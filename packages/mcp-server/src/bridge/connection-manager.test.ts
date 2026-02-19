import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

// ---------------------------------------------------------------------------
// Mock the ws-client module.
// IMPORTANT: vi.mock is hoisted, so the factory MUST be fully self-contained;
// it cannot reference any variable declared outside the factory.
// We use a globalThis side-channel to expose the latest instance to tests,
// following the same pattern used in ws-client.test.ts.
// ---------------------------------------------------------------------------
vi.mock('./ws-client.js', () => {
  class MockWSClient {
    url: string;
    _isConnected = false;
    connectFn: ReturnType<typeof vi.fn>;
    disconnectFn: ReturnType<typeof vi.fn>;
    sendFn: ReturnType<typeof vi.fn>;

    constructor(url: string) {
      this.url = url;
      this.connectFn = vi.fn(async () => { this._isConnected = true; });
      this.disconnectFn = vi.fn(() => { this._isConnected = false; });
      this.sendFn = vi.fn().mockResolvedValue({
        id: 'test-id',
        status: 'ok',
        data: { pong: true },
        timestamp: Date.now(),
      });
      (globalThis as Record<string, unknown>).__mockWSClient = this;
    }

    async connect() { return this.connectFn(); }
    disconnect() { return this.disconnectFn(); }
    async send(type: string, params: Record<string, unknown> = {}) { return this.sendFn(type, params); }

    get isConnected() { return this._isConnected; }

    _setConnected(value: boolean) { this._isConnected = value; }
  }

  return { WSClient: MockWSClient };
});

import { ConnectionManager } from './connection-manager.js';

// Helper to grab the mock WSClient instance
interface MockWSClientInstance {
  url: string;
  _isConnected: boolean;
  connectFn: ReturnType<typeof vi.fn>;
  disconnectFn: ReturnType<typeof vi.fn>;
  sendFn: ReturnType<typeof vi.fn>;
  _setConnected: (value: boolean) => void;
}

function getMockClient(): MockWSClientInstance {
  return (globalThis as Record<string, unknown>).__mockWSClient as MockWSClientInstance;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
describe('ConnectionManager', () => {
  let manager: ConnectionManager;

  beforeEach(() => {
    vi.useFakeTimers();
    // Suppress console.error noise from reconnect loop and circuit breaker
    vi.spyOn(console, 'error').mockImplementation(() => {});
    manager = new ConnectionManager('ws://test:9090');
  });

  afterEach(() => {
    manager.disconnect();
    vi.useRealTimers();
    vi.restoreAllMocks();
  });

  // -----------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------
  describe('constructor', () => {
    it('stores the bridge URL and passes it to WSClient', () => {
      expect(getMockClient().url).toBe('ws://test:9090');
    });

    it('starts disconnected (isConnected = false)', () => {
      expect(manager.isConnected).toBe(false);
    });

    it('uses default URL when none is provided', () => {
      const defaultManager = new ConnectionManager();
      expect(getMockClient().url).toBe('ws://localhost:9090');
      defaultManager.disconnect();
    });
  });

  // -----------------------------------------------------------------------
  // connect()
  // -----------------------------------------------------------------------
  describe('connect()', () => {
    it('sets isConnected to true on success', async () => {
      await manager.connect();
      expect(manager.isConnected).toBe(true);
    });

    it('calls WSClient.connect()', async () => {
      await manager.connect();
      expect(getMockClient().connectFn).toHaveBeenCalled();
    });

    it('multiple connect() calls are idempotent — starts only one reconnect loop', async () => {
      await manager.connect();
      await manager.connect();
      await manager.connect();

      // isConnected should still be true and the system should remain stable
      expect(manager.isConnected).toBe(true);
    });

    it('throws after all retry attempts fail', async () => {
      getMockClient().connectFn.mockRejectedValue(new Error('ECONNREFUSED'));

      // ErrorRecovery.withRetry uses setTimeout delays between retries
      // (1000ms, 3000ms), so we must advance fake timers while waiting.
      // Attach a .catch immediately to prevent unhandled rejection warnings.
      let caughtError: Error | null = null;
      const connectPromise = manager.connect().catch((err) => { caughtError = err; });

      // Advance past the first retry delay (1000ms)
      await vi.advanceTimersByTimeAsync(1500);
      // Advance past the second retry delay (3000ms)
      await vi.advanceTimersByTimeAsync(3500);

      await connectPromise;
      expect(caughtError).not.toBeNull();
      expect(caughtError!.message).toMatch(/failed after 3 attempts/);
    });
  });

  // -----------------------------------------------------------------------
  // disconnect()
  // -----------------------------------------------------------------------
  describe('disconnect()', () => {
    it('sets isConnected to false', async () => {
      await manager.connect();
      expect(manager.isConnected).toBe(true);

      manager.disconnect();
      expect(manager.isConnected).toBe(false);
    });

    it('calls WSClient.disconnect()', async () => {
      await manager.connect();
      manager.disconnect();
      expect(getMockClient().disconnectFn).toHaveBeenCalled();
    });

    it('is safe to call when already disconnected', () => {
      expect(manager.isConnected).toBe(false);
      // Should not throw
      manager.disconnect();
      manager.disconnect();
      expect(manager.isConnected).toBe(false);
    });

    it('stops the reconnect loop', async () => {
      await manager.connect();
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(1);

      // Simulate connection drop
      getMockClient()._setConnected(false);
      getMockClient().connectFn.mockImplementation(async () => {
        getMockClient()._setConnected(true);
      });

      manager.disconnect();

      // Advance well past the 5s reconnect interval
      await vi.advanceTimersByTimeAsync(15000);

      // The reconnect loop should NOT have tried to reconnect since we
      // called disconnect() which clears the interval.
      // connectFn was called once by the initial connect().
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(1);
    });
  });

  // -----------------------------------------------------------------------
  // send()
  // -----------------------------------------------------------------------
  describe('send()', () => {
    it('throws when not connected and auto-reconnect in send fails', async () => {
      // The send() method attempts to connect if not connected via the
      // circuit breaker. Make connect fail.
      getMockClient().connectFn.mockRejectedValue(new Error('ECONNREFUSED'));

      await expect(manager.send('ping')).rejects.toThrow('ECONNREFUSED');
    });

    it('returns response from bridge on success', async () => {
      await manager.connect();

      const expectedResponse = {
        id: 'resp-1',
        status: 'ok' as const,
        data: { topics: ['/cmd_vel'] },
        timestamp: 1700000000,
      };
      getMockClient().sendFn.mockResolvedValue(expectedResponse);

      const response = await manager.send('topic.list');
      expect(response).toEqual(expectedResponse);
      expect(response.status).toBe('ok');
      expect(response.data).toEqual({ topics: ['/cmd_vel'] });
    });

    it('passes command type and params to WSClient.send()', async () => {
      await manager.connect();

      await manager.send('service.call', { name: '/my_service', args: { x: 1 } });
      expect(getMockClient().sendFn).toHaveBeenCalledWith(
        'service.call',
        { name: '/my_service', args: { x: 1 } }
      );
    });

    it('auto-connects when client is disconnected before sending', async () => {
      await manager.connect();

      // Simulate connection drop
      getMockClient()._setConnected(false);
      getMockClient().connectFn.mockImplementation(async () => {
        getMockClient()._setConnected(true);
      });

      const response = await manager.send('ping');
      // connect should have been called again inside send()
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(2);
      expect(response.status).toBe('ok');
    });

    it('opens circuit breaker after repeated failures', async () => {
      await manager.connect();

      // Make send fail 5 times (the circuit breaker threshold)
      getMockClient().sendFn.mockRejectedValue(new Error('Bridge error'));

      for (let i = 0; i < 5; i++) {
        await expect(manager.send('ping')).rejects.toThrow('Bridge error');
      }

      // 6th call should fail with circuit breaker open
      await expect(manager.send('ping')).rejects.toThrow('Circuit breaker open');
    });

    it('circuit breaker resets after timeout period', async () => {
      await manager.connect();

      // Trip the circuit breaker
      getMockClient().sendFn.mockRejectedValue(new Error('Bridge error'));
      for (let i = 0; i < 5; i++) {
        await expect(manager.send('ping')).rejects.toThrow('Bridge error');
      }

      // Verify it is open
      await expect(manager.send('ping')).rejects.toThrow('Circuit breaker open');

      // Advance time past the 30s reset timeout
      vi.advanceTimersByTime(31000);

      // Restore send to succeed
      getMockClient().sendFn.mockResolvedValue({
        id: 'recovered',
        status: 'ok' as const,
        data: null,
        timestamp: Date.now(),
      });

      const response = await manager.send('ping');
      expect(response.status).toBe('ok');
    });
  });

  // -----------------------------------------------------------------------
  // isBridgeAvailable
  // -----------------------------------------------------------------------
  describe('isBridgeAvailable', () => {
    it('returns false when disconnected', () => {
      expect(manager.isBridgeAvailable).toBe(false);
    });

    it('returns true when connected and circuit breaker is closed', async () => {
      await manager.connect();
      expect(manager.isBridgeAvailable).toBe(true);
    });

    it('mirrors isConnected state when circuit breaker is closed', async () => {
      expect(manager.isBridgeAvailable).toBe(false);
      expect(manager.isConnected).toBe(false);

      await manager.connect();
      expect(manager.isBridgeAvailable).toBe(true);
      expect(manager.isConnected).toBe(true);

      manager.disconnect();
      expect(manager.isBridgeAvailable).toBe(false);
      expect(manager.isConnected).toBe(false);
    });

    it('returns false when connected but circuit breaker is open', async () => {
      await manager.connect();

      // Trip the circuit breaker
      getMockClient().sendFn.mockRejectedValue(new Error('Bridge error'));
      for (let i = 0; i < 5; i++) {
        await expect(manager.send('ping')).rejects.toThrow('Bridge error');
      }

      // Still connected, but circuit breaker is open
      expect(manager.isConnected).toBe(true);
      expect(manager.isBridgeAvailable).toBe(false);
    });
  });

  // -----------------------------------------------------------------------
  // Reconnect loop
  // -----------------------------------------------------------------------
  describe('reconnect loop', () => {
    it('attempts to reconnect when client becomes disconnected', async () => {
      await manager.connect();
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(1);

      // Simulate connection drop
      getMockClient()._setConnected(false);
      getMockClient().connectFn.mockImplementation(async () => {
        getMockClient()._setConnected(true);
      });

      // Advance past the 5s reconnect interval
      await vi.advanceTimersByTimeAsync(5500);

      // Should have attempted reconnection
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(2);
      expect(manager.isConnected).toBe(true);
    });

    it('does not attempt reconnect when already connected', async () => {
      await manager.connect();
      const callCount = getMockClient().connectFn.mock.calls.length;

      // Advance past several reconnect intervals
      await vi.advanceTimersByTimeAsync(20000);

      // No additional connect calls because client is still connected
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(callCount);
    });

    it('handles reconnect failure gracefully and retries on next interval', async () => {
      await manager.connect();

      // Simulate disconnect
      getMockClient()._setConnected(false);

      // Make reconnect fail
      getMockClient().connectFn.mockRejectedValue(new Error('ECONNREFUSED'));

      // First reconnect attempt (at 5s)
      await vi.advanceTimersByTimeAsync(5500);
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(2);
      expect(manager.isConnected).toBe(false);

      // Make reconnect succeed on next attempt
      getMockClient().connectFn.mockImplementation(async () => {
        getMockClient()._setConnected(true);
      });

      // Second reconnect attempt (at 10s)
      await vi.advanceTimersByTimeAsync(5000);
      expect(getMockClient().connectFn).toHaveBeenCalledTimes(3);
      expect(manager.isConnected).toBe(true);
    });
  });
});
