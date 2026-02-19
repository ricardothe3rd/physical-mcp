import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { WSClient } from './ws-client.js';

// ---------------------------------------------------------------------------
// The most recently constructed mock WS instance so tests can drive events.
// This is set by the mock factory below.
// ---------------------------------------------------------------------------
let latestMockWs: {
  readyState: number;
  lastSentData: string | null;
  emit: (event: string, ...args: unknown[]) => void;
};

// ---------------------------------------------------------------------------
// Mock the 'ws' module.
// IMPORTANT: vi.mock is hoisted, so the factory MUST be fully self-contained;
// it cannot reference any variable declared outside the factory.
// We use a globalThis side-channel to expose the latest instance to tests.
// ---------------------------------------------------------------------------
vi.mock('ws', () => {
  const OPEN = 1;
  const CLOSED = 3;
  const CONNECTING = 0;

  function MockWebSocketFactory() {
    const handlers = new Map<string, ((...args: unknown[]) => void)[]>();

    const ws = {
      readyState: OPEN,
      lastSentData: null as string | null,

      on(event: string, handler: (...args: unknown[]) => void) {
        if (!handlers.has(event)) handlers.set(event, []);
        handlers.get(event)!.push(handler);
      },

      emit(event: string, ...args: unknown[]) {
        const h = handlers.get(event);
        if (h) h.forEach(fn => fn(...args));
      },

      send(data: string) {
        ws.lastSentData = data;
      },

      ping() {
        /* no-op */
      },

      close() {
        ws.readyState = CLOSED;
        ws.emit('close', 1000, 'Normal');
      },

      terminate() {
        ws.readyState = CLOSED;
        ws.emit('close', 1006, 'Terminated');
      },
    };

    // Expose instance to tests via globalThis
    (globalThis as Record<string, unknown>).__latestMockWs = ws;

    // Emit 'open' on the next microtask so WSClient can register its handlers first
    queueMicrotask(() => ws.emit('open'));
    return ws;
  }

  MockWebSocketFactory.OPEN = OPEN;
  MockWebSocketFactory.CLOSED = CLOSED;
  MockWebSocketFactory.CONNECTING = CONNECTING;

  return { default: MockWebSocketFactory };
});

// Helper to grab the latest instance after connect()
function getMockWs() {
  return (globalThis as Record<string, unknown>).__latestMockWs as typeof latestMockWs;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
describe('WSClient', () => {
  let client: WSClient;

  beforeEach(() => {
    vi.useFakeTimers();
    client = new WSClient('ws://localhost:9090', 500);
  });

  afterEach(() => {
    client.disconnect();
    vi.useRealTimers();
  });

  // -----------------------------------------------------------------------
  // isConnected
  // -----------------------------------------------------------------------
  describe('isConnected', () => {
    it('returns false before connect() is called', () => {
      expect(client.isConnected).toBe(false);
    });

    it('returns true after a successful connect()', async () => {
      await client.connect();
      expect(client.isConnected).toBe(true);
    });

    it('returns false after disconnect()', async () => {
      await client.connect();
      client.disconnect();
      expect(client.isConnected).toBe(false);
    });
  });

  // -----------------------------------------------------------------------
  // send() throws when not connected
  // -----------------------------------------------------------------------
  describe('send() when not connected', () => {
    it('throws "Not connected to bridge" before connect', async () => {
      await expect(client.send('ping')).rejects.toThrow('Not connected to bridge');
    });

    it('throws after disconnect', async () => {
      await client.connect();
      client.disconnect();
      await expect(client.send('topic.list')).rejects.toThrow('Not connected to bridge');
    });
  });

  // -----------------------------------------------------------------------
  // send() + handleMessage with a valid response
  // -----------------------------------------------------------------------
  describe('handleMessage with valid response', () => {
    it('resolves the pending request when a matching response arrives', async () => {
      await client.connect();
      const ws = getMockWs();

      const sendPromise = client.send('ping');

      // Inspect the command that was sent over the wire
      const sentCommand = JSON.parse(ws.lastSentData!);
      expect(sentCommand.type).toBe('ping');
      expect(sentCommand.params).toEqual({});
      expect(typeof sentCommand.id).toBe('string');

      // Simulate bridge response
      ws.emit(
        'message',
        JSON.stringify({
          id: sentCommand.id,
          status: 'ok',
          data: { pong: true },
          timestamp: 1700000000,
        }),
      );

      const response = await sendPromise;
      expect(response.status).toBe('ok');
      expect(response.data).toEqual({ pong: true });
      expect(response.id).toBe(sentCommand.id);
    });

    it('resolves with error status when bridge returns error response', async () => {
      await client.connect();
      const ws = getMockWs();

      const sendPromise = client.send('service.call', { name: '/missing' });
      const sentCommand = JSON.parse(ws.lastSentData!);

      ws.emit(
        'message',
        JSON.stringify({
          id: sentCommand.id,
          status: 'error',
          data: 'service not found',
          timestamp: 1700000000,
        }),
      );

      const response = await sendPromise;
      expect(response.status).toBe('error');
      expect(response.data).toBe('service not found');
    });
  });

  // -----------------------------------------------------------------------
  // handleMessage with malformed JSON
  // -----------------------------------------------------------------------
  describe('handleMessage with malformed JSON', () => {
    it('does not throw; logs error to stderr', async () => {
      await client.connect();
      const ws = getMockWs();
      const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});

      ws.emit('message', '<<<not json>>>');

      expect(consoleSpy).toHaveBeenCalledWith(
        expect.stringContaining('[WSClient] Failed to parse message'),
      );
      consoleSpy.mockRestore();
    });

    it('does not affect subsequent valid messages', async () => {
      await client.connect();
      const ws = getMockWs();
      const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});

      const sendPromise = client.send('ping');
      const sentCommand = JSON.parse(ws.lastSentData!);

      // Inject garbage first
      ws.emit('message', '{{{bad');

      // Then the real response
      ws.emit(
        'message',
        JSON.stringify({
          id: sentCommand.id,
          status: 'ok',
          data: null,
          timestamp: 123,
        }),
      );

      const response = await sendPromise;
      expect(response.status).toBe('ok');
      consoleSpy.mockRestore();
    });
  });

  // -----------------------------------------------------------------------
  // handleMessage with valid JSON but invalid schema
  // -----------------------------------------------------------------------
  describe('handleMessage with valid JSON but invalid schema', () => {
    it('logs error when response does not match BridgeResponseSchema', async () => {
      await client.connect();
      const ws = getMockWs();
      const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});

      ws.emit('message', JSON.stringify({ foo: 'bar' }));

      expect(consoleSpy).toHaveBeenCalledWith(
        expect.stringContaining('[WSClient] Failed to parse message'),
      );
      consoleSpy.mockRestore();
    });
  });

  // -----------------------------------------------------------------------
  // handleMessage with unknown request ID
  // -----------------------------------------------------------------------
  describe('handleMessage with unknown request ID', () => {
    it('silently ignores a response whose ID is not in pending requests', async () => {
      await client.connect();
      const ws = getMockWs();
      const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});

      ws.emit(
        'message',
        JSON.stringify({
          id: 'unknown-id-999',
          status: 'ok',
          data: null,
          timestamp: 123,
        }),
      );

      // Should NOT have logged an error (valid parse, just no matching request)
      expect(consoleSpy).not.toHaveBeenCalled();
      consoleSpy.mockRestore();
    });

    it('does not resolve any other pending request', async () => {
      await client.connect();
      const ws = getMockWs();

      const sendPromise = client.send('ping');

      // Send a response with a different ID
      ws.emit(
        'message',
        JSON.stringify({
          id: 'different-id',
          status: 'ok',
          data: null,
          timestamp: 0,
        }),
      );

      // The original request should still be pending; advance time to trigger timeout
      vi.advanceTimersByTime(600);

      await expect(sendPromise).rejects.toThrow('timed out');
    });
  });

  // -----------------------------------------------------------------------
  // Request timeout behaviour
  // -----------------------------------------------------------------------
  describe('request timeout', () => {
    it('rejects the promise after the configured timeout', async () => {
      await client.connect();

      const sendPromise = client.send('topic.list');

      vi.advanceTimersByTime(600);

      await expect(sendPromise).rejects.toThrow(/timed out after 500ms/);
    });

    it('includes the request ID in the timeout error message', async () => {
      await client.connect();
      const ws = getMockWs();

      const sendPromise = client.send('ping');
      const sentCommand = JSON.parse(ws.lastSentData!);

      vi.advanceTimersByTime(600);

      await expect(sendPromise).rejects.toThrow(sentCommand.id);
    });

    it('cleans up the pending request entry on timeout', async () => {
      await client.connect();
      const ws = getMockWs();

      const sendPromise = client.send('ping');
      const sentCommand = JSON.parse(ws.lastSentData!);

      vi.advanceTimersByTime(600);

      // Eat the rejection
      await sendPromise.catch(() => {});

      // Now deliver a late response -- nothing should happen (no unhandled rejection)
      ws.emit(
        'message',
        JSON.stringify({
          id: sentCommand.id,
          status: 'ok',
          data: null,
          timestamp: 0,
        }),
      );
    });
  });

  // -----------------------------------------------------------------------
  // disconnect() cleans up resources
  // -----------------------------------------------------------------------
  describe('disconnect()', () => {
    it('sets isConnected to false', async () => {
      await client.connect();
      expect(client.isConnected).toBe(true);
      client.disconnect();
      expect(client.isConnected).toBe(false);
    });

    it('rejects all pending requests with "Disconnecting"', async () => {
      await client.connect();

      const p1 = client.send('ping');
      const p2 = client.send('topic.list');

      client.disconnect();

      await expect(p1).rejects.toThrow('Disconnecting');
      await expect(p2).rejects.toThrow('Disconnecting');
    });

    it('is safe to call multiple times', async () => {
      await client.connect();
      client.disconnect();
      client.disconnect();
      client.disconnect();
      expect(client.isConnected).toBe(false);
    });

    it('is safe to call without ever connecting', () => {
      client.disconnect();
      expect(client.isConnected).toBe(false);
    });
  });

  // -----------------------------------------------------------------------
  // rejectAllPending on connection close
  // -----------------------------------------------------------------------
  describe('rejectAllPending on connection close', () => {
    it('rejects all pending requests when the WebSocket closes unexpectedly', async () => {
      await client.connect();
      const ws = getMockWs();

      const p1 = client.send('ping');
      const p2 = client.send('service.list');

      // Simulate unexpected server-side close
      ws.readyState = 3; // CLOSED
      ws.emit('close', 1006, 'Abnormal');

      await expect(p1).rejects.toThrow('Connection closed');
      await expect(p2).rejects.toThrow('Connection closed');
    });

    it('sets isConnected to false after close event', async () => {
      await client.connect();
      const ws = getMockWs();
      expect(client.isConnected).toBe(true);

      ws.readyState = 3; // CLOSED
      ws.emit('close', 1000, 'Normal');

      expect(client.isConnected).toBe(false);
    });
  });

  // -----------------------------------------------------------------------
  // Constructor defaults
  // -----------------------------------------------------------------------
  describe('constructor defaults', () => {
    it('uses ws://localhost:9090 and 10000ms timeout by default', () => {
      const defaultClient = new WSClient();
      expect(defaultClient.isConnected).toBe(false);
      defaultClient.disconnect();
    });

    it('accepts a custom URL and timeout', () => {
      const custom = new WSClient('ws://10.0.0.1:5555', 30000);
      expect(custom.isConnected).toBe(false);
      custom.disconnect();
    });
  });

  // -----------------------------------------------------------------------
  // Multiple concurrent requests
  // -----------------------------------------------------------------------
  describe('multiple concurrent requests', () => {
    it('correctly routes responses to the right pending request', async () => {
      await client.connect();
      const ws = getMockWs();

      // Send two requests
      const p1 = client.send('topic.list');
      const cmd1Id = JSON.parse(ws.lastSentData!).id;

      const p2 = client.send('service.list');
      const cmd2Id = JSON.parse(ws.lastSentData!).id;

      // Respond in reverse order
      ws.emit(
        'message',
        JSON.stringify({
          id: cmd2Id,
          status: 'ok',
          data: { services: ['a'] },
          timestamp: 2,
        }),
      );
      ws.emit(
        'message',
        JSON.stringify({
          id: cmd1Id,
          status: 'ok',
          data: { topics: ['b'] },
          timestamp: 1,
        }),
      );

      const r1 = await p1;
      const r2 = await p2;

      expect(r1.data).toEqual({ topics: ['b'] });
      expect(r2.data).toEqual({ services: ['a'] });
    });
  });
});
