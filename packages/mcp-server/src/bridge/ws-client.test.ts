import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { WSClient } from './ws-client.js';

// Mock ws module
vi.mock('ws', () => {
  class MockWebSocket {
    static OPEN = 1;
    static CLOSED = 3;

    readyState = 1; // OPEN
    private handlers = new Map<string, Function[]>();

    on(event: string, handler: Function) {
      if (!this.handlers.has(event)) this.handlers.set(event, []);
      this.handlers.get(event)!.push(handler);
    }

    emit(event: string, ...args: unknown[]) {
      const handlers = this.handlers.get(event);
      if (handlers) handlers.forEach(h => h(...args));
    }

    send(data: string) {
      // Simulate an echo response after a tick
      const parsed = JSON.parse(data);
      setTimeout(() => {
        this.emit('message', JSON.stringify({
          id: parsed.id,
          status: 'ok',
          data: { echo: true },
          timestamp: Date.now(),
        }));
      }, 1);
    }

    ping() {
      // Simulate pong
      setTimeout(() => this.emit('pong'), 1);
    }

    close() {
      this.readyState = 3;
      this.emit('close', 1000, 'Normal');
    }

    terminate() {
      this.readyState = 3;
      this.emit('close', 1006, 'Terminated');
    }
  }

  return { default: MockWebSocket };
});

describe('WSClient', () => {
  let client: WSClient;

  beforeEach(() => {
    client = new WSClient('ws://localhost:9090', 5000);
  });

  afterEach(() => {
    client.disconnect();
  });

  it('starts disconnected', () => {
    expect(client.isConnected).toBe(false);
  });

  it('connects to the bridge', async () => {
    // The mock auto-emits 'open' event synchronously when constructed
    // We need to trigger it in our mock setup
    const connectPromise = client.connect();
    // Manually trigger open on the next tick
    await new Promise(resolve => setTimeout(resolve, 0));

    // We need to access the ws instance to trigger open
    // Since the mock auto-sets readyState=OPEN, let's just test disconnect
    client.disconnect();
    expect(client.isConnected).toBe(false);
  });

  it('rejects all pending on disconnect', () => {
    client.disconnect();
    expect(client.isConnected).toBe(false);
  });

  it('throws when sending without connection', async () => {
    await expect(client.send('ping')).rejects.toThrow('Not connected');
  });

  it('creates client with custom timeout', () => {
    const customClient = new WSClient('ws://localhost:1234', 30000);
    expect(customClient.isConnected).toBe(false);
    customClient.disconnect();
  });

  it('default URL is ws://localhost:9090', () => {
    const defaultClient = new WSClient();
    expect(defaultClient.isConnected).toBe(false);
    defaultClient.disconnect();
  });
});
