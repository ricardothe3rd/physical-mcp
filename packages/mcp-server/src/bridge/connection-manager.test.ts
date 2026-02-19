import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

// Mock WSClient before importing ConnectionManager
vi.mock('./ws-client.js', () => {
  return {
    WSClient: class MockWSClient {
      isConnected = true;
      connect = vi.fn().mockResolvedValue(undefined);
      send = vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() });
      disconnect = vi.fn();
    },
  };
});

// Mock ErrorRecovery to avoid delays
vi.mock('../utils/error-recovery.js', () => ({
  ErrorRecovery: {
    withRetry: vi.fn(async (fn: () => Promise<unknown>) => fn()),
    createCircuitBreaker: vi.fn(() => ({
      call: vi.fn(async (fn: () => Promise<unknown>) => fn()),
      isOpen: vi.fn(() => false),
      reset: vi.fn(),
    })),
  },
}));

import { ConnectionManager } from './connection-manager.js';

describe('ConnectionManager', () => {
  let manager: ConnectionManager;

  beforeEach(() => {
    vi.useFakeTimers();
    manager = new ConnectionManager('ws://localhost:9090');
  });

  afterEach(() => {
    manager.disconnect();
    vi.useRealTimers();
  });

  it('creates with default URL', () => {
    const m = new ConnectionManager();
    expect(m).toBeDefined();
    m.disconnect();
  });

  it('creates with custom URL', () => {
    const m = new ConnectionManager('ws://custom:1234');
    expect(m).toBeDefined();
    m.disconnect();
  });

  it('connects successfully', async () => {
    await manager.connect();
    expect(manager.isConnected).toBe(true);
  });

  it('sends commands through WebSocket client', async () => {
    await manager.connect();
    const result = await manager.send('ping');
    expect(result).toBeDefined();
    expect(result.status).toBe('ok');
  });

  it('reports connection status', () => {
    // With mocked WSClient that returns isConnected: true
    expect(manager.isConnected).toBe(true);
  });

  it('reports bridge availability', () => {
    expect(manager.isBridgeAvailable).toBe(true);
  });

  it('disconnects cleanly', async () => {
    await manager.connect();
    manager.disconnect();
    // Should not throw
  });
});
