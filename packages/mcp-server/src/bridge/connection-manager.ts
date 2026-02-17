/**
 * Manages WebSocket connection to the Python ROS2 bridge with
 * automatic reconnection and circuit breaker.
 */

import { WSClient, type BridgeResponse } from './ws-client.js';
import { type CommandTypeValue } from './protocol.js';
import { ErrorRecovery } from '../utils/error-recovery.js';

export class ConnectionManager {
  private client: WSClient;
  private circuitBreaker: ReturnType<typeof ErrorRecovery.createCircuitBreaker>;
  private reconnecting = false;
  private reconnectInterval: ReturnType<typeof setInterval> | null = null;
  private url: string;

  constructor(url = 'ws://localhost:9090') {
    this.url = url;
    this.client = new WSClient(url);
    this.circuitBreaker = ErrorRecovery.createCircuitBreaker(5, 30000);
  }

  async connect(): Promise<void> {
    await ErrorRecovery.withRetry(
      () => this.client.connect(),
      { component: 'ConnectionManager', operation: 'connect' }
    );
    this.startReconnectLoop();
  }

  async send(type: CommandTypeValue, params: Record<string, unknown> = {}): Promise<BridgeResponse> {
    return this.circuitBreaker.call(async () => {
      if (!this.client.isConnected) {
        await this.client.connect();
      }
      return this.client.send(type, params);
    });
  }

  private startReconnectLoop() {
    if (this.reconnectInterval) return;

    this.reconnectInterval = setInterval(async () => {
      if (!this.client.isConnected && !this.reconnecting) {
        this.reconnecting = true;
        console.error('[ConnectionManager] Bridge disconnected, attempting reconnect...');
        try {
          await this.client.connect();
          console.error('[ConnectionManager] Reconnected to bridge');
        } catch {
          // Will retry on next interval
        } finally {
          this.reconnecting = false;
        }
      }
    }, 5000);
  }

  get isConnected(): boolean {
    return this.client.isConnected;
  }

  get isBridgeAvailable(): boolean {
    return this.client.isConnected && !this.circuitBreaker.isOpen();
  }

  disconnect() {
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
    this.client.disconnect();
  }
}
