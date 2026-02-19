/**
 * Manages WebSocket connection to the Python ROS2 bridge with
 * automatic reconnection and circuit breaker.
 */

import { WSClient, type BridgeResponse } from './ws-client.js';
import { type CommandTypeValue } from './protocol.js';
import { ErrorRecovery } from '../utils/error-recovery.js';

/**
 * Manages the WebSocket connection to the Python ROS2 bridge.
 *
 * Wraps WSClient with automatic reconnection (every 5s) and a circuit
 * breaker that prevents cascading failures when the bridge is down.
 * All MCP tool handlers send commands through this manager.
 */
export class ConnectionManager {
  private client: WSClient;
  private circuitBreaker: ReturnType<typeof ErrorRecovery.createCircuitBreaker>;
  private reconnecting = false;
  private reconnectInterval: ReturnType<typeof setInterval> | null = null;
  private url: string;

  /**
   * @param url - WebSocket URL of the ROS2 bridge (default: ws://localhost:9090)
   */
  constructor(url = 'ws://localhost:9090') {
    this.url = url;
    this.client = new WSClient(url);
    this.circuitBreaker = ErrorRecovery.createCircuitBreaker(5, 30000);
  }

  /**
   * Connect to the ROS2 bridge with automatic retry.
   * Starts a background reconnect loop that attempts to restore the connection every 5 seconds.
   */
  async connect(): Promise<void> {
    await ErrorRecovery.withRetry(
      () => this.client.connect(),
      { component: 'ConnectionManager', operation: 'connect' }
    );
    this.startReconnectLoop();
  }

  /**
   * Send a command to the ROS2 bridge through the circuit breaker.
   * Automatically reconnects if the WebSocket is not connected.
   * @param type - The bridge command type (e.g., "publish", "subscribe", "call_service")
   * @param params - Command parameters to send
   * @returns The bridge response
   * @throws When the circuit breaker is open or the request times out
   */
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

  /** Whether the underlying WebSocket is currently open. */
  get isConnected(): boolean {
    return this.client.isConnected;
  }

  /** Whether the bridge is connected and the circuit breaker is closed (healthy). */
  get isBridgeAvailable(): boolean {
    return this.client.isConnected && !this.circuitBreaker.isOpen();
  }

  /** Disconnect from the bridge and stop the automatic reconnect loop. */
  disconnect() {
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
    this.client.disconnect();
  }
}
