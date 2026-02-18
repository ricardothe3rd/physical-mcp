#!/usr/bin/env node

/**
 * WebSocket client that sends commands to the Python ROS2 bridge.
 */

import WebSocket from 'ws';
import { randomUUID } from 'crypto';
import { BridgeResponseSchema, type CommandTypeValue } from './protocol.js';

export interface BridgeResponse {
  id: string | null;
  status: 'ok' | 'error';
  data: unknown;
  timestamp: number;
}

/**
 * WebSocket client for communicating with the Python ROS2 bridge.
 *
 * Handles connection lifecycle, request/response correlation via UUIDs,
 * heartbeat pings, and stale connection detection. Each `send()` call
 * returns a promise that resolves when the bridge responds or rejects
 * on timeout.
 */
export class WSClient {
  private ws: WebSocket | null = null;
  private pendingRequests = new Map<string, {
    resolve: (value: BridgeResponse) => void;
    reject: (reason: Error) => void;
    timer: ReturnType<typeof setTimeout>;
  }>();
  private url: string;
  private requestTimeout: number;
  private heartbeatInterval: ReturnType<typeof setInterval> | null = null;
  private lastPongTime = 0;

  constructor(url = 'ws://localhost:9090', requestTimeout = 10000) {
    this.url = url;
    this.requestTimeout = requestTimeout;
  }

  async connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(this.url);

      this.ws.on('open', () => {
        console.error(`[WSClient] Connected to bridge at ${this.url}`);
        this.lastPongTime = Date.now();
        this.startHeartbeat();
        resolve();
      });

      this.ws.on('message', (data) => {
        this.handleMessage(data.toString());
      });

      this.ws.on('pong', () => {
        this.lastPongTime = Date.now();
      });

      this.ws.on('error', (err) => {
        console.error(`[WSClient] WebSocket error: ${err.message}`);
        reject(err);
      });

      this.ws.on('close', (code, reason) => {
        console.error(`[WSClient] Connection closed: ${code} ${reason}`);
        this.stopHeartbeat();
        this.rejectAllPending(new Error('Connection closed'));
        this.ws = null;
      });
    });
  }

  private startHeartbeat() {
    this.stopHeartbeat();
    this.heartbeatInterval = setInterval(() => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;

      // If no pong received in 30s, connection is stale
      if (Date.now() - this.lastPongTime > 30000) {
        console.error('[WSClient] Heartbeat timeout — closing stale connection');
        this.ws.terminate();
        return;
      }

      this.ws.ping();
    }, 15000); // ping every 15s
  }

  private stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  async send(type: CommandTypeValue, params: Record<string, unknown> = {}): Promise<BridgeResponse> {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error('Not connected to bridge');
    }

    const id = randomUUID();
    const command = JSON.stringify({ id, type, params });

    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingRequests.delete(id);
        reject(new Error(`Request ${id} timed out after ${this.requestTimeout}ms`));
      }, this.requestTimeout);

      this.pendingRequests.set(id, { resolve, reject, timer });
      this.ws!.send(command);
    });
  }

  private handleMessage(raw: string) {
    try {
      const parsed = JSON.parse(raw);
      const response = BridgeResponseSchema.parse(parsed);

      if (response.id && this.pendingRequests.has(response.id)) {
        const pending = this.pendingRequests.get(response.id)!;
        clearTimeout(pending.timer);
        this.pendingRequests.delete(response.id);
        pending.resolve(response as BridgeResponse);
      }
    } catch (err) {
      console.error(`[WSClient] Failed to parse message: ${err}`);
    }
  }

  private rejectAllPending(error: Error) {
    for (const [id, pending] of this.pendingRequests) {
      clearTimeout(pending.timer);
      pending.reject(error);
    }
    this.pendingRequests.clear();
  }

  disconnect() {
    this.stopHeartbeat();
    if (this.ws) {
      this.rejectAllPending(new Error('Disconnecting'));
      this.ws.close();
      this.ws = null;
    }
  }

  get isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN;
  }
}
