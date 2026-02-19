/**
 * WebSocket heartbeat / keepalive manager.
 *
 * Sends periodic pings and tracks pong responses to detect stale
 * connections. After `maxMissed` consecutive missed pongs the
 * `onTimeout` callback fires and the heartbeat stops automatically.
 */

/** Options accepted by {@link HeartbeatManager}. */
export interface HeartbeatOptions {
  /** Milliseconds between pings. @default 10000 */
  intervalMs?: number;
  /** Milliseconds to wait for a pong after each ping. @default 5000 */
  timeoutMs?: number;
  /** Consecutive missed pongs before declaring the connection dead. @default 3 */
  maxMissed?: number;
  /** Called when `maxMissed` consecutive pongs are missed. */
  onTimeout?: () => void;
  /** Called every time a pong is received, with the round-trip latency. */
  onPong?: (latencyMs: number) => void;
}

const RING_BUFFER_SIZE = 10;

export class HeartbeatManager {
  // ── Configuration ───────────────────────────────────────────────
  private readonly intervalMs: number;
  private readonly timeoutMs: number;
  private readonly maxMissedThreshold: number;
  private readonly onTimeout?: () => void;
  private readonly onPongCb?: (latencyMs: number) => void;

  // ── Timers ──────────────────────────────────────────────────────
  private pingTimer: ReturnType<typeof setInterval> | null = null;
  private pongTimer: ReturnType<typeof setTimeout> | null = null;

  // ── State ───────────────────────────────────────────────────────
  private _missedCount = 0;
  private _latencyMs = 0;
  private _isRunning = false;
  private pingSentAt = 0;
  private waitingForPong = false;

  // ── Ring buffer for rolling average ─────────────────────────────
  private latencyBuffer: number[] = [];
  private latencyBufferIndex = 0;
  private latencyBufferFilled = false;

  constructor(options: HeartbeatOptions = {}) {
    this.intervalMs = options.intervalMs ?? 10_000;
    this.timeoutMs = options.timeoutMs ?? 5_000;
    this.maxMissedThreshold = options.maxMissed ?? 3;
    this.onTimeout = options.onTimeout;
    this.onPongCb = options.onPong;
    this.latencyBuffer = new Array<number>(RING_BUFFER_SIZE).fill(0);
  }

  // ── Public API ──────────────────────────────────────────────────

  /**
   * Start sending periodic pings.
   *
   * If already running this is a no-op (idempotent).
   *
   * @param sendPing - async function that transmits a ping frame to the peer.
   */
  start(sendPing: () => Promise<void>): void {
    if (this._isRunning) return;
    this._isRunning = true;
    this._missedCount = 0;

    this.pingTimer = setInterval(() => {
      this.sendOnePing(sendPing);
    }, this.intervalMs);
  }

  /** Stop all timers and reset running state. Safe to call multiple times. */
  stop(): void {
    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }
    this.clearPongTimer();
    this._isRunning = false;
    this.waitingForPong = false;
  }

  /**
   * Notify the manager that a pong was received from the peer.
   * Must be called by the consumer when a pong frame arrives.
   */
  receivePong(): void {
    if (!this.waitingForPong) return;

    this.waitingForPong = false;
    this.clearPongTimer();

    const now = Date.now();
    this._latencyMs = now - this.pingSentAt;
    this._missedCount = 0;

    // Record in ring buffer
    this.latencyBuffer[this.latencyBufferIndex] = this._latencyMs;
    this.latencyBufferIndex++;
    if (this.latencyBufferIndex >= RING_BUFFER_SIZE) {
      this.latencyBufferIndex = 0;
      this.latencyBufferFilled = true;
    }

    this.onPongCb?.(this._latencyMs);
  }

  // ── Getters ─────────────────────────────────────────────────────

  /** True when the connection is considered alive (missed < max). */
  get isAlive(): boolean {
    return this._missedCount < this.maxMissedThreshold;
  }

  /** Last recorded round-trip latency in milliseconds. */
  get latencyMs(): number {
    return this._latencyMs;
  }

  /** Rolling average latency over the last 10 pongs (0 if none recorded). */
  get averageLatencyMs(): number {
    const count = this.latencyBufferFilled
      ? RING_BUFFER_SIZE
      : this.latencyBufferIndex;

    if (count === 0) return 0;

    let sum = 0;
    for (let i = 0; i < count; i++) {
      sum += this.latencyBuffer[i];
    }
    return sum / count;
  }

  /** Current number of consecutive missed pongs. */
  get missedCount(): number {
    return this._missedCount;
  }

  /** Whether the heartbeat loop is currently active. */
  get isRunning(): boolean {
    return this._isRunning;
  }

  // ── Internals ───────────────────────────────────────────────────

  private sendOnePing(sendPing: () => Promise<void>): void {
    // If we are still waiting for a previous pong, count it as missed
    if (this.waitingForPong) {
      this.handleMissedPong();
      if (!this._isRunning) return; // timeout may have stopped us
    }

    this.pingSentAt = Date.now();
    this.waitingForPong = true;

    // Start a pong deadline timer
    this.pongTimer = setTimeout(() => {
      if (this.waitingForPong) {
        this.waitingForPong = false;
        this.handleMissedPong();
      }
    }, this.timeoutMs);

    // Fire-and-forget the actual ping
    sendPing().catch(() => {
      // If the ping itself fails, treat as missed
      this.waitingForPong = false;
      this.clearPongTimer();
      this.handleMissedPong();
    });
  }

  private handleMissedPong(): void {
    this._missedCount++;

    if (this._missedCount >= this.maxMissedThreshold) {
      this.stop();
      this.onTimeout?.();
    }
  }

  private clearPongTimer(): void {
    if (this.pongTimer) {
      clearTimeout(this.pongTimer);
      this.pongTimer = null;
    }
  }
}
