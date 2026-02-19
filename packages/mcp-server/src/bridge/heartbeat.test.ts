import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { HeartbeatManager } from './heartbeat.js';

describe('HeartbeatManager', () => {
  let hb: HeartbeatManager;
  let sendPing: ReturnType<typeof vi.fn>;

  beforeEach(() => {
    vi.useFakeTimers();
    sendPing = vi.fn<() => Promise<void>>().mockResolvedValue(undefined);
  });

  afterEach(() => {
    hb?.stop();
    vi.useRealTimers();
  });

  // ── Constructor / initial state ──────────────────────────────────

  it('creates a manager in stopped state with sane defaults', () => {
    hb = new HeartbeatManager();
    expect(hb.isRunning).toBe(false);
    expect(hb.isAlive).toBe(true); // missedCount = 0 < 3
    expect(hb.latencyMs).toBe(0);
    expect(hb.averageLatencyMs).toBe(0);
    expect(hb.missedCount).toBe(0);
  });

  // ── start / stop lifecycle ────────────────────────────────────────

  it('start() sets isRunning to true', () => {
    hb = new HeartbeatManager();
    hb.start(sendPing);
    expect(hb.isRunning).toBe(true);
  });

  it('stop() sets isRunning to false', () => {
    hb = new HeartbeatManager();
    hb.start(sendPing);
    hb.stop();
    expect(hb.isRunning).toBe(false);
  });

  it('double start is idempotent — does not create a second timer', () => {
    hb = new HeartbeatManager({ intervalMs: 1000 });
    hb.start(sendPing);
    hb.start(sendPing); // should be a no-op

    vi.advanceTimersByTime(1000);
    expect(sendPing).toHaveBeenCalledTimes(1);
  });

  it('double stop is safe and does not throw', () => {
    hb = new HeartbeatManager();
    hb.start(sendPing);
    expect(() => {
      hb.stop();
      hb.stop();
    }).not.toThrow();
    expect(hb.isRunning).toBe(false);
  });

  it('stop() on a never-started manager is safe', () => {
    hb = new HeartbeatManager();
    expect(() => hb.stop()).not.toThrow();
  });

  // ── Ping sends at correct intervals ───────────────────────────────

  it('sends a ping after each interval tick', () => {
    hb = new HeartbeatManager({ intervalMs: 1000 });
    hb.start(sendPing);

    expect(sendPing).not.toHaveBeenCalled();

    vi.advanceTimersByTime(1000);
    expect(sendPing).toHaveBeenCalledTimes(1);

    vi.advanceTimersByTime(1000);
    expect(sendPing).toHaveBeenCalledTimes(2);

    vi.advanceTimersByTime(1000);
    expect(sendPing).toHaveBeenCalledTimes(3);
  });

  it('stop() prevents further pings from being sent', () => {
    hb = new HeartbeatManager({ intervalMs: 1000 });
    hb.start(sendPing);

    vi.advanceTimersByTime(1000);
    expect(sendPing).toHaveBeenCalledTimes(1);

    hb.stop();

    vi.advanceTimersByTime(5000);
    expect(sendPing).toHaveBeenCalledTimes(1); // no more after stop
  });

  // ── Pong resets missed count ──────────────────────────────────────

  it('receivePong() resets missedCount to 0', () => {
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 500, maxMissed: 5 });
    hb.start(sendPing);

    // First ping — let timeout expire (miss)
    vi.advanceTimersByTime(1000); // ping sent
    vi.advanceTimersByTime(500);  // pong timeout
    expect(hb.missedCount).toBe(1);

    // Second ping — respond with pong
    vi.advanceTimersByTime(500);  // next interval tick
    hb.receivePong();
    expect(hb.missedCount).toBe(0);
  });

  // ── Latency calculation ───────────────────────────────────────────

  it('records latency when pong is received', () => {
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 5000 });
    hb.start(sendPing);

    vi.advanceTimersByTime(1000); // ping sent at t=1000
    vi.advanceTimersByTime(50);   // pong arrives 50ms later
    hb.receivePong();

    expect(hb.latencyMs).toBe(50);
  });

  // ── Average latency with ring buffer ──────────────────────────────

  it('averageLatencyMs computes over recorded pongs', () => {
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 5000 });
    hb.start(sendPing);

    // Pong 1: latency 100ms
    vi.advanceTimersByTime(1000);
    vi.advanceTimersByTime(100);
    hb.receivePong();

    // Pong 2: latency 200ms
    vi.advanceTimersByTime(900); // rest of interval
    vi.advanceTimersByTime(200);
    hb.receivePong();

    expect(hb.averageLatencyMs).toBe(150); // (100 + 200) / 2
  });

  it('averageLatencyMs uses ring buffer of last 10', () => {
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 500 });
    hb.start(sendPing);

    // Record 12 pongs — only last 10 should count
    // First 2 pongs with 100ms latency
    for (let i = 0; i < 2; i++) {
      vi.advanceTimersByTime(1000); // ping sent
      vi.advanceTimersByTime(100);  // pong arrives 100ms later
      hb.receivePong();
      vi.advanceTimersByTime(900 - 100); // rest of interval before next tick
    }

    // Next 10 pongs with 50ms latency — these fill the ring buffer
    for (let i = 0; i < 10; i++) {
      vi.advanceTimersByTime(1000); // ping sent
      vi.advanceTimersByTime(50);   // pong arrives 50ms later
      hb.receivePong();
      vi.advanceTimersByTime(950 - 50); // rest of interval before next tick
    }

    // Ring buffer should contain only the last 10 (all 50ms)
    expect(hb.averageLatencyMs).toBe(50);
  });

  it('averageLatencyMs is 0 when no pongs have been received', () => {
    hb = new HeartbeatManager();
    expect(hb.averageLatencyMs).toBe(0);
  });

  // ── Timeout after maxMissed consecutive misses ────────────────────

  it('isAlive becomes false after maxMissed timeouts', () => {
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 500, maxMissed: 2 });
    hb.start(sendPing);

    // Ping 1 — miss
    vi.advanceTimersByTime(1000);
    vi.advanceTimersByTime(500);
    expect(hb.missedCount).toBe(1);
    expect(hb.isAlive).toBe(true);

    // Ping 2 — miss (at next interval the pending pong triggers missed)
    vi.advanceTimersByTime(500); // next interval tick
    // At this point sendOnePing sees waitingForPong was handled by timeout,
    // and a new ping is sent; its timeout will fire maxMissed
    vi.advanceTimersByTime(500);
    expect(hb.isAlive).toBe(false);
  });

  it('stops automatically when maxMissed is reached', () => {
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 500, maxMissed: 2 });
    hb.start(sendPing);

    // Miss enough pongs to trigger timeout
    vi.advanceTimersByTime(1000); // ping 1
    vi.advanceTimersByTime(500);  // timeout 1
    vi.advanceTimersByTime(500);  // ping 2
    vi.advanceTimersByTime(500);  // timeout 2

    expect(hb.isRunning).toBe(false);
  });

  // ── onTimeout callback ────────────────────────────────────────────

  it('calls onTimeout when maxMissed consecutive pongs are missed', () => {
    const onTimeout = vi.fn();
    hb = new HeartbeatManager({
      intervalMs: 1000,
      timeoutMs: 500,
      maxMissed: 2,
      onTimeout,
    });
    hb.start(sendPing);

    // Miss 2 pongs
    vi.advanceTimersByTime(1000);
    vi.advanceTimersByTime(500);
    vi.advanceTimersByTime(500);
    vi.advanceTimersByTime(500);

    expect(onTimeout).toHaveBeenCalledTimes(1);
  });

  it('onTimeout is not called if pong arrives in time', () => {
    const onTimeout = vi.fn();
    hb = new HeartbeatManager({
      intervalMs: 1000,
      timeoutMs: 500,
      maxMissed: 2,
      onTimeout,
    });
    hb.start(sendPing);

    // Ping 1 — respond in time
    vi.advanceTimersByTime(1000);
    vi.advanceTimersByTime(100);
    hb.receivePong();

    // Ping 2 — respond in time
    vi.advanceTimersByTime(900);
    vi.advanceTimersByTime(100);
    hb.receivePong();

    expect(onTimeout).not.toHaveBeenCalled();
    expect(hb.isRunning).toBe(true);
  });

  // ── onPong callback ───────────────────────────────────────────────

  it('calls onPong with latency when pong is received', () => {
    const onPong = vi.fn();
    hb = new HeartbeatManager({
      intervalMs: 1000,
      timeoutMs: 5000,
      onPong,
    });
    hb.start(sendPing);

    vi.advanceTimersByTime(1000); // ping sent
    vi.advanceTimersByTime(42);   // pong arrives 42ms later
    hb.receivePong();

    expect(onPong).toHaveBeenCalledTimes(1);
    expect(onPong).toHaveBeenCalledWith(42);
  });

  // ── sendPing failure ──────────────────────────────────────────────

  it('treats a rejected sendPing as a missed pong', async () => {
    const failingPing = vi.fn<() => Promise<void>>().mockRejectedValue(new Error('send failed'));
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 500, maxMissed: 5 });
    hb.start(failingPing);

    vi.advanceTimersByTime(1000);
    // Allow the rejected promise microtask to flush
    await vi.advanceTimersByTimeAsync(0);

    expect(hb.missedCount).toBe(1);
  });

  // ── receivePong when not waiting is a no-op ───────────────────────

  it('receivePong() is a no-op when no ping is pending', () => {
    hb = new HeartbeatManager();
    // Should not throw or change state
    expect(() => hb.receivePong()).not.toThrow();
    expect(hb.latencyMs).toBe(0);
    expect(hb.missedCount).toBe(0);
  });

  // ── Restart after stop ────────────────────────────────────────────

  it('can be restarted after stop and resets missedCount', () => {
    hb = new HeartbeatManager({ intervalMs: 1000, timeoutMs: 500, maxMissed: 3 });
    hb.start(sendPing);

    // Miss one pong
    vi.advanceTimersByTime(1000);
    vi.advanceTimersByTime(500);
    expect(hb.missedCount).toBe(1);

    hb.stop();
    hb.start(sendPing);

    expect(hb.isRunning).toBe(true);
    expect(hb.missedCount).toBe(0);
  });

  // ── Custom options override defaults ──────────────────────────────

  it('respects custom intervalMs', () => {
    hb = new HeartbeatManager({ intervalMs: 500 });
    hb.start(sendPing);

    vi.advanceTimersByTime(500);
    expect(sendPing).toHaveBeenCalledTimes(1);

    vi.advanceTimersByTime(500);
    expect(sendPing).toHaveBeenCalledTimes(2);
  });

  it('respects custom timeoutMs', () => {
    const onTimeout = vi.fn();
    hb = new HeartbeatManager({
      intervalMs: 1000,
      timeoutMs: 200,
      maxMissed: 1,
      onTimeout,
    });
    hb.start(sendPing);

    vi.advanceTimersByTime(1000); // ping sent
    vi.advanceTimersByTime(200);  // custom timeout fires, maxMissed=1 reached

    expect(onTimeout).toHaveBeenCalledTimes(1);
    expect(hb.isRunning).toBe(false);
  });
});
