/**
 * Performance benchmark tests for PhysicalMCP safety subsystems.
 *
 * These tests measure latency and throughput of the critical hot-path
 * components: safety checks, audit logging, rate limiting, command
 * creation, and tool dispatch routing.
 *
 * Each benchmark runs N iterations, collects per-iteration timings, and
 * computes avg / p50 / p95 / p99. Results are printed to stderr for
 * visibility in CI logs. Generous upper-bound assertions ensure the
 * tests stay green on slow machines while still catching regressions.
 */

import { describe, it, expect, afterEach } from 'vitest';
import { PolicyEngine } from '../safety/policy-engine.js';
import { AuditLogger } from '../safety/audit-logger.js';
import { RateLimiter } from '../safety/rate-limiter.js';
import { checkGeofence, type Position } from '../safety/geofence.js';
import { createCommand, CommandType } from '../bridge/protocol.js';
import { getTopicTools } from '../tools/topic-tools.js';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Compute percentile from a sorted array. */
function percentile(sorted: number[], p: number): number {
  const idx = Math.ceil((p / 100) * sorted.length) - 1;
  return sorted[Math.max(0, idx)];
}

/** Collect timing stats from an array of durations (ms). */
function stats(durations: number[]) {
  const sorted = [...durations].sort((a, b) => a - b);
  const sum = sorted.reduce((a, b) => a + b, 0);
  return {
    avg: sum / sorted.length,
    p50: percentile(sorted, 50),
    p95: percentile(sorted, 95),
    p99: percentile(sorted, 99),
    min: sorted[0],
    max: sorted[sorted.length - 1],
    count: sorted.length,
  };
}

/** Pretty-print a stats object to stderr. */
function report(label: string, s: ReturnType<typeof stats>) {
  console.error(
    `  [BENCH] ${label}: avg=${s.avg.toFixed(4)}ms  p50=${s.p50.toFixed(4)}ms  p95=${s.p95.toFixed(4)}ms  p99=${s.p99.toFixed(4)}ms  (n=${s.count})`
  );
}

// ---------------------------------------------------------------------------
// Benchmarks
// ---------------------------------------------------------------------------

describe('Performance Benchmarks', () => {
  // PolicyEngine starts a deadman switch interval — clean up after each test.
  let engineInstance: PolicyEngine | null = null;

  afterEach(() => {
    if (engineInstance) {
      engineInstance.destroy();
      engineInstance = null;
    }
  });

  // -----------------------------------------------------------------------
  // 1. Safety check overhead
  // -----------------------------------------------------------------------
  describe('Safety check overhead', () => {
    const N = 10_000;

    it(`velocity check: ${N} iterations avg < 1ms`, () => {
      const engine = new PolicyEngine();
      engineInstance = engine;

      const durations: number[] = [];
      for (let i = 0; i < N; i++) {
        const start = performance.now();
        engine.checkPublish('/cmd_vel', {
          linear: { x: 0.1, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0.2 },
        });
        durations.push(performance.now() - start);
      }

      const s = stats(durations);
      report(`Velocity check (${N} iters)`, s);
      expect(s.avg).toBeLessThan(1);
    });

    it(`geofence check: ${N} iterations avg < 1ms`, () => {
      const bounds = { xMin: -5, xMax: 5, yMin: -5, yMax: 5, zMin: 0, zMax: 3 };
      const position: Position = { x: 1.0, y: 2.0, z: 0.5 };

      const durations: number[] = [];
      for (let i = 0; i < N; i++) {
        const start = performance.now();
        checkGeofence(position, bounds);
        durations.push(performance.now() - start);
      }

      const s = stats(durations);
      report(`Geofence check (${N} iters)`, s);
      expect(s.avg).toBeLessThan(1);
    });

    it(`rate limiter check: ${N} iterations avg < 1ms`, () => {
      const limiter = new RateLimiter({
        publishHz: 100_000,
        servicePerMinute: 100_000,
        actionPerMinute: 100_000,
      });

      const durations: number[] = [];
      for (let i = 0; i < N; i++) {
        const start = performance.now();
        limiter.checkPublish('/cmd_vel');
        durations.push(performance.now() - start);
      }

      const s = stats(durations);
      report(`Rate limiter check (${N} iters)`, s);
      expect(s.avg).toBeLessThan(1);
    });
  });

  // -----------------------------------------------------------------------
  // 2. Audit logger throughput
  // -----------------------------------------------------------------------
  describe('Audit logger throughput', () => {
    const N = 10_000;

    it(`${N} sequential writes avg < 0.5ms`, () => {
      const logger = new AuditLogger(N + 1000);
      const safetyResult = { allowed: true, violations: [] as never[] };

      const durations: number[] = [];
      for (let i = 0; i < N; i++) {
        const start = performance.now();
        logger.log('publish', '/cmd_vel', { linear: { x: 0.1 } }, safetyResult);
        durations.push(performance.now() - start);
      }

      const s = stats(durations);
      report(`Audit log write (${N} iters)`, s);

      const totalMs = durations.reduce((a, b) => a + b, 0);
      const entriesPerSec = (N / totalMs) * 1000;
      console.error(`  [BENCH] Audit throughput: ${entriesPerSec.toFixed(0)} entries/sec`);

      expect(s.avg).toBeLessThan(0.5);
    });
  });

  // -----------------------------------------------------------------------
  // 3. Rate limiter under load
  // -----------------------------------------------------------------------
  describe('Rate limiter under load', () => {
    const N = 10_000;

    it(`${N} rapid checks measure throughput`, () => {
      // Use a very high limit so checks do not get blocked
      const limiter = new RateLimiter({
        publishHz: 100_000,
        servicePerMinute: 100_000,
        actionPerMinute: 100_000,
      });

      const start = performance.now();
      for (let i = 0; i < N; i++) {
        limiter.checkPublish('/cmd_vel');
      }
      const totalMs = performance.now() - start;

      const opsPerSec = (N / totalMs) * 1000;
      console.error(
        `  [BENCH] Rate limiter load: ${N} checks in ${totalMs.toFixed(2)}ms (${opsPerSec.toFixed(0)} ops/sec)`
      );

      // Should comfortably handle 10k checks in under 3 seconds (generous for CI)
      expect(totalMs).toBeLessThan(3000);
    });
  });

  // -----------------------------------------------------------------------
  // 4. Policy engine full pipeline
  // -----------------------------------------------------------------------
  describe('Policy engine full pipeline', () => {
    const N = 1_000;

    it(`checkPublish full pipeline: ${N} calls avg < 1ms`, () => {
      const engine = new PolicyEngine();
      engineInstance = engine;

      const durations: number[] = [];
      for (let i = 0; i < N; i++) {
        const start = performance.now();
        engine.checkPublish('/cmd_vel', {
          linear: { x: 0.1, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0.2 },
        });
        durations.push(performance.now() - start);
      }

      const s = stats(durations);
      report(`Full pipeline checkPublish (${N} iters)`, s);
      expect(s.avg).toBeLessThan(1);
    });
  });

  // -----------------------------------------------------------------------
  // 5. Tool dispatch routing
  // -----------------------------------------------------------------------
  describe('Tool dispatch routing', () => {
    const N = 10_000;

    it(`Set.has() dispatch lookup: ${N} iterations avg < 0.01ms`, () => {
      const topicToolNames = new Set(getTopicTools().map(t => t.name));

      const durations: number[] = [];
      for (let i = 0; i < N; i++) {
        const start = performance.now();
        topicToolNames.has('ros2_topic_publish');
        durations.push(performance.now() - start);
      }

      const s = stats(durations);
      report(`Set.has() dispatch (${N} iters)`, s);
      expect(s.avg).toBeLessThan(0.01);
    });
  });

  // -----------------------------------------------------------------------
  // 6. Command creation
  // -----------------------------------------------------------------------
  describe('Command creation', () => {
    const N = 10_000;

    it(`createCommand: ${N} calls avg < 0.01ms`, () => {
      const durations: number[] = [];
      for (let i = 0; i < N; i++) {
        const start = performance.now();
        createCommand(CommandType.TOPIC_PUBLISH, { topic: '/cmd_vel', message: {} });
        durations.push(performance.now() - start);
      }

      const s = stats(durations);
      report(`createCommand (${N} iters)`, s);
      expect(s.avg).toBeLessThan(0.01);
    });
  });
});
