import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { PolicyWatcher } from './policy-watcher.js';

// Mock fs.watch
vi.mock('fs', async (importOriginal) => {
  const actual = await importOriginal() as Record<string, unknown>;
  return {
    ...actual,
    watch: vi.fn((_path: string, callback: Function) => {
      // Store callback for manual triggering
      (globalThis as any).__fsMockCallback = callback;
      return {
        close: vi.fn(),
      };
    }),
  };
});

describe('PolicyWatcher', () => {
  let watcher: PolicyWatcher;
  let callback: ReturnType<typeof vi.fn>;

  beforeEach(() => {
    vi.useFakeTimers();
    callback = vi.fn();
    // Use a path that will return defaults via loadPolicy
    watcher = new PolicyWatcher('/nonexistent/test-policy.yaml', callback, 100);
  });

  afterEach(() => {
    watcher.stop();
    vi.useRealTimers();
    vi.restoreAllMocks();
  });

  it('starts watching', () => {
    watcher.start();
    expect(watcher.isWatching).toBe(true);
  });

  it('stops watching', () => {
    watcher.start();
    watcher.stop();
    expect(watcher.isWatching).toBe(false);
  });

  it('does not start twice', () => {
    watcher.start();
    expect(watcher.isWatching).toBe(true);
    watcher.start(); // second call should be no-op — still watching
    expect(watcher.isWatching).toBe(true);
  });

  it('debounces rapid changes', () => {
    watcher.start();

    // Simulate rapid file changes
    const fsCb = (globalThis as any).__fsMockCallback;
    fsCb('change');
    fsCb('change');
    fsCb('change');

    // Before debounce, callback should not be called
    expect(callback).not.toHaveBeenCalled();

    // After debounce period
    vi.advanceTimersByTime(200);

    // Should only be called once due to debounce
    expect(callback).toHaveBeenCalledTimes(1);
  });

  it('ignores non-change events', () => {
    watcher.start();

    const fsCb = (globalThis as any).__fsMockCallback;
    fsCb('rename'); // Not 'change'

    vi.advanceTimersByTime(200);
    expect(callback).not.toHaveBeenCalled();
  });
});
