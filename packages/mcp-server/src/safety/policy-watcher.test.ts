import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

// ── Mocks ────────────────────────────────────────────────────────────
// We mock `fs` and `yaml` so tests never touch the real filesystem and
// can fully control what the watcher sees.

const mockClose = vi.fn();
const mockOn = vi.fn();

// Stores the listener that fs.watch passes back so we can trigger it.
let watchListener: (eventType: string, filename?: string) => void;

vi.mock('fs', async (importOriginal) => {
  const actual = (await importOriginal()) as Record<string, unknown>;
  return {
    ...actual,
    existsSync: vi.fn(() => true),
    readFileSync: vi.fn(() => 'mocked-yaml-content'),
    watch: vi.fn((_path: string, listener: (eventType: string, filename?: string) => void) => {
      watchListener = listener;
      return { close: mockClose, on: mockOn };
    }),
    watchFile: vi.fn(),
    unwatchFile: vi.fn(),
  };
});

vi.mock('yaml', () => ({
  parse: vi.fn(() => ({
    name: 'test-policy',
    description: 'A test policy',
    velocity: { linearMax: 1.0, angularMax: 2.0, clampMode: false },
    geofence: { xMin: -10, xMax: 10, yMin: -10, yMax: 10, zMin: 0, zMax: 3 },
    rateLimits: { publishHz: 20, servicePerMinute: 120, actionPerMinute: 60 },
  })),
}));

// Import after mocks are set up
import { PolicyWatcher } from './policy-watcher.js';
import { existsSync, readFileSync } from 'fs';
import { parse as parseYaml } from 'yaml';
import type { SafetyPolicy } from './types.js';

describe('PolicyWatcher', () => {
  let watcher: PolicyWatcher;
  let onReload: ReturnType<typeof vi.fn>;

  beforeEach(() => {
    vi.useFakeTimers();
    vi.clearAllMocks();

    // Reset default mock behaviours
    vi.mocked(existsSync).mockReturnValue(true);
    vi.mocked(readFileSync).mockReturnValue('mocked-yaml');
    vi.mocked(parseYaml).mockReturnValue({
      name: 'test-policy',
      description: 'A test policy',
      velocity: { linearMax: 1.0, angularMax: 2.0, clampMode: false },
      geofence: { xMin: -10, xMax: 10, yMin: -10, yMax: 10, zMin: 0, zMax: 3 },
      rateLimits: { publishHz: 20, servicePerMinute: 120, actionPerMinute: 60 },
    });

    onReload = vi.fn();
    watcher = new PolicyWatcher('/tmp/test-policy.yaml', onReload);
  });

  afterEach(() => {
    watcher.stop();
    vi.useRealTimers();
  });

  // ── Constructor / initial state ──────────────────────────────────

  it('creates a watcher in non-watching state', () => {
    expect(watcher.isWatching).toBe(false);
    expect(watcher.reloadCount).toBe(0);
    expect(watcher.lastReloadAt).toBeNull();
    expect(watcher.lastError).toBeNull();
  });

  // ── start() ──────────────────────────────────────────────────────

  it('start() returns true and sets isWatching when file exists', () => {
    const result = watcher.start();
    expect(result).toBe(true);
    expect(watcher.isWatching).toBe(true);
  });

  it('start() returns false if the policy file does not exist', () => {
    vi.mocked(existsSync).mockReturnValue(false);

    const result = watcher.start();
    expect(result).toBe(false);
    expect(watcher.isWatching).toBe(false);
    expect(watcher.lastError).toContain('does not exist');
  });

  it('start() is idempotent — calling twice does not create a second watcher', () => {
    watcher.start();
    watcher.start();
    expect(watcher.isWatching).toBe(true);
  });

  // ── stop() ───────────────────────────────────────────────────────

  it('stop() after start() cleans up and sets isWatching to false', () => {
    watcher.start();
    expect(watcher.isWatching).toBe(true);

    watcher.stop();
    expect(watcher.isWatching).toBe(false);
    expect(mockClose).toHaveBeenCalledTimes(1);
  });

  it('stop() is safe to call when not watching', () => {
    expect(() => watcher.stop()).not.toThrow();
    expect(watcher.isWatching).toBe(false);
  });

  // ── Reload on file change ────────────────────────────────────────

  it('calls onReload with parsed policy after debounce when file changes', () => {
    watcher.start();

    // Simulate a file change event
    watchListener('change');

    // Before debounce expires — callback should NOT have fired
    expect(onReload).not.toHaveBeenCalled();

    // Advance past the 500ms debounce window
    vi.advanceTimersByTime(500);

    expect(onReload).toHaveBeenCalledTimes(1);
    const policy = onReload.mock.calls[0][0] as SafetyPolicy;
    expect(policy.name).toBe('test-policy');
  });

  it('tracks reloadCount and lastReloadAt after successful reload', () => {
    watcher.start();

    watchListener('change');
    vi.advanceTimersByTime(500);

    expect(watcher.reloadCount).toBe(1);
    expect(watcher.lastReloadAt).toBeTypeOf('number');
    expect(watcher.lastReloadAt).not.toBeNull();
  });

  it('increments reloadCount on successive reloads', () => {
    watcher.start();

    watchListener('change');
    vi.advanceTimersByTime(500);

    watchListener('change');
    vi.advanceTimersByTime(500);

    expect(watcher.reloadCount).toBe(2);
  });

  // ── Debouncing ───────────────────────────────────────────────────

  it('debounces multiple rapid changes into a single reload', () => {
    watcher.start();

    // Fire several changes in quick succession
    watchListener('change');
    vi.advanceTimersByTime(100);
    watchListener('change');
    vi.advanceTimersByTime(100);
    watchListener('change');

    // Advance less than debounce from last change — should not fire yet
    vi.advanceTimersByTime(400);

    // Now past the debounce window from the last change
    vi.advanceTimersByTime(200);

    // Should have triggered exactly once
    expect(onReload).toHaveBeenCalledTimes(1);
    expect(watcher.reloadCount).toBe(1);
  });

  it('debounce timer resets on each new change', () => {
    watcher.start();

    watchListener('change');
    vi.advanceTimersByTime(499); // just under threshold
    watchListener('change');     // reset the timer
    vi.advanceTimersByTime(499); // still under new threshold
    expect(onReload).not.toHaveBeenCalled();

    vi.advanceTimersByTime(1);   // now at 500ms from last change
    expect(onReload).toHaveBeenCalledTimes(1);
  });

  // ── Validation / errors ──────────────────────────────────────────

  it('sets lastError and does NOT call onReload when YAML is missing required fields', () => {
    vi.mocked(parseYaml).mockReturnValue({ name: 'incomplete' }); // missing velocity, geofence, rateLimits

    watcher.start();
    watchListener('change');
    vi.advanceTimersByTime(500);

    expect(onReload).not.toHaveBeenCalled();
    expect(watcher.lastError).toContain('missing required fields');
    expect(watcher.reloadCount).toBe(0);
  });

  it('sets lastError when readFileSync throws (file deleted)', () => {
    vi.mocked(readFileSync).mockImplementation(() => {
      throw new Error('ENOENT: no such file or directory');
    });

    watcher.start();
    watchListener('change');
    vi.advanceTimersByTime(500);

    expect(onReload).not.toHaveBeenCalled();
    expect(watcher.lastError).toContain('ENOENT');
    expect(watcher.reloadCount).toBe(0);
  });

  it('sets lastError when yaml.parse throws (invalid YAML)', () => {
    vi.mocked(parseYaml).mockImplementation(() => {
      throw new Error('Invalid YAML syntax');
    });

    watcher.start();
    watchListener('change');
    vi.advanceTimersByTime(500);

    expect(onReload).not.toHaveBeenCalled();
    expect(watcher.lastError).toContain('Invalid YAML syntax');
  });

  it('clears lastError on a subsequent successful reload', () => {
    // First reload: bad YAML
    vi.mocked(parseYaml).mockImplementationOnce(() => {
      throw new Error('bad yaml');
    });

    watcher.start();
    watchListener('change');
    vi.advanceTimersByTime(500);
    expect(watcher.lastError).toContain('bad yaml');

    // Second reload: good YAML — restore default mock
    vi.mocked(parseYaml).mockReturnValue({
      name: 'fixed-policy',
      description: 'Fixed',
      velocity: { linearMax: 1.0, angularMax: 2.0, clampMode: false },
      geofence: { xMin: -5, xMax: 5, yMin: -5, yMax: 5, zMin: 0, zMax: 2 },
      rateLimits: { publishHz: 10, servicePerMinute: 60, actionPerMinute: 30 },
    });

    watchListener('change');
    vi.advanceTimersByTime(500);

    expect(watcher.lastError).toBeNull();
    expect(watcher.reloadCount).toBe(1);
    expect(onReload).toHaveBeenCalledTimes(1);
  });

  // ── lastError is null when no error ──────────────────────────────

  it('lastError is null when no error has occurred', () => {
    expect(watcher.lastError).toBeNull();

    watcher.start();
    watchListener('change');
    vi.advanceTimersByTime(500);

    expect(watcher.lastError).toBeNull();
  });

  // ── isWatching reflects state correctly ──────────────────────────

  it('isWatching reflects state transitions: false -> true -> false', () => {
    expect(watcher.isWatching).toBe(false);

    watcher.start();
    expect(watcher.isWatching).toBe(true);

    watcher.stop();
    expect(watcher.isWatching).toBe(false);
  });

  // ── stop() cancels pending debounce ──────────────────────────────

  it('stop() cancels a pending debounced reload', () => {
    watcher.start();
    watchListener('change');

    // Stop before debounce fires
    vi.advanceTimersByTime(200);
    watcher.stop();

    // Advance past where the debounce would have fired
    vi.advanceTimersByTime(500);

    expect(onReload).not.toHaveBeenCalled();
    expect(watcher.reloadCount).toBe(0);
  });
});
