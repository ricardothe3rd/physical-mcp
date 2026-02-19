/**
 * Watches a YAML policy file for changes and reloads automatically.
 *
 * Uses fs.watch() with a debounce to prevent multiple rapid reloads when
 * editors write files in stages. Validates the parsed YAML before calling
 * the onReload callback — invalid policies are rejected and surfaced via
 * the lastError property.
 */

import { existsSync, readFileSync, watch, watchFile, unwatchFile, type FSWatcher } from 'fs';
import { parse as parseYaml } from 'yaml';
import type { SafetyPolicy } from './types.js';

const DEBOUNCE_MS = 500;

/**
 * Validate that a parsed object has the minimum required fields to be
 * considered a valid SafetyPolicy.  We check for the three fields called
 * out in the spec: velocity (velocityLimits in the YAML may map to
 * "velocity" on the typed interface), geofence, and rateLimits.
 */
function hasRequiredPolicyFields(data: unknown): data is Record<string, unknown> {
  if (data == null || typeof data !== 'object') return false;
  const obj = data as Record<string, unknown>;
  return (
    ('velocity' in obj || 'velocityLimits' in obj) &&
    ('geofence' in obj) &&
    ('rateLimits' in obj)
  );
}

export class PolicyWatcher {
  private readonly policyPath: string;
  private readonly onReload: (newPolicy: SafetyPolicy) => void;

  private watcher: FSWatcher | null = null;
  private fallbackActive = false;
  private debounceTimer: ReturnType<typeof setTimeout> | null = null;

  private _reloadCount = 0;
  private _lastReloadAt: number | null = null;
  private _lastError: string | null = null;

  constructor(policyPath: string, onReload: (newPolicy: SafetyPolicy) => void) {
    this.policyPath = policyPath;
    this.onReload = onReload;
  }

  // ── Public API ──────────────────────────────────────────────────────

  /**
   * Start watching the policy file.
   * Returns true if watching started successfully, false otherwise
   * (e.g. file does not exist).
   */
  start(): boolean {
    if (this.watcher || this.fallbackActive) {
      // Already watching
      return true;
    }

    if (!existsSync(this.policyPath)) {
      this._lastError = `Policy file does not exist: ${this.policyPath}`;
      return false;
    }

    try {
      this.watcher = watch(this.policyPath, (_eventType) => {
        this.scheduleReload();
      });

      // Handle watcher errors (e.g. file deleted while watching)
      this.watcher.on('error', (err) => {
        this._lastError = `Watcher error: ${err.message}`;
      });

      return true;
    } catch {
      // fs.watch() not supported — fall back to polling with watchFile
      try {
        watchFile(this.policyPath, { interval: 1000 }, () => {
          this.scheduleReload();
        });
        this.fallbackActive = true;
        return true;
      } catch (fallbackErr) {
        this._lastError = `Failed to watch policy file: ${fallbackErr}`;
        return false;
      }
    }
  }

  /** Stop watching and clean up resources. */
  stop(): void {
    if (this.debounceTimer) {
      clearTimeout(this.debounceTimer);
      this.debounceTimer = null;
    }

    if (this.watcher) {
      this.watcher.close();
      this.watcher = null;
    }

    if (this.fallbackActive) {
      try {
        unwatchFile(this.policyPath);
      } catch {
        // Ignore — file may have been deleted
      }
      this.fallbackActive = false;
    }
  }

  /** Whether the watcher is currently active. */
  get isWatching(): boolean {
    return this.watcher !== null || this.fallbackActive;
  }

  /** Timestamp (ms since epoch) of the last successful reload, or null. */
  get lastReloadAt(): number | null {
    return this._lastReloadAt;
  }

  /** Number of successful reloads since the watcher was created. */
  get reloadCount(): number {
    return this._reloadCount;
  }

  /** The error message from the last failed reload attempt, or null. */
  get lastError(): string | null {
    return this._lastError;
  }

  // ── Internals ───────────────────────────────────────────────────────

  /**
   * Schedule a reload after the debounce window.  If another change
   * arrives before the window expires the timer resets.
   */
  private scheduleReload(): void {
    if (this.debounceTimer) {
      clearTimeout(this.debounceTimer);
    }

    this.debounceTimer = setTimeout(() => {
      this.debounceTimer = null;
      this.performReload();
    }, DEBOUNCE_MS);
  }

  /**
   * Read, parse, validate, and propagate the policy file.
   */
  private performReload(): void {
    try {
      const raw = readFileSync(this.policyPath, 'utf-8');
      const data = parseYaml(raw);

      if (!hasRequiredPolicyFields(data)) {
        this._lastError =
          'Invalid policy: missing required fields (velocity/velocityLimits, geofence, rateLimits)';
        return;
      }

      // Cast — the caller is responsible for deeper type correctness via
      // their own SafetyPolicy schema; we ensure the structural minimum.
      const policy = data as unknown as SafetyPolicy;

      this._lastError = null;
      this._reloadCount += 1;
      this._lastReloadAt = Date.now();

      this.onReload(policy);
    } catch (err) {
      this._lastError = `Reload failed: ${err instanceof Error ? err.message : String(err)}`;
    }
  }
}
