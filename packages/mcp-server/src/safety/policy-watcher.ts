/**
 * Watches a YAML policy file for changes and reloads automatically.
 */

import { watch, type FSWatcher } from 'fs';
import { loadPolicy } from './policy-loader.js';
import type { SafetyPolicy } from './types.js';

export type PolicyChangeCallback = (newPolicy: SafetyPolicy, oldPolicy: SafetyPolicy) => void;

export class PolicyWatcher {
  private watcher: FSWatcher | null = null;
  private filePath: string;
  private callback: PolicyChangeCallback;
  private debounceTimer: ReturnType<typeof setTimeout> | null = null;
  private debounceMs: number;

  constructor(filePath: string, callback: PolicyChangeCallback, debounceMs = 500) {
    this.filePath = filePath;
    this.callback = callback;
    this.debounceMs = debounceMs;
  }

  start(): void {
    if (this.watcher) return;

    try {
      this.watcher = watch(this.filePath, (eventType) => {
        if (eventType === 'change') {
          this.handleChange();
        }
      });
      console.error(`[PolicyWatcher] Watching ${this.filePath} for changes`);
    } catch (err) {
      console.error(`[PolicyWatcher] Failed to watch ${this.filePath}: ${err}`);
    }
  }

  private handleChange(): void {
    // Debounce rapid file changes (editors often save multiple times)
    if (this.debounceTimer) {
      clearTimeout(this.debounceTimer);
    }

    this.debounceTimer = setTimeout(() => {
      try {
        const oldPolicy = loadPolicy(this.filePath);
        const newPolicy = loadPolicy(this.filePath);
        console.error(`[PolicyWatcher] Policy file changed, reloading: ${newPolicy.name}`);
        this.callback(newPolicy, oldPolicy);
      } catch (err) {
        console.error(`[PolicyWatcher] Failed to reload policy: ${err}`);
      }
    }, this.debounceMs);
  }

  stop(): void {
    if (this.debounceTimer) {
      clearTimeout(this.debounceTimer);
      this.debounceTimer = null;
    }
    if (this.watcher) {
      this.watcher.close();
      this.watcher = null;
      console.error('[PolicyWatcher] Stopped watching policy file');
    }
  }

  get isWatching(): boolean {
    return this.watcher !== null;
  }
}
