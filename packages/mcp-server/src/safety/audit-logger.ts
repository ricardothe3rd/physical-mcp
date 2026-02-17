/**
 * Command audit trail logging.
 */

import { randomUUID } from 'crypto';
import type { AuditEntry, SafetyCheckResult } from './types.js';

export class AuditLogger {
  private entries: AuditEntry[] = [];
  private maxEntries: number;

  constructor(maxEntries = 10000) {
    this.maxEntries = maxEntries;
  }

  log(
    command: string,
    target: string,
    params: Record<string, unknown>,
    safetyResult: SafetyCheckResult
  ): AuditEntry {
    const entry: AuditEntry = {
      id: randomUUID(),
      timestamp: Date.now(),
      command,
      target,
      params,
      safetyResult,
    };

    this.entries.push(entry);

    if (this.entries.length > this.maxEntries) {
      this.entries = this.entries.slice(-this.maxEntries);
    }

    if (!safetyResult.allowed) {
      console.error(
        `[AUDIT] BLOCKED ${command} on ${target}: ${safetyResult.violations.map(v => v.message).join('; ')}`
      );
    }

    return entry;
  }

  updateResult(entryId: string, result: 'success' | 'error', error?: string) {
    const entry = this.entries.find(e => e.id === entryId);
    if (entry) {
      entry.executionResult = result;
      entry.executionError = error;
    }
  }

  getEntries(options?: {
    limit?: number;
    command?: string;
    violationsOnly?: boolean;
  }): AuditEntry[] {
    let result = [...this.entries];

    if (options?.command) {
      result = result.filter(e => e.command === options.command);
    }

    if (options?.violationsOnly) {
      result = result.filter(e => !e.safetyResult.allowed);
    }

    result.reverse(); // newest first

    if (options?.limit) {
      result = result.slice(0, options.limit);
    }

    return result;
  }

  getStats() {
    const total = this.entries.length;
    const blocked = this.entries.filter(e => !e.safetyResult.allowed).length;
    const errors = this.entries.filter(e => e.executionResult === 'error').length;

    return { total, allowed: total - blocked, blocked, errors };
  }

  clear() {
    this.entries = [];
  }
}
