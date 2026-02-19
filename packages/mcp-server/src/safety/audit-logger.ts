/**
 * Command audit trail logging.
 */

import { randomUUID } from 'crypto';
import { writeFileSync } from 'fs';
import type { AuditEntry, SafetyCheckResult } from './types.js';

/**
 * In-memory audit trail for all safety-checked commands.
 * Records every command evaluation with its safety result, maintaining
 * a bounded buffer (default 10 000 entries) with automatic eviction.
 */
export class AuditLogger {
  private entries: AuditEntry[] = [];
  private maxEntries: number;

  /**
   * @param maxEntries - Maximum number of entries to keep in memory (oldest are evicted)
   */
  constructor(maxEntries = 10000) {
    this.maxEntries = maxEntries;
  }

  /**
   * Record a command evaluation in the audit trail.
   * Blocked commands are also logged to stderr for immediate visibility.
   * @param command - Command type (e.g., "publish", "service_call", "action_goal")
   * @param target - The topic, service, or action name
   * @param params - The command parameters/payload
   * @param safetyResult - The result of the safety check
   * @returns The created audit entry (includes a unique id)
   */
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

  /**
   * Update an existing entry with the execution outcome (after the command was sent to the bridge).
   * @param entryId - The UUID of the audit entry to update
   * @param result - Whether execution succeeded or failed
   * @param error - Error message if result is 'error'
   */
  updateResult(entryId: string, result: 'success' | 'error', error?: string) {
    const entry = this.entries.find(e => e.id === entryId);
    if (entry) {
      entry.executionResult = result;
      entry.executionError = error;
    }
  }

  /**
   * Retrieve audit entries, optionally filtered and limited.
   * @param options.limit - Max number of entries to return
   * @param options.command - Filter by command type
   * @param options.violationsOnly - If true, only return blocked entries
   * @returns Entries sorted newest-first
   */
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

  /** Return aggregate counts: total, allowed, blocked, and errored commands. */
  getStats() {
    const total = this.entries.length;
    const blocked = this.entries.filter(e => !e.safetyResult.allowed).length;
    const errors = this.entries.filter(e => e.executionResult === 'error').length;

    return { total, allowed: total - blocked, blocked, errors };
  }

  /**
   * Export audit log to a JSON file.
   * @returns The number of entries exported.
   */
  exportToFile(filePath: string, options?: {
    violationsOnly?: boolean;
    command?: string;
  }): number {
    let entries = this.getEntries(options);
    const data = {
      exportedAt: new Date().toISOString(),
      stats: this.getStats(),
      entries,
    };
    writeFileSync(filePath, JSON.stringify(data, null, 2));
    return entries.length;
  }

  /**
   * Get entries within a time range (inclusive on both ends).
   * @param startMs - Start of range as Unix epoch milliseconds
   * @param endMs - End of range as Unix epoch milliseconds
   */
  getEntriesByTimeRange(startMs: number, endMs: number): AuditEntry[] {
    return this.entries.filter(e => e.timestamp >= startMs && e.timestamp <= endMs);
  }

  /** Remove all entries from the audit log. */
  clear() {
    this.entries = [];
  }
}
