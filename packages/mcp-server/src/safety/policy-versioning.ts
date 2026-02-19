/**
 * Safety policy versioning system.
 * Tracks policy changes over time and supports rollback to previous versions.
 */

import type { SafetyPolicy } from './types.js';

export interface PolicyVersion {
  version: number;
  timestamp: number;
  policy: SafetyPolicy;
  reason: string;
  changedFields: string[];
}

export interface PolicyFieldDiff {
  field: string;
  before: unknown;
  after: unknown;
}

/**
 * Deep-clone a SafetyPolicy so stored versions are never mutated externally.
 */
function clonePolicy(policy: SafetyPolicy): SafetyPolicy {
  return JSON.parse(JSON.stringify(policy));
}

/**
 * Return the top-level SafetyPolicy field names where the two policies differ.
 */
function detectChangedFields(a: SafetyPolicy, b: SafetyPolicy): string[] {
  const allKeys = new Set<string>([
    ...Object.keys(a),
    ...Object.keys(b),
  ]);

  const changed: string[] = [];
  for (const key of allKeys) {
    const valA = (a as unknown as Record<string, unknown>)[key];
    const valB = (b as unknown as Record<string, unknown>)[key];
    if (JSON.stringify(valA) !== JSON.stringify(valB)) {
      changed.push(key);
    }
  }
  return changed.sort();
}

export class PolicyVersionManager {
  private versions: PolicyVersion[] = [];
  private nextVersion = 1;
  private readonly maxVersions: number;

  constructor(maxVersions = 50) {
    this.maxVersions = maxVersions;
  }

  /**
   * Record a new policy version. Returns the version number.
   */
  recordVersion(policy: SafetyPolicy, reason: string): number {
    const version = this.nextVersion++;
    const previousPolicy = this.versions.length > 0
      ? this.versions[this.versions.length - 1].policy
      : null;

    const changedFields = previousPolicy
      ? detectChangedFields(previousPolicy, policy)
      : Object.keys(policy).sort();

    const entry: PolicyVersion = {
      version,
      timestamp: Date.now(),
      policy: clonePolicy(policy),
      reason,
      changedFields,
    };

    this.versions.push(entry);

    // Enforce the max-versions cap by dropping the oldest entries.
    while (this.versions.length > this.maxVersions) {
      this.versions.shift();
    }

    return version;
  }

  /**
   * Get all recorded versions, newest first.
   */
  getVersions(): PolicyVersion[] {
    return [...this.versions].reverse();
  }

  /**
   * Get a specific version by number.
   */
  getVersion(version: number): PolicyVersion | null {
    return this.versions.find((v) => v.version === version) ?? null;
  }

  /**
   * Get the diff between two versions (which top-level fields changed).
   * Returns null if either version is not found.
   */
  diff(v1: number, v2: number): PolicyFieldDiff[] | null {
    const entry1 = this.getVersion(v1);
    const entry2 = this.getVersion(v2);

    if (!entry1 || !entry2) {
      return null;
    }

    const changedKeys = detectChangedFields(entry1.policy, entry2.policy);

    return changedKeys.map((field) => ({
      field,
      before: (entry1.policy as unknown as Record<string, unknown>)[field],
      after: (entry2.policy as unknown as Record<string, unknown>)[field],
    }));
  }

  /**
   * Rollback to a specific version. Returns a deep clone of that version's
   * policy, or null if the version is not found.
   */
  rollback(version: number): SafetyPolicy | null {
    const entry = this.getVersion(version);
    if (!entry) {
      return null;
    }
    return clonePolicy(entry.policy);
  }

  /**
   * Get the current (latest) version number.
   * Returns 0 if no versions have been recorded.
   */
  getCurrentVersion(): number {
    if (this.versions.length === 0) {
      return 0;
    }
    return this.versions[this.versions.length - 1].version;
  }
}
