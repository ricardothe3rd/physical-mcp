/**
 * Configurable violation mode: "block" vs "warn".
 *
 * In "block" mode (default), safety violations prevent command execution.
 * In "warn" mode, violations are logged and reported but commands are
 * still allowed through. Useful during development and testing.
 */

import type { SafetyCheckResult, SafetyViolation } from './types.js';

export type ViolationMode = 'block' | 'warn';

/**
 * Apply violation mode to a safety check result.
 *
 * In "block" mode, returns the result as-is.
 * In "warn" mode, sets allowed=true but preserves violations for logging.
 */
export function applyViolationMode(
  result: SafetyCheckResult,
  mode: ViolationMode,
): SafetyCheckResult {
  if (mode === 'block') {
    return result;
  }

  // Warn mode: allow the command but mark violations as warnings
  if (!result.allowed && result.violations.length > 0) {
    return {
      allowed: true,
      violations: result.violations.map(v => ({
        ...v,
        message: `[WARN MODE] ${v.message}`,
      })),
    };
  }

  return result;
}

/**
 * Tracks violation mode state and provides a summary.
 */
export class ViolationModeManager {
  private mode: ViolationMode = 'block';
  private warnedCount = 0;
  private blockedCount = 0;

  getMode(): ViolationMode {
    return this.mode;
  }

  setMode(mode: ViolationMode): void {
    this.mode = mode;
  }

  /**
   * Process a safety check result through the violation mode filter.
   * Tracks statistics and applies mode-appropriate behavior.
   */
  process(result: SafetyCheckResult): SafetyCheckResult {
    const processed = applyViolationMode(result, this.mode);

    if (!result.allowed) {
      if (this.mode === 'warn') {
        this.warnedCount++;
      } else {
        this.blockedCount++;
      }
    }

    return processed;
  }

  getStats(): { mode: ViolationMode; warnedCount: number; blockedCount: number } {
    return {
      mode: this.mode,
      warnedCount: this.warnedCount,
      blockedCount: this.blockedCount,
    };
  }

  reset(): void {
    this.warnedCount = 0;
    this.blockedCount = 0;
  }
}
