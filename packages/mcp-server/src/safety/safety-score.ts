/**
 * Safety score tracking per session.
 *
 * Tracks the percentage of commands that were blocked by the safety system,
 * providing a quick measure of how "safe" an AI agent session has been.
 */

export interface SafetyScoreSnapshot {
  totalCommands: number;
  blockedCommands: number;
  allowedCommands: number;
  blockRate: number;        // 0.0 - 1.0
  safetyScore: number;      // 0 - 100 (100 = all allowed, 0 = all blocked)
  violationsByType: Record<string, number>;
  sessionStartedAt: number;
  sessionDurationMs: number;
}

export class SafetyScoreTracker {
  private totalCommands = 0;
  private blockedCommands = 0;
  private violationsByType = new Map<string, number>();
  private sessionStart = Date.now();

  recordAllowed(): void {
    this.totalCommands++;
  }

  recordBlocked(violationTypes: string[]): void {
    this.totalCommands++;
    this.blockedCommands++;
    for (const type of violationTypes) {
      this.violationsByType.set(type, (this.violationsByType.get(type) || 0) + 1);
    }
  }

  getSnapshot(): SafetyScoreSnapshot {
    const allowed = this.totalCommands - this.blockedCommands;
    const blockRate = this.totalCommands > 0 ? this.blockedCommands / this.totalCommands : 0;

    return {
      totalCommands: this.totalCommands,
      blockedCommands: this.blockedCommands,
      allowedCommands: allowed,
      blockRate: Math.round(blockRate * 1000) / 1000,
      safetyScore: this.totalCommands > 0
        ? Math.round((1 - blockRate) * 100)
        : 100,
      violationsByType: Object.fromEntries(this.violationsByType),
      sessionStartedAt: this.sessionStart,
      sessionDurationMs: Date.now() - this.sessionStart,
    };
  }

  reset(): void {
    this.totalCommands = 0;
    this.blockedCommands = 0;
    this.violationsByType.clear();
    this.sessionStart = Date.now();
  }
}
