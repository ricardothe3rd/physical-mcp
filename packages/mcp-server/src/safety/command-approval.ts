/**
 * Command approval system (human-in-the-loop).
 *
 * When enabled, certain commands require explicit approval before execution.
 * Pending approvals expire after a configurable timeout.
 */

import type { CommandApprovalConfig } from './types.js';

export interface PendingApproval {
  id: string;
  toolName: string;
  target: string;
  args: Record<string, unknown>;
  createdAt: number;
  expiresAt: number;
}

export class CommandApprovalManager {
  private config: CommandApprovalConfig;
  private pending = new Map<string, PendingApproval>();
  private approvedIds = new Set<string>();
  private nextId = 1;

  constructor(config: CommandApprovalConfig) {
    this.config = config;
  }

  /** Check if a tool requires approval. */
  requiresApproval(toolName: string): boolean {
    if (!this.config.enabled) return false;
    return this.config.requireApprovalFor.includes(toolName);
  }

  /** Create a pending approval request. Returns the approval ID. */
  requestApproval(toolName: string, target: string, args: Record<string, unknown>): PendingApproval {
    this.cleanup();
    const id = `approval_${this.nextId++}`;
    const now = Date.now();
    const pending: PendingApproval = {
      id,
      toolName,
      target,
      args,
      createdAt: now,
      expiresAt: now + this.config.pendingTimeout,
    };
    this.pending.set(id, pending);
    return pending;
  }

  /** Approve a pending request. Returns true if the approval was valid. */
  approve(approvalId: string): boolean {
    this.cleanup();
    const pending = this.pending.get(approvalId);
    if (!pending) return false;
    if (Date.now() > pending.expiresAt) {
      this.pending.delete(approvalId);
      return false;
    }
    this.pending.delete(approvalId);
    this.approvedIds.add(approvalId);
    return true;
  }

  /** Deny a pending request. */
  deny(approvalId: string): boolean {
    const existed = this.pending.has(approvalId);
    this.pending.delete(approvalId);
    return existed;
  }

  /** Check if an approval ID has been approved. Consumes the approval (one-time use). */
  isApproved(approvalId: string): boolean {
    if (this.approvedIds.has(approvalId)) {
      this.approvedIds.delete(approvalId);
      return true;
    }
    return false;
  }

  /** Get all pending approvals. */
  getPendingApprovals(): PendingApproval[] {
    this.cleanup();
    return Array.from(this.pending.values());
  }

  /** Update the approval config. */
  updateConfig(config: Partial<CommandApprovalConfig>): void {
    Object.assign(this.config, config);
  }

  /** Get the current config. */
  getConfig(): CommandApprovalConfig {
    return { ...this.config };
  }

  /** Remove expired pending approvals. */
  private cleanup(): void {
    const now = Date.now();
    for (const [id, pending] of this.pending) {
      if (now > pending.expiresAt) {
        this.pending.delete(id);
      }
    }
  }
}
