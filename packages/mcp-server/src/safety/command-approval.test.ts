/**
 * Tests for command approval system.
 */

import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { CommandApprovalManager } from './command-approval.js';

describe('CommandApprovalManager', () => {
  let manager: CommandApprovalManager;

  beforeEach(() => {
    vi.useFakeTimers();
    manager = new CommandApprovalManager({
      enabled: true,
      requireApprovalFor: ['ros2_topic_publish', 'ros2_action_send_goal'],
      pendingTimeout: 30000,
    });
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  describe('requiresApproval', () => {
    it('returns true for tools in the approval list', () => {
      expect(manager.requiresApproval('ros2_topic_publish')).toBe(true);
      expect(manager.requiresApproval('ros2_action_send_goal')).toBe(true);
    });

    it('returns false for tools not in the approval list', () => {
      expect(manager.requiresApproval('ros2_topic_list')).toBe(false);
      expect(manager.requiresApproval('safety_status')).toBe(false);
    });

    it('returns false when approval is disabled', () => {
      manager.updateConfig({ enabled: false });
      expect(manager.requiresApproval('ros2_topic_publish')).toBe(false);
    });
  });

  describe('requestApproval', () => {
    it('creates a pending approval with unique ID', () => {
      const a1 = manager.requestApproval('ros2_topic_publish', '/cmd_vel', { message: {} });
      const a2 = manager.requestApproval('ros2_topic_publish', '/cmd_vel', { message: {} });

      expect(a1.id).not.toBe(a2.id);
      expect(a1.toolName).toBe('ros2_topic_publish');
      expect(a1.target).toBe('/cmd_vel');
    });

    it('sets correct expiration time', () => {
      const now = Date.now();
      const approval = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});

      expect(approval.createdAt).toBe(now);
      expect(approval.expiresAt).toBe(now + 30000);
    });
  });

  describe('approve', () => {
    it('approves a valid pending request', () => {
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      expect(manager.approve(pending.id)).toBe(true);
    });

    it('returns false for nonexistent approval', () => {
      expect(manager.approve('nonexistent')).toBe(false);
    });

    it('returns false for expired approval', () => {
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      vi.advanceTimersByTime(31000);
      expect(manager.approve(pending.id)).toBe(false);
    });

    it('removes from pending after approval', () => {
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      manager.approve(pending.id);
      expect(manager.getPendingApprovals()).toHaveLength(0);
    });
  });

  describe('deny', () => {
    it('removes a pending request', () => {
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      expect(manager.deny(pending.id)).toBe(true);
      expect(manager.getPendingApprovals()).toHaveLength(0);
    });

    it('returns false for nonexistent approval', () => {
      expect(manager.deny('nonexistent')).toBe(false);
    });
  });

  describe('isApproved', () => {
    it('returns true for approved requests', () => {
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      manager.approve(pending.id);
      expect(manager.isApproved(pending.id)).toBe(true);
    });

    it('consumes the approval (one-time use)', () => {
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      manager.approve(pending.id);
      expect(manager.isApproved(pending.id)).toBe(true);
      expect(manager.isApproved(pending.id)).toBe(false);
    });

    it('returns false for unapproved requests', () => {
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      expect(manager.isApproved(pending.id)).toBe(false);
    });
  });

  describe('getPendingApprovals', () => {
    it('returns all pending approvals', () => {
      manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      manager.requestApproval('ros2_action_send_goal', '/navigate', {});

      expect(manager.getPendingApprovals()).toHaveLength(2);
    });

    it('excludes expired approvals', () => {
      manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      vi.advanceTimersByTime(31000);

      expect(manager.getPendingApprovals()).toHaveLength(0);
    });
  });

  describe('updateConfig', () => {
    it('updates the timeout', () => {
      manager.updateConfig({ pendingTimeout: 5000 });
      const pending = manager.requestApproval('ros2_topic_publish', '/cmd_vel', {});
      expect(pending.expiresAt - pending.createdAt).toBe(5000);
    });

    it('can add tools to approval list', () => {
      manager.updateConfig({ requireApprovalFor: ['ros2_topic_publish', 'ros2_service_call'] });
      expect(manager.requiresApproval('ros2_service_call')).toBe(true);
    });
  });

  describe('getConfig', () => {
    it('returns a copy of the config', () => {
      const config = manager.getConfig();
      expect(config.enabled).toBe(true);
      expect(config.requireApprovalFor).toContain('ros2_topic_publish');
      // Modifying the returned config shouldn't affect the manager
      config.enabled = false;
      expect(manager.getConfig().enabled).toBe(true);
    });
  });
});
