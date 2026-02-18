/**
 * Tests for time-based safety policies.
 */

import { describe, it, expect } from 'vitest';
import {
  isInTimeWindow,
  isDayMatch,
  getActiveSchedule,
  applyTimePolicy,
  type TimeSchedule,
  type TimePolicyConfig,
} from './time-policy.js';
import { getDefaultPolicy } from './policy-loader.js';

describe('Time-Based Policies', () => {
  describe('isInTimeWindow', () => {
    const daytimeSchedule: TimeSchedule = {
      name: 'daytime',
      startHour: 8,
      endHour: 18,
      overrides: {},
    };

    const nightSchedule: TimeSchedule = {
      name: 'night',
      startHour: 22,
      endHour: 6,
      overrides: {},
    };

    it('matches hours within a normal range', () => {
      expect(isInTimeWindow(8, daytimeSchedule)).toBe(true);
      expect(isInTimeWindow(12, daytimeSchedule)).toBe(true);
      expect(isInTimeWindow(17, daytimeSchedule)).toBe(true);
    });

    it('excludes hours outside a normal range', () => {
      expect(isInTimeWindow(7, daytimeSchedule)).toBe(false);
      expect(isInTimeWindow(18, daytimeSchedule)).toBe(false);
      expect(isInTimeWindow(0, daytimeSchedule)).toBe(false);
    });

    it('matches hours in an overnight range', () => {
      expect(isInTimeWindow(22, nightSchedule)).toBe(true);
      expect(isInTimeWindow(23, nightSchedule)).toBe(true);
      expect(isInTimeWindow(0, nightSchedule)).toBe(true);
      expect(isInTimeWindow(3, nightSchedule)).toBe(true);
      expect(isInTimeWindow(5, nightSchedule)).toBe(true);
    });

    it('excludes hours outside an overnight range', () => {
      expect(isInTimeWindow(6, nightSchedule)).toBe(false);
      expect(isInTimeWindow(12, nightSchedule)).toBe(false);
      expect(isInTimeWindow(21, nightSchedule)).toBe(false);
    });

    it('handles full-day range (0-24 equivalent)', () => {
      const allDay: TimeSchedule = { name: 'all', startHour: 0, endHour: 0, overrides: {} };
      // startHour <= endHour and both 0, so hour must be >= 0 AND < 0 → always false
      expect(isInTimeWindow(0, allDay)).toBe(false);
      expect(isInTimeWindow(12, allDay)).toBe(false);
    });
  });

  describe('isDayMatch', () => {
    it('matches when daysOfWeek is empty', () => {
      const schedule: TimeSchedule = { name: 'test', startHour: 0, endHour: 24, overrides: {} };
      expect(isDayMatch(0, schedule)).toBe(true);
      expect(isDayMatch(6, schedule)).toBe(true);
    });

    it('matches when daysOfWeek is undefined', () => {
      const schedule: TimeSchedule = { name: 'test', startHour: 0, endHour: 24, daysOfWeek: undefined, overrides: {} };
      expect(isDayMatch(3, schedule)).toBe(true);
    });

    it('matches specific days', () => {
      const weekdays: TimeSchedule = {
        name: 'weekdays',
        startHour: 0,
        endHour: 24,
        daysOfWeek: [1, 2, 3, 4, 5],
        overrides: {},
      };
      expect(isDayMatch(1, weekdays)).toBe(true); // Monday
      expect(isDayMatch(5, weekdays)).toBe(true); // Friday
      expect(isDayMatch(0, weekdays)).toBe(false); // Sunday
      expect(isDayMatch(6, weekdays)).toBe(false); // Saturday
    });
  });

  describe('getActiveSchedule', () => {
    it('returns null when disabled', () => {
      const config: TimePolicyConfig = {
        enabled: false,
        schedules: [{ name: 'test', startHour: 0, endHour: 24, overrides: {} }],
      };
      expect(getActiveSchedule(config)).toBeNull();
    });

    it('returns null when no schedules match', () => {
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          { name: 'daytime', startHour: 8, endHour: 18, overrides: {} },
        ],
      };
      // Hour 3 is outside 8-18
      const threeAm = new Date('2025-01-15T03:00:00');
      expect(getActiveSchedule(config, threeAm)).toBeNull();
    });

    it('returns matching schedule', () => {
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          { name: 'daytime', startHour: 8, endHour: 18, overrides: {} },
        ],
      };
      const noon = new Date('2025-01-15T12:00:00');
      const result = getActiveSchedule(config, noon);
      expect(result).not.toBeNull();
      expect(result!.name).toBe('daytime');
    });

    it('last matching schedule wins (higher priority)', () => {
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          { name: 'general', startHour: 8, endHour: 18, overrides: {} },
          { name: 'specific', startHour: 10, endHour: 14, overrides: {} },
        ],
      };
      const elevenAm = new Date('2025-01-15T11:00:00');
      const result = getActiveSchedule(config, elevenAm);
      expect(result!.name).toBe('specific');
    });

    it('matches day-of-week filter', () => {
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          {
            name: 'weekend-lockdown',
            startHour: 0,
            endHour: 23,
            daysOfWeek: [0, 6], // Sunday, Saturday
            overrides: {},
          },
        ],
      };
      // Jan 18, 2025 is a Saturday
      const saturday = new Date('2025-01-18T12:00:00');
      expect(getActiveSchedule(config, saturday)?.name).toBe('weekend-lockdown');

      // Jan 15, 2025 is a Wednesday
      const wednesday = new Date('2025-01-15T12:00:00');
      expect(getActiveSchedule(config, wednesday)).toBeNull();
    });
  });

  describe('applyTimePolicy', () => {
    it('returns base policy when no schedule matches', () => {
      const base = getDefaultPolicy();
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [],
      };
      const { policy, activeSchedule } = applyTimePolicy(base, config);
      expect(policy.velocity.linearMax).toBe(base.velocity.linearMax);
      expect(activeSchedule).toBeNull();
    });

    it('applies velocity override during matching window', () => {
      const base = getDefaultPolicy();
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          {
            name: 'night-mode',
            startHour: 22,
            endHour: 6,
            overrides: {
              velocity: { linearMax: 0.1, angularMax: 0.5, clampMode: false },
            },
          },
        ],
      };
      const midnight = new Date('2025-01-15T00:00:00');
      const { policy, activeSchedule } = applyTimePolicy(base, config, midnight);
      expect(policy.velocity.linearMax).toBe(0.1);
      expect(policy.velocity.angularMax).toBe(0.5);
      expect(activeSchedule).toBe('night-mode');
    });

    it('does not apply override outside matching window', () => {
      const base = getDefaultPolicy();
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          {
            name: 'night-mode',
            startHour: 22,
            endHour: 6,
            overrides: {
              velocity: { linearMax: 0.1, angularMax: 0.5, clampMode: false },
            },
          },
        ],
      };
      const noon = new Date('2025-01-15T12:00:00');
      const { policy, activeSchedule } = applyTimePolicy(base, config, noon);
      expect(policy.velocity.linearMax).toBe(0.5); // default
      expect(activeSchedule).toBeNull();
    });

    it('does not mutate base policy', () => {
      const base = getDefaultPolicy();
      const originalMax = base.velocity.linearMax;
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          {
            name: 'test',
            startHour: 0,
            endHour: 23,
            overrides: {
              velocity: { linearMax: 10.0, angularMax: 10.0, clampMode: true },
            },
          },
        ],
      };
      applyTimePolicy(base, config, new Date('2025-01-15T12:00:00'));
      expect(base.velocity.linearMax).toBe(originalMax);
    });

    it('applies multiple overrides from matching schedule', () => {
      const base = getDefaultPolicy();
      const config: TimePolicyConfig = {
        enabled: true,
        schedules: [
          {
            name: 'maintenance',
            startHour: 2,
            endHour: 4,
            overrides: {
              velocity: { linearMax: 0, angularMax: 0, clampMode: false },
              blockedTopics: ['/cmd_vel', '/rosout', '/parameter_events'],
              description: 'Maintenance window - all motion blocked',
            },
          },
        ],
      };
      const threeAm = new Date('2025-01-15T03:00:00');
      const { policy } = applyTimePolicy(base, config, threeAm);
      expect(policy.velocity.linearMax).toBe(0);
      expect(policy.blockedTopics).toContain('/cmd_vel');
      expect(policy.description).toBe('Maintenance window - all motion blocked');
    });
  });
});
