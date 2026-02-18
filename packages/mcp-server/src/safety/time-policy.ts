/**
 * Time-based safety policies: apply different limits at different times of day.
 *
 * Example use cases:
 * - Reduce velocity at night when operators are not watching
 * - Block all commands during maintenance windows
 * - Allow higher limits during supervised testing hours
 */

import type { SafetyPolicy } from './types.js';
import { mergePolicies } from './policy-inheritance.js';

export interface TimeSchedule {
  /** Human-readable name for this schedule. */
  name: string;
  /** Start hour (0-23, inclusive). */
  startHour: number;
  /** End hour (0-23, exclusive). If endHour < startHour, wraps past midnight. */
  endHour: number;
  /** Days of week this schedule applies (0=Sunday, 6=Saturday). If empty, applies every day. */
  daysOfWeek?: number[];
  /** Policy overrides to apply during this window. */
  overrides: Partial<SafetyPolicy>;
}

export interface TimePolicyConfig {
  enabled: boolean;
  schedules: TimeSchedule[];
}

/**
 * Check if a given hour (0-23) falls within a schedule's time window.
 * Handles overnight ranges (e.g., 22:00 to 06:00).
 */
export function isInTimeWindow(hour: number, schedule: TimeSchedule): boolean {
  if (schedule.startHour <= schedule.endHour) {
    // Normal range: e.g., 08:00 to 18:00
    return hour >= schedule.startHour && hour < schedule.endHour;
  } else {
    // Overnight range: e.g., 22:00 to 06:00
    return hour >= schedule.startHour || hour < schedule.endHour;
  }
}

/**
 * Check if a given day of week matches a schedule.
 * If daysOfWeek is empty/undefined, matches every day.
 */
export function isDayMatch(dayOfWeek: number, schedule: TimeSchedule): boolean {
  if (!schedule.daysOfWeek || schedule.daysOfWeek.length === 0) {
    return true;
  }
  return schedule.daysOfWeek.includes(dayOfWeek);
}

/**
 * Get the active time schedule for a given date.
 * Returns the first matching schedule, or null if none match.
 * Later schedules in the array have priority over earlier ones.
 */
export function getActiveSchedule(
  config: TimePolicyConfig,
  now: Date = new Date(),
): TimeSchedule | null {
  if (!config.enabled || config.schedules.length === 0) {
    return null;
  }

  const hour = now.getHours();
  const dayOfWeek = now.getDay();

  // Last matching schedule wins (higher priority at end of array)
  let match: TimeSchedule | null = null;
  for (const schedule of config.schedules) {
    if (isInTimeWindow(hour, schedule) && isDayMatch(dayOfWeek, schedule)) {
      match = schedule;
    }
  }

  return match;
}

/**
 * Apply time-based overrides to a base policy.
 * Returns the modified policy if a schedule matches, or the base policy if none match.
 */
export function applyTimePolicy(
  base: SafetyPolicy,
  config: TimePolicyConfig,
  now: Date = new Date(),
): { policy: SafetyPolicy; activeSchedule: string | null } {
  const schedule = getActiveSchedule(config, now);

  if (!schedule) {
    return { policy: base, activeSchedule: null };
  }

  const merged = mergePolicies(base, schedule.overrides);
  return { policy: merged, activeSchedule: schedule.name };
}
