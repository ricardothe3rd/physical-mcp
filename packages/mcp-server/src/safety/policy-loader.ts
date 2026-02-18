/**
 * Load safety policies from YAML config files.
 */

import { readFileSync } from 'fs';
import { parse as parseYaml } from 'yaml';
import type { SafetyPolicy, TopicVelocityOverride } from './types.js';

const DEFAULT_POLICY: SafetyPolicy = {
  name: 'default',
  description: 'Conservative default safety policy',
  velocity: {
    linearMax: 0.5,
    angularMax: 1.5,
    clampMode: false,
  },
  acceleration: {
    linearMaxAccel: 1.0,
    angularMaxAccel: 3.0,
    enabled: false,
  },
  geofence: {
    xMin: -5, xMax: 5,
    yMin: -5, yMax: 5,
    zMin: 0, zMax: 2,
  },
  geofenceWarningMargin: 1.0,
  rateLimits: {
    publishHz: 10,
    servicePerMinute: 60,
    actionPerMinute: 30,
  },
  deadmanSwitch: {
    enabled: false,
    timeoutMs: 30000,
  },
  blockedTopics: ['/rosout', '/parameter_events'],
  blockedServices: ['/kill', '/shutdown'],
  commandApproval: {
    enabled: false,
    requireApprovalFor: [],
    pendingTimeout: 30000,
  },
};

export function loadPolicy(filePath?: string): SafetyPolicy {
  if (!filePath) {
    return { ...DEFAULT_POLICY };
  }

  try {
    const raw = readFileSync(filePath, 'utf-8');
    const data = parseYaml(raw);
    return mergePolicyWithDefaults(data);
  } catch (err) {
    console.error(`[PolicyLoader] Failed to load ${filePath}, using defaults: ${err}`);
    return { ...DEFAULT_POLICY };
  }
}

function mergePolicyWithDefaults(data: Record<string, unknown>): SafetyPolicy {
  return {
    name: (data.name as string) || DEFAULT_POLICY.name,
    description: (data.description as string) || DEFAULT_POLICY.description,
    velocity: {
      ...DEFAULT_POLICY.velocity,
      ...(data.velocity as Record<string, unknown> || {}),
    },
    acceleration: {
      ...DEFAULT_POLICY.acceleration,
      ...(data.acceleration as Record<string, unknown> || {}),
    },
    geofence: {
      ...DEFAULT_POLICY.geofence,
      ...(data.geofence as Record<string, number> || {}),
    },
    geofenceWarningMargin: (data.geofenceWarningMargin as number) ?? DEFAULT_POLICY.geofenceWarningMargin,
    rateLimits: {
      ...DEFAULT_POLICY.rateLimits,
      ...(data.rateLimits as Record<string, number> || {}),
    },
    deadmanSwitch: {
      ...DEFAULT_POLICY.deadmanSwitch,
      ...(data.deadmanSwitch as Record<string, unknown> || {}),
    },
    blockedTopics: (data.blockedTopics as string[]) || DEFAULT_POLICY.blockedTopics,
    blockedServices: (data.blockedServices as string[]) || DEFAULT_POLICY.blockedServices,
    allowedTopics: data.allowedTopics as string[] | undefined,
    allowedServices: data.allowedServices as string[] | undefined,
    topicVelocityOverrides: data.topicVelocityOverrides as SafetyPolicy['topicVelocityOverrides'],
    commandApproval: {
      ...DEFAULT_POLICY.commandApproval,
      ...(data.commandApproval as Record<string, unknown> || {}),
    },
  };
}

export function getDefaultPolicy(): SafetyPolicy {
  return {
    ...DEFAULT_POLICY,
    velocity: { ...DEFAULT_POLICY.velocity },
    acceleration: { ...DEFAULT_POLICY.acceleration },
    geofence: { ...DEFAULT_POLICY.geofence },
    rateLimits: { ...DEFAULT_POLICY.rateLimits },
    deadmanSwitch: { ...DEFAULT_POLICY.deadmanSwitch },
    blockedTopics: [...DEFAULT_POLICY.blockedTopics],
    blockedServices: [...DEFAULT_POLICY.blockedServices],
    commandApproval: { ...DEFAULT_POLICY.commandApproval, requireApprovalFor: [...DEFAULT_POLICY.commandApproval.requireApprovalFor] },
  };
}
