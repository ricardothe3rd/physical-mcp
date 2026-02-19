/**
 * Tests for waypoint management tools.
 */

import { describe, it, expect, vi, beforeEach } from 'vitest';
import { getWaypointTools, handleWaypointTool, waypointStore } from './waypoint-tools.js';
import { PolicyEngine } from '../safety/policy-engine.js';

function createMockConnection() {
  return {
    isConnected: true,
    connect: vi.fn(),
    send: vi.fn().mockResolvedValue({
      id: '1',
      status: 'ok',
      data: { goal_id: 'goal_123' },
      timestamp: Date.now(),
    }),
    disconnect: vi.fn(),
  } as any;
}

describe('Waypoint Tools', () => {
  let safety: PolicyEngine;

  beforeEach(() => {
    waypointStore.clear();
    safety = new PolicyEngine();
  });

  describe('tool definitions', () => {
    it('returns 4 tools', () => {
      const tools = getWaypointTools();
      expect(tools.length).toBe(4);
    });

    it('has all expected tool names', () => {
      const names = getWaypointTools().map(t => t.name);
      expect(names).toContain('ros2_waypoint_save');
      expect(names).toContain('ros2_waypoint_list');
      expect(names).toContain('ros2_waypoint_delete');
      expect(names).toContain('ros2_waypoint_navigate');
    });

    it('all tools have descriptions', () => {
      for (const tool of getWaypointTools()) {
        expect(tool.description.length).toBeGreaterThan(10);
      }
    });
  });

  describe('ros2_waypoint_save', () => {
    it('saves a new waypoint', async () => {
      const connection = createMockConnection();
      const result = await handleWaypointTool('ros2_waypoint_save', {
        name: 'home',
        x: 1.0,
        y: 2.0,
        z: 0,
      }, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('saved');
      expect(waypointStore.has('home')).toBe(true);
    });

    it('updates an existing waypoint', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', {
        name: 'home',
        x: 1.0,
        y: 2.0,
      }, connection, safety);
      const result = await handleWaypointTool('ros2_waypoint_save', {
        name: 'home',
        x: 3.0,
        y: 4.0,
      }, connection, safety);
      expect(result.content[0].text).toContain('updated');
      expect(waypointStore.get('home')!.x).toBe(3.0);
    });

    it('saves waypoint with orientation', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', {
        name: 'dock',
        x: 0,
        y: 0,
        orientationX: 0,
        orientationY: 0,
        orientationZ: 0.707,
        orientationW: 0.707,
      }, connection, safety);
      const wp = waypointStore.get('dock')!;
      expect(wp.orientation).toBeDefined();
      expect(wp.orientation!.z).toBeCloseTo(0.707);
    });

    it('uses default frame "map"', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', {
        name: 'test',
        x: 0,
        y: 0,
      }, connection, safety);
      expect(waypointStore.get('test')!.frame).toBe('map');
    });

    it('accepts custom frame', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', {
        name: 'test',
        x: 0,
        y: 0,
        frame: 'odom',
      }, connection, safety);
      expect(waypointStore.get('test')!.frame).toBe('odom');
    });
  });

  describe('ros2_waypoint_list', () => {
    it('returns empty when no waypoints', async () => {
      const connection = createMockConnection();
      const result = await handleWaypointTool('ros2_waypoint_list', {}, connection, safety);
      expect(result.content[0].text).toContain('No waypoints');
    });

    it('lists all waypoints', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', { name: 'a', x: 1, y: 2 }, connection, safety);
      await handleWaypointTool('ros2_waypoint_save', { name: 'b', x: 3, y: 4 }, connection, safety);
      const result = await handleWaypointTool('ros2_waypoint_list', {}, connection, safety);
      const data = JSON.parse(result.content[0].text);
      expect(data.length).toBe(2);
      expect(data[0].name).toBe('a');
      expect(data[1].name).toBe('b');
    });
  });

  describe('ros2_waypoint_delete', () => {
    it('deletes an existing waypoint', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', { name: 'temp', x: 0, y: 0 }, connection, safety);
      const result = await handleWaypointTool('ros2_waypoint_delete', { name: 'temp' }, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('deleted');
      expect(waypointStore.has('temp')).toBe(false);
    });

    it('returns error for unknown waypoint', async () => {
      const connection = createMockConnection();
      const result = await handleWaypointTool('ros2_waypoint_delete', { name: 'unknown' }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('not found');
    });
  });

  describe('ros2_waypoint_navigate', () => {
    it('sends navigation goal to saved waypoint', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', { name: 'goal', x: 2, y: 3 }, connection, safety);
      const result = await handleWaypointTool('ros2_waypoint_navigate', { name: 'goal' }, connection, safety);
      expect(result.isError).toBeUndefined();
      expect(result.content[0].text).toContain('Navigating to waypoint');
      expect(connection.send).toHaveBeenCalledWith('action.send_goal', expect.objectContaining({
        action: '/navigate_to_pose',
      }));
    });

    it('returns error for unknown waypoint', async () => {
      const connection = createMockConnection();
      const result = await handleWaypointTool('ros2_waypoint_navigate', { name: 'unknown' }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('not found');
    });

    it('blocks navigation to waypoint outside geofence', async () => {
      const connection = createMockConnection();
      // Default geofence is 10x10 centered at origin, so (100, 100) is outside
      await handleWaypointTool('ros2_waypoint_save', { name: 'far', x: 100, y: 100 }, connection, safety);
      const result = await handleWaypointTool('ros2_waypoint_navigate', { name: 'far' }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('outside geofence');
    });

    it('allows navigation to waypoint inside geofence', async () => {
      const connection = createMockConnection();
      await handleWaypointTool('ros2_waypoint_save', { name: 'near', x: 1, y: 1 }, connection, safety);
      const result = await handleWaypointTool('ros2_waypoint_navigate', { name: 'near' }, connection, safety);
      expect(result.isError).toBeUndefined();
    });

    it('handles bridge error during navigation', async () => {
      const connection = createMockConnection();
      connection.send.mockRejectedValueOnce(new Error('ok')).mockRejectedValue(new Error('Bridge unavailable'));
      // First send is for save (will fail but waypoint still saved locally)
      await handleWaypointTool('ros2_waypoint_save', { name: 'test', x: 1, y: 1 }, connection, safety);
      const result = await handleWaypointTool('ros2_waypoint_navigate', { name: 'test' }, connection, safety);
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Navigation failed');
    });
  });

  describe('unknown tool', () => {
    it('returns error for unknown waypoint tool', async () => {
      const connection = createMockConnection();
      const result = await handleWaypointTool('nonexistent', {}, connection, safety);
      expect(result.isError).toBe(true);
    });
  });
});
