/**
 * ROS2 type definitions used across the MCP server.
 */

export interface RosTopic {
  name: string;
  type: string;
  publisherCount?: number;
  subscriberCount?: number;
}

export interface RosService {
  name: string;
  type: string;
}

export interface RosAction {
  name: string;
  type: string;
}

export interface RosNode {
  name: string;
  namespace: string;
}

export interface Twist {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}
