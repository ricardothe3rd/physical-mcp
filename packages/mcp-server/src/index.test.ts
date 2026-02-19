/**
 * Integration tests for the PhysicalMCP MCP server entry point (index.ts).
 *
 * Since index.ts has module-level side effects (parseArgs, server creation,
 * process.on handlers), we mock all external dependencies and capture the
 * registered request handlers via the MCP SDK Server mock.
 */

import { describe, it, expect, vi, beforeEach, beforeAll } from 'vitest';

// ---------------------------------------------------------------------------
// Side-channel: expose captured handlers and mock instances via globalThis
// ---------------------------------------------------------------------------
declare global {
  // eslint-disable-next-line no-var
  var __test_captured_handlers: Map<unknown, Function>;
  // eslint-disable-next-line no-var
  var __test_mock_connection: {
    isConnected: boolean;
    connect: ReturnType<typeof vi.fn>;
    disconnect: ReturnType<typeof vi.fn>;
    send: ReturnType<typeof vi.fn>;
    isBridgeAvailable: boolean;
  };
  // eslint-disable-next-line no-var
  var __test_mock_safety: {
    destroy: ReturnType<typeof vi.fn>;
  };
  // eslint-disable-next-line no-var
  var __test_mock_server: {
    setRequestHandler: ReturnType<typeof vi.fn>;
    connect: ReturnType<typeof vi.fn>;
  };
}

globalThis.__test_captured_handlers = new Map();

// ---------------------------------------------------------------------------
// Mock: @modelcontextprotocol/sdk
// ---------------------------------------------------------------------------
vi.mock('@modelcontextprotocol/sdk/server/index.js', () => {
  class MockServer {
    setRequestHandler = vi.fn((schema: unknown, handler: Function) => {
      globalThis.__test_captured_handlers.set(schema, handler);
    });
    connect = vi.fn().mockResolvedValue(undefined);
  }
  const instance = new MockServer();
  globalThis.__test_mock_server = instance as any;
  return { Server: vi.fn(() => instance) };
});

vi.mock('@modelcontextprotocol/sdk/server/stdio.js', () => {
  class MockStdioTransport {}
  return { StdioServerTransport: vi.fn(() => new MockStdioTransport()) };
});

vi.mock('@modelcontextprotocol/sdk/types.js', () => {
  const ListToolsRequestSchema = Symbol('ListToolsRequestSchema');
  const CallToolRequestSchema = Symbol('CallToolRequestSchema');
  return { ListToolsRequestSchema, CallToolRequestSchema };
});

// ---------------------------------------------------------------------------
// Mock: ConnectionManager
// ---------------------------------------------------------------------------
vi.mock('./bridge/connection-manager.js', () => {
  class MockConnectionManager {
    isConnected = false;
    isBridgeAvailable = false;
    connect = vi.fn().mockRejectedValue(new Error('Bridge not available'));
    disconnect = vi.fn();
    send = vi.fn().mockResolvedValue({ id: '1', status: 'ok', data: {}, timestamp: Date.now() });
  }
  return {
    ConnectionManager: vi.fn(() => {
      const instance = new MockConnectionManager();
      globalThis.__test_mock_connection = instance as any;
      return instance;
    }),
  };
});

// ---------------------------------------------------------------------------
// Mock: PolicyEngine
// ---------------------------------------------------------------------------
vi.mock('./safety/policy-engine.js', () => {
  class MockPolicyEngine {
    destroy = vi.fn();
  }
  return {
    PolicyEngine: vi.fn(() => {
      const instance = new MockPolicyEngine();
      globalThis.__test_mock_safety = instance as any;
      return instance;
    }),
  };
});

// ---------------------------------------------------------------------------
// Mock: startup-check (prevent process.exit on failed checks)
// ---------------------------------------------------------------------------
vi.mock('./utils/startup-check.js', () => ({
  runStartupChecks: vi.fn(() => ({ passed: true, checks: [] })),
  printStartupChecks: vi.fn(),
}));

// ---------------------------------------------------------------------------
// Mock: all 17 tool modules
// We give each category one fake tool and a mock handler so we can verify
// dispatch routing without pulling in real implementations.
// ---------------------------------------------------------------------------
function makeMockToolModule(category: string, toolNames: string[]) {
  const getTools = vi.fn(() =>
    toolNames.map(name => ({
      name,
      description: `Mock ${category} tool`,
      inputSchema: { type: 'object', properties: {} },
    }))
  );
  const handleTool = vi.fn(async (name: string) => ({
    content: [{ type: 'text', text: `${category}:${name}:ok` }],
  }));
  return { getTools, handleTool };
}

const topicMock = makeMockToolModule('topic', ['ros2_topic_list', 'ros2_topic_publish']);
const serviceMock = makeMockToolModule('service', ['ros2_service_list', 'ros2_service_call']);
const actionMock = makeMockToolModule('action', ['ros2_action_list', 'ros2_action_send_goal']);
const safetyMock = makeMockToolModule('safety', ['safety_status', 'safety_emergency_stop']);
const systemMock = makeMockToolModule('system', ['system_bridge_status', 'system_node_list']);
const batchMock = makeMockToolModule('batch', ['ros2_batch_execute']);
const recordingMock = makeMockToolModule('recording', ['ros2_topic_record_start']);
const conditionalMock = makeMockToolModule('conditional', ['ros2_conditional_execute']);
const scheduledMock = makeMockToolModule('scheduled', ['ros2_schedule_command']);
const tfMock = makeMockToolModule('tf', ['ros2_tf_lookup']);
const diagnosticMock = makeMockToolModule('diagnostic', ['ros2_diagnostic_summary']);
const fleetMock = makeMockToolModule('fleet', ['ros2_fleet_list']);
const launchMock = makeMockToolModule('launch', ['ros2_launch_start']);
const waypointMock = makeMockToolModule('waypoint', ['ros2_waypoint_follow']);
const introspectionMock = makeMockToolModule('introspection', ['ros2_msg_type_info']);
const namespaceMock = makeMockToolModule('namespace', ['ros2_namespace_list']);
const sensorMock = makeMockToolModule('sensor', ['ros2_sensor_summary']);
const paramMock = makeMockToolModule('param', ['ros2_param_list', 'ros2_param_get']);
const descriptionMock = makeMockToolModule('description', ['ros2_robot_description']);
const powerMock = makeMockToolModule('power', ['ros2_battery_status']);
const cameraMock = makeMockToolModule('camera', ['ros2_camera_info']);
const mapMock = makeMockToolModule('map', ['ros2_map_info']);
const historyMock = makeMockToolModule('history', ['ros2_command_history']);
const pathMock = makeMockToolModule('path', ['ros2_plan_path']);
const networkMock = makeMockToolModule('network', ['ros2_network_stats']);

vi.mock('./tools/topic-tools.js', () => ({
  getTopicTools: topicMock.getTools,
  handleTopicTool: topicMock.handleTool,
}));
vi.mock('./tools/service-tools.js', () => ({
  getServiceTools: serviceMock.getTools,
  handleServiceTool: serviceMock.handleTool,
}));
vi.mock('./tools/action-tools.js', () => ({
  getActionTools: actionMock.getTools,
  handleActionTool: actionMock.handleTool,
}));
vi.mock('./tools/safety-tools.js', () => ({
  getSafetyTools: safetyMock.getTools,
  handleSafetyTool: safetyMock.handleTool,
}));
vi.mock('./tools/system-tools.js', () => ({
  getSystemTools: systemMock.getTools,
  handleSystemTool: systemMock.handleTool,
}));
vi.mock('./tools/batch-tools.js', () => ({
  getBatchTools: batchMock.getTools,
  handleBatchTool: batchMock.handleTool,
}));
vi.mock('./tools/recording-tools.js', () => ({
  getRecordingTools: recordingMock.getTools,
  handleRecordingTool: recordingMock.handleTool,
}));
vi.mock('./tools/conditional-tools.js', () => ({
  getConditionalTools: conditionalMock.getTools,
  handleConditionalTool: conditionalMock.handleTool,
}));
vi.mock('./tools/scheduled-tools.js', () => ({
  getScheduledTools: scheduledMock.getTools,
  handleScheduledTool: scheduledMock.handleTool,
}));
vi.mock('./tools/tf-tools.js', () => ({
  getTfTools: tfMock.getTools,
  handleTfTool: tfMock.handleTool,
}));
vi.mock('./tools/diagnostic-tools.js', () => ({
  getDiagnosticTools: diagnosticMock.getTools,
  handleDiagnosticTool: diagnosticMock.handleTool,
}));
vi.mock('./tools/fleet-tools.js', () => ({
  getFleetTools: fleetMock.getTools,
  handleFleetTool: fleetMock.handleTool,
}));
vi.mock('./tools/launch-tools.js', () => ({
  getLaunchTools: launchMock.getTools,
  handleLaunchTool: launchMock.handleTool,
}));
vi.mock('./tools/waypoint-tools.js', () => ({
  getWaypointTools: waypointMock.getTools,
  handleWaypointTool: waypointMock.handleTool,
}));
vi.mock('./tools/introspection-tools.js', () => ({
  getIntrospectionTools: introspectionMock.getTools,
  handleIntrospectionTool: introspectionMock.handleTool,
}));
vi.mock('./tools/namespace-tools.js', () => ({
  getNamespaceTools: namespaceMock.getTools,
  handleNamespaceTool: namespaceMock.handleTool,
}));
vi.mock('./tools/sensor-tools.js', () => ({
  getSensorTools: sensorMock.getTools,
  handleSensorTool: sensorMock.handleTool,
}));
vi.mock('./tools/param-tools.js', () => ({
  getParamTools: paramMock.getTools,
  handleParamTool: paramMock.handleTool,
}));
vi.mock('./tools/description-tools.js', () => ({
  getDescriptionTools: descriptionMock.getTools,
  handleDescriptionTool: descriptionMock.handleTool,
}));
vi.mock('./tools/power-tools.js', () => ({
  getPowerTools: powerMock.getTools,
  handlePowerTool: powerMock.handleTool,
}));
vi.mock('./tools/camera-tools.js', () => ({
  getCameraTools: cameraMock.getTools,
  handleCameraTool: cameraMock.handleTool,
}));
vi.mock('./tools/map-tools.js', () => ({
  getMapTools: mapMock.getTools,
  handleMapTool: mapMock.handleTool,
}));
vi.mock('./tools/history-tools.js', () => ({
  getHistoryTools: historyMock.getTools,
  handleHistoryTool: historyMock.handleTool,
}));
vi.mock('./tools/path-tools.js', () => ({
  getPathTools: pathMock.getTools,
  handlePathTool: pathMock.handleTool,
}));
vi.mock('./tools/network-tools.js', () => ({
  getNetworkTools: networkMock.getTools,
  handleNetworkTool: networkMock.handleTool,
}));

// ---------------------------------------------------------------------------
// Suppress console.error output from the module under test
// ---------------------------------------------------------------------------
vi.spyOn(console, 'error').mockImplementation(() => {});

// Prevent process.exit from actually exiting during tests
const exitSpy = vi.spyOn(process, 'exit').mockImplementation((() => {}) as any);

// ---------------------------------------------------------------------------
// Import index.ts AFTER all mocks are set up (side effects execute here)
// ---------------------------------------------------------------------------
let ListToolsRequestSchema: unknown;
let CallToolRequestSchema: unknown;
let listToolsHandler: Function;
let callToolHandler: Function;

beforeAll(async () => {
  // Import the schemas so we can look up the captured handlers
  const types = await import('@modelcontextprotocol/sdk/types.js');
  ListToolsRequestSchema = types.ListToolsRequestSchema;
  CallToolRequestSchema = types.CallToolRequestSchema;

  // Import index.ts - this triggers all side effects
  await import('./index.js');

  // Extract the handlers that were registered via setRequestHandler
  listToolsHandler = globalThis.__test_captured_handlers.get(ListToolsRequestSchema)!;
  callToolHandler = globalThis.__test_captured_handlers.get(CallToolRequestSchema)!;
});

// Reset mocks between tests (but not the captured handlers)
beforeEach(() => {
  topicMock.handleTool.mockClear();
  serviceMock.handleTool.mockClear();
  actionMock.handleTool.mockClear();
  safetyMock.handleTool.mockClear();
  systemMock.handleTool.mockClear();
  batchMock.handleTool.mockClear();
  recordingMock.handleTool.mockClear();
  conditionalMock.handleTool.mockClear();
  scheduledMock.handleTool.mockClear();
  tfMock.handleTool.mockClear();
  diagnosticMock.handleTool.mockClear();
  fleetMock.handleTool.mockClear();
  launchMock.handleTool.mockClear();
  waypointMock.handleTool.mockClear();
  introspectionMock.handleTool.mockClear();
  namespaceMock.handleTool.mockClear();
  sensorMock.handleTool.mockClear();
  paramMock.handleTool.mockClear();
  descriptionMock.handleTool.mockClear();
  powerMock.handleTool.mockClear();
  cameraMock.handleTool.mockClear();
  mapMock.handleTool.mockClear();
  historyMock.handleTool.mockClear();
  pathMock.handleTool.mockClear();
  networkMock.handleTool.mockClear();

  // Default: bridge is disconnected and connect fails
  globalThis.__test_mock_connection.isConnected = false;
  globalThis.__test_mock_connection.connect.mockRejectedValue(new Error('Bridge not available'));
});

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

describe('index.ts integration', () => {
  // =========================================================================
  // 1. Tool Registration
  // =========================================================================
  describe('Tool registration', () => {
    it('registers a ListToolsRequest handler', () => {
      expect(listToolsHandler).toBeDefined();
      expect(typeof listToolsHandler).toBe('function');
    });

    it('registers a CallToolRequest handler', () => {
      expect(callToolHandler).toBeDefined();
      expect(typeof callToolHandler).toBe('function');
    });

    it('ListTools returns all tools from all 25 modules', async () => {
      const result = await listToolsHandler();
      expect(result.tools).toBeDefined();
      expect(Array.isArray(result.tools)).toBe(true);
      // Each mock module returns 1 or 2 tools; total should be sum of all mock tool arrays
      const expectedCount =
        topicMock.getTools().length +
        serviceMock.getTools().length +
        actionMock.getTools().length +
        safetyMock.getTools().length +
        systemMock.getTools().length +
        batchMock.getTools().length +
        recordingMock.getTools().length +
        conditionalMock.getTools().length +
        scheduledMock.getTools().length +
        tfMock.getTools().length +
        diagnosticMock.getTools().length +
        fleetMock.getTools().length +
        launchMock.getTools().length +
        waypointMock.getTools().length +
        introspectionMock.getTools().length +
        namespaceMock.getTools().length +
        sensorMock.getTools().length +
        paramMock.getTools().length +
        descriptionMock.getTools().length +
        powerMock.getTools().length +
        cameraMock.getTools().length +
        mapMock.getTools().length +
        historyMock.getTools().length +
        pathMock.getTools().length +
        networkMock.getTools().length;
      expect(result.tools.length).toBe(expectedCount);
    });

    it('all 25 getter functions are called during module import', () => {
      // getTools is called twice per module: once for allTools array, once for the name Set
      expect(topicMock.getTools).toHaveBeenCalled();
      expect(serviceMock.getTools).toHaveBeenCalled();
      expect(actionMock.getTools).toHaveBeenCalled();
      expect(safetyMock.getTools).toHaveBeenCalled();
      expect(systemMock.getTools).toHaveBeenCalled();
      expect(batchMock.getTools).toHaveBeenCalled();
      expect(recordingMock.getTools).toHaveBeenCalled();
      expect(conditionalMock.getTools).toHaveBeenCalled();
      expect(scheduledMock.getTools).toHaveBeenCalled();
      expect(tfMock.getTools).toHaveBeenCalled();
      expect(diagnosticMock.getTools).toHaveBeenCalled();
      expect(fleetMock.getTools).toHaveBeenCalled();
      expect(launchMock.getTools).toHaveBeenCalled();
      expect(waypointMock.getTools).toHaveBeenCalled();
      expect(introspectionMock.getTools).toHaveBeenCalled();
      expect(namespaceMock.getTools).toHaveBeenCalled();
      expect(sensorMock.getTools).toHaveBeenCalled();
      expect(paramMock.getTools).toHaveBeenCalled();
      expect(descriptionMock.getTools).toHaveBeenCalled();
      expect(powerMock.getTools).toHaveBeenCalled();
      expect(cameraMock.getTools).toHaveBeenCalled();
      expect(mapMock.getTools).toHaveBeenCalled();
      expect(historyMock.getTools).toHaveBeenCalled();
      expect(pathMock.getTools).toHaveBeenCalled();
      expect(networkMock.getTools).toHaveBeenCalled();
    });

    it('each tool in ListTools has required name and description fields', async () => {
      const result = await listToolsHandler();
      for (const tool of result.tools) {
        expect(typeof tool.name).toBe('string');
        expect(tool.name.length).toBeGreaterThan(0);
        expect(typeof tool.description).toBe('string');
      }
    });
  });

  // =========================================================================
  // 2. Tool Dispatch Routing
  // =========================================================================
  describe('Tool dispatch routing', () => {
    // Helper to make a CallTool request
    function makeRequest(name: string, args: Record<string, unknown> = {}) {
      return { params: { name, arguments: args } };
    }

    it('dispatches topic tool to handleTopicTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_topic_list'));
      expect(topicMock.handleTool).toHaveBeenCalledTimes(1);
      expect(topicMock.handleTool).toHaveBeenCalledWith(
        'ros2_topic_list',
        {},
        expect.anything(), // connection
        expect.anything(), // safety
      );
    });

    it('dispatches service tool to handleServiceTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_service_call', { service: '/test' }));
      expect(serviceMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches action tool to handleActionTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_action_send_goal'));
      expect(actionMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches safety tool to handleSafetyTool', async () => {
      await callToolHandler(makeRequest('safety_status'));
      expect(safetyMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches system tool to handleSystemTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('system_bridge_status'));
      expect(systemMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches batch tool to handleBatchTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_batch_execute'));
      expect(batchMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches recording tool to handleRecordingTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_topic_record_start'));
      expect(recordingMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches conditional tool to handleConditionalTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_conditional_execute'));
      expect(conditionalMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches scheduled tool to handleScheduledTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_schedule_command'));
      expect(scheduledMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches tf tool to handleTfTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_tf_lookup'));
      expect(tfMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches diagnostic tool to handleDiagnosticTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_diagnostic_summary'));
      expect(diagnosticMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches fleet tool to handleFleetTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_fleet_list'));
      expect(fleetMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches launch tool to handleLaunchTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_launch_start'));
      expect(launchMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches waypoint tool to handleWaypointTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_waypoint_follow'));
      expect(waypointMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches introspection tool to handleIntrospectionTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_msg_type_info'));
      expect(introspectionMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches namespace tool to handleNamespaceTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_namespace_list'));
      expect(namespaceMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches sensor tool to handleSensorTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_sensor_summary'));
      expect(sensorMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches param tool to handleParamTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_param_list'));
      expect(paramMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches description tool to handleDescriptionTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_robot_description'));
      expect(descriptionMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches power tool to handlePowerTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_battery_status'));
      expect(powerMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches camera tool to handleCameraTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_camera_info'));
      expect(cameraMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches map tool to handleMapTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_map_info'));
      expect(mapMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches history tool to handleHistoryTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_command_history'));
      expect(historyMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches path tool to handlePathTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_plan_path'));
      expect(pathMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('dispatches network tool to handleNetworkTool', async () => {
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_network_stats'));
      expect(networkMock.handleTool).toHaveBeenCalledTimes(1);
    });
  });

  // =========================================================================
  // 3. Bridge connection failure for non-safety tools
  // =========================================================================
  describe('Bridge connection failure', () => {
    function makeRequest(name: string, args: Record<string, unknown> = {}) {
      return { params: { name, arguments: args } };
    }

    it('returns helpful error when bridge is down for a topic tool', async () => {
      // connection.isConnected = false (default), connect rejects
      const result = await callToolHandler(makeRequest('ros2_topic_list'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Bridge not connected');
      expect(result.content[0].text).toContain('Troubleshooting');
    });

    it('includes bridge URL in error message', async () => {
      const result = await callToolHandler(makeRequest('ros2_service_call'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Expected bridge at');
    });

    it('returns bridge error for system tools when bridge is down', async () => {
      const result = await callToolHandler(makeRequest('system_bridge_status'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toContain('Bridge not connected');
    });

    it('does not call the handler when bridge connection fails', async () => {
      await callToolHandler(makeRequest('ros2_topic_list'));
      expect(topicMock.handleTool).not.toHaveBeenCalled();
    });
  });

  // =========================================================================
  // 4. Safety tools work without bridge
  // =========================================================================
  describe('Safety tools work without bridge', () => {
    function makeRequest(name: string, args: Record<string, unknown> = {}) {
      return { params: { name, arguments: args } };
    }

    it('safety_status works even when bridge is disconnected', async () => {
      // connection.isConnected = false (default), connect will NOT be attempted
      const result = await callToolHandler(makeRequest('safety_status'));
      expect(result.content[0].text).toContain('safety:safety_status:ok');
      expect(result.isError).toBeUndefined();
      expect(safetyMock.handleTool).toHaveBeenCalledTimes(1);
    });

    it('safety_emergency_stop works without bridge', async () => {
      const result = await callToolHandler(makeRequest('safety_emergency_stop'));
      expect(result.content[0].text).toContain('safety:safety_emergency_stop:ok');
      expect(result.isError).toBeUndefined();
    });

    it('does not attempt bridge connection for safety tools', async () => {
      globalThis.__test_mock_connection.connect.mockClear();
      await callToolHandler(makeRequest('safety_status'));
      expect(globalThis.__test_mock_connection.connect).not.toHaveBeenCalled();
    });
  });

  // =========================================================================
  // 5. Unknown tool name
  // =========================================================================
  describe('Unknown tool name', () => {
    function makeRequest(name: string, args: Record<string, unknown> = {}) {
      return { params: { name, arguments: args } };
    }

    it('returns "Unknown tool: xxx" with isError for completely unknown tool', async () => {
      globalThis.__test_mock_connection.isConnected = true; // skip bridge connect
      const result = await callToolHandler(makeRequest('nonexistent_tool'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toBe('Unknown tool: nonexistent_tool');
    });

    it('returns unknown tool for empty string name', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      const result = await callToolHandler(makeRequest(''));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toBe('Unknown tool: ');
    });

    it('returns unknown tool for similar but wrong name', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      const result = await callToolHandler(makeRequest('ros2_topic_delete'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toBe('Unknown tool: ros2_topic_delete');
    });
  });

  // =========================================================================
  // 6. Error handling
  // =========================================================================
  describe('Error handling', () => {
    function makeRequest(name: string, args: Record<string, unknown> = {}) {
      return { params: { name, arguments: args } };
    }

    it('returns isError with message when handler throws an Error', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      topicMock.handleTool.mockRejectedValueOnce(new Error('Connection timeout'));
      const result = await callToolHandler(makeRequest('ros2_topic_list'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toBe('Error: Connection timeout');
    });

    it('returns isError with stringified value when handler throws a non-Error', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      topicMock.handleTool.mockRejectedValueOnce('string error');
      const result = await callToolHandler(makeRequest('ros2_topic_list'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toBe('Error: string error');
    });

    it('returns isError when handler throws undefined', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      topicMock.handleTool.mockRejectedValueOnce(undefined);
      const result = await callToolHandler(makeRequest('ros2_topic_list'));
      expect(result.isError).toBe(true);
      expect(result.content[0].text).toBe('Error: undefined');
    });

    it('handles null arguments gracefully', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      const result = await callToolHandler({ params: { name: 'ros2_topic_list', arguments: null } });
      // Should not throw - args default to {}
      // The handler should be called
      expect(topicMock.handleTool).toHaveBeenCalledWith(
        'ros2_topic_list',
        {},
        expect.anything(),
        expect.anything(),
      );
    });

    it('handles missing arguments gracefully', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      const result = await callToolHandler({ params: { name: 'ros2_topic_list' } });
      expect(topicMock.handleTool).toHaveBeenCalledWith(
        'ros2_topic_list',
        {},
        expect.anything(),
        expect.anything(),
      );
    });
  });

  // =========================================================================
  // 7. Bridge auto-connect for non-safety tools
  // =========================================================================
  describe('Bridge auto-connect behavior', () => {
    function makeRequest(name: string, args: Record<string, unknown> = {}) {
      return { params: { name, arguments: args } };
    }

    it('attempts to connect when bridge is disconnected for non-safety tool', async () => {
      globalThis.__test_mock_connection.isConnected = false;
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      await callToolHandler(makeRequest('ros2_topic_list'));
      expect(globalThis.__test_mock_connection.connect).toHaveBeenCalledTimes(1);
    });

    it('skips connect when bridge is already connected', async () => {
      globalThis.__test_mock_connection.isConnected = true;
      globalThis.__test_mock_connection.connect.mockClear();
      await callToolHandler(makeRequest('ros2_topic_list'));
      expect(globalThis.__test_mock_connection.connect).not.toHaveBeenCalled();
    });

    it('dispatches to handler after successful auto-connect', async () => {
      globalThis.__test_mock_connection.isConnected = false;
      globalThis.__test_mock_connection.connect.mockResolvedValueOnce(undefined);
      const result = await callToolHandler(makeRequest('ros2_topic_list'));
      expect(topicMock.handleTool).toHaveBeenCalledTimes(1);
      expect(result.content[0].text).toContain('topic:ros2_topic_list:ok');
    });
  });

  // =========================================================================
  // 8. Server setup
  // =========================================================================
  describe('Server setup', () => {
    it('Server constructor was called with correct metadata', async () => {
      const { Server } = await import('@modelcontextprotocol/sdk/server/index.js');
      expect(Server).toHaveBeenCalledWith(
        expect.objectContaining({
          name: '@ricardothe3rd/physical-mcp',
          version: '0.1.0',
        }),
        expect.objectContaining({
          capabilities: { tools: {} },
        }),
      );
    });

    it('server.connect was called with transport', () => {
      expect(globalThis.__test_mock_server.connect).toHaveBeenCalled();
    });

    it('setRequestHandler was called exactly 2 times (ListTools + CallTool)', () => {
      expect(globalThis.__test_mock_server.setRequestHandler).toHaveBeenCalledTimes(2);
    });
  });
});

// ---------------------------------------------------------------------------
// parseArgs tests (indirect)
//
// parseArgs runs at module load time, so we verify its output indirectly
// through the arguments passed to ConnectionManager, PolicyEngine, and
// runStartupChecks. process.argv has no custom flags during tests, so
// we expect the default config.
// ---------------------------------------------------------------------------
describe('parseArgs (indirect via constructor args)', () => {
  it('ConnectionManager is constructed with default bridge URL', async () => {
    const { ConnectionManager } = await import('./bridge/connection-manager.js');
    expect(ConnectionManager).toHaveBeenCalledWith('ws://localhost:9090');
  });

  it('PolicyEngine is constructed with undefined policyPath (default)', async () => {
    const { PolicyEngine } = await import('./safety/policy-engine.js');
    expect(PolicyEngine).toHaveBeenCalledWith(undefined);
  });

  it('runStartupChecks receives the parsed config with defaults', async () => {
    const { runStartupChecks } = await import('./utils/startup-check.js');
    expect(runStartupChecks).toHaveBeenCalledWith(
      expect.objectContaining({
        bridgeUrl: 'ws://localhost:9090',
        verbose: false,
      }),
    );
  });

  it('printStartupChecks is called with the startup result', async () => {
    const { printStartupChecks } = await import('./utils/startup-check.js');
    expect(printStartupChecks).toHaveBeenCalledWith(
      expect.objectContaining({ passed: true }),
    );
  });

  it('default policyPath is undefined when no env var or arg is set', async () => {
    const { runStartupChecks } = await import('./utils/startup-check.js');
    const callArgs = (runStartupChecks as ReturnType<typeof vi.fn>).mock.calls[0][0];
    expect(callArgs.policyPath).toBeUndefined();
  });

  it('PHYSICAL_MCP_BRIDGE_URL env var is not set during tests', () => {
    expect(process.env.PHYSICAL_MCP_BRIDGE_URL).toBeUndefined();
  });

  it('PHYSICAL_MCP_POLICY env var is not set during tests', () => {
    expect(process.env.PHYSICAL_MCP_POLICY).toBeUndefined();
  });
});
