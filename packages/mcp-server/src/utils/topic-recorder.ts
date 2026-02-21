/**
 * Topic recording utility for PhysicalMCP.
 *
 * Subscribes to ROS2 topics and records messages to an in-memory buffer
 * that can be exported for debugging and replay.
 */

export interface RecordedMessage {
  topic: string;
  timestamp: number;
  sequenceNumber: number;
  data: unknown;
}

export interface RecordingConfig {
  maxMessages?: number;     // Default 1000
  maxDurationMs?: number;   // Default 60000 (60 seconds)
  topics?: string[];        // Topics to record (empty = all)
}

export interface RecordingStats {
  id: string;
  status: 'idle' | 'recording' | 'stopped';
  startedAt: number | null;
  stoppedAt: number | null;
  messageCount: number;
  topics: string[];
  durationMs: number;
  bytesEstimate: number;
}

const DEFAULT_MAX_MESSAGES = 1000;
const DEFAULT_MAX_DURATION_MS = 60_000;

export class TopicRecorder {
  private readonly maxMessages: number;
  private readonly maxDurationMs: number;
  private readonly topicFilter: string[];

  private status: 'idle' | 'recording' | 'stopped' = 'idle';
  private recordingId = '';
  private startedAt: number | null = null;
  private stoppedAt: number | null = null;
  private messages: RecordedMessage[] = [];
  private sequenceCounter = 0;
  private bytesEstimate = 0;

  constructor(config?: RecordingConfig) {
    this.maxMessages = config?.maxMessages ?? DEFAULT_MAX_MESSAGES;
    this.maxDurationMs = config?.maxDurationMs ?? DEFAULT_MAX_DURATION_MS;
    this.topicFilter = config?.topics ?? [];
  }

  /** Start a new recording session. Returns the recording ID. */
  start(): string {
    if (this.status === 'recording') {
      throw new Error('Already recording');
    }

    this.messages = [];
    this.sequenceCounter = 0;
    this.bytesEstimate = 0;
    this.stoppedAt = null;

    this.recordingId = `rec_${Date.now()}`;
    this.startedAt = Date.now();
    this.status = 'recording';

    return this.recordingId;
  }

  /** Stop the current recording session. */
  stop(): void {
    if (this.status !== 'recording') {
      throw new Error('Not recording');
    }
    this.stoppedAt = Date.now();
    this.status = 'stopped';
  }

  /** Record a message. Called externally when a message arrives on a topic. */
  recordMessage(topic: string, data: unknown): void {
    if (this.status !== 'recording') {
      return;
    }

    // Check topic filter
    if (!this.isTopicIncluded(topic)) {
      return;
    }

    // Check max messages
    if (this.messages.length >= this.maxMessages) {
      this.stop();
      return;
    }

    // Check max duration
    if (this.startedAt !== null && Date.now() - this.startedAt >= this.maxDurationMs) {
      this.stop();
      return;
    }

    const message: RecordedMessage = {
      topic,
      timestamp: Date.now(),
      sequenceNumber: this.sequenceCounter++,
      data,
    };

    const serialized = JSON.stringify(message);
    this.bytesEstimate += serialized.length;

    this.messages.push(message);
  }

  /** Get current recording status. */
  getStatus(): 'idle' | 'recording' | 'stopped' {
    return this.status;
  }

  /** Get recording statistics. */
  getStats(): RecordingStats {
    let durationMs = 0;
    if (this.startedAt !== null) {
      const endTime = this.stoppedAt ?? Date.now();
      durationMs = endTime - this.startedAt;
    }

    return {
      id: this.recordingId,
      status: this.status,
      startedAt: this.startedAt,
      stoppedAt: this.stoppedAt,
      messageCount: this.messages.length,
      topics: this.getRecordedTopics(),
      durationMs,
      bytesEstimate: this.bytesEstimate,
    };
  }

  /** Get all recorded messages in order. */
  getMessages(): RecordedMessage[] {
    return [...this.messages];
  }

  /** Get recorded messages for a specific topic. */
  getMessagesByTopic(topic: string): RecordedMessage[] {
    return this.messages.filter((m) => m.topic === topic);
  }

  /** Export all recorded messages as a JSON string (array). */
  exportJson(): string {
    return JSON.stringify(this.messages);
  }

  /** Export all recorded messages as JSON Lines (one JSON object per line). */
  exportJsonLines(): string {
    return this.messages.map((m) => JSON.stringify(m)).join('\n');
  }

  /** Clear all recorded data and reset to idle state. */
  reset(): void {
    this.status = 'idle';
    this.recordingId = '';
    this.startedAt = null;
    this.stoppedAt = null;
    this.messages = [];
    this.sequenceCounter = 0;
    this.bytesEstimate = 0;
  }

  /** Check whether a topic is included by the recording filter. */
  isTopicIncluded(topic: string): boolean {
    if (this.topicFilter.length === 0) {
      return true;
    }
    return this.topicFilter.includes(topic);
  }

  /** Get the unique set of topics that have been recorded. */
  getRecordedTopics(): string[] {
    return [...new Set(this.messages.map((m) => m.topic))];
  }
}
