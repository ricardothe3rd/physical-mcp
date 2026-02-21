import { describe, it, expect, vi, beforeEach } from 'vitest';
import { TopicRecorder } from './topic-recorder.js';
import type { RecordedMessage, RecordingStats } from './topic-recorder.js';

describe('TopicRecorder', () => {
  let recorder: TopicRecorder;

  beforeEach(() => {
    recorder = new TopicRecorder();
  });

  // --- start() ---

  describe('start()', () => {
    it('sets status to recording and returns an ID', () => {
      const id = recorder.start();
      expect(recorder.getStatus()).toBe('recording');
      expect(typeof id).toBe('string');
      expect(id.length).toBeGreaterThan(0);
    });

    it('returns an ID that starts with rec_', () => {
      const id = recorder.start();
      expect(id.startsWith('rec_')).toBe(true);
    });

    it('throws if already recording', () => {
      recorder.start();
      expect(() => recorder.start()).toThrow('Already recording');
    });

    it('can start again after stop', () => {
      recorder.start();
      recorder.stop();
      const id = recorder.start();
      expect(recorder.getStatus()).toBe('recording');
      expect(id.startsWith('rec_')).toBe(true);
    });
  });

  // --- stop() ---

  describe('stop()', () => {
    it('sets status to stopped', () => {
      recorder.start();
      recorder.stop();
      expect(recorder.getStatus()).toBe('stopped');
    });

    it('throws if not recording', () => {
      expect(() => recorder.stop()).toThrow('Not recording');
    });

    it('throws if already stopped', () => {
      recorder.start();
      recorder.stop();
      expect(() => recorder.stop()).toThrow('Not recording');
    });
  });

  // --- recordMessage() ---

  describe('recordMessage()', () => {
    it('adds a message with correct fields', () => {
      recorder.start();
      recorder.recordMessage('/cmd_vel', { linear: { x: 1 } });

      const messages = recorder.getMessages();
      expect(messages).toHaveLength(1);
      expect(messages[0].topic).toBe('/cmd_vel');
      expect(messages[0].data).toEqual({ linear: { x: 1 } });
      expect(messages[0].sequenceNumber).toBe(0);
      expect(typeof messages[0].timestamp).toBe('number');
    });

    it('ignores messages when not recording (idle)', () => {
      recorder.recordMessage('/cmd_vel', { x: 1 });
      expect(recorder.getMessages()).toHaveLength(0);
    });

    it('ignores messages when stopped', () => {
      recorder.start();
      recorder.stop();
      recorder.recordMessage('/cmd_vel', { x: 1 });
      expect(recorder.getMessages()).toHaveLength(0);
    });

    it('stops recording when maxMessages reached', () => {
      const small = new TopicRecorder({ maxMessages: 3 });
      small.start();
      small.recordMessage('/a', 1);
      small.recordMessage('/a', 2);
      small.recordMessage('/a', 3);
      // The 4th message triggers the stop
      small.recordMessage('/a', 4);
      expect(small.getStatus()).toBe('stopped');
      expect(small.getMessages()).toHaveLength(3);
    });

    it('respects topic filter (only records matching topics)', () => {
      const filtered = new TopicRecorder({ topics: ['/cmd_vel', '/odom'] });
      filtered.start();
      filtered.recordMessage('/cmd_vel', 1);
      filtered.recordMessage('/scan', 2);
      filtered.recordMessage('/odom', 3);
      filtered.recordMessage('/tf', 4);

      const messages = filtered.getMessages();
      expect(messages).toHaveLength(2);
      expect(messages[0].topic).toBe('/cmd_vel');
      expect(messages[1].topic).toBe('/odom');
    });

    it('records all topics when filter is empty', () => {
      const noFilter = new TopicRecorder({ topics: [] });
      noFilter.start();
      noFilter.recordMessage('/cmd_vel', 1);
      noFilter.recordMessage('/scan', 2);
      noFilter.recordMessage('/tf', 3);

      expect(noFilter.getMessages()).toHaveLength(3);
    });

    it('records all topics when no topics config provided', () => {
      recorder.start();
      recorder.recordMessage('/cmd_vel', 1);
      recorder.recordMessage('/scan', 2);
      expect(recorder.getMessages()).toHaveLength(2);
    });

    it('increments sequenceNumber correctly', () => {
      recorder.start();
      recorder.recordMessage('/a', 1);
      recorder.recordMessage('/b', 2);
      recorder.recordMessage('/c', 3);

      const messages = recorder.getMessages();
      expect(messages[0].sequenceNumber).toBe(0);
      expect(messages[1].sequenceNumber).toBe(1);
      expect(messages[2].sequenceNumber).toBe(2);
    });

    it('stops recording when maxDurationMs elapsed', () => {
      const short = new TopicRecorder({ maxDurationMs: 100 });
      short.start();

      // Simulate time passing by mocking Date.now
      const originalNow = Date.now;
      const startTime = originalNow();
      vi.spyOn(Date, 'now').mockReturnValue(startTime + 200);

      short.recordMessage('/a', 1);
      expect(short.getStatus()).toBe('stopped');
      expect(short.getMessages()).toHaveLength(0);

      vi.restoreAllMocks();
    });
  });

  // --- getMessages() ---

  describe('getMessages()', () => {
    it('returns all recorded messages in order', () => {
      recorder.start();
      recorder.recordMessage('/a', 'first');
      recorder.recordMessage('/b', 'second');
      recorder.recordMessage('/c', 'third');

      const messages = recorder.getMessages();
      expect(messages).toHaveLength(3);
      expect(messages[0].data).toBe('first');
      expect(messages[1].data).toBe('second');
      expect(messages[2].data).toBe('third');
    });

    it('returns a copy (not the internal array)', () => {
      recorder.start();
      recorder.recordMessage('/a', 1);
      const messages = recorder.getMessages();
      messages.push({ topic: '/fake', timestamp: 0, sequenceNumber: 99, data: null });
      expect(recorder.getMessages()).toHaveLength(1);
    });
  });

  // --- getMessagesByTopic() ---

  describe('getMessagesByTopic()', () => {
    it('filters messages correctly by topic', () => {
      recorder.start();
      recorder.recordMessage('/cmd_vel', 'vel1');
      recorder.recordMessage('/odom', 'odom1');
      recorder.recordMessage('/cmd_vel', 'vel2');
      recorder.recordMessage('/scan', 'scan1');

      const velMessages = recorder.getMessagesByTopic('/cmd_vel');
      expect(velMessages).toHaveLength(2);
      expect(velMessages[0].data).toBe('vel1');
      expect(velMessages[1].data).toBe('vel2');
    });

    it('returns empty array for unknown topic', () => {
      recorder.start();
      recorder.recordMessage('/cmd_vel', 1);
      expect(recorder.getMessagesByTopic('/nonexistent')).toHaveLength(0);
    });
  });

  // --- exportJson() ---

  describe('exportJson()', () => {
    it('returns valid JSON array', () => {
      recorder.start();
      recorder.recordMessage('/a', { x: 1 });
      recorder.recordMessage('/b', { y: 2 });

      const json = recorder.exportJson();
      const parsed = JSON.parse(json) as RecordedMessage[];
      expect(Array.isArray(parsed)).toBe(true);
      expect(parsed).toHaveLength(2);
      expect(parsed[0].topic).toBe('/a');
      expect(parsed[1].topic).toBe('/b');
    });

    it('returns empty array JSON when no messages', () => {
      const json = recorder.exportJson();
      expect(json).toBe('[]');
    });
  });

  // --- exportJsonLines() ---

  describe('exportJsonLines()', () => {
    it('returns one JSON object per line', () => {
      recorder.start();
      recorder.recordMessage('/a', 1);
      recorder.recordMessage('/b', 2);
      recorder.recordMessage('/c', 3);

      const lines = recorder.exportJsonLines().split('\n');
      expect(lines).toHaveLength(3);

      for (const line of lines) {
        const parsed = JSON.parse(line) as RecordedMessage;
        expect(parsed).toHaveProperty('topic');
        expect(parsed).toHaveProperty('timestamp');
        expect(parsed).toHaveProperty('sequenceNumber');
        expect(parsed).toHaveProperty('data');
      }
    });

    it('returns empty string when no messages', () => {
      expect(recorder.exportJsonLines()).toBe('');
    });
  });

  // --- getStats() ---

  describe('getStats()', () => {
    it('returns correct values when idle', () => {
      const stats = recorder.getStats();
      expect(stats.id).toBe('');
      expect(stats.status).toBe('idle');
      expect(stats.startedAt).toBeNull();
      expect(stats.stoppedAt).toBeNull();
      expect(stats.messageCount).toBe(0);
      expect(stats.topics).toEqual([]);
      expect(stats.durationMs).toBe(0);
      expect(stats.bytesEstimate).toBe(0);
    });

    it('returns correct values when recording', () => {
      const id = recorder.start();
      recorder.recordMessage('/cmd_vel', { x: 1 });
      recorder.recordMessage('/odom', { y: 2 });

      const stats = recorder.getStats();
      expect(stats.id).toBe(id);
      expect(stats.status).toBe('recording');
      expect(stats.startedAt).not.toBeNull();
      expect(stats.stoppedAt).toBeNull();
      expect(stats.messageCount).toBe(2);
      expect(stats.topics).toContain('/cmd_vel');
      expect(stats.topics).toContain('/odom');
      expect(stats.durationMs).toBeGreaterThanOrEqual(0);
      expect(stats.bytesEstimate).toBeGreaterThan(0);
    });

    it('returns correct values when stopped', () => {
      recorder.start();
      recorder.recordMessage('/a', 1);
      recorder.stop();

      const stats = recorder.getStats();
      expect(stats.status).toBe('stopped');
      expect(stats.stoppedAt).not.toBeNull();
      expect(stats.messageCount).toBe(1);
    });
  });

  // --- getRecordedTopics() ---

  describe('getRecordedTopics()', () => {
    it('returns unique topic names', () => {
      recorder.start();
      recorder.recordMessage('/cmd_vel', 1);
      recorder.recordMessage('/odom', 2);
      recorder.recordMessage('/cmd_vel', 3);
      recorder.recordMessage('/scan', 4);
      recorder.recordMessage('/odom', 5);

      const topics = recorder.getRecordedTopics();
      expect(topics).toHaveLength(3);
      expect(topics).toContain('/cmd_vel');
      expect(topics).toContain('/odom');
      expect(topics).toContain('/scan');
    });

    it('returns empty array when no messages', () => {
      expect(recorder.getRecordedTopics()).toEqual([]);
    });
  });

  // --- isTopicIncluded() ---

  describe('isTopicIncluded()', () => {
    it('returns true for all topics when filter is empty', () => {
      expect(recorder.isTopicIncluded('/anything')).toBe(true);
      expect(recorder.isTopicIncluded('/cmd_vel')).toBe(true);
    });

    it('returns true for included topics', () => {
      const filtered = new TopicRecorder({ topics: ['/cmd_vel', '/odom'] });
      expect(filtered.isTopicIncluded('/cmd_vel')).toBe(true);
      expect(filtered.isTopicIncluded('/odom')).toBe(true);
    });

    it('returns false for excluded topics', () => {
      const filtered = new TopicRecorder({ topics: ['/cmd_vel', '/odom'] });
      expect(filtered.isTopicIncluded('/scan')).toBe(false);
      expect(filtered.isTopicIncluded('/tf')).toBe(false);
    });
  });

  // --- reset() ---

  describe('reset()', () => {
    it('clears everything back to idle', () => {
      recorder.start();
      recorder.recordMessage('/a', 1);
      recorder.recordMessage('/b', 2);
      recorder.stop();

      recorder.reset();

      expect(recorder.getStatus()).toBe('idle');
      expect(recorder.getMessages()).toHaveLength(0);
      expect(recorder.getRecordedTopics()).toEqual([]);
      expect(recorder.exportJson()).toBe('[]');

      const stats = recorder.getStats();
      expect(stats.id).toBe('');
      expect(stats.status).toBe('idle');
      expect(stats.startedAt).toBeNull();
      expect(stats.stoppedAt).toBeNull();
      expect(stats.messageCount).toBe(0);
      expect(stats.bytesEstimate).toBe(0);
      expect(stats.durationMs).toBe(0);
    });

    it('allows starting a new recording after reset', () => {
      recorder.start();
      recorder.stop();
      recorder.reset();

      const id = recorder.start();
      expect(recorder.getStatus()).toBe('recording');
      expect(id.startsWith('rec_')).toBe(true);
    });
  });

  // --- bytesEstimate ---

  describe('bytesEstimate', () => {
    it('provides a reasonable estimate', () => {
      recorder.start();
      const data = { linear: { x: 1.0, y: 0.0, z: 0.0 } };
      recorder.recordMessage('/cmd_vel', data);

      const stats = recorder.getStats();
      // The estimate should be at least the size of the data when serialized
      expect(stats.bytesEstimate).toBeGreaterThan(0);
      // Rough sanity check: a single message with that data should be < 500 bytes
      expect(stats.bytesEstimate).toBeLessThan(500);
    });

    it('grows with more messages', () => {
      recorder.start();
      recorder.recordMessage('/a', { value: 1 });
      const after1 = recorder.getStats().bytesEstimate;

      recorder.recordMessage('/b', { value: 2 });
      const after2 = recorder.getStats().bytesEstimate;

      expect(after2).toBeGreaterThan(after1);
    });
  });

  // --- recording ID ---

  describe('recording ID', () => {
    it('starts with rec_', () => {
      const id = recorder.start();
      expect(id).toMatch(/^rec_\d+$/);
    });

    it('generates different IDs for different sessions', async () => {
      const id1 = recorder.start();
      recorder.stop();
      // Small delay to ensure different timestamp
      await new Promise((resolve) => setTimeout(resolve, 5));
      const id2 = recorder.start();
      expect(id1).not.toBe(id2);
    });
  });

  // --- default config ---

  describe('default config', () => {
    it('uses default maxMessages of 1000', () => {
      const rec = new TopicRecorder();
      rec.start();
      for (let i = 0; i < 1001; i++) {
        rec.recordMessage('/a', i);
      }
      expect(rec.getMessages()).toHaveLength(1000);
      expect(rec.getStatus()).toBe('stopped');
    });
  });
});
