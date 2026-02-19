import { describe, it, expect, beforeEach } from 'vitest';
import {
  AuditEntry,
  AuditExporter,
  AuditRotator,
  ExportOptions,
} from './audit-export.js';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function makeEntry(overrides: Partial<AuditEntry> = {}): AuditEntry {
  return {
    timestamp: 1000,
    type: 'publish',
    action: 'send',
    target: '/cmd_vel',
    result: 'allowed',
    ...overrides,
  };
}

function makeSampleEntries(): AuditEntry[] {
  return [
    makeEntry({ timestamp: 1000, type: 'publish', result: 'allowed', robotId: 'r1' }),
    makeEntry({ timestamp: 2000, type: 'service', result: 'blocked', robotId: 'r2' }),
    makeEntry({ timestamp: 3000, type: 'action', result: 'error', robotId: 'r1' }),
    makeEntry({ timestamp: 4000, type: 'safety', result: 'allowed', robotId: 'r2' }),
    makeEntry({ timestamp: 5000, type: 'system', result: 'blocked', robotId: 'r1' }),
  ];
}

// ---------------------------------------------------------------------------
// AuditExporter
// ---------------------------------------------------------------------------

describe('AuditExporter', () => {
  let entries: AuditEntry[];

  beforeEach(() => {
    entries = makeSampleEntries();
  });

  // ---- export() ----------------------------------------------------------

  describe('export()', () => {
    it('with format "json" returns valid JSON', () => {
      const result = AuditExporter.export(entries, { format: 'json' });
      const parsed = JSON.parse(result);
      expect(Array.isArray(parsed)).toBe(true);
      expect(parsed).toHaveLength(entries.length);
    });

    it('with format "csv" returns CSV with headers', () => {
      const result = AuditExporter.export(entries, { format: 'csv' });
      const lines = result.split('\n');
      expect(lines[0]).toBe('timestamp,type,action,target,result,details,robotId');
      expect(lines).toHaveLength(entries.length + 1); // header + data rows
    });

    it('with format "csv" and includeHeaders false omits headers', () => {
      const result = AuditExporter.export(entries, { format: 'csv', includeHeaders: false });
      const lines = result.split('\n');
      expect(lines).toHaveLength(entries.length);
      // First line should be data, not the header
      expect(lines[0]).not.toContain('timestamp,type,action');
    });

    it('with format "jsonl" returns one JSON per line', () => {
      const result = AuditExporter.export(entries, { format: 'jsonl' });
      const lines = result.split('\n');
      expect(lines).toHaveLength(entries.length);
      for (const line of lines) {
        expect(() => JSON.parse(line)).not.toThrow();
      }
    });

    it('with filter applies filter before formatting', () => {
      const result = AuditExporter.export(entries, {
        format: 'json',
        filter: { type: 'publish' },
      });
      const parsed = JSON.parse(result);
      expect(parsed).toHaveLength(1);
      expect(parsed[0].type).toBe('publish');
    });

    it('with empty entries returns appropriate empty result for json', () => {
      expect(AuditExporter.export([], { format: 'json' })).toBe('[]');
    });

    it('with empty entries returns appropriate empty result for csv', () => {
      const result = AuditExporter.export([], { format: 'csv' });
      expect(result).toBe('timestamp,type,action,target,result,details,robotId');
    });

    it('with empty entries returns appropriate empty result for csv without headers', () => {
      const result = AuditExporter.export([], { format: 'csv', includeHeaders: false });
      expect(result).toBe('');
    });

    it('with empty entries returns appropriate empty result for jsonl', () => {
      expect(AuditExporter.export([], { format: 'jsonl' })).toBe('');
    });
  });

  // ---- filter() ----------------------------------------------------------

  describe('filter()', () => {
    it('by type returns only matching entries', () => {
      const result = AuditExporter.filter(entries, { type: 'service' });
      expect(result).toHaveLength(1);
      expect(result[0].type).toBe('service');
    });

    it('by result returns only matching entries', () => {
      const result = AuditExporter.filter(entries, { result: 'blocked' });
      expect(result).toHaveLength(2);
      result.forEach((e) => expect(e.result).toBe('blocked'));
    });

    it('by startTime/endTime returns entries in range', () => {
      const result = AuditExporter.filter(entries, { startTime: 2000, endTime: 4000 });
      expect(result).toHaveLength(3);
      result.forEach((e) => {
        expect(e.timestamp).toBeGreaterThanOrEqual(2000);
        expect(e.timestamp).toBeLessThanOrEqual(4000);
      });
    });

    it('by robotId returns only matching entries', () => {
      const result = AuditExporter.filter(entries, { robotId: 'r2' });
      expect(result).toHaveLength(2);
      result.forEach((e) => expect(e.robotId).toBe('r2'));
    });

    it('with multiple criteria applies all (AND logic)', () => {
      const result = AuditExporter.filter(entries, { type: 'publish', result: 'allowed', robotId: 'r1' });
      expect(result).toHaveLength(1);
      expect(result[0].type).toBe('publish');
      expect(result[0].result).toBe('allowed');
      expect(result[0].robotId).toBe('r1');
    });

    it('with no criteria returns all entries', () => {
      const result = AuditExporter.filter(entries, {});
      expect(result).toHaveLength(entries.length);
    });

    it('with undefined filter returns all entries', () => {
      const result = AuditExporter.filter(entries, undefined);
      expect(result).toHaveLength(entries.length);
    });
  });

  // ---- entryToCsv() ------------------------------------------------------

  describe('entryToCsv()', () => {
    it('escapes commas in values', () => {
      const entry = makeEntry({ details: 'value,with,commas' });
      const csv = AuditExporter.entryToCsv(entry);
      expect(csv).toContain('"value,with,commas"');
    });

    it('escapes quotes in values', () => {
      const entry = makeEntry({ details: 'value "with" quotes' });
      const csv = AuditExporter.entryToCsv(entry);
      expect(csv).toContain('"value ""with"" quotes"');
    });

    it('handles undefined details', () => {
      const entry = makeEntry({ details: undefined });
      const csv = AuditExporter.entryToCsv(entry);
      // The details field should appear as an empty string
      const fields = csv.split(',');
      // details is the 6th field (index 5)
      expect(fields[5]).toBe('');
    });

    it('handles undefined robotId', () => {
      const entry = makeEntry({ robotId: undefined });
      const csv = AuditExporter.entryToCsv(entry);
      // The last field should be empty string for undefined robotId
      // CSV rows have 7 fields; split on comma to verify
      const fields = csv.split(',');
      expect(fields).toHaveLength(7);
      expect(fields[6]).toBe('');
    });
  });

  // ---- toCsv() -----------------------------------------------------------

  describe('toCsv()', () => {
    it('includes header row by default', () => {
      const result = AuditExporter.toCsv(entries);
      const lines = result.split('\n');
      expect(lines[0]).toBe('timestamp,type,action,target,result,details,robotId');
    });

    it('omits header row when includeHeaders is false', () => {
      const result = AuditExporter.toCsv(entries, false);
      const lines = result.split('\n');
      expect(lines).toHaveLength(entries.length);
      expect(lines[0]).not.toContain('timestamp,type,action');
    });
  });

  // ---- toJson() ----------------------------------------------------------

  describe('toJson()', () => {
    it('produces parseable JSON array', () => {
      const result = AuditExporter.toJson(entries);
      const parsed = JSON.parse(result);
      expect(Array.isArray(parsed)).toBe(true);
      expect(parsed).toHaveLength(entries.length);
      expect(parsed[0].timestamp).toBe(entries[0].timestamp);
    });

    it('is pretty-printed', () => {
      const result = AuditExporter.toJson(entries);
      expect(result).toContain('\n');
      expect(result).toContain('  ');
    });
  });

  // ---- toJsonLines() -----------------------------------------------------

  describe('toJsonLines()', () => {
    it('produces one valid JSON per line', () => {
      const result = AuditExporter.toJsonLines(entries);
      const lines = result.split('\n');
      expect(lines).toHaveLength(entries.length);
      for (let i = 0; i < lines.length; i++) {
        const parsed = JSON.parse(lines[i]);
        expect(parsed.timestamp).toBe(entries[i].timestamp);
      }
    });

    it('each line is compact (no extra whitespace)', () => {
      const result = AuditExporter.toJsonLines(entries);
      const lines = result.split('\n');
      for (const line of lines) {
        // compact JSON should not start with whitespace or have indentation
        expect(line).toBe(line.trim());
        expect(line).not.toContain('\n');
      }
    });
  });
});

// ---------------------------------------------------------------------------
// AuditRotator
// ---------------------------------------------------------------------------

describe('AuditRotator', () => {
  // ---- shouldRotate() ----------------------------------------------------

  describe('shouldRotate()', () => {
    it('returns false when under limit', () => {
      const rotator = new AuditRotator({ maxEntries: 100 });
      expect(rotator.shouldRotate(50)).toBe(false);
    });

    it('returns true when at limit', () => {
      const rotator = new AuditRotator({ maxEntries: 100 });
      expect(rotator.shouldRotate(100)).toBe(true);
    });

    it('returns true when over limit', () => {
      const rotator = new AuditRotator({ maxEntries: 100 });
      expect(rotator.shouldRotate(150)).toBe(true);
    });
  });

  // ---- shouldRotateBySize() ----------------------------------------------

  describe('shouldRotateBySize()', () => {
    it('returns false when under limit', () => {
      const rotator = new AuditRotator({ maxFileSize: 1024 });
      expect(rotator.shouldRotateBySize(512)).toBe(false);
    });

    it('returns true when at limit', () => {
      const rotator = new AuditRotator({ maxFileSize: 1024 });
      expect(rotator.shouldRotateBySize(1024)).toBe(true);
    });

    it('returns true when over limit', () => {
      const rotator = new AuditRotator({ maxFileSize: 1024 });
      expect(rotator.shouldRotateBySize(2048)).toBe(true);
    });
  });

  // ---- getRotatedFilename() ----------------------------------------------

  describe('getRotatedFilename()', () => {
    it('generates correct names for files with extensions', () => {
      const rotator = new AuditRotator();
      expect(rotator.getRotatedFilename('audit.log', 1)).toBe('audit.1.log');
      expect(rotator.getRotatedFilename('audit.log', 2)).toBe('audit.2.log');
      expect(rotator.getRotatedFilename('audit.log', 5)).toBe('audit.5.log');
    });

    it('handles dotless filenames', () => {
      const rotator = new AuditRotator();
      expect(rotator.getRotatedFilename('audit', 1)).toBe('audit.1');
      expect(rotator.getRotatedFilename('audit', 3)).toBe('audit.3');
    });

    it('handles filenames with multiple dots', () => {
      const rotator = new AuditRotator();
      expect(rotator.getRotatedFilename('my.audit.log', 1)).toBe('my.audit.1.log');
    });
  });

  // ---- getRotatedFilenames() ---------------------------------------------

  describe('getRotatedFilenames()', () => {
    it('returns correct count of filenames', () => {
      const rotator = new AuditRotator({ rotateCount: 3 });
      const filenames = rotator.getRotatedFilenames('audit.log');
      expect(filenames).toHaveLength(3);
    });

    it('returns filenames oldest first (highest index first)', () => {
      const rotator = new AuditRotator({ rotateCount: 3 });
      const filenames = rotator.getRotatedFilenames('audit.log');
      expect(filenames).toEqual([
        'audit.3.log',
        'audit.2.log',
        'audit.1.log',
      ]);
    });

    it('uses default rotateCount of 5', () => {
      const rotator = new AuditRotator();
      const filenames = rotator.getRotatedFilenames('audit.log');
      expect(filenames).toHaveLength(5);
    });
  });

  // ---- partition() -------------------------------------------------------

  describe('partition()', () => {
    it('keeps most recent maxEntries entries', () => {
      const rotator = new AuditRotator({ maxEntries: 3 });
      const entries: AuditEntry[] = [];
      for (let i = 0; i < 5; i++) {
        entries.push(makeEntry({ timestamp: i * 1000 }));
      }
      const [keep, archive] = rotator.partition(entries);
      expect(keep).toHaveLength(3);
      expect(keep[0].timestamp).toBe(2000);
      expect(keep[1].timestamp).toBe(3000);
      expect(keep[2].timestamp).toBe(4000);
    });

    it('returns empty archive when under limit', () => {
      const rotator = new AuditRotator({ maxEntries: 10 });
      const entries = makeSampleEntries();
      const [keep, archive] = rotator.partition(entries);
      expect(keep).toHaveLength(entries.length);
      expect(archive).toHaveLength(0);
    });

    it('returns correct archive when over limit', () => {
      const rotator = new AuditRotator({ maxEntries: 2 });
      const entries = makeSampleEntries(); // 5 entries
      const [keep, archive] = rotator.partition(entries);
      expect(keep).toHaveLength(2);
      expect(archive).toHaveLength(3);
      // archive should contain the oldest entries
      expect(archive[0].timestamp).toBe(1000);
      expect(archive[1].timestamp).toBe(2000);
      expect(archive[2].timestamp).toBe(3000);
    });

    it('returns all entries in keep when exactly at limit', () => {
      const rotator = new AuditRotator({ maxEntries: 5 });
      const entries = makeSampleEntries(); // 5 entries
      const [keep, archive] = rotator.partition(entries);
      expect(keep).toHaveLength(5);
      expect(archive).toHaveLength(0);
    });
  });

  // ---- config defaults ---------------------------------------------------

  describe('config', () => {
    it('default config uses 10000 entries and 10MB', () => {
      const rotator = new AuditRotator();
      const config = rotator.getConfig();
      expect(config.maxEntries).toBe(10000);
      expect(config.maxFileSize).toBe(10 * 1024 * 1024);
      expect(config.rotateCount).toBe(5);
    });

    it('custom config overrides defaults', () => {
      const rotator = new AuditRotator({
        maxEntries: 500,
        maxFileSize: 2048,
        rotateCount: 3,
      });
      const config = rotator.getConfig();
      expect(config.maxEntries).toBe(500);
      expect(config.maxFileSize).toBe(2048);
      expect(config.rotateCount).toBe(3);
    });

    it('partial custom config keeps remaining defaults', () => {
      const rotator = new AuditRotator({ maxEntries: 500 });
      const config = rotator.getConfig();
      expect(config.maxEntries).toBe(500);
      expect(config.maxFileSize).toBe(10 * 1024 * 1024);
      expect(config.rotateCount).toBe(5);
    });
  });
});
