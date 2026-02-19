/**
 * Audit log export and rotation utilities.
 *
 * Provides CSV, JSON, and JSON Lines export formats with filtering,
 * plus a rotation strategy based on entry count or data size.
 */

export interface AuditEntry {
  timestamp: number;
  type: 'publish' | 'service' | 'action' | 'safety' | 'system';
  action: string;
  target: string;
  result: 'allowed' | 'blocked' | 'error';
  details?: string;
  robotId?: string;
}

export interface ExportOptions {
  format: 'json' | 'csv' | 'jsonl';
  includeHeaders?: boolean;
  filter?: {
    type?: AuditEntry['type'];
    result?: AuditEntry['result'];
    startTime?: number;
    endTime?: number;
    robotId?: string;
  };
}

export interface RotationConfig {
  maxEntries?: number;
  maxFileSize?: number;
  rotateCount?: number;
}

const CSV_HEADERS = ['timestamp', 'type', 'action', 'target', 'result', 'details', 'robotId'];

/**
 * Escape a value for inclusion in a CSV field.
 * Wraps in double quotes if the value contains a comma, double quote, or newline.
 * Double quotes within the value are escaped by doubling them.
 */
function escapeCsvField(value: string): string {
  if (value.includes(',') || value.includes('"') || value.includes('\n') || value.includes('\r')) {
    return '"' + value.replace(/"/g, '""') + '"';
  }
  return value;
}

export class AuditExporter {
  /**
   * Export audit entries to a string in the specified format.
   * Applies optional filtering before formatting.
   */
  static export(entries: AuditEntry[], options: ExportOptions): string {
    const filtered = options.filter
      ? AuditExporter.filter(entries, options.filter)
      : entries;

    switch (options.format) {
      case 'json':
        return AuditExporter.toJson(filtered);
      case 'csv':
        return AuditExporter.toCsv(filtered, options.includeHeaders ?? true);
      case 'jsonl':
        return AuditExporter.toJsonLines(filtered);
    }
  }

  /**
   * Filter entries based on criteria. All provided filters are combined with AND logic.
   * Returns all entries if filter is undefined or empty.
   */
  static filter(entries: AuditEntry[], filter: ExportOptions['filter']): AuditEntry[] {
    if (!filter) {
      return entries;
    }

    return entries.filter((entry) => {
      if (filter.type !== undefined && entry.type !== filter.type) {
        return false;
      }
      if (filter.result !== undefined && entry.result !== filter.result) {
        return false;
      }
      if (filter.startTime !== undefined && entry.timestamp < filter.startTime) {
        return false;
      }
      if (filter.endTime !== undefined && entry.timestamp > filter.endTime) {
        return false;
      }
      if (filter.robotId !== undefined && entry.robotId !== filter.robotId) {
        return false;
      }
      return true;
    });
  }

  /**
   * Format a single entry as a CSV row.
   * Fields are escaped according to RFC 4180.
   */
  static entryToCsv(entry: AuditEntry): string {
    const values = [
      String(entry.timestamp),
      entry.type,
      entry.action,
      entry.target,
      entry.result,
      entry.details ?? '',
      entry.robotId ?? '',
    ];
    return values.map(escapeCsvField).join(',');
  }

  /**
   * Format entries as CSV with an optional header row.
   */
  static toCsv(entries: AuditEntry[], includeHeaders = true): string {
    const rows: string[] = [];
    if (includeHeaders) {
      rows.push(CSV_HEADERS.join(','));
    }
    for (const entry of entries) {
      rows.push(AuditExporter.entryToCsv(entry));
    }
    return rows.join('\n');
  }

  /**
   * Format entries as a pretty-printed JSON array.
   */
  static toJson(entries: AuditEntry[]): string {
    return JSON.stringify(entries, null, 2);
  }

  /**
   * Format entries as JSON Lines (one JSON object per line).
   * Good for streaming and appending.
   */
  static toJsonLines(entries: AuditEntry[]): string {
    return entries.map((entry) => JSON.stringify(entry)).join('\n');
  }
}

export class AuditRotator {
  private config: Required<RotationConfig>;

  constructor(config?: RotationConfig) {
    this.config = {
      maxEntries: config?.maxEntries ?? 10000,
      maxFileSize: config?.maxFileSize ?? 10 * 1024 * 1024,
      rotateCount: config?.rotateCount ?? 5,
    };
  }

  /** Check if rotation is needed based on entry count. */
  shouldRotate(entryCount: number): boolean {
    return entryCount >= this.config.maxEntries;
  }

  /** Check if rotation is needed based on file/data size in bytes. */
  shouldRotateBySize(sizeBytes: number): boolean {
    return sizeBytes >= this.config.maxFileSize;
  }

  /**
   * Generate a rotated filename.
   * Examples:
   *   audit.log, 1 -> audit.1.log
   *   audit.log, 2 -> audit.2.log
   *   audit, 1     -> audit.1
   */
  getRotatedFilename(baseName: string, index: number): string {
    const dotIndex = baseName.lastIndexOf('.');
    if (dotIndex === -1) {
      return `${baseName}.${index}`;
    }
    const name = baseName.substring(0, dotIndex);
    const ext = baseName.substring(dotIndex);
    return `${name}.${index}${ext}`;
  }

  /**
   * Get the list of rotated filenames that should be kept, oldest first.
   * Returns filenames from index rotateCount down to 1.
   */
  getRotatedFilenames(baseName: string): string[] {
    const filenames: string[] = [];
    for (let i = this.config.rotateCount; i >= 1; i--) {
      filenames.push(this.getRotatedFilename(baseName, i));
    }
    return filenames;
  }

  /**
   * Partition entries into [keep, archive].
   * Keep is the most recent maxEntries entries; archive is everything else.
   * Entries are assumed to be in chronological order (oldest first).
   */
  partition(entries: AuditEntry[]): [AuditEntry[], AuditEntry[]] {
    if (entries.length <= this.config.maxEntries) {
      return [entries, []];
    }
    const archiveCount = entries.length - this.config.maxEntries;
    const archive = entries.slice(0, archiveCount);
    const keep = entries.slice(archiveCount);
    return [keep, archive];
  }

  /** Get the resolved configuration. */
  getConfig(): Required<RotationConfig> {
    return { ...this.config };
  }
}
