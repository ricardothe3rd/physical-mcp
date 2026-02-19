/**
 * Structured logger for PhysicalMCP.
 *
 * Supports both human-readable and JSON output formats.
 * All output goes to stderr (stdout is reserved for MCP protocol).
 */

export type LogLevel = 'debug' | 'info' | 'warn' | 'error';

const LOG_LEVELS: Record<LogLevel, number> = {
  debug: 0,
  info: 1,
  warn: 2,
  error: 3,
};

export interface LogEntry {
  timestamp: string;
  level: LogLevel;
  component: string;
  message: string;
  data?: Record<string, unknown>;
}

export interface LoggerOptions {
  level?: LogLevel;
  format?: 'text' | 'json';
  component?: string;
}

export class Logger {
  private level: number;
  private format: 'text' | 'json';
  private component: string;
  private output: (msg: string) => void;

  constructor(options: LoggerOptions = {}) {
    this.level = LOG_LEVELS[options.level ?? 'info'];
    this.format = options.format ?? 'text';
    this.component = options.component ?? 'PhysicalMCP';
    this.output = (msg: string) => process.stderr.write(msg + '\n');
  }

  /** Create a child logger with a specific component name */
  child(component: string): Logger {
    const child = new Logger({
      level: this.getLevelName(),
      format: this.format,
      component,
    });
    child.output = this.output;
    return child;
  }

  /** Set the output function (useful for testing) */
  setOutput(fn: (msg: string) => void): void {
    this.output = fn;
  }

  /** Get current level name */
  getLevelName(): LogLevel {
    for (const [name, val] of Object.entries(LOG_LEVELS)) {
      if (val === this.level) return name as LogLevel;
    }
    return 'info';
  }

  /** Set the minimum log level */
  setLevel(level: LogLevel): void {
    this.level = LOG_LEVELS[level];
  }

  debug(message: string, data?: Record<string, unknown>): void {
    this.log('debug', message, data);
  }

  info(message: string, data?: Record<string, unknown>): void {
    this.log('info', message, data);
  }

  warn(message: string, data?: Record<string, unknown>): void {
    this.log('warn', message, data);
  }

  error(message: string, data?: Record<string, unknown>): void {
    this.log('error', message, data);
  }

  private log(level: LogLevel, message: string, data?: Record<string, unknown>): void {
    if (LOG_LEVELS[level] < this.level) return;

    const entry: LogEntry = {
      timestamp: new Date().toISOString(),
      level,
      component: this.component,
      message,
      ...(data && Object.keys(data).length > 0 ? { data } : {}),
    };

    if (this.format === 'json') {
      this.output(JSON.stringify(entry));
    } else {
      const prefix = `[${this.component}]`;
      const levelTag = level.toUpperCase().padEnd(5);
      const dataStr = data && Object.keys(data).length > 0
        ? ' ' + JSON.stringify(data)
        : '';
      this.output(`${prefix} ${levelTag} ${message}${dataStr}`);
    }
  }
}

/** Default global logger instance */
export const logger = new Logger();
