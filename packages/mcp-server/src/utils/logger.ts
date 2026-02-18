/**
 * Configurable logger with log levels and optional JSON output.
 */

export type LogLevel = 'debug' | 'info' | 'warn' | 'error';

const LEVEL_ORDER: Record<LogLevel, number> = {
  debug: 0,
  info: 1,
  warn: 2,
  error: 3,
};

export interface LoggerOptions {
  level: LogLevel;
  json: boolean;
  prefix: string;
}

export class Logger {
  private level: number;
  private json: boolean;
  private prefix: string;

  constructor(options: Partial<LoggerOptions> = {}) {
    this.level = LEVEL_ORDER[options.level || 'info'];
    this.json = options.json || false;
    this.prefix = options.prefix || 'PhysicalMCP';
  }

  private shouldLog(level: LogLevel): boolean {
    return LEVEL_ORDER[level] >= this.level;
  }

  private format(level: LogLevel, component: string, message: string, data?: Record<string, unknown>): string {
    if (this.json) {
      return JSON.stringify({
        timestamp: new Date().toISOString(),
        level,
        component,
        message,
        ...data,
      });
    }
    const tag = `[${this.prefix}:${component}]`;
    const dataStr = data ? ` ${JSON.stringify(data)}` : '';
    return `${tag} ${message}${dataStr}`;
  }

  debug(component: string, message: string, data?: Record<string, unknown>): void {
    if (this.shouldLog('debug')) {
      console.error(this.format('debug', component, message, data));
    }
  }

  info(component: string, message: string, data?: Record<string, unknown>): void {
    if (this.shouldLog('info')) {
      console.error(this.format('info', component, message, data));
    }
  }

  warn(component: string, message: string, data?: Record<string, unknown>): void {
    if (this.shouldLog('warn')) {
      console.error(this.format('warn', component, message, data));
    }
  }

  error(component: string, message: string, data?: Record<string, unknown>): void {
    if (this.shouldLog('error')) {
      console.error(this.format('error', component, message, data));
    }
  }

  setLevel(level: LogLevel): void {
    this.level = LEVEL_ORDER[level];
  }

  setJson(json: boolean): void {
    this.json = json;
  }
}

/** Global logger instance. */
export const logger = new Logger();
