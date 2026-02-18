import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { Logger } from './logger.js';

describe('Logger', () => {
  let consoleSpy: ReturnType<typeof vi.spyOn>;

  beforeEach(() => {
    consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});
  });

  afterEach(() => {
    consoleSpy.mockRestore();
  });

  it('logs at info level by default', () => {
    const logger = new Logger();
    logger.info('Test', 'hello');
    expect(consoleSpy).toHaveBeenCalledTimes(1);
    expect(consoleSpy.mock.calls[0][0]).toContain('hello');
  });

  it('does not log debug when level is info', () => {
    const logger = new Logger({ level: 'info' });
    logger.debug('Test', 'debug message');
    expect(consoleSpy).not.toHaveBeenCalled();
  });

  it('logs debug when level is debug', () => {
    const logger = new Logger({ level: 'debug' });
    logger.debug('Test', 'debug message');
    expect(consoleSpy).toHaveBeenCalledTimes(1);
  });

  it('logs warn at info level', () => {
    const logger = new Logger({ level: 'info' });
    logger.warn('Test', 'warning');
    expect(consoleSpy).toHaveBeenCalledTimes(1);
  });

  it('does not log info when level is error', () => {
    const logger = new Logger({ level: 'error' });
    logger.info('Test', 'info message');
    expect(consoleSpy).not.toHaveBeenCalled();
    logger.error('Test', 'error message');
    expect(consoleSpy).toHaveBeenCalledTimes(1);
  });

  it('includes prefix and component in output', () => {
    const logger = new Logger({ prefix: 'MyApp' });
    logger.info('Safety', 'check passed');
    expect(consoleSpy.mock.calls[0][0]).toContain('[MyApp:Safety]');
    expect(consoleSpy.mock.calls[0][0]).toContain('check passed');
  });

  it('outputs JSON format when enabled', () => {
    const logger = new Logger({ json: true });
    logger.info('Test', 'hello', { extra: 42 });

    const output = consoleSpy.mock.calls[0][0];
    const parsed = JSON.parse(output);
    expect(parsed.level).toBe('info');
    expect(parsed.component).toBe('Test');
    expect(parsed.message).toBe('hello');
    expect(parsed.extra).toBe(42);
    expect(parsed.timestamp).toBeDefined();
  });

  it('includes data in text format', () => {
    const logger = new Logger();
    logger.info('Test', 'event', { count: 5 });
    expect(consoleSpy.mock.calls[0][0]).toContain('"count":5');
  });

  it('setLevel changes minimum log level', () => {
    const logger = new Logger({ level: 'error' });
    logger.info('Test', 'should not show');
    expect(consoleSpy).not.toHaveBeenCalled();

    logger.setLevel('debug');
    logger.info('Test', 'should show');
    expect(consoleSpy).toHaveBeenCalledTimes(1);
  });

  it('setJson toggles JSON output', () => {
    const logger = new Logger();
    logger.setJson(true);
    logger.info('Test', 'json mode');

    const output = consoleSpy.mock.calls[0][0];
    expect(() => JSON.parse(output)).not.toThrow();
  });

  it('logs all levels correctly', () => {
    const logger = new Logger({ level: 'debug' });
    logger.debug('C', 'debug');
    logger.info('C', 'info');
    logger.warn('C', 'warn');
    logger.error('C', 'error');
    expect(consoleSpy).toHaveBeenCalledTimes(4);
  });
});
