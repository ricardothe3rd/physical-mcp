import { describe, it, expect, vi, beforeEach } from 'vitest';
import { Logger, logger } from './logger.js';

describe('Logger', () => {
  let output: ReturnType<typeof vi.fn>;
  let log: Logger;

  beforeEach(() => {
    output = vi.fn();
  });

  // --- Text format ---

  describe('text format', () => {
    beforeEach(() => {
      log = new Logger({ level: 'debug', format: 'text', component: 'TestComp' });
      log.setOutput(output);
    });

    it('debug produces correct text format with component and level tag', () => {
      log.debug('a debug message');
      expect(output).toHaveBeenCalledTimes(1);
      const msg = output.mock.calls[0][0] as string;
      expect(msg).toBe('[TestComp] DEBUG a debug message');
    });

    it('info produces correct text format with component and level tag', () => {
      log.info('an info message');
      const msg = output.mock.calls[0][0] as string;
      expect(msg).toBe('[TestComp] INFO  an info message');
    });

    it('warn produces correct text format with component and level tag', () => {
      log.warn('a warn message');
      const msg = output.mock.calls[0][0] as string;
      expect(msg).toBe('[TestComp] WARN  a warn message');
    });

    it('error produces correct text format with component and level tag', () => {
      log.error('an error message');
      const msg = output.mock.calls[0][0] as string;
      expect(msg).toBe('[TestComp] ERROR an error message');
    });

    it('includes data in text format when provided', () => {
      log.info('with data', { count: 5, key: 'val' });
      const msg = output.mock.calls[0][0] as string;
      expect(msg).toContain('[TestComp] INFO  with data ');
      expect(msg).toContain('"count":5');
      expect(msg).toContain('"key":"val"');
    });
  });

  // --- JSON format ---

  describe('JSON format', () => {
    beforeEach(() => {
      log = new Logger({ level: 'debug', format: 'json', component: 'JsonComp' });
      log.setOutput(output);
    });

    it('produces valid JSON with all required fields', () => {
      log.info('json test');
      const msg = output.mock.calls[0][0] as string;
      const parsed = JSON.parse(msg);
      expect(parsed.timestamp).toBeDefined();
      expect(typeof parsed.timestamp).toBe('string');
      expect(parsed.level).toBe('info');
      expect(parsed.component).toBe('JsonComp');
      expect(parsed.message).toBe('json test');
    });

    it('includes data field in JSON when provided', () => {
      log.warn('warning', { detail: 'abc' });
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed.data).toEqual({ detail: 'abc' });
    });

    it('omits data field in JSON when not provided', () => {
      log.error('no data');
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed).not.toHaveProperty('data');
    });

    it('timestamp is a valid ISO 8601 string', () => {
      log.info('timestamp check');
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      const date = new Date(parsed.timestamp);
      expect(date.toISOString()).toBe(parsed.timestamp);
    });
  });

  // --- Level filtering ---

  describe('level filtering', () => {
    it('suppresses debug messages at info level', () => {
      log = new Logger({ level: 'info' });
      log.setOutput(output);
      log.debug('should not appear');
      expect(output).not.toHaveBeenCalled();
    });

    it('allows info messages at info level', () => {
      log = new Logger({ level: 'info' });
      log.setOutput(output);
      log.info('should appear');
      expect(output).toHaveBeenCalledTimes(1);
    });

    it('error messages are always shown regardless of level', () => {
      for (const lvl of ['debug', 'info', 'warn', 'error'] as const) {
        const fn = vi.fn();
        const l = new Logger({ level: lvl });
        l.setOutput(fn);
        l.error('always shown');
        expect(fn).toHaveBeenCalledTimes(1);
      }
    });

    it('suppresses warn at error level', () => {
      log = new Logger({ level: 'error' });
      log.setOutput(output);
      log.warn('suppressed');
      expect(output).not.toHaveBeenCalled();
    });

    it('suppresses info and debug at warn level', () => {
      log = new Logger({ level: 'warn' });
      log.setOutput(output);
      log.debug('no');
      log.info('no');
      expect(output).not.toHaveBeenCalled();
      log.warn('yes');
      log.error('yes');
      expect(output).toHaveBeenCalledTimes(2);
    });
  });

  // --- setLevel ---

  describe('setLevel', () => {
    it('changes minimum level dynamically', () => {
      log = new Logger({ level: 'error' });
      log.setOutput(output);
      log.info('suppressed');
      expect(output).not.toHaveBeenCalled();

      log.setLevel('debug');
      log.info('now visible');
      expect(output).toHaveBeenCalledTimes(1);
    });

    it('can raise the level to suppress previously visible messages', () => {
      log = new Logger({ level: 'debug' });
      log.setOutput(output);
      log.debug('visible');
      expect(output).toHaveBeenCalledTimes(1);

      log.setLevel('error');
      log.debug('gone');
      log.info('gone');
      log.warn('gone');
      expect(output).toHaveBeenCalledTimes(1);
    });
  });

  // --- child logger ---

  describe('child logger', () => {
    it('inherits level and format from parent', () => {
      const parent = new Logger({ level: 'warn', format: 'json', component: 'Parent' });
      parent.setOutput(output);

      const child = parent.child('ChildComp');

      // child should inherit the same output function
      child.info('suppressed by warn level');
      expect(output).not.toHaveBeenCalled();

      child.warn('visible');
      expect(output).toHaveBeenCalledTimes(1);
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed.component).toBe('ChildComp');
      expect(parsed.level).toBe('warn');
    });

    it('has different component name than parent', () => {
      const parent = new Logger({ level: 'debug', format: 'text', component: 'Parent' });
      parent.setOutput(output);

      const child = parent.child('Safety');
      child.info('check');

      const msg = output.mock.calls[0][0] as string;
      expect(msg).toContain('[Safety]');
      expect(msg).not.toContain('[Parent]');
    });

    it('inherits the output function from parent', () => {
      const parentOutput = vi.fn();
      const parent = new Logger({ level: 'debug' });
      parent.setOutput(parentOutput);

      const child = parent.child('Sub');
      child.info('hello');
      expect(parentOutput).toHaveBeenCalledTimes(1);
    });
  });

  // --- data field ---

  describe('data field', () => {
    beforeEach(() => {
      log = new Logger({ level: 'debug', format: 'json' });
      log.setOutput(output);
    });

    it('is included when provided with values', () => {
      log.info('msg', { foo: 'bar', num: 42 });
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed.data).toEqual({ foo: 'bar', num: 42 });
    });

    it('is omitted when data is an empty object', () => {
      log.info('msg', {});
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed).not.toHaveProperty('data');
    });

    it('is omitted when data is undefined', () => {
      log.info('msg');
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed).not.toHaveProperty('data');
    });

    it('is omitted when data is explicitly undefined', () => {
      log.info('msg', undefined);
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed).not.toHaveProperty('data');
    });

    it('empty data object omitted in text format too', () => {
      const textLog = new Logger({ level: 'debug', format: 'text', component: 'T' });
      textLog.setOutput(output);
      textLog.info('msg', {});
      const msg = output.mock.calls[0][0] as string;
      // Should NOT have trailing JSON of empty object
      expect(msg).toBe('[T] INFO  msg');
    });
  });

  // --- setOutput ---

  describe('setOutput', () => {
    it('custom output function receives log strings', () => {
      const customOutput = vi.fn();
      log = new Logger({ level: 'debug' });
      log.setOutput(customOutput);

      log.info('test message');
      expect(customOutput).toHaveBeenCalledTimes(1);
      expect(typeof customOutput.mock.calls[0][0]).toBe('string');
      expect(customOutput.mock.calls[0][0]).toContain('test message');
    });

    it('replacing output redirects subsequent messages', () => {
      const first = vi.fn();
      const second = vi.fn();
      log = new Logger({ level: 'debug' });
      log.setOutput(first);
      log.info('to first');
      expect(first).toHaveBeenCalledTimes(1);

      log.setOutput(second);
      log.info('to second');
      expect(first).toHaveBeenCalledTimes(1);
      expect(second).toHaveBeenCalledTimes(1);
    });
  });

  // --- Default instance ---

  describe('default instance', () => {
    it('logger export is an instance of Logger', () => {
      expect(logger).toBeInstanceOf(Logger);
    });

    it('default instance has info level', () => {
      expect(logger.getLevelName()).toBe('info');
    });
  });

  // --- getLevelName ---

  describe('getLevelName', () => {
    it('returns correct name for each level', () => {
      log = new Logger();
      for (const lvl of ['debug', 'info', 'warn', 'error'] as const) {
        log.setLevel(lvl);
        expect(log.getLevelName()).toBe(lvl);
      }
    });

    it('returns correct name after setLevel', () => {
      log = new Logger({ level: 'debug' });
      expect(log.getLevelName()).toBe('debug');
      log.setLevel('error');
      expect(log.getLevelName()).toBe('error');
      log.setLevel('warn');
      expect(log.getLevelName()).toBe('warn');
    });
  });

  // --- Edge cases ---

  describe('edge cases', () => {
    it('handles very long messages', () => {
      log = new Logger({ level: 'debug', format: 'text' });
      log.setOutput(output);
      const longMsg = 'x'.repeat(10000);
      log.info(longMsg);
      expect(output).toHaveBeenCalledTimes(1);
      expect((output.mock.calls[0][0] as string)).toContain(longMsg);
    });

    it('handles very long messages in JSON format', () => {
      log = new Logger({ level: 'debug', format: 'json' });
      log.setOutput(output);
      const longMsg = 'y'.repeat(10000);
      log.info(longMsg);
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed.message).toBe(longMsg);
    });

    it('handles data with nested objects', () => {
      log = new Logger({ level: 'debug', format: 'json' });
      log.setOutput(output);
      log.info('nested', { a: { b: { c: 1 } } });
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed.data).toEqual({ a: { b: { c: 1 } } });
    });

    it('handles special characters in messages', () => {
      log = new Logger({ level: 'debug', format: 'json' });
      log.setOutput(output);
      log.info('message with "quotes" and \nnewlines');
      const parsed = JSON.parse(output.mock.calls[0][0] as string);
      expect(parsed.message).toBe('message with "quotes" and \nnewlines');
    });

    it('default component is PhysicalMCP', () => {
      log = new Logger({ level: 'debug', format: 'text' });
      log.setOutput(output);
      log.info('default component');
      expect((output.mock.calls[0][0] as string)).toContain('[PhysicalMCP]');
    });

    it('constructor defaults work with empty options', () => {
      log = new Logger({});
      log.setOutput(output);
      expect(log.getLevelName()).toBe('info');
      log.info('works');
      expect(output).toHaveBeenCalledTimes(1);
    });

    it('constructor defaults work with no options', () => {
      log = new Logger();
      log.setOutput(output);
      expect(log.getLevelName()).toBe('info');
      log.info('works');
      expect(output).toHaveBeenCalledTimes(1);
    });
  });
});
