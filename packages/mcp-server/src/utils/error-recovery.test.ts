import { describe, it, expect, vi, beforeEach } from 'vitest';
import { ErrorRecovery } from './error-recovery.js';

describe('ErrorRecovery', () => {
  describe('withRetry', () => {
    beforeEach(() => {
      vi.useFakeTimers();
    });

    it('returns result on first success', async () => {
      const op = vi.fn().mockResolvedValue('ok');
      const result = await ErrorRecovery.withRetry(op, { component: 'Test', operation: 'test' });
      expect(result).toBe('ok');
      expect(op).toHaveBeenCalledTimes(1);
    });

    it('retries on failure and succeeds', async () => {
      const op = vi.fn()
        .mockRejectedValueOnce(new Error('fail 1'))
        .mockResolvedValue('ok');

      const promise = ErrorRecovery.withRetry(op, { component: 'Test', operation: 'test' });
      // Advance past retry delay
      await vi.advanceTimersByTimeAsync(2000);
      const result = await promise;

      expect(result).toBe('ok');
      expect(op).toHaveBeenCalledTimes(2);
    });

    it('throws after max attempts', async () => {
      vi.useRealTimers(); // Use real timers for this test to avoid async leaks
      const op = vi.fn().mockRejectedValue(new Error('always fails'));

      await expect(
        ErrorRecovery.withRetry(op, { component: 'Test', operation: 'test' }, 1)
      ).rejects.toThrow('Test: test failed after 1 attempts');
      expect(op).toHaveBeenCalledTimes(1);
    });

    it('respects custom max attempts', async () => {
      vi.useRealTimers();
      const op = vi.fn().mockRejectedValue(new Error('fail'));

      await expect(
        ErrorRecovery.withRetry(op, { component: 'Test', operation: 'test' }, 1)
      ).rejects.toThrow('Test: test failed after 1 attempts');
      expect(op).toHaveBeenCalledTimes(1);
    });
  });

  describe('createCircuitBreaker', () => {
    it('passes through successful calls', async () => {
      const cb = ErrorRecovery.createCircuitBreaker(3, 1000);
      const result = await cb.call(async () => 'success');
      expect(result).toBe('success');
      expect(cb.isOpen()).toBe(false);
    });

    it('opens after threshold failures', async () => {
      const cb = ErrorRecovery.createCircuitBreaker(3, 1000);

      for (let i = 0; i < 3; i++) {
        await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      }

      expect(cb.isOpen()).toBe(true);
      await expect(cb.call(async () => 'ok')).rejects.toThrow('Circuit breaker open');
    });

    it('resets failure count on success', async () => {
      const cb = ErrorRecovery.createCircuitBreaker(3, 1000);

      // 2 failures
      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();

      // 1 success — resets count
      const result = await cb.call(async () => 'ok');
      expect(result).toBe('ok');

      // 2 more failures — should not open (count reset)
      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      expect(cb.isOpen()).toBe(false);
    });

    it('auto-resets after timeout', async () => {
      vi.useFakeTimers();
      const cb = ErrorRecovery.createCircuitBreaker(2, 5000);

      // Trip the breaker
      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      expect(cb.isOpen()).toBe(true);

      // Advance past reset timeout
      vi.advanceTimersByTime(6000);

      // Should allow calls again
      const result = await cb.call(async () => 'recovered');
      expect(result).toBe('recovered');
      expect(cb.isOpen()).toBe(false);

      vi.useRealTimers();
    });

    it('can be manually reset', async () => {
      const cb = ErrorRecovery.createCircuitBreaker(2, 60000);

      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      await expect(cb.call(async () => { throw new Error('fail'); })).rejects.toThrow();
      expect(cb.isOpen()).toBe(true);

      cb.reset();
      expect(cb.isOpen()).toBe(false);

      const result = await cb.call(async () => 'ok');
      expect(result).toBe('ok');
    });
  });
});
