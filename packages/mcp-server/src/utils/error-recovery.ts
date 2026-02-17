/**
 * Error recovery and circuit breaker for bridge connection.
 */

export class ErrorRecovery {
  private static retryDelays = [1000, 3000, 5000];

  static async withRetry<T>(
    operation: () => Promise<T>,
    context: { component: string; operation: string },
    maxAttempts = 3
  ): Promise<T> {
    for (let attempt = 0; attempt < maxAttempts; attempt++) {
      try {
        return await operation();
      } catch (error) {
        const message = error instanceof Error ? error.message : String(error);
        if (attempt < maxAttempts - 1) {
          const delay = this.retryDelays[attempt] || 5000;
          console.error(`[${context.component}] ${context.operation} failed (attempt ${attempt + 1}/${maxAttempts}): ${message}, retrying in ${delay}ms...`);
          await new Promise(resolve => setTimeout(resolve, delay));
        } else {
          throw new Error(`${context.component}: ${context.operation} failed after ${maxAttempts} attempts: ${message}`);
        }
      }
    }
    throw new Error('unreachable');
  }

  static createCircuitBreaker(threshold = 5, resetTimeout = 60000) {
    let failureCount = 0;
    let lastFailureTime = 0;
    let isOpen = false;

    return {
      async call<T>(operation: () => Promise<T>): Promise<T> {
        if (isOpen && Date.now() - lastFailureTime > resetTimeout) {
          isOpen = false;
          failureCount = 0;
        }

        if (isOpen) {
          throw new Error('Circuit breaker open - bridge unavailable');
        }

        try {
          const result = await operation();
          failureCount = 0;
          return result;
        } catch (error) {
          failureCount++;
          lastFailureTime = Date.now();
          if (failureCount >= threshold) {
            isOpen = true;
            console.error(`Circuit breaker opened after ${threshold} failures`);
          }
          throw error;
        }
      },
      isOpen: () => isOpen,
      reset: () => { isOpen = false; failureCount = 0; },
    };
  }
}
