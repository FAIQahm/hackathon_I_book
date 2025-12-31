/**
 * Personalization Cache Layer
 * In-memory cache with TTL support for personalization data
 * TTL: Profile=5min, Response=1hr, Recommendations=30min
 */

interface CacheEntry<T> {
  data: T;
  expiresAt: number;
  createdAt: number;
}

interface CacheConfig {
  profileTTL: number;      // 5 minutes
  responseTTL: number;     // 1 hour
  recommendationTTL: number; // 30 minutes
}

const DEFAULT_CONFIG: CacheConfig = {
  profileTTL: 5 * 60 * 1000,        // 5 minutes in ms
  responseTTL: 60 * 60 * 1000,      // 1 hour in ms
  recommendationTTL: 30 * 60 * 1000, // 30 minutes in ms
};

class PersonalizationCache {
  private cache: Map<string, CacheEntry<unknown>> = new Map();
  private config: CacheConfig;

  constructor(config: Partial<CacheConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    // Run cleanup every minute
    setInterval(() => this.cleanup(), 60 * 1000);
  }

  /**
   * Generate cache key for user profile
   */
  profileKey(userId: string): string {
    return `profile:${userId}`;
  }

  /**
   * Generate cache key for chatbot response
   */
  responseKey(userId: string, queryHash: string): string {
    return `response:${userId}:${queryHash}`;
  }

  /**
   * Generate cache key for recommendations
   */
  recommendationKey(userId: string): string {
    return `recommendations:${userId}`;
  }

  /**
   * Generate cache key for learning path
   */
  learningPathKey(userId: string): string {
    return `learning-path:${userId}`;
  }

  /**
   * Get item from cache
   */
  get<T>(key: string): T | null {
    const entry = this.cache.get(key) as CacheEntry<T> | undefined;

    if (!entry) {
      return null;
    }

    if (Date.now() > entry.expiresAt) {
      this.cache.delete(key);
      return null;
    }

    return entry.data;
  }

  /**
   * Set item in cache with TTL based on key type
   */
  set<T>(key: string, data: T, customTTL?: number): void {
    const ttl = customTTL ?? this.getTTLForKey(key);
    const now = Date.now();

    this.cache.set(key, {
      data,
      createdAt: now,
      expiresAt: now + ttl,
    });
  }

  /**
   * Determine TTL based on key prefix
   */
  private getTTLForKey(key: string): number {
    if (key.startsWith('profile:')) {
      return this.config.profileTTL;
    }
    if (key.startsWith('response:')) {
      return this.config.responseTTL;
    }
    if (key.startsWith('recommendations:') || key.startsWith('learning-path:')) {
      return this.config.recommendationTTL;
    }
    return this.config.profileTTL; // Default
  }

  /**
   * Invalidate all cache entries for a user
   * Called when user preferences are updated
   */
  invalidateUser(userId: string): void {
    const keysToDelete: string[] = [];

    for (const key of this.cache.keys()) {
      if (key.includes(userId)) {
        keysToDelete.push(key);
      }
    }

    keysToDelete.forEach(key => this.cache.delete(key));
  }

  /**
   * Invalidate specific key
   */
  invalidate(key: string): void {
    this.cache.delete(key);
  }

  /**
   * Check if key exists and is not expired
   */
  has(key: string): boolean {
    return this.get(key) !== null;
  }

  /**
   * Get cache statistics
   */
  getStats(): { size: number; keys: string[] } {
    return {
      size: this.cache.size,
      keys: Array.from(this.cache.keys()),
    };
  }

  /**
   * Clear all cache entries
   */
  clear(): void {
    this.cache.clear();
  }

  /**
   * Remove expired entries
   */
  private cleanup(): void {
    const now = Date.now();
    const keysToDelete: string[] = [];

    for (const [key, entry] of this.cache.entries()) {
      if (now > entry.expiresAt) {
        keysToDelete.push(key);
      }
    }

    keysToDelete.forEach(key => this.cache.delete(key));
  }
}

// Singleton instance
export const personalizationCache = new PersonalizationCache();

// Export class for testing
export { PersonalizationCache, CacheConfig };
