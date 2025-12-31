/**
 * Preference Reader
 * Reads user preferences from auth database (003-authentication integration)
 * Provides fallback to defaults for incomplete profiles
 */

import {
  UserPreferences,
  DEFAULT_PREFERENCES,
  TechnicalBackground,
  DomainKnowledge,
  PreferredDepth,
  CodeExamplesImportance,
  FocusArea,
  TimeCommitment,
  LearningGoal,
  LanguagePreference,
} from '../types/personalization';

interface AuthDatabaseConfig {
  connectionString?: string;
  mockMode?: boolean;
}

/**
 * Mock user data for development/testing
 */
const MOCK_USERS: Record<string, Partial<UserPreferences>> = {
  'user-beginner': {
    technicalBackground: 'beginner',
    domainKnowledge: 'none',
    preferredDepth: 'overview',
    codeExamplesImportance: 'minimal',
    focusArea: 'general',
    timeCommitment: 'minimal',
    learningGoal: 'hobby',
    languagePreference: 'en',
  },
  'user-advanced': {
    technicalBackground: 'advanced',
    domainKnowledge: 'experienced',
    preferredDepth: 'deep-dive',
    codeExamplesImportance: 'essential',
    focusArea: 'ros2',
    timeCommitment: 'dedicated',
    learningGoal: 'career-change',
    languagePreference: 'en',
  },
  'user-urdu': {
    technicalBackground: 'intermediate',
    domainKnowledge: 'some',
    preferredDepth: 'balanced',
    codeExamplesImportance: 'helpful',
    focusArea: 'simulation',
    timeCommitment: 'moderate',
    learningGoal: 'skill-enhancement',
    languagePreference: 'ur',
  },
  'user-incomplete': {
    technicalBackground: 'beginner',
    // Other fields intentionally missing
  },
};

class PreferenceReader {
  private config: AuthDatabaseConfig;

  constructor(config: AuthDatabaseConfig = {}) {
    this.config = {
      mockMode: process.env.NODE_ENV !== 'production',
      ...config,
    };
  }

  /**
   * Fetch user preferences from auth database
   * Returns null if user not found
   */
  async getUserPreferences(userId: string): Promise<UserPreferences | null> {
    if (this.config.mockMode) {
      return this.getMockPreferences(userId);
    }

    return this.fetchFromDatabase(userId);
  }

  /**
   * Get preferences with defaults applied for missing fields
   */
  async getPreferencesWithDefaults(userId: string): Promise<UserPreferences> {
    const prefs = await this.getUserPreferences(userId);

    if (!prefs) {
      // Return full defaults for unknown user
      return {
        userId,
        ...DEFAULT_PREFERENCES,
        updatedAt: new Date(),
      };
    }

    // Merge with defaults for any missing fields
    return this.applyDefaults(prefs);
  }

  /**
   * Check if user has complete preferences
   */
  async hasCompletePreferences(userId: string): Promise<boolean> {
    const prefs = await this.getUserPreferences(userId);

    if (!prefs) return false;

    const requiredFields: (keyof UserPreferences)[] = [
      'technicalBackground',
      'domainKnowledge',
      'preferredDepth',
      'codeExamplesImportance',
      'focusArea',
      'timeCommitment',
      'learningGoal',
      'languagePreference',
    ];

    return requiredFields.every(
      field => prefs[field] !== undefined && prefs[field] !== null
    );
  }

  /**
   * Get list of missing preference fields
   */
  async getMissingFields(userId: string): Promise<string[]> {
    const prefs = await this.getUserPreferences(userId);

    if (!prefs) {
      return [
        'technicalBackground',
        'domainKnowledge',
        'preferredDepth',
        'codeExamplesImportance',
        'focusArea',
        'timeCommitment',
        'learningGoal',
        'languagePreference',
      ];
    }

    const requiredFields: (keyof Omit<UserPreferences, 'userId' | 'updatedAt'>)[] = [
      'technicalBackground',
      'domainKnowledge',
      'preferredDepth',
      'codeExamplesImportance',
      'focusArea',
      'timeCommitment',
      'learningGoal',
      'languagePreference',
    ];

    return requiredFields.filter(
      field => prefs[field] === undefined || prefs[field] === null
    );
  }

  /**
   * Apply defaults to incomplete preferences
   */
  private applyDefaults(prefs: Partial<UserPreferences> & { userId: string }): UserPreferences {
    return {
      userId: prefs.userId,
      technicalBackground: prefs.technicalBackground ?? DEFAULT_PREFERENCES.technicalBackground,
      domainKnowledge: prefs.domainKnowledge ?? DEFAULT_PREFERENCES.domainKnowledge,
      preferredDepth: prefs.preferredDepth ?? DEFAULT_PREFERENCES.preferredDepth,
      codeExamplesImportance: prefs.codeExamplesImportance ?? DEFAULT_PREFERENCES.codeExamplesImportance,
      focusArea: prefs.focusArea ?? DEFAULT_PREFERENCES.focusArea,
      timeCommitment: prefs.timeCommitment ?? DEFAULT_PREFERENCES.timeCommitment,
      learningGoal: prefs.learningGoal ?? DEFAULT_PREFERENCES.learningGoal,
      languagePreference: prefs.languagePreference ?? DEFAULT_PREFERENCES.languagePreference,
      updatedAt: prefs.updatedAt ?? new Date(),
    };
  }

  /**
   * Mock implementation for development
   */
  private getMockPreferences(userId: string): UserPreferences | null {
    const mockData = MOCK_USERS[userId];

    if (!mockData) {
      return null;
    }

    return this.applyDefaults({
      userId,
      ...mockData,
      updatedAt: new Date(),
    });
  }

  /**
   * Real database fetch (placeholder for 003-auth integration)
   */
  private async fetchFromDatabase(userId: string): Promise<UserPreferences | null> {
    // TODO: Integrate with 003-authentication Neon PostgreSQL
    // This will be replaced with actual database query:
    //
    // const { rows } = await pool.query(
    //   'SELECT * FROM user_preferences WHERE user_id = $1',
    //   [userId]
    // );
    //
    // if (rows.length === 0) return null;
    // return this.mapRowToPreferences(rows[0]);

    console.warn('[PreferenceReader] Database integration pending - using mock mode');
    return this.getMockPreferences(userId);
  }

  /**
   * Validate preference values
   */
  validatePreferences(prefs: Partial<UserPreferences>): string[] {
    const errors: string[] = [];

    const validValues: Record<string, string[]> = {
      technicalBackground: ['beginner', 'intermediate', 'advanced'],
      domainKnowledge: ['none', 'some', 'experienced'],
      preferredDepth: ['overview', 'balanced', 'deep-dive'],
      codeExamplesImportance: ['essential', 'helpful', 'minimal'],
      focusArea: ['ros2', 'simulation', 'computer-vision', 'vla-models', 'general'],
      timeCommitment: ['minimal', 'moderate', 'dedicated'],
      learningGoal: ['hobby', 'career-change', 'skill-enhancement', 'academic'],
      languagePreference: ['en', 'ur'],
    };

    for (const [field, allowed] of Object.entries(validValues)) {
      const value = prefs[field as keyof UserPreferences];
      if (value !== undefined && !allowed.includes(value as string)) {
        errors.push(`Invalid ${field}: ${value}. Must be one of: ${allowed.join(', ')}`);
      }
    }

    return errors;
  }
}

// Singleton instance
export const preferenceReader = new PreferenceReader();

// Export class for testing
export { PreferenceReader };
