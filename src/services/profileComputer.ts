/**
 * Profile Computer
 * Transforms raw user preferences into an actionable PersonalizationProfile
 * Implements dimension mapping and complexity calculation
 */

import {
  UserPreferences,
  PersonalizationProfile,
  ComplexityLevel,
  PathType,
  TechnicalBackground,
  DomainKnowledge,
  PreferredDepth,
  TimeCommitment,
} from '../types/personalization';
import { preferenceReader } from './preferenceReader';
import { personalizationCache } from '../cache/personalizationCache';

/**
 * Complexity mapping matrix
 * Technical Background + Domain Knowledge → Complexity Level
 */
const COMPLEXITY_MATRIX: Record<TechnicalBackground, Record<DomainKnowledge, ComplexityLevel>> = {
  beginner: {
    none: 'simple',
    some: 'simple',
    experienced: 'balanced',
  },
  intermediate: {
    none: 'balanced',
    some: 'balanced',
    experienced: 'balanced',
  },
  advanced: {
    none: 'balanced',
    some: 'technical',
    experienced: 'technical',
  },
};

/**
 * Word limit mapping based on preferred depth
 */
const WORD_LIMITS: Record<PreferredDepth, number> = {
  'overview': 150,
  'balanced': 250,
  'deep-dive': 400,
};

/**
 * Path type mapping based on time commitment
 */
const PATH_TYPE_MAPPING: Record<TimeCommitment, PathType> = {
  'minimal': 'condensed',
  'moderate': 'balanced',
  'dedicated': 'comprehensive',
};

class ProfileComputer {
  /**
   * Compute personalization profile from user preferences
   */
  async computeProfile(userId: string): Promise<PersonalizationProfile> {
    // Check cache first
    const cacheKey = personalizationCache.profileKey(userId);
    const cached = personalizationCache.get<PersonalizationProfile>(cacheKey);

    if (cached) {
      return cached;
    }

    // Get preferences with defaults
    const prefs = await preferenceReader.getPreferencesWithDefaults(userId);

    // Compute profile
    const profile = this.computeFromPreferences(prefs);

    // Cache the result
    personalizationCache.set(cacheKey, profile);

    return profile;
  }

  /**
   * Compute profile from preferences object (for testing/direct use)
   */
  computeFromPreferences(prefs: UserPreferences): PersonalizationProfile {
    const complexityLevel = this.computeComplexity(
      prefs.technicalBackground,
      prefs.domainKnowledge
    );

    const activeDimensions = this.getActiveDimensions(prefs);
    const cacheKey = this.generateCacheKey(prefs);

    return {
      userId: prefs.userId,

      // Computed values
      complexityLevel,
      includeCodeExamples: prefs.codeExamplesImportance !== 'minimal',
      responseWordLimit: WORD_LIMITS[prefs.preferredDepth],
      pathType: PATH_TYPE_MAPPING[prefs.timeCommitment],

      // Direct mappings
      focusArea: prefs.focusArea,
      learningGoal: prefs.learningGoal,
      languagePreference: prefs.languagePreference,

      // Content filtering (for FR-071, FR-072)
      hideAdvancedMath: prefs.technicalBackground === 'beginner',
      showConceptPrimers: prefs.technicalBackground === 'beginner' || prefs.domainKnowledge === 'none',

      // Metadata
      activeDimensions,
      computedAt: new Date(),
      cacheKey,
    };
  }

  /**
   * Compute complexity level from technical background and domain knowledge
   */
  private computeComplexity(
    technical: TechnicalBackground,
    domain: DomainKnowledge
  ): ComplexityLevel {
    return COMPLEXITY_MATRIX[technical][domain];
  }

  /**
   * Get list of active (non-default) dimensions
   */
  private getActiveDimensions(prefs: UserPreferences): string[] {
    const dimensions: string[] = [];

    // All 7 dimensions are always active
    dimensions.push('technicalBackground');
    dimensions.push('domainKnowledge');
    dimensions.push('preferredDepth');
    dimensions.push('codeExamplesImportance');
    dimensions.push('focusArea');
    dimensions.push('timeCommitment');
    dimensions.push('learningGoal');

    return dimensions;
  }

  /**
   * Generate a unique cache key based on preference values
   */
  private generateCacheKey(prefs: UserPreferences): string {
    const keyParts = [
      prefs.userId,
      prefs.technicalBackground,
      prefs.domainKnowledge,
      prefs.preferredDepth,
      prefs.codeExamplesImportance,
      prefs.focusArea,
      prefs.timeCommitment,
      prefs.learningGoal,
      prefs.languagePreference,
    ];

    return keyParts.join(':');
  }

  /**
   * Get prompt template name based on complexity level
   */
  getPromptTemplateName(complexityLevel: ComplexityLevel): string {
    return `${complexityLevel}.txt`;
  }

  /**
   * Check if profile requires content adaptation
   */
  requiresAdaptation(profile: PersonalizationProfile): boolean {
    return (
      profile.complexityLevel !== 'balanced' ||
      !profile.includeCodeExamples ||
      profile.hideAdvancedMath ||
      profile.showConceptPrimers ||
      profile.languagePreference !== 'en'
    );
  }

  /**
   * Get priority score for content matching
   * Used for recommendations - higher score = better match
   */
  getTopicMatchScore(profile: PersonalizationProfile, contentTopics: string[]): number {
    const focusAreaTopics: Record<string, string[]> = {
      'ros2': ['ros2', 'robot operating system', 'nodes', 'topics', 'services'],
      'simulation': ['gazebo', 'simulation', 'physics', 'world', 'urdf'],
      'computer-vision': ['opencv', 'vision', 'camera', 'image', 'detection'],
      'vla-models': ['vla', 'vision-language', 'transformer', 'model', 'ai'],
      'general': [], // Matches everything equally
    };

    const userTopics = focusAreaTopics[profile.focusArea];

    if (userTopics.length === 0) {
      return 0.5; // Neutral score for general focus
    }

    const matchCount = contentTopics.filter(topic =>
      userTopics.some(ut => topic.toLowerCase().includes(ut))
    ).length;

    return Math.min(1, matchCount / Math.max(1, userTopics.length));
  }

  /**
   * Get difficulty match score
   */
  getDifficultyMatchScore(
    profile: PersonalizationProfile,
    contentDifficulty: 'beginner' | 'intermediate' | 'advanced'
  ): number {
    const complexityToDifficulty: Record<ComplexityLevel, string[]> = {
      'simple': ['beginner'],
      'balanced': ['beginner', 'intermediate'],
      'technical': ['intermediate', 'advanced'],
    };

    const matchingDifficulties = complexityToDifficulty[profile.complexityLevel];

    return matchingDifficulties.includes(contentDifficulty) ? 1 : 0.3;
  }
}

// Singleton instance
export const profileComputer = new ProfileComputer();

// Export class for testing
export { ProfileComputer, COMPLEXITY_MATRIX, WORD_LIMITS, PATH_TYPE_MAPPING };
