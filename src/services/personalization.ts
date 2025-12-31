/**
 * Personalization Service
 * Main orchestrator for all personalization functionality
 * Coordinates ProfileComputer, ResponseAdapter, RecommendEngine, etc.
 */

import {
  PersonalizationProfile,
  UserPreferences,
  ContentRecommendation,
  RecommendationResult,
  LearningPath,
  PersonalizedResponse,
  PersonalizedChapterView,
  ContentModification,
  PersonalizationApiResponse,
} from '../types/personalization';
import { profileComputer } from './profileComputer';
import { preferenceReader } from './preferenceReader';
import { personalizationCache } from '../cache/personalizationCache';

class PersonalizationService {
  /**
   * Get computed personalization profile for a user
   * FR-067: Supports all 7 dimensions simultaneously
   */
  async getProfile(userId: string): Promise<PersonalizationApiResponse<PersonalizationProfile>> {
    const startTime = Date.now();

    try {
      const profile = await profileComputer.computeProfile(userId);

      return {
        success: true,
        data: profile,
        cached: personalizationCache.has(personalizationCache.profileKey(userId)),
        responseTime: Date.now() - startTime,
      };
    } catch (error) {
      console.error('[PersonalizationService] Error getting profile:', error);

      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        cached: false,
        responseTime: Date.now() - startTime,
      };
    }
  }

  /**
   * Check if user has preferences set
   * Used to determine if onboarding modal should be shown (FR-073)
   */
  async hasPreferences(userId: string): Promise<boolean> {
    return preferenceReader.hasCompletePreferences(userId);
  }

  /**
   * Get missing preference fields for a user
   */
  async getMissingPreferences(userId: string): Promise<string[]> {
    return preferenceReader.getMissingFields(userId);
  }

  /**
   * Get personalized chapter view (FR-070 to FR-073)
   * Returns content modifications based on user profile
   */
  async getPersonalizedChapterView(
    userId: string,
    chapterId: string,
    sectionMetadata: Array<{
      sectionId: string;
      difficulty: 'beginner' | 'intermediate' | 'advanced';
      contentTypes: string[];
      requiresPrimer: boolean;
    }>
  ): Promise<PersonalizationApiResponse<PersonalizedChapterView>> {
    const startTime = Date.now();

    try {
      // Check if user has preferences (FR-073)
      const hasPrefs = await this.hasPreferences(userId);

      if (!hasPrefs) {
        return {
          success: false,
          error: 'PREFERENCES_REQUIRED',
          cached: false,
          responseTime: Date.now() - startTime,
        };
      }

      const profile = await profileComputer.computeProfile(userId);
      const modifications: ContentModification[] = [];

      // Process each section based on profile
      for (const section of sectionMetadata) {
        // FR-071: Collapse advanced content for beginners
        if (profile.hideAdvancedMath && section.contentTypes.includes('math')) {
          modifications.push({
            sectionId: section.sectionId,
            modificationType: 'collapse',
            reason: 'Advanced math content collapsed for beginner profile',
          });
        }

        if (profile.complexityLevel === 'simple' && section.difficulty === 'advanced') {
          modifications.push({
            sectionId: section.sectionId,
            modificationType: 'collapse',
            reason: 'Advanced section collapsed for simple complexity profile',
          });
        }

        // FR-072: Prepend concept primers for beginners
        if (profile.showConceptPrimers && section.requiresPrimer) {
          modifications.push({
            sectionId: section.sectionId,
            modificationType: 'prepend-primer',
            reason: 'Concept primer added for beginner/novice profile',
          });
        }
      }

      const chapterView: PersonalizedChapterView = {
        chapterId,
        userId,
        modifications,
        appliedAt: new Date(),
      };

      return {
        success: true,
        data: chapterView,
        cached: false,
        responseTime: Date.now() - startTime,
      };
    } catch (error) {
      console.error('[PersonalizationService] Error getting chapter view:', error);

      return {
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error',
        cached: false,
        responseTime: Date.now() - startTime,
      };
    }
  }

  /**
   * Invalidate cache when user preferences are updated
   * FR-068: Next request applies new preferences
   */
  async onPreferencesUpdated(userId: string): Promise<void> {
    personalizationCache.invalidateUser(userId);
  }

  /**
   * Get fallback (unpersonalized) profile
   * FR-063, FR-069: Graceful degradation
   */
  getFallbackProfile(): PersonalizationProfile {
    return {
      userId: 'guest',
      complexityLevel: 'balanced',
      includeCodeExamples: true,
      responseWordLimit: 250,
      pathType: 'balanced',
      focusArea: 'general',
      learningGoal: 'skill-enhancement',
      languagePreference: 'en',
      hideAdvancedMath: false,
      showConceptPrimers: false,
      activeDimensions: [],
      computedAt: new Date(),
      cacheKey: 'fallback',
    };
  }

  /**
   * Check if personalization service is healthy
   */
  async healthCheck(): Promise<{ healthy: boolean; details: Record<string, unknown> }> {
    try {
      const cacheStats = personalizationCache.getStats();

      return {
        healthy: true,
        details: {
          cacheSize: cacheStats.size,
          timestamp: new Date().toISOString(),
        },
      };
    } catch (error) {
      return {
        healthy: false,
        details: {
          error: error instanceof Error ? error.message : 'Unknown error',
        },
      };
    }
  }

  /**
   * Get cache statistics
   */
  getCacheStats(): { size: number; keys: string[] } {
    return personalizationCache.getStats();
  }
}

// Singleton instance
export const personalizationService = new PersonalizationService();

// Export class for testing
export { PersonalizationService };
