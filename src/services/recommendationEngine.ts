/**
 * Recommendation Engine
 * Generates personalized content recommendations
 * FR-058, FR-059: Content recommendations based on user profile
 */

import {
  PersonalizationProfile,
  ContentRecommendation,
  RecommendationResult,
  LearningPath,
  LearningPathStep,
} from '../types/personalization';
import { profileComputer } from './profileComputer';
import { personalizationCache } from '../cache/personalizationCache';

/**
 * Content metadata interface (from 001-textbook-generation)
 */
interface ContentMetadata {
  id: string;
  type: 'chapter' | 'section';
  title: string;
  primaryTopic: string;
  topics: string[];
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  estimatedReadTime: number;
  contentTypes: string[];
  requiresPrimer: boolean;
}

/**
 * Book content structure (mock data - will integrate with 001-textbook-generation)
 */
const BOOK_CONTENT: ContentMetadata[] = [
  {
    id: 'chapter-1',
    type: 'chapter',
    title: 'Introduction to Physical AI & ROS 2',
    primaryTopic: 'ros2',
    topics: ['ros2', 'physical-ai', 'robotics', 'introduction'],
    difficulty: 'beginner',
    estimatedReadTime: 30,
    contentTypes: ['text', 'diagrams'],
    requiresPrimer: false,
  },
  {
    id: 'chapter-1-section-1',
    type: 'section',
    title: 'What is Physical AI?',
    primaryTopic: 'physical-ai',
    topics: ['physical-ai', 'ai', 'robotics', 'embodied-ai'],
    difficulty: 'beginner',
    estimatedReadTime: 10,
    contentTypes: ['text'],
    requiresPrimer: false,
  },
  {
    id: 'chapter-1-section-2',
    type: 'section',
    title: 'ROS 2 Architecture Overview',
    primaryTopic: 'ros2',
    topics: ['ros2', 'architecture', 'nodes', 'topics'],
    difficulty: 'intermediate',
    estimatedReadTime: 15,
    contentTypes: ['text', 'diagrams', 'code'],
    requiresPrimer: true,
  },
  {
    id: 'chapter-2',
    type: 'chapter',
    title: 'Simulation with Gazebo',
    primaryTopic: 'simulation',
    topics: ['gazebo', 'simulation', 'physics', 'urdf'],
    difficulty: 'intermediate',
    estimatedReadTime: 45,
    contentTypes: ['text', 'code', 'diagrams'],
    requiresPrimer: true,
  },
  {
    id: 'chapter-2-section-1',
    type: 'section',
    title: 'Setting up Gazebo',
    primaryTopic: 'simulation',
    topics: ['gazebo', 'setup', 'installation'],
    difficulty: 'beginner',
    estimatedReadTime: 15,
    contentTypes: ['text', 'code'],
    requiresPrimer: false,
  },
  {
    id: 'chapter-2-section-2',
    type: 'section',
    title: 'Physics Simulation Deep Dive',
    primaryTopic: 'simulation',
    topics: ['physics', 'simulation', 'dynamics', 'math'],
    difficulty: 'advanced',
    estimatedReadTime: 25,
    contentTypes: ['text', 'math', 'code'],
    requiresPrimer: true,
  },
  {
    id: 'chapter-3',
    type: 'chapter',
    title: 'Vision-Language-Action Models',
    primaryTopic: 'vla-models',
    topics: ['vla', 'vision', 'language', 'transformer', 'ai'],
    difficulty: 'advanced',
    estimatedReadTime: 60,
    contentTypes: ['text', 'math', 'code', 'diagrams'],
    requiresPrimer: true,
  },
  {
    id: 'chapter-3-section-1',
    type: 'section',
    title: 'Introduction to VLA Models',
    primaryTopic: 'vla-models',
    topics: ['vla', 'introduction', 'multimodal'],
    difficulty: 'intermediate',
    estimatedReadTime: 20,
    contentTypes: ['text', 'diagrams'],
    requiresPrimer: true,
  },
  {
    id: 'chapter-3-section-2',
    type: 'section',
    title: 'Training VLA Models',
    primaryTopic: 'vla-models',
    topics: ['training', 'vla', 'deep-learning', 'transformer'],
    difficulty: 'advanced',
    estimatedReadTime: 30,
    contentTypes: ['text', 'math', 'code'],
    requiresPrimer: true,
  },
];

/**
 * Relevance scoring weights
 */
const SCORING_WEIGHTS = {
  topicMatch: 0.4,
  difficultyMatch: 0.3,
  goalAlignment: 0.3,
};

class RecommendationEngine {
  /**
   * Generate content recommendations for a user
   * FR-058, FR-059
   */
  async getRecommendations(
    userId: string,
    limit: number = 5
  ): Promise<RecommendationResult> {
    // Check cache first
    const cacheKey = personalizationCache.recommendationKey(userId);
    const cached = personalizationCache.get<RecommendationResult>(cacheKey);

    if (cached) {
      return cached;
    }

    // Get user profile
    const profile = await profileComputer.computeProfile(userId);

    // Score and rank all content
    const scoredContent = BOOK_CONTENT.map(content => ({
      content,
      score: this.calculateRelevanceScore(profile, content),
    }));

    // Sort by score descending
    scoredContent.sort((a, b) => b.score - a.score);

    // Take top N recommendations
    const recommendations: ContentRecommendation[] = scoredContent
      .slice(0, limit)
      .map(({ content, score }) => ({
        contentId: content.id,
        contentType: content.type,
        title: content.title,
        relevanceScore: score,
        reason: this.generateRecommendationReason(profile, content, score),
        estimatedReadTime: content.estimatedReadTime,
        difficulty: content.difficulty,
        topics: content.topics,
      }));

    const result: RecommendationResult = {
      userId,
      recommendations,
      generatedAt: new Date(),
      basedOnProfile: profile.cacheKey,
    };

    // Cache the result
    personalizationCache.set(cacheKey, result);

    return result;
  }

  /**
   * Calculate relevance score for content
   * Score = (topic_match * 0.4) + (difficulty_match * 0.3) + (goal_alignment * 0.3)
   */
  calculateRelevanceScore(
    profile: PersonalizationProfile,
    content: ContentMetadata
  ): number {
    const topicScore = profileComputer.getTopicMatchScore(profile, content.topics);
    const difficultyScore = profileComputer.getDifficultyMatchScore(
      profile,
      content.difficulty
    );
    const goalScore = this.calculateGoalAlignment(profile, content);

    return (
      topicScore * SCORING_WEIGHTS.topicMatch +
      difficultyScore * SCORING_WEIGHTS.difficultyMatch +
      goalScore * SCORING_WEIGHTS.goalAlignment
    );
  }

  /**
   * Calculate goal alignment score
   */
  private calculateGoalAlignment(
    profile: PersonalizationProfile,
    content: ContentMetadata
  ): number {
    const goalContentTypes: Record<string, string[]> = {
      hobby: ['text', 'diagrams'], // Fun, visual content
      'career-change': ['code', 'text'], // Practical, job-relevant
      'skill-enhancement': ['code', 'text', 'diagrams'], // Balanced
      academic: ['math', 'text'], // Theoretical
    };

    const preferredTypes = goalContentTypes[profile.learningGoal] || [];
    const matchCount = content.contentTypes.filter(type =>
      preferredTypes.includes(type)
    ).length;

    return matchCount / Math.max(1, preferredTypes.length);
  }

  /**
   * Generate human-readable recommendation reason
   */
  private generateRecommendationReason(
    profile: PersonalizationProfile,
    content: ContentMetadata,
    score: number
  ): string {
    const reasons: string[] = [];

    // Topic match reason
    if (profile.focusArea !== 'general' && content.primaryTopic === profile.focusArea) {
      reasons.push(`Matches your focus on ${profile.focusArea}`);
    }

    // Difficulty match reason
    if (
      (profile.complexityLevel === 'simple' && content.difficulty === 'beginner') ||
      (profile.complexityLevel === 'technical' && content.difficulty === 'advanced')
    ) {
      reasons.push('Appropriate difficulty for your level');
    }

    // Time-based reason
    if (profile.pathType === 'condensed' && content.estimatedReadTime <= 15) {
      reasons.push('Quick read for your schedule');
    }

    if (reasons.length === 0) {
      reasons.push('Recommended based on your learning profile');
    }

    return reasons.join('. ');
  }

  /**
   * Generate learning path for a user
   * FR-060
   */
  async generateLearningPath(userId: string): Promise<LearningPath> {
    // Check cache
    const cacheKey = personalizationCache.learningPathKey(userId);
    const cached = personalizationCache.get<LearningPath>(cacheKey);

    if (cached) {
      return cached;
    }

    const profile = await profileComputer.computeProfile(userId);

    // Filter content based on path type
    const filteredContent = this.filterContentForPath(profile);

    // Order content logically
    const orderedContent = this.orderContentForLearning(filteredContent, profile);

    // Create path steps
    const steps: LearningPathStep[] = orderedContent.map((content, index) => ({
      order: index + 1,
      contentId: content.id,
      contentType: content.type,
      title: content.title,
      estimatedTime: content.estimatedReadTime,
      isOptional: this.isOptionalForPath(content, profile),
      reason: this.getStepReason(content, profile),
    }));

    const totalTime = steps.reduce((sum, step) => sum + step.estimatedTime, 0);

    const path: LearningPath = {
      userId,
      pathType: profile.pathType,
      steps,
      totalEstimatedTime: totalTime,
      generatedAt: new Date(),
    };

    // Cache the result
    personalizationCache.set(cacheKey, path);

    return path;
  }

  /**
   * Filter content based on path type
   * Condensed: <60% of content
   * Balanced: All core content
   * Comprehensive: Everything
   */
  private filterContentForPath(profile: PersonalizationProfile): ContentMetadata[] {
    switch (profile.pathType) {
      case 'condensed':
        // Only beginner/intermediate, chapters only, short reads
        return BOOK_CONTENT.filter(
          c =>
            c.type === 'chapter' ||
            (c.difficulty !== 'advanced' && c.estimatedReadTime <= 20)
        );

      case 'balanced':
        // All chapters, most sections except advanced math
        return BOOK_CONTENT.filter(
          c => c.type === 'chapter' || !c.contentTypes.includes('math')
        );

      case 'comprehensive':
      default:
        return [...BOOK_CONTENT];
    }
  }

  /**
   * Order content for logical learning progression
   */
  private orderContentForLearning(
    content: ContentMetadata[],
    profile: PersonalizationProfile
  ): ContentMetadata[] {
    // Sort by: difficulty (easier first), then by focus area match, then by ID
    return content.sort((a, b) => {
      const difficultyOrder = { beginner: 0, intermediate: 1, advanced: 2 };
      const diffDiff = difficultyOrder[a.difficulty] - difficultyOrder[b.difficulty];

      if (diffDiff !== 0) return diffDiff;

      // Prioritize focus area
      const aMatchesFocus = a.primaryTopic === profile.focusArea ? -1 : 0;
      const bMatchesFocus = b.primaryTopic === profile.focusArea ? -1 : 0;

      if (aMatchesFocus !== bMatchesFocus) return aMatchesFocus - bMatchesFocus;

      return a.id.localeCompare(b.id);
    });
  }

  /**
   * Determine if content is optional for the path
   */
  private isOptionalForPath(
    content: ContentMetadata,
    profile: PersonalizationProfile
  ): boolean {
    if (profile.pathType === 'condensed') {
      return content.type === 'section' || content.difficulty === 'advanced';
    }

    if (profile.pathType === 'balanced') {
      return content.difficulty === 'advanced' && content.contentTypes.includes('math');
    }

    return false;
  }

  /**
   * Get reason for including step in path
   */
  private getStepReason(content: ContentMetadata, profile: PersonalizationProfile): string {
    if (content.type === 'chapter') {
      return 'Core chapter content';
    }

    if (content.primaryTopic === profile.focusArea) {
      return `Directly relevant to your ${profile.focusArea} focus`;
    }

    if (content.difficulty === 'beginner') {
      return 'Foundation building';
    }

    return 'Recommended for comprehensive understanding';
  }

  /**
   * Get content metadata by ID
   */
  getContentById(contentId: string): ContentMetadata | undefined {
    return BOOK_CONTENT.find(c => c.id === contentId);
  }

  /**
   * Get all content metadata
   */
  getAllContent(): ContentMetadata[] {
    return [...BOOK_CONTENT];
  }
}

// Singleton instance
export const recommendationEngine = new RecommendationEngine();

// Export class for testing
export { RecommendationEngine, BOOK_CONTENT, SCORING_WEIGHTS };
