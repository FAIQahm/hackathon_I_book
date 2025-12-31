/**
 * Personalization Types
 * Defines the 7-dimension personalization model for content adaptation
 */

// ===========================================
// User Preference Dimensions (7 Required)
// ===========================================

export type TechnicalBackground = 'beginner' | 'intermediate' | 'advanced';
export type DomainKnowledge = 'none' | 'some' | 'experienced';
export type PreferredDepth = 'overview' | 'balanced' | 'deep-dive';
export type CodeExamplesImportance = 'essential' | 'helpful' | 'minimal';
export type FocusArea = 'ros2' | 'simulation' | 'computer-vision' | 'vla-models' | 'general';
export type TimeCommitment = 'minimal' | 'moderate' | 'dedicated'; // <2h, 2-5h, 5+h weekly
export type LearningGoal = 'hobby' | 'career-change' | 'skill-enhancement' | 'academic';
export type LanguagePreference = 'en' | 'ur';

/**
 * Raw user preferences as stored in the auth database
 */
export interface UserPreferences {
  userId: string;
  technicalBackground: TechnicalBackground;
  domainKnowledge: DomainKnowledge;
  preferredDepth: PreferredDepth;
  codeExamplesImportance: CodeExamplesImportance;
  focusArea: FocusArea;
  timeCommitment: TimeCommitment;
  learningGoal: LearningGoal;
  languagePreference: LanguagePreference;
  updatedAt: Date;
}

// ===========================================
// Computed Personalization Profile
// ===========================================

export type ComplexityLevel = 'simple' | 'balanced' | 'technical';
export type PathType = 'condensed' | 'balanced' | 'comprehensive';

/**
 * Computed profile derived from user preferences
 * Used for content adaptation decisions
 */
export interface PersonalizationProfile {
  userId: string;

  // Computed values
  complexityLevel: ComplexityLevel;
  includeCodeExamples: boolean;
  responseWordLimit: number; // Overview: 150, Balanced: 250, Deep-dive: 400+
  pathType: PathType;

  // Direct mappings
  focusArea: FocusArea;
  learningGoal: LearningGoal;
  languagePreference: LanguagePreference;

  // Content filtering
  hideAdvancedMath: boolean;
  showConceptPrimers: boolean;

  // Metadata
  activeDimensions: string[];
  computedAt: Date;
  cacheKey: string;
}

// ===========================================
// Content Recommendation
// ===========================================

export interface ContentRecommendation {
  contentId: string;
  contentType: 'chapter' | 'section' | 'topic';
  title: string;
  relevanceScore: number; // 0-1
  reason: string;
  estimatedReadTime: number; // minutes
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  topics: string[];
}

export interface RecommendationResult {
  userId: string;
  recommendations: ContentRecommendation[];
  generatedAt: Date;
  basedOnProfile: string; // cacheKey reference
}

// ===========================================
// Learning Path
// ===========================================

export interface LearningPathStep {
  order: number;
  contentId: string;
  contentType: 'chapter' | 'section';
  title: string;
  estimatedTime: number; // minutes
  isOptional: boolean;
  reason: string;
}

export interface LearningPath {
  userId: string;
  pathType: PathType;
  steps: LearningPathStep[];
  totalEstimatedTime: number; // minutes
  generatedAt: Date;
}

// ===========================================
// Chatbot Response Adaptation
// ===========================================

export interface ResponseAdaptation {
  complexityLevel: ComplexityLevel;
  wordLimit: number;
  includeCodeExamples: boolean;
  languagePreference: LanguagePreference;
  promptTemplate: string;
}

export interface PersonalizedResponse {
  originalQuery: string;
  adaptedResponse: string;
  adaptationsApplied: string[];
  profile: PersonalizationProfile;
  responseTime: number; // ms
}

// ===========================================
// Chapter Personalization (FR-070 to FR-073)
// ===========================================

export interface ContentModification {
  sectionId: string;
  modificationType: 'collapse' | 'expand' | 'prepend-primer' | 'hide';
  reason: string;
}

export interface PersonalizedChapterView {
  chapterId: string;
  userId: string;
  modifications: ContentModification[];
  primerContent?: string;
  appliedAt: Date;
}

// ===========================================
// API Response Types
// ===========================================

export interface PersonalizationApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
  cached: boolean;
  responseTime: number;
}

// ===========================================
// Default Preferences
// ===========================================

export const DEFAULT_PREFERENCES: Omit<UserPreferences, 'userId' | 'updatedAt'> = {
  technicalBackground: 'beginner',
  domainKnowledge: 'none',
  preferredDepth: 'balanced',
  codeExamplesImportance: 'helpful',
  focusArea: 'general',
  timeCommitment: 'moderate',
  learningGoal: 'skill-enhancement',
  languagePreference: 'en',
};
