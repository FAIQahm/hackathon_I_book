/**
 * Response Adapter
 * Adapts chatbot responses based on user personalization profile
 * FR-055, FR-056, FR-057: Chatbot response adaptation
 */

import {
  PersonalizationProfile,
  ResponseAdaptation,
  PersonalizedResponse,
  ComplexityLevel,
} from '../types/personalization';
import { profileComputer } from './profileComputer';
import { personalizationCache } from '../cache/personalizationCache';

/**
 * Prompt templates for different complexity levels
 */
const PROMPT_TEMPLATES: Record<ComplexityLevel, string> = {
  simple: `You are a friendly AI tutor explaining Physical AI and robotics concepts to a beginner.

GUIDELINES:
- Use simple, everyday language - avoid jargon
- Include analogies and real-world examples
- Break down complex concepts into small, digestible pieces
- Define any technical terms you must use
- Be encouraging and supportive
- Keep explanations concise but thorough

WORD LIMIT: {{wordLimit}} words maximum

USER QUESTION: {{question}}

Provide a beginner-friendly response:`,

  balanced: `You are an AI tutor explaining Physical AI and robotics concepts.

GUIDELINES:
- Use standard technical language with brief explanations when needed
- Balance theory with practical examples
- Assume basic programming knowledge
- Include relevant context without over-explaining
- Be clear and informative

WORD LIMIT: {{wordLimit}} words maximum

{{codeInstructions}}

USER QUESTION: {{question}}

Provide a balanced, informative response:`,

  technical: `You are an expert AI assistant discussing Physical AI and robotics with an experienced developer.

GUIDELINES:
- Use precise technical terminology freely
- Include edge cases, nuances, and advanced considerations
- Reference relevant APIs, libraries, and best practices
- Assume strong prerequisite knowledge
- Be thorough and technically accurate

WORD LIMIT: {{wordLimit}} words maximum

{{codeInstructions}}

USER QUESTION: {{question}}

Provide a technically detailed response:`,
};

/**
 * Code example instructions based on user preference
 */
const CODE_INSTRUCTIONS = {
  include: 'Include relevant code examples to illustrate concepts.',
  exclude: 'Focus on conceptual explanation without code examples.',
};

class ResponseAdapter {
  /**
   * Get adaptation settings for a user's profile
   */
  getAdaptation(profile: PersonalizationProfile): ResponseAdaptation {
    const templateName = profileComputer.getPromptTemplateName(profile.complexityLevel);

    return {
      complexityLevel: profile.complexityLevel,
      wordLimit: profile.responseWordLimit,
      includeCodeExamples: profile.includeCodeExamples,
      languagePreference: profile.languagePreference,
      promptTemplate: templateName,
    };
  }

  /**
   * Build adapted prompt for chatbot
   * FR-055, FR-056, FR-057
   */
  buildAdaptedPrompt(profile: PersonalizationProfile, userQuestion: string): string {
    const template = PROMPT_TEMPLATES[profile.complexityLevel];

    const codeInstructions = profile.includeCodeExamples
      ? CODE_INSTRUCTIONS.include
      : CODE_INSTRUCTIONS.exclude;

    let prompt = template
      .replace('{{wordLimit}}', profile.responseWordLimit.toString())
      .replace('{{question}}', userQuestion)
      .replace('{{codeInstructions}}', codeInstructions);

    // Add language instruction for non-English preferences
    if (profile.languagePreference !== 'en') {
      const languageNames: Record<string, string> = {
        ur: 'Urdu',
      };

      prompt += `\n\nIMPORTANT: Respond in ${languageNames[profile.languagePreference] || profile.languagePreference}.`;
    }

    return prompt;
  }

  /**
   * Create personalized response record
   */
  createPersonalizedResponse(
    originalQuery: string,
    adaptedResponse: string,
    profile: PersonalizationProfile,
    responseTime: number
  ): PersonalizedResponse {
    const adaptations: string[] = [];

    // Record which adaptations were applied
    adaptations.push(`complexity:${profile.complexityLevel}`);
    adaptations.push(`wordLimit:${profile.responseWordLimit}`);

    if (!profile.includeCodeExamples) {
      adaptations.push('codeExamples:excluded');
    }

    if (profile.languagePreference !== 'en') {
      adaptations.push(`language:${profile.languagePreference}`);
    }

    return {
      originalQuery,
      adaptedResponse,
      adaptationsApplied: adaptations,
      profile,
      responseTime,
    };
  }

  /**
   * Cache a personalized response
   */
  cacheResponse(userId: string, queryHash: string, response: PersonalizedResponse): void {
    const cacheKey = personalizationCache.responseKey(userId, queryHash);
    personalizationCache.set(cacheKey, response);
  }

  /**
   * Get cached response if available
   */
  getCachedResponse(userId: string, queryHash: string): PersonalizedResponse | null {
    const cacheKey = personalizationCache.responseKey(userId, queryHash);
    return personalizationCache.get<PersonalizedResponse>(cacheKey);
  }

  /**
   * Generate hash for query (for cache key)
   */
  hashQuery(query: string): string {
    // Simple hash function for demo - use crypto in production
    let hash = 0;
    for (let i = 0; i < query.length; i++) {
      const char = query.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash; // Convert to 32bit integer
    }
    return Math.abs(hash).toString(16);
  }

  /**
   * Get prompt template content by name
   */
  getPromptTemplate(complexityLevel: ComplexityLevel): string {
    return PROMPT_TEMPLATES[complexityLevel];
  }

  /**
   * Validate that response meets word limit
   */
  validateResponseLength(response: string, wordLimit: number): {
    valid: boolean;
    wordCount: number;
    exceededBy: number;
  } {
    const wordCount = response.split(/\s+/).filter(word => word.length > 0).length;
    const exceededBy = Math.max(0, wordCount - wordLimit);

    return {
      valid: wordCount <= wordLimit * 1.2, // Allow 20% tolerance
      wordCount,
      exceededBy,
    };
  }
}

// Singleton instance
export const responseAdapter = new ResponseAdapter();

// Export class and templates for testing
export { ResponseAdapter, PROMPT_TEMPLATES, CODE_INSTRUCTIONS };
