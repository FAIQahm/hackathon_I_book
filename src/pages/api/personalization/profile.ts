/**
 * API Route: /api/personalization/profile
 * Returns computed personalization profile for authenticated user
 *
 * GET /api/personalization/profile?userId=<userId>
 *
 * Response: PersonalizationApiResponse<PersonalizationProfile>
 */

import type { NextApiRequest, NextApiResponse } from 'next';
import { personalizationService } from '../../../services/personalization';
import { PersonalizationProfile, PersonalizationApiResponse } from '../../../types/personalization';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse<PersonalizationApiResponse<PersonalizationProfile>>
) {
  // Only allow GET requests
  if (req.method !== 'GET') {
    return res.status(405).json({
      success: false,
      error: 'Method not allowed',
      cached: false,
      responseTime: 0,
    });
  }

  const startTime = Date.now();

  try {
    // Get userId from query or session
    // In production, this should come from authenticated session
    const userId = req.query.userId as string;

    if (!userId) {
      // FR-063: Return fallback for unauthenticated users
      const fallbackProfile = personalizationService.getFallbackProfile();

      return res.status(200).json({
        success: true,
        data: fallbackProfile,
        cached: false,
        responseTime: Date.now() - startTime,
      });
    }

    // Get personalized profile
    const result = await personalizationService.getProfile(userId);

    // FR-065: Check response time (4 second threshold)
    if (result.responseTime > 4000) {
      console.warn(`[API] Profile response exceeded 4s threshold: ${result.responseTime}ms`);
    }

    // FR-066: Log cache performance (1 second for cached)
    if (result.cached && result.responseTime > 1000) {
      console.warn(`[API] Cached response exceeded 1s threshold: ${result.responseTime}ms`);
    }

    return res.status(result.success ? 200 : 500).json(result);
  } catch (error) {
    console.error('[API] Profile endpoint error:', error);

    // FR-069: Graceful degradation - return fallback
    const fallbackProfile = personalizationService.getFallbackProfile();

    return res.status(200).json({
      success: true,
      data: fallbackProfile,
      error: 'Fallback used due to service error',
      cached: false,
      responseTime: Date.now() - startTime,
    });
  }
}
