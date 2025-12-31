/**
 * API Route: /api/personalization/recommendations
 * Returns personalized content recommendations
 *
 * GET /api/personalization/recommendations?userId=<userId>&limit=<number>
 */

import type { NextApiRequest, NextApiResponse } from 'next';
import { recommendationEngine } from '../../../services/recommendationEngine';
import { personalizationService } from '../../../services/personalization';
import { RecommendationResult, PersonalizationApiResponse } from '../../../types/personalization';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse<PersonalizationApiResponse<RecommendationResult>>
) {
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
    const userId = req.query.userId as string;
    const limit = parseInt(req.query.limit as string) || 5;

    if (!userId) {
      return res.status(400).json({
        success: false,
        error: 'userId is required',
        cached: false,
        responseTime: Date.now() - startTime,
      });
    }

    const recommendations = await recommendationEngine.getRecommendations(userId, limit);

    return res.status(200).json({
      success: true,
      data: recommendations,
      cached: false, // Cache check happens inside the engine
      responseTime: Date.now() - startTime,
    });
  } catch (error) {
    console.error('[API] Recommendations endpoint error:', error);

    return res.status(500).json({
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error',
      cached: false,
      responseTime: Date.now() - startTime,
    });
  }
}
