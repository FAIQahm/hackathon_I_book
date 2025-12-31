/**
 * API Route: /api/personalization/learning-path
 * Returns personalized learning path
 *
 * GET /api/personalization/learning-path?userId=<userId>
 */

import type { NextApiRequest, NextApiResponse } from 'next';
import { recommendationEngine } from '../../../services/recommendationEngine';
import { LearningPath, PersonalizationApiResponse } from '../../../types/personalization';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse<PersonalizationApiResponse<LearningPath>>
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

    if (!userId) {
      return res.status(400).json({
        success: false,
        error: 'userId is required',
        cached: false,
        responseTime: Date.now() - startTime,
      });
    }

    const learningPath = await recommendationEngine.generateLearningPath(userId);

    return res.status(200).json({
      success: true,
      data: learningPath,
      cached: false,
      responseTime: Date.now() - startTime,
    });
  } catch (error) {
    console.error('[API] Learning path endpoint error:', error);

    return res.status(500).json({
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error',
      cached: false,
      responseTime: Date.now() - startTime,
    });
  }
}
