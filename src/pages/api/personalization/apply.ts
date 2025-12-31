/**
 * API Route: /api/personalization/apply
 * Apply personalization to chapter content
 * FR-070, FR-071, FR-072, FR-073
 *
 * POST /api/personalization/apply
 * Body: { userId: string, chapterId: string, sections: SectionMetadata[] }
 */

import type { NextApiRequest, NextApiResponse } from 'next';
import { personalizationService } from '../../../services/personalization';
import { PersonalizedChapterView, PersonalizationApiResponse } from '../../../types/personalization';

interface ApplyRequest {
  userId: string;
  chapterId: string;
  sections: Array<{
    sectionId: string;
    difficulty: 'beginner' | 'intermediate' | 'advanced';
    contentTypes: string[];
    requiresPrimer: boolean;
  }>;
}

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse<PersonalizationApiResponse<PersonalizedChapterView>>
) {
  if (req.method !== 'POST') {
    return res.status(405).json({
      success: false,
      error: 'Method not allowed',
      cached: false,
      responseTime: 0,
    });
  }

  const startTime = Date.now();

  try {
    const { userId, chapterId, sections } = req.body as ApplyRequest;

    if (!userId || !chapterId || !sections) {
      return res.status(400).json({
        success: false,
        error: 'Missing required fields: userId, chapterId, sections',
        cached: false,
        responseTime: Date.now() - startTime,
      });
    }

    const result = await personalizationService.getPersonalizedChapterView(
      userId,
      chapterId,
      sections
    );

    // FR-073: Check if preferences are required
    if (!result.success && result.error === 'PREFERENCES_REQUIRED') {
      return res.status(200).json({
        success: false,
        error: 'PREFERENCES_REQUIRED',
        cached: false,
        responseTime: Date.now() - startTime,
      });
    }

    return res.status(result.success ? 200 : 500).json(result);
  } catch (error) {
    console.error('[API] Apply personalization endpoint error:', error);

    return res.status(500).json({
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error',
      cached: false,
      responseTime: Date.now() - startTime,
    });
  }
}
