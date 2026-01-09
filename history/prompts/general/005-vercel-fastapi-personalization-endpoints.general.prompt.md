---
id: 005
title: Add Personalization Endpoints to vercel-fastapi-link Skill
stage: general
date: 2026-01-01
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: developer
command: skill enhancement
labels: ["skill", "fastapi", "personalization", "api", "endpoints"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/skills/vercel-fastapi-link/scripts/setup.sh
  - .claude/skills/vercel-fastapi-link/skill.md
tests:
  - curl /api/personalization/profile
  - curl /api/personalization/recommendations
  - curl /api/personalization/learning-path
  - curl /api/personalization/apply
---

## Prompt

We need to iterate on the vercel-fastapi-link skill in .claude/skills/vercel-fastapi-link/SKILL.md. Please update the api/main.py template in the Bundled Resources to include these personalization endpoints:

/api/personalization/profile: For user data management.

/api/personalization/recommendations: For AI-driven content suggestions.

/api/personalization/learning-path: To track educational progress.

/api/personalization/apply: To save personalization settings.

Also, update the Verification Checklist in the skill to include testing these specific endpoints using curl or pytest.

## Response snapshot

Enhanced vercel-fastapi-link skill from v1.2.0 to v1.3.0 with personalization endpoints:

### New Pydantic Models Added:
- `UserPreferences` - Learning preferences (language, difficulty, topics, style)
- `UserProfile` - User profile with preferences
- `Recommendation` - Content recommendation with relevance score
- `RecommendationsResponse` - List of recommendations
- `LearningPathItem` - Single learning path item with progress
- `LearningPath` - Complete personalized learning path
- `PersonalizationSettings` - Settings to apply
- `PersonalizationApplyResponse` - Apply confirmation

### New Endpoints Added:
1. `GET /api/personalization/profile` - Get/create user profile
2. `POST /api/personalization/profile` - Update user profile
3. `GET /api/personalization/recommendations` - AI-driven content suggestions
4. `GET /api/personalization/learning-path` - Personalized learning path
5. `POST /api/personalization/apply` - Save personalization settings

### Updated Verification Checklist:
- Added curl examples for all personalization endpoints
- Added pytest test examples for all endpoints
- Organized into Basic Setup, Endpoint Testing, and Deployment sections

## Outcome

- ✅ Impact: Skill now includes complete personalization API for educational content
- 🧪 Tests: curl and pytest examples provided in verification checklist
- 📁 Files: 2 files modified (setup.sh, skill.md)
- 🔁 Next prompts: Re-apply skill to project, deploy to Vercel
- 🧠 Reflection: In-memory storage used for demo; production needs database integration

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A - skill template update
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add database integration (Neon/Postgres)
