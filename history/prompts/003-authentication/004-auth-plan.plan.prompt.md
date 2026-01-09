---
id: 004
title: Authentication Technical Plan Creation
stage: plan
date: 2025-12-31
surface: agent
model: claude-opus-4-5-20251101
feature: 003-authentication
branch: 003-authentication
user: PMLS
command: /sp.plan 003-authentication
labels: ["plan", "architecture", "authentication", "better-auth", "personalization"]
links:
  spec: specs/003-authentication/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-authentication/plan.md
  - specs/003-authentication/research.md
  - specs/003-authentication/data-model.md
  - specs/003-authentication/contracts/openapi.yaml
  - specs/003-authentication/quickstart.md
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan 003-authentication

Execute the implementation planning workflow for the User Authentication & Personalization feature using Better-Auth library with email/password registration, 8-10 onboarding questions, and preference management.

## Response snapshot

Created comprehensive technical plan for 003-authentication feature with complete Phase 1 artifacts.

**Architecture**: 3-layer design
- Frontend: Docusaurus with React components (SignupForm, LoginForm, OnboardingWizard, ProfileSettings)
- Backend: FastAPI with Better-Auth (AuthService, PreferencesService)
- External Services: Neon Postgres (users, preferences, sessions), Email Service (SMTP)

**Implementation Phases**: 6 phases
1. Better-Auth Integration (register, login, logout, session management)
2. Password Security & Recovery (bcrypt, reset flow, account lockout)
3. Onboarding Questions (10 preference dimensions, wizard steps)
4. Frontend Components (SignupForm, LoginForm, OnboardingWizard, ProfileSettings)
5. Preference Caching (5-min TTL, cache invalidation)
6. Security Hardening (HTTPS, CORS, rate limiting, audit logging)

**Technology Decisions** (research.md):
- Auth Framework: Better-Auth (per constitution)
- Password Hashing: bcrypt with cost factor 12
- Session Management: JWT in HttpOnly cookies
- Database: Neon Serverless Postgres
- Onboarding UX: Multi-step wizard (3 steps, 10 questions)
- Account Lockout: 5 attempts → 15-minute lockout
- Email: Standard SMTP
- Caching: In-memory with 5-minute TTL

**Data Model** (data-model.md):
- 5 entities: User, UserPreference, Session, PasswordResetToken, OnboardingProgress
- SQL schema for Neon Postgres
- Pydantic models (Python) and TypeScript interfaces (Frontend)
- 10 preference dimensions with enums and valid values

**API Contract** (openapi.yaml):
- POST /api/auth/register - Create account
- POST /api/auth/login - Authenticate
- POST /api/auth/logout - End session
- GET /api/auth/me - Current user
- POST /api/auth/password/reset-request - Request reset
- POST /api/auth/password/reset-confirm - Confirm reset
- POST /api/preferences/onboarding - Submit all preferences
- GET /api/preferences - Get preferences
- PATCH /api/preferences - Update preferences
- GET/PUT /api/preferences/progress - Onboarding progress

**Design Decisions**: 6 key decisions
- D1: Better-Auth for all authentication
- D2: bcrypt with cost factor 12
- D3: JWT tokens with HttpOnly cookies
- D4: Multi-step onboarding wizard with progress saving
- D5: In-memory preference cache with 5-min TTL
- D6: 5-attempt lockout for 15 minutes

## Outcome

- Impact: Complete technical blueprint for authentication and personalization implementation
- Tests: N/A (planning phase)
- Files: plan.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md created
- Next prompts: /sp.tasks to generate actionable task list
- Reflection: Plan addresses all success criteria (SC-024 to SC-033) with clear mappings

## Evaluation notes (flywheel)

- Failure modes observed: Setup script pointed to wrong feature (002 instead of 003), manually corrected paths
- Graders run and results (PASS/FAIL): N/A (planning)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
