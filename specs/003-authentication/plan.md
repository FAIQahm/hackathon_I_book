# Implementation Plan: User Authentication & Personalization

**Branch**: `003-authentication` | **Date**: 2025-12-31 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-authentication/spec.md`

## Summary

Implement user authentication using Better-Auth library with email/password registration, 8-10 onboarding questions for personalization preferences, session management, and password reset functionality. The system stores user preferences in Neon Postgres and delivers personalized content responses within 4 seconds.

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), Python 3.11 (backend API)
**Primary Dependencies**:
- Frontend: React, Docusaurus, Better-Auth Client
- Backend: FastAPI, Better-Auth, SQLAlchemy, Pydantic
**Storage**: Neon Serverless Postgres (users, preferences, sessions)
**Testing**: Vitest (frontend), pytest (backend), k6 (load testing)
**Target Platform**: Vercel (FastAPI serverless functions)
**Project Type**: Web application (frontend embedded in Docusaurus + backend API)
**Performance Goals**:
- <4s personalized response (p95)
- <1s cached response
- 100 concurrent authenticated users
**Constraints**:
- Password hashing with bcrypt/argon2
- HTTPS required for all auth requests
- 5-attempt lockout for 15 minutes
**Scale/Scope**: ~1000 users initially, 10 preference dimensions

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status |
|-----------|-------------|--------|
| Educational Excellence | Real-world examples, tested code | ✅ Auth flow demonstrates practical implementation |
| Technical Accuracy | No placeholder content | ✅ All auth endpoints fully functional |
| AI-Native Development | 4+ skills, 3+ agents | ✅ Using personalization via preferences |
| User-Centric Design | <4s response, accessible | ✅ SC-029, FR-051 personalized response |
| Quality Over Quantity | Secure, tested | ✅ Password hashing, lockout, HTTPS |

**Tech Stack Compliance**:
- ✅ Better-Auth (per constitution requirement)
- ✅ Neon Postgres (per constitution requirement)
- ✅ 8-10 signup questions (per constitution requirement)
- ✅ 5+ personalization dimensions (per constitution requirement)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         DOCUSAURUS FRONTEND                              │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ SignupForm   │  │ LoginForm    │  │ Onboarding   │  │ Profile      │ │
│  │ Component    │  │ Component    │  │ Wizard       │  │ Settings     │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
└─────────┼──────────────────┼──────────────────┼──────────────────┼───────┘
          │                  │                  │                  │
          ▼                  ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         AUTH API (FastAPI + Better-Auth)                 │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                      AuthService                                  │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ │   │
│  │  │ Registration│ │ Login       │ │ Session     │ │ Password    │ │   │
│  │  │ Handler     │ │ Handler     │ │ Manager     │ │ Reset       │ │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                  │                                       │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                      PreferencesService                           │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐                 │   │
│  │  │ Onboarding  │ │ Preference  │ │ Preference  │                 │   │
│  │  │ Collector   │ │ Storage     │ │ Cache       │                 │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘                 │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          EXTERNAL SERVICES                               │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                   │
│  │ Neon         │  │ Email        │  │ Better-Auth  │                   │
│  │ Postgres     │  │ Service      │  │ Library      │                   │
│  │ (users/prefs)│  │ (reset)      │  │ (auth logic) │                   │
│  └──────────────┘  └──────────────┘  └──────────────┘                   │
└─────────────────────────────────────────────────────────────────────────┘
```

## Implementation Phases

### Phase 1: Better-Auth Integration
**Goal**: Set up Better-Auth with email/password authentication

| Task | Description | Files |
|------|-------------|-------|
| 1.1 | Install and configure Better-Auth | `backend/src/auth/better_auth.py` |
| 1.2 | Create user registration endpoint | `backend/src/api/auth/register.py` |
| 1.3 | Create login endpoint | `backend/src/api/auth/login.py` |
| 1.4 | Implement session management | `backend/src/auth/session.py` |
| 1.5 | Create logout endpoint | `backend/src/api/auth/logout.py` |
| 1.6 | Create Vercel deployment configuration | `backend/vercel.json` |
| 1.7 | Create environment configuration | `backend/src/config.py` |

**Better-Auth Configuration**:
```python
# Password requirements
MIN_PASSWORD_LENGTH = 8
REQUIRE_UPPERCASE = True
REQUIRE_NUMBER = True

# Session settings
SESSION_EXPIRY_HOURS = 24
SESSION_COOKIE_SECURE = True
SESSION_COOKIE_HTTPONLY = True
```

**Environment Variables** (configured in Vercel dashboard):
```
# Required - Database
DATABASE_URL=postgresql://...@xxx.neon.tech/dbname?sslmode=require

# Required - Auth
JWT_SECRET=...
BETTER_AUTH_SECRET=...

# Required - Email (for password reset)
SMTP_HOST=...
SMTP_PORT=587
SMTP_USER=...
SMTP_PASSWORD=...
FROM_EMAIL=noreply@yourbook.com

# Optional - Configuration
SESSION_EXPIRY_HOURS=24
LOCKOUT_ATTEMPTS=5
LOCKOUT_DURATION_MINUTES=15
```

### Phase 2: Password Security & Recovery
**Goal**: Implement secure password handling and reset flow

| Task | Description | Files |
|------|-------------|-------|
| 2.1 | Implement password hashing (bcrypt/argon2) | `backend/src/auth/password.py` |
| 2.2 | Create password validation rules | `backend/src/auth/validators.py` |
| 2.3 | Implement password reset request | `backend/src/api/auth/reset_request.py` |
| 2.4 | Implement password reset confirmation | `backend/src/api/auth/reset_confirm.py` |
| 2.5 | Add account lockout logic | `backend/src/auth/lockout.py` |
| 2.6 | Create email service for reset links | `backend/src/services/email.py` |

**Password Validation Rules**:
```
- Minimum 8 characters
- At least 1 uppercase letter
- At least 1 number
- Reset links expire after 24 hours
- Account locks after 5 failed attempts for 15 minutes
```

### Phase 3: Onboarding Questions
**Goal**: Collect and store user preferences during signup

| Task | Description | Files |
|------|-------------|-------|
| 3.1 | Create onboarding questions schema | `backend/src/models/preferences.py` |
| 3.2 | Create preferences storage service | `backend/src/services/preferences.py` |
| 3.3 | Create onboarding endpoint | `backend/src/api/preferences/onboarding.py` |
| 3.4 | Create preference update endpoint | `backend/src/api/preferences/update.py` |
| 3.5 | Add partial progress saving | `backend/src/services/onboarding_progress.py` |

**Onboarding Questions (10 Required)**:
| # | Dimension | Question | Options |
|---|-----------|----------|---------|
| 1 | technical_background | Programming experience level? | Beginner/Intermediate/Advanced |
| 2 | domain_knowledge | Familiarity with robotics? | None/Some/Experienced |
| 3 | learning_goal | What do you want to achieve? | Career change/Skill enhancement/Academic research/Hobby |
| 4 | preferred_depth | Prefer overviews or details? | Overview/Balanced/Deep-dive |
| 5 | code_examples | Importance of code examples? | Not important/Somewhat/Very important |
| 6 | time_commitment | Weekly time available? | <2 hours/2-5 hours/5+ hours |
| 7 | focus_area | Most interesting topic? | ROS 2/Simulation/Computer Vision/Machine Learning |
| 8 | language_preference | Preferred language? | English/Urdu |
| 9 | prior_ai_experience | Worked with AI/ML before? | Yes/No/Learning |
| 10 | notification_preference | Email updates on new content? | Yes/No |

### Phase 4: Frontend Components
**Goal**: Build authentication UI in Docusaurus

| Task | Description | Files |
|------|-------------|-------|
| 4.1 | Create SignupForm component | `frontend/src/components/auth/SignupForm.tsx` |
| 4.2 | Create LoginForm component | `frontend/src/components/auth/LoginForm.tsx` |
| 4.3 | Create OnboardingWizard component | `frontend/src/components/auth/OnboardingWizard.tsx` |
| 4.4 | Create ProfileSettings component | `frontend/src/components/auth/ProfileSettings.tsx` |
| 4.5 | Create PasswordResetForm component | `frontend/src/components/auth/PasswordResetForm.tsx` |
| 4.6 | Add authentication context | `frontend/src/contexts/AuthContext.tsx` |
| 4.7 | Create protected route wrapper | `frontend/src/components/auth/ProtectedRoute.tsx` |

**Component Hierarchy**:
```
AuthProvider (Context)
├── SignupForm
│   └── OnboardingWizard (10 questions)
├── LoginForm
├── PasswordResetForm
└── ProfileSettings
    └── PreferencesEditor
```

### Phase 5: Preference Caching
**Goal**: Meet <1s cached response requirement

| Task | Description | Files |
|------|-------------|-------|
| 5.1 | Implement preference caching | `backend/src/cache/preference_cache.py` |
| 5.2 | Add cache invalidation on update | `backend/src/services/cache_invalidation.py` |
| 5.3 | Create preference retrieval endpoint | `backend/src/api/preferences/get.py` |
| 5.4 | Add performance monitoring | `backend/src/utils/metrics.py` |

**Cache Strategy**:
```
- Cache key: user:{user_id}:preferences
- TTL: 5 minutes
- Invalidation: On preference update
- Storage: In-memory (for serverless, use Redis if needed)
```

### Phase 6: Security Hardening
**Goal**: Ensure security compliance per constitution

| Task | Description | Files |
|------|-------------|-------|
| 6.1 | Add HTTPS enforcement | `backend/src/middleware/https.py` |
| 6.2 | Implement CORS configuration (MUST be first middleware) | `backend/src/middleware/cors.py` |
| 6.3 | Add rate limiting | `backend/src/middleware/rate_limit.py` |
| 6.4 | Create security audit logging | `backend/src/utils/audit_log.py` |
| 6.5 | Add input sanitization | `backend/src/utils/sanitize.py` |

**CORS Configuration** (Task 6.2 - Critical for Better-Auth):
```python
# backend/src/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config import settings

app = FastAPI()

# IMPORTANT: CORSMiddleware MUST be added FIRST before any other middleware
# This is required for Better-Auth credential passing to work correctly
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,  # From ALLOWED_ORIGINS env var
    allow_credentials=True,  # REQUIRED for Better-Auth session cookies
    allow_methods=["GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# Add other middleware AFTER CORSMiddleware
# app.add_middleware(HTTPSRedirectMiddleware)
# app.add_middleware(RateLimitMiddleware)
```

**Environment Variable** (add to config.py and Vercel dashboard):
```python
# backend/src/config.py
class Settings(BaseSettings):
    # CORS - comma-separated list of allowed origins
    allowed_origins: list[str] = Field(default_factory=lambda: [
        "http://localhost:3000",
        "http://localhost:5173",
    ])

    @field_validator("allowed_origins", mode="before")
    @classmethod
    def parse_origins(cls, v):
        if isinstance(v, str):
            return [origin.strip() for origin in v.split(",")]
        return v

# .env / Vercel Environment Variables
ALLOWED_ORIGINS=https://<org>.github.io,http://localhost:3000,http://localhost:5173
```

**Security Requirements**:
```
- All auth endpoints: HTTPS only
- CORS: CORSMiddleware FIRST, allow_credentials=True for Better-Auth
- CORS Origins: GitHub Pages URL + localhost for development
- Rate limiting: 10 requests/minute for auth endpoints
- Passwords: bcrypt with cost factor 12
- Sessions: HttpOnly, Secure, SameSite cookies
```

## Project Structure

### Documentation (this feature)

```text
specs/003-authentication/
├── plan.md              # This file
├── research.md          # Phase 0 research findings
├── data-model.md        # Entity definitions
├── quickstart.md        # Developer setup guide
├── contracts/           # API contracts
│   └── openapi.yaml     # REST API specification
└── tasks.md             # Generated by /sp.tasks
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── api/
│   │   ├── auth/
│   │   │   ├── register.py       # User registration
│   │   │   ├── login.py          # User login
│   │   │   ├── logout.py         # Session termination
│   │   │   ├── reset_request.py  # Password reset request
│   │   │   └── reset_confirm.py  # Password reset confirmation
│   │   └── preferences/
│   │       ├── onboarding.py     # Initial preference collection
│   │       ├── update.py         # Preference updates
│   │       └── get.py            # Preference retrieval
│   ├── auth/
│   │   ├── better_auth.py        # Better-Auth configuration
│   │   ├── session.py            # Session management
│   │   ├── password.py           # Password hashing
│   │   ├── validators.py         # Input validation
│   │   └── lockout.py            # Account lockout logic
│   ├── services/
│   │   ├── preferences.py        # Preference storage
│   │   ├── email.py              # Email sending
│   │   ├── onboarding_progress.py # Partial progress
│   │   └── cache_invalidation.py # Cache management
│   ├── cache/
│   │   └── preference_cache.py   # Preference caching
│   ├── middleware/
│   │   ├── https.py              # HTTPS enforcement
│   │   ├── cors.py               # CORS configuration
│   │   └── rate_limit.py         # Rate limiting
│   ├── models/
│   │   ├── user.py               # User model
│   │   ├── preferences.py        # Preferences model
│   │   └── session.py            # Session model
│   ├── utils/
│   │   ├── audit_log.py          # Security logging
│   │   ├── sanitize.py           # Input sanitization
│   │   └── metrics.py            # Performance metrics
│   ├── config.py                 # Environment configuration
│   └── main.py                   # FastAPI app
├── tests/
│   ├── unit/
│   ├── integration/
│   └── load/                     # k6 load tests
├── vercel.json                   # Vercel deployment
└── requirements.txt

frontend/
├── src/
│   ├── components/auth/
│   │   ├── SignupForm.tsx
│   │   ├── LoginForm.tsx
│   │   ├── OnboardingWizard.tsx
│   │   ├── ProfileSettings.tsx
│   │   ├── PasswordResetForm.tsx
│   │   └── ProtectedRoute.tsx
│   ├── contexts/
│   │   └── AuthContext.tsx
│   ├── hooks/
│   │   ├── useAuth.ts
│   │   └── usePreferences.ts
│   └── services/
│       └── authApi.ts
└── tests/
```

**Structure Decision**: Web application with separate backend (FastAPI + Better-Auth) and frontend (Docusaurus React components). Backend handles all authentication and preference logic; frontend provides embedded UI.

## Design Decisions

### D1: Better-Auth Library
**Decision**: Use Better-Auth for all authentication functionality
**Rationale**: Required by constitution; provides secure, tested auth patterns
**Alternative Rejected**: Custom auth (security risk), other libraries (not constitution-compliant)

### D2: Password Hashing
**Decision**: bcrypt with cost factor 12
**Rationale**: Industry standard, good balance of security and performance
**Alternative Rejected**: argon2 (more complex setup), SHA-256 (not suitable for passwords)

### D3: Session Storage
**Decision**: JWT tokens with HttpOnly cookies
**Rationale**: Secure, stateless, works with serverless (Vercel)
**Alternative Rejected**: Server-side sessions (requires persistent state)

### D4: Onboarding Flow
**Decision**: Multi-step wizard with progress saving
**Rationale**: Better UX than single long form; handles abandonment (per edge case)
**Implementation**: Save to localStorage during wizard; persist to DB on completion

### D5: Preference Caching
**Decision**: In-memory cache with 5-minute TTL
**Rationale**: Meets <1s cached response; simple for serverless
**Cache Keys**: `user:{user_id}:preferences`

### D6: Account Lockout
**Decision**: 5 failed attempts → 15-minute lockout
**Rationale**: Per FR-049; balances security with usability
**Implementation**: Track attempts in DB; notify user via email

## Success Criteria Mapping

| SC | Implementation |
|----|----------------|
| SC-024 | OnboardingWizard with progress indicator + time tracking |
| SC-025 | Login endpoint + Better-Auth session validation |
| SC-026 | Email service with timestamp logging |
| SC-027 | k6 load test with 100 concurrent users |
| SC-028 | Analytics tracking in OnboardingWizard |
| SC-029 | Preference cache + optimized retrieval |
| SC-030 | preference_cache.py with 5-min TTL |
| SC-031 | password.py with bcrypt + DB inspection script |
| SC-032 | lockout.py automated test |
| SC-033 | https.py middleware + SSL Labs verification |

## Dependencies

```
001-textbook-generation ─────┐
  (content to personalize)    │
                             ▼
002-rag-chatbot ────────► 003-authentication ────► 004-personalization
  (optional auth for        │                       (consumes preferences)
   history persistence)     │
                           ▼
                    005-translation
                    (language preference)
```

**Blocking Dependencies**:
- None; can develop independently

**Integration Points**:
- 002-rag-chatbot: Optional session ID for authenticated history
- 004-personalization: Consumes user preferences for response adaptation
- 005-translation: Uses language_preference dimension
