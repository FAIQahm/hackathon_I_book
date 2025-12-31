# Research: User Authentication & Personalization

**Feature**: 003-authentication
**Date**: 2025-12-31

## Technology Decisions

### 1. Authentication Framework: Better-Auth

**Decision**: Use Better-Auth library for authentication
**Rationale**:
- Required by constitution (Tech Stack Mandatory)
- Provides secure, tested authentication patterns out of the box
- Supports email/password, session management, password reset
- Works with FastAPI and modern Python backends
- Handles security best practices (CSRF, secure cookies, etc.)

**Alternatives Considered**:
- Custom auth: Higher security risk, more development time
- Auth0/Firebase: External dependency, not in constitution
- Authlib: Less opinionated, more configuration needed

**Integration Notes**:
```python
# Better-Auth configuration for FastAPI
from better_auth import BetterAuth

auth = BetterAuth(
    secret=settings.better_auth_secret,
    database_url=settings.database_url,
    session_expiry=timedelta(hours=24),
    password_hash_rounds=12
)
```

### 2. Password Hashing: bcrypt

**Decision**: Use bcrypt with cost factor 12
**Rationale**:
- Industry standard for password hashing
- Adaptive cost factor for future-proofing
- Better-Auth natively supports bcrypt
- Good balance of security and performance (~250ms hash time)

**Configuration**:
```python
BCRYPT_COST_FACTOR = 12
# Results in ~250ms per hash on modern hardware
# Provides protection against brute-force attacks
```

**Alternatives Considered**:
- argon2: Newer, memory-hard, but more complex setup
- scrypt: Good but less ecosystem support
- PBKDF2: Older, less resistant to GPU attacks

### 3. Session Management: JWT in HttpOnly Cookies

**Decision**: JWT tokens stored in HttpOnly, Secure, SameSite cookies
**Rationale**:
- Stateless: Works with Vercel serverless
- Secure: HttpOnly prevents XSS token theft
- Better-Auth provides built-in JWT support
- SameSite=Strict prevents CSRF

**Token Configuration**:
```python
JWT_EXPIRY = 24  # hours
JWT_ALGORITHM = "HS256"
COOKIE_SETTINGS = {
    "httponly": True,
    "secure": True,  # HTTPS only
    "samesite": "strict",
    "max_age": 86400  # 24 hours
}
```

**Alternatives Considered**:
- Server-side sessions: Requires persistent state (Redis)
- localStorage tokens: XSS vulnerability
- Refresh tokens: Added complexity for this use case

### 4. Database: Neon Serverless Postgres

**Decision**: Use Neon Postgres for users, sessions, and preferences
**Rationale**:
- Required by constitution (Tech Stack Mandatory)
- Serverless: Scales with Vercel
- Connection pooling built-in
- Native support for async Python (asyncpg)

**Schema Design**:
```sql
-- Users table
CREATE TABLE users (
    id UUID PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    last_login TIMESTAMP,
    account_status VARCHAR(20) DEFAULT 'active',
    failed_attempts INT DEFAULT 0,
    locked_until TIMESTAMP
);

-- Preferences table
CREATE TABLE user_preferences (
    id UUID PRIMARY KEY,
    user_id UUID REFERENCES users(id),
    preference_key VARCHAR(50) NOT NULL,
    preference_value VARCHAR(255) NOT NULL,
    updated_at TIMESTAMP DEFAULT NOW(),
    UNIQUE(user_id, preference_key)
);
```

### 5. Onboarding UX: Multi-Step Wizard

**Decision**: 10 questions across 3 steps with progress saving
**Rationale**:
- Better UX than single long form
- Handles abandonment (edge case requirement)
- Progress bar increases completion rate
- Groups related questions logically

**Step Structure**:
```
Step 1: Background (3 questions)
  - Technical background
  - Domain knowledge
  - Prior AI experience

Step 2: Goals & Preferences (4 questions)
  - Learning goal
  - Preferred depth
  - Code examples importance
  - Focus area

Step 3: Logistics (3 questions)
  - Time commitment
  - Language preference
  - Notification preference
```

**Progress Saving**:
- Save to localStorage after each question
- On auth failure during wizard: preserve answers
- On completion: persist to DB, clear localStorage

### 6. Account Lockout Strategy

**Decision**: 5 failed attempts → 15-minute lockout with email notification
**Rationale**:
- Per FR-049 requirement
- Balances security with usability
- Email notification allows legitimate users to be aware
- 15 minutes is enough to deter brute force without frustrating users

**Implementation**:
```python
MAX_FAILED_ATTEMPTS = 5
LOCKOUT_DURATION_MINUTES = 15

def check_lockout(user):
    if user.failed_attempts >= MAX_FAILED_ATTEMPTS:
        if user.locked_until > datetime.now():
            raise AccountLockedException(
                f"Account locked. Try again in {remaining_time} minutes"
            )
        else:
            # Lockout expired, reset
            user.failed_attempts = 0
            user.locked_until = None
```

### 7. Email Service: SMTP

**Decision**: Standard SMTP for password reset emails
**Rationale**:
- Simple, reliable delivery
- Works with any email provider
- No additional service dependencies
- Easy to configure via environment variables

**Email Templates**:
```
Password Reset Email:
- Subject: "Reset your Physical AI Book password"
- Body: Link with 24-hour expiry token
- Link format: {frontend_url}/reset-password?token={token}
```

**Alternatives Considered**:
- SendGrid: Good but adds dependency
- AWS SES: Overkill for this use case
- Resend: Modern but not necessary

### 8. Preference Caching

**Decision**: In-memory cache with 5-minute TTL
**Rationale**:
- Meets <1s cached response requirement (SC-030)
- Simple for serverless (no Redis needed initially)
- 5-minute TTL balances freshness with performance

**Cache Implementation**:
```python
from functools import lru_cache
from datetime import datetime, timedelta

preference_cache = {}
CACHE_TTL = timedelta(minutes=5)

def get_cached_preferences(user_id: str):
    cache_entry = preference_cache.get(user_id)
    if cache_entry and cache_entry["expires"] > datetime.now():
        return cache_entry["data"]
    return None

def set_cached_preferences(user_id: str, preferences: dict):
    preference_cache[user_id] = {
        "data": preferences,
        "expires": datetime.now() + CACHE_TTL
    }
```

**Note**: For high scale, consider Vercel KV or Redis

## Best Practices Applied

### Security Best Practices

1. **Password Requirements**: Min 8 chars, 1 uppercase, 1 number (per FR-041)
2. **No Plaintext Passwords**: bcrypt hashing only (per SC-031)
3. **HTTPS Enforcement**: All auth endpoints (per FR-054, SC-033)
4. **Rate Limiting**: 10 req/min on auth endpoints
5. **Input Sanitization**: Prevent injection attacks
6. **Audit Logging**: Track auth events for security review

### UX Best Practices

1. **Immediate Activation**: No email verification (per clarification)
2. **Progress Saving**: Handle wizard abandonment
3. **Clear Error Messages**: Specific feedback for each failure mode
4. **Inline Validation**: Email format, password strength shown in real-time
5. **Session Persistence**: Remember login across browser sessions

### Performance Best Practices

1. **Connection Pooling**: Neon built-in pooling
2. **Preference Caching**: Meet <1s requirement
3. **Async Operations**: Non-blocking I/O throughout
4. **Lazy Loading**: Load preferences only when needed

## Security Considerations

1. **Password Storage**: bcrypt with cost 12, never plaintext
2. **Session Tokens**: JWT in HttpOnly cookies, short expiry
3. **CSRF Protection**: SameSite cookies, origin validation
4. **XSS Prevention**: HttpOnly cookies, CSP headers
5. **Brute Force**: Account lockout after 5 attempts
6. **Reset Token Security**: Single-use, 24-hour expiry, cryptographically random

## Performance Targets

| Metric | Target | Implementation |
|--------|--------|----------------|
| Login latency | <500ms | Direct DB query + bcrypt verify |
| Preference fetch (cached) | <1s | In-memory cache |
| Preference fetch (uncached) | <2s | DB query + cache population |
| Personalized response | <4s | Cached preferences + response generation |
| Concurrent users | 100 | Vercel serverless auto-scaling |

## All Research Questions Resolved

No NEEDS CLARIFICATION remaining.
