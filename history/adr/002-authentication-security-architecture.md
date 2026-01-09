# ADR-002: Authentication & Security Architecture

**Status**: Accepted
**Date**: 2025-12-31
**Deciders**: Project Team

## Context

The Physical AI Educational Book requires user authentication to:
- Enable personalized learning experiences based on user preferences
- Collect 10 onboarding questions to customize content delivery
- Provide secure account management with password reset
- Protect user data while supporting serverless deployment

The solution must integrate with Vercel serverless functions, work across GitHub Pages frontend and Vercel backend, and meet security best practices.

## Decision

We adopt a **Better-Auth based authentication stack**:

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Auth Library** | Better-Auth | Session management, credential validation |
| **Password Hashing** | bcrypt (cost factor 12) | Secure password storage |
| **Session Tokens** | JWT in HttpOnly cookies | Stateless auth for serverless |
| **Account Protection** | 5-attempt lockout (15 min) | Brute force prevention |
| **Email Service** | SMTP (configurable) | Password reset delivery |

### Security Configuration

**Password Requirements**:
- Minimum 8 characters
- At least 1 uppercase letter
- At least 1 number

**Session Management**:
- 24-hour session expiry
- HttpOnly, Secure, SameSite cookies
- JWT tokens (not server-side sessions)

**CORS Configuration** (Critical):
```python
# CORSMiddleware MUST be first middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,  # GitHub Pages + localhost
    allow_credentials=True,  # Required for cookie auth
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Account Lockout**:
- 5 failed login attempts triggers lockout
- 15-minute lockout duration
- Email notification on lockout

## Consequences

### Positive

- **Constitution Compliant**: Better-Auth is mandated by project constitution
- **Serverless Compatible**: JWT tokens work without persistent session storage
- **Industry Standard**: bcrypt is proven, widely audited
- **User Experience**: Cookie-based auth is seamless (no token management in frontend)
- **Security**: Multiple layers (hashing, lockout, HTTPS, secure cookies)

### Negative

- **Token Revocation**: JWT cannot be revoked before expiry (mitigated by short TTL)
- **CORS Complexity**: Cross-origin cookie auth requires precise configuration
- **Cold Start Impact**: bcrypt hashing during cold starts may add latency

### Risks

- **Misconfigured CORS**: `allow_credentials=True` with proper origins is critical
- **Token Leakage**: JWT in cookies must have proper security flags

## Alternatives Considered

### Alternative 1: OAuth Only (Google, GitHub)
- **Pros**: No password management, trusted providers
- **Cons**: Requires third-party accounts, less control, privacy concerns for educational users

### Alternative 2: Custom Auth Implementation
- **Pros**: Full control, no dependencies
- **Cons**: Security risk, reinventing wheel, not constitution-compliant

### Alternative 3: Server-Side Sessions (Redis)
- **Pros**: Easy token revocation, traditional approach
- **Cons**: Requires additional infrastructure, not serverless-friendly, adds cost

### Alternative 4: Argon2 Password Hashing
- **Pros**: Newer, memory-hard algorithm
- **Cons**: More complex configuration, bcrypt is sufficient for this scale

## References

- `specs/003-authentication/plan.md` - Full authentication plan
- `specs/003-authentication/research.md` - Technology decisions
- `specs/003-authentication/contracts/openapi.yaml` - API specification
- `.specify/memory/constitution.md` - Better-Auth mandate
