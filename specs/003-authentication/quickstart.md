# Quickstart: Authentication & Personalization Development

**Feature Branch**: `003-authentication`
**Created**: 2025-12-31

## Prerequisites

- Python 3.11+
- Node.js 18+ and npm
- Git
- Access to Neon Postgres instance

## Environment Setup

### 1. Clone and Branch

```bash
git clone <repository-url>
cd hackathon-1
git checkout 003-authentication
```

### 2. Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt
```

### 3. Environment Variables

Create `.env` file in `backend/`:

```env
# Database (Neon Postgres)
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Authentication
JWT_SECRET=your-secure-random-string-min-32-chars
BETTER_AUTH_SECRET=another-secure-random-string

# Email (for password reset)
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your-email@gmail.com
SMTP_PASSWORD=your-app-password
FROM_EMAIL=noreply@yourbook.com

# Security Settings
SESSION_EXPIRY_HOURS=24
LOCKOUT_ATTEMPTS=5
LOCKOUT_DURATION_MINUTES=15
BCRYPT_COST_FACTOR=12

# Environment
DEBUG=true
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000
```

### 4. Database Setup

```bash
# Run migrations
python -m alembic upgrade head

# Or manually create tables
python scripts/init_db.py

# Verify tables created
python scripts/check_db.py
```

### 5. Frontend Setup

```bash
cd frontend

# Install dependencies
npm install

# Create .env.local
echo "NEXT_PUBLIC_API_URL=http://localhost:8000/api" > .env.local
```

## Running the Application

### Start Backend (Development)

```bash
cd backend
source venv/bin/activate
uvicorn src.main:app --reload --port 8000
```

Backend available at `http://localhost:8000`
- API docs: `http://localhost:8000/docs`
- Health check: `http://localhost:8000/api/health`

### Start Frontend (Development)

```bash
cd frontend
npm run dev
```

Frontend available at `http://localhost:3000`

## Testing

### Run Backend Tests

```bash
cd backend

# All tests
pytest

# Unit tests only
pytest tests/unit/

# Integration tests
pytest tests/integration/

# With coverage
pytest --cov=src --cov-report=html

# Specific test file
pytest tests/unit/test_auth.py -v
```

### Run Frontend Tests

```bash
cd frontend

# All tests
npm test

# Watch mode
npm test -- --watch

# Coverage
npm test -- --coverage
```

### Load Testing (k6)

```bash
# Install k6 (https://k6.io/docs/getting-started/installation/)
brew install k6  # macOS
# or: apt install k6  # Ubuntu

# Run load test
k6 run tests/load/auth_load_test.js

# Expected output:
# ✓ 100 concurrent users
# ✓ p95 response time < 4s
```

## API Quick Reference

### Register New User

```bash
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123"
  }'
```

### Login

```bash
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123"
  }' \
  -c cookies.txt  # Save session cookie
```

### Submit Onboarding Preferences

```bash
curl -X POST http://localhost:8000/api/preferences/onboarding \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <token>" \
  -d '{
    "preferences": {
      "technical_background": "intermediate",
      "domain_knowledge": "some",
      "learning_goal": "skill_enhancement",
      "preferred_depth": "balanced",
      "code_examples": "very_important",
      "time_commitment": "2_to_5_hours",
      "focus_area": "ros2",
      "language_preference": "english",
      "prior_ai_experience": "learning",
      "notification_preference": "yes"
    }
  }'
```

### Get Preferences

```bash
curl http://localhost:8000/api/preferences \
  -H "Authorization: Bearer <token>"
```

### Request Password Reset

```bash
curl -X POST http://localhost:8000/api/auth/password/reset-request \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com"}'
```

## Debugging

### Enable Debug Logging

```env
DEBUG=true
LOG_LEVEL=DEBUG
```

### Common Issues

| Issue | Solution |
|-------|----------|
| "Invalid password" on valid password | Check bcrypt cost factor matches |
| Session not persisting | Verify cookie settings (Secure, SameSite) |
| CORS errors | Add frontend origin to CORS_ORIGINS |
| Email not sending | Check SMTP credentials and app password |
| Account locked | Wait 15 minutes or reset via DB |

### Database Debugging

```bash
# Connect to Neon Postgres
psql $DATABASE_URL

# Check users
SELECT id, email, account_status, failed_attempts FROM users;

# Unlock account manually
UPDATE users SET failed_attempts = 0, locked_until = NULL WHERE email = 'test@example.com';

# View preferences
SELECT * FROM user_preferences WHERE user_id = '<uuid>';
```

### Security Testing

```bash
# Test password validation
python -c "from src.auth.validators import validate_password; print(validate_password('weak'))"

# Test bcrypt timing
python -c "import time; from src.auth.password import hash_password; start=time.time(); hash_password('test'); print(f'{time.time()-start:.2f}s')"
```

## Project Structure

```
backend/
├── src/
│   ├── api/
│   │   ├── auth/          # Auth endpoints
│   │   └── preferences/   # Preference endpoints
│   ├── auth/              # Auth logic
│   ├── services/          # Business logic
│   ├── models/            # Pydantic models
│   ├── cache/             # Caching layer
│   └── middleware/        # Security middleware
├── tests/
│   ├── unit/
│   ├── integration/
│   └── load/
├── vercel.json
└── requirements.txt

frontend/
├── src/
│   ├── components/auth/   # Auth UI components
│   ├── contexts/          # Auth context
│   ├── hooks/             # Auth hooks
│   └── services/          # API client
└── tests/
```

## Deployment

### Vercel Deployment

1. Connect repository to Vercel
2. Set environment variables in Vercel dashboard:
   - `DATABASE_URL`
   - `JWT_SECRET`
   - `BETTER_AUTH_SECRET`
   - `SMTP_*` variables
3. Deploy

### Environment Variable Checklist

- [ ] `DATABASE_URL` - Neon connection string
- [ ] `JWT_SECRET` - Secure random string (32+ chars)
- [ ] `BETTER_AUTH_SECRET` - Secure random string
- [ ] `SMTP_HOST` - Email server host
- [ ] `SMTP_PORT` - Email server port (587 for TLS)
- [ ] `SMTP_USER` - Email username
- [ ] `SMTP_PASSWORD` - Email password/app password
- [ ] `FROM_EMAIL` - Sender email address

## Next Steps

1. Complete Phase 1 (Better-Auth Integration)
2. Implement Phase 2 (Password Security)
3. Build Phase 3 (Onboarding Questions)
4. Develop Phase 4 (Frontend Components)

See `plan.md` for detailed implementation phases.
