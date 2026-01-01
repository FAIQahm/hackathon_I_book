---
name: vercel-fastapi-link
description: |
  Configure FastAPI for Vercel deployment.
  Bundled Resources: Includes the 'vercel.json' configuration template and the CORSMiddleware Python boilerplate to allow requests from GitHub Pages.
version: 1.1.0
inputs:
  github_pages_url:
    description: Your GitHub Pages URL for CORS configuration
    required: false
    default: "https://faiqahm.github.io"
    example: "https://username.github.io"
  extra_origins:
    description: Additional CORS origins (comma-separated)
    required: false
    default: ""
    example: "https://staging.example.com,https://preview.example.com"
  api_entry:
    description: Path to FastAPI main.py file
    required: false
    default: "api/main.py"
  project_name:
    description: Project name for API title
    required: false
    default: "Physical AI Book API"
  python_version:
    description: Python version for Vercel runtime
    required: false
    default: "3.11"
---

# Vercel FastAPI Link

Configure FastAPI for Vercel serverless deployment with CORS support for GitHub Pages frontend.

## Quick Setup

**Full automated setup with testing (recommended):**

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh --github-pages https://faiqahm.github.io --test
```

**Basic setup:**

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh
```

**With multiple CORS origins:**

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh \
  --github-pages https://faiqahm.github.io \
  --extra-origins "https://staging.mysite.com,https://preview.mysite.com" \
  --test
```

## Command Options

| Option | Description | Default |
|--------|-------------|---------|
| `--github-pages URL` | GitHub Pages URL for CORS | `https://faiqahm.github.io` |
| `--extra-origins URLS` | Additional CORS origins (comma-separated) | - |
| `--api-entry PATH` | Path to FastAPI main.py | `api/main.py` |
| `--project-name NAME` | API project name | `Physical AI Book API` |
| `--python-version VER` | Python version for Vercel | `3.11` |
| `--skip-vercel-json` | Don't create vercel.json | off |
| `--skip-main` | Don't create main.py template | off |
| `--test` | Auto-test setup (starts server, hits /health) | off |
| `-h, --help` | Show help message | - |

## What It Does

### 1. Creates `runtime.txt`
Specifies Python version for Vercel deployment (e.g., `python-3.11`)

### 2. Creates `.env.example` and `.env`
Environment variable templates with:
- `GITHUB_PAGES_URL` - Primary CORS origin
- `EXTRA_CORS_ORIGINS` - Additional origins (comma-separated)
- Placeholders for database, auth, and API config

### 3. Creates `vercel.json`
Configures Vercel to:
- Use `@vercel/python` runtime
- Route `/api/*` requests to FastAPI
- Expose `/docs`, `/health`, and `/openapi.json`

### 4. Creates `api/main.py`
FastAPI application with:
- CORS middleware configured for GitHub Pages + extra origins
- Dynamic CORS from `EXTRA_CORS_ORIGINS` env var
- Health check endpoint at `/health`
- OpenAPI docs at `/docs`
- Example API routes

### 5. Auto-Test (--test flag)
When `--test` is provided:
- Starts uvicorn on port 8765
- Hits `/health` endpoint
- Reports success/failure
- Catches import errors immediately

---

## Bundled Resources

### 1. Runtime Configuration

**File**: `runtime.txt`

```
python-3.11
```

### 2. Environment Template

**File**: `.env.example`

```bash
# GitHub Pages URL for CORS (required for production)
GITHUB_PAGES_URL=https://faiqahm.github.io

# Additional allowed origins (comma-separated, optional)
# EXTRA_CORS_ORIGINS=https://staging.example.com,https://preview.example.com

# Database (if needed)
# DATABASE_URL=postgresql://user:pass@host:5432/dbname
```

### 3. Vercel Configuration

**File**: `vercel.json`

```json
{
  "version": 2,
  "builds": [
    {
      "src": "api/main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    { "src": "/api/(.*)", "dest": "api/main.py" },
    { "src": "/health", "dest": "api/main.py" },
    { "src": "/docs", "dest": "api/main.py" },
    { "src": "/openapi.json", "dest": "api/main.py" }
  ]
}
```

### 4. CORS Middleware

**File**: `api/main.py` (CORS section)

```python
# Get GitHub Pages URL from environment or use default
GITHUB_PAGES_URL = os.getenv("GITHUB_PAGES_URL", "https://faiqahm.github.io")

# Base allowed origins
allowed_origins = [
    "http://localhost:3000",      # Local Docusaurus dev
    "http://localhost:8000",      # Local FastAPI dev
    GITHUB_PAGES_URL,             # Production GitHub Pages
]

# Add extra origins from environment variable (comma-separated)
extra_origins_env = os.getenv("EXTRA_CORS_ORIGINS", "")
if extra_origins_env:
    for origin in extra_origins_env.split(","):
        origin = origin.strip()
        if origin and origin not in allowed_origins:
            allowed_origins.append(origin)

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
    allow_headers=["Authorization", "Content-Type", "Accept", "Origin"],
    max_age=600,
)
```

### 5. Input Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `github_pages_url` | No | `https://faiqahm.github.io` | Primary frontend URL |
| `extra_origins` | No | - | Additional CORS origins |
| `api_entry` | No | `api/main.py` | FastAPI entry point |
| `project_name` | No | `Physical AI Book API` | API title |
| `python_version` | No | `3.11` | Python version for Vercel |

## Usage Instructions

### Step 1: Run Setup with Test

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh --test
```

### Step 2: Verify Created Files

```
project/
├── api/
│   ├── __init__.py
│   └── main.py          # FastAPI application
├── vercel.json          # Vercel configuration
├── runtime.txt          # Python version
├── requirements.txt     # Python dependencies
├── .env.example         # Environment template
├── .env                 # Local environment (gitignored)
└── .gitignore           # Updated with .env
```

### Step 3: Test Locally (if --test not used)

```bash
# Install dependencies
pip install -r requirements.txt

# Run locally
uvicorn api.main:app --reload --port 8000

# Test endpoints
curl http://localhost:8000/health
curl http://localhost:8000/docs
```

### Step 4: Deploy to Vercel

```bash
# Install Vercel CLI
npm i -g vercel

# Deploy
vercel

# Set environment variables
vercel env add GITHUB_PAGES_URL
vercel env add EXTRA_CORS_ORIGINS  # Optional
```

### Step 5: Update Frontend

In your Docusaurus site, configure the API URL:

```javascript
// src/config.js
export const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-project.vercel.app'
  : 'http://localhost:8000';
```

## Verification Checklist

- [ ] `runtime.txt` exists with Python version
- [ ] `.env.example` documents all environment variables
- [ ] `.env` is gitignored
- [ ] `vercel.json` exists in project root
- [ ] `api/main.py` contains FastAPI app with CORS
- [ ] `requirements.txt` includes fastapi and uvicorn
- [ ] `--test` flag passes health check
- [ ] Vercel deployment successful
- [ ] Frontend can call API without CORS errors

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **CORS error in browser** | Verify `allowed_origins` includes your GitHub Pages URL |
| **404 on Vercel** | Check `vercel.json` routes match your endpoints |
| **Module not found** | Ensure `requirements.txt` is in project root |
| **Wrong Python version** | Check `runtime.txt` matches your code requirements |
| **Cold start slow** | Normal for serverless; first request takes longer |
| **Environment variable not set** | Use `vercel env add VARIABLE_NAME` |
| **--test fails** | Check for import errors in api/main.py |
| **Multiple origins needed** | Use `--extra-origins` or set `EXTRA_CORS_ORIGINS` env var |

### Common CORS Issues

**Problem**: `Access-Control-Allow-Origin` header missing

**Solution**: Ensure the exact origin (including protocol) is in `allowed_origins`:
```python
# In Vercel Dashboard > Settings > Environment Variables:
GITHUB_PAGES_URL=https://username.github.io
EXTRA_CORS_ORIGINS=https://staging.example.com,https://preview.example.com
```

### Testing the --test Flag

```bash
# Run with test
.claude/skills/vercel-fastapi-link/scripts/setup.sh --test

# Expected output:
# ✓ Health check passed: {"status":"healthy","service":"physical-ai-book-api"}
# ✓ All tests passed! Your API is ready.
```

## Requirements

- Python 3.9+ (default: 3.11)
- FastAPI 0.100.0+
- Vercel account
- Vercel CLI (`npm i -g vercel`)

## Environment Variables

| Variable | Description | Where to Set |
|----------|-------------|--------------|
| `GITHUB_PAGES_URL` | Primary frontend URL for CORS | Vercel Dashboard |
| `EXTRA_CORS_ORIGINS` | Additional origins (comma-separated) | Vercel Dashboard |

## Related

- [Vercel Python Runtime Docs](https://vercel.com/docs/functions/runtimes/python)
- [FastAPI CORS Docs](https://fastapi.tiangolo.com/tutorial/cors/)
- [GitHub Pages Deployment](https://docs.github.com/en/pages)
- Skill: `github-pages-deploy` (for frontend deployment)
- ADR-001: Deployment Infrastructure Stack
