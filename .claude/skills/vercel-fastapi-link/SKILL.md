---
name: vercel-fastapi-link
description: |
  Configure FastAPI for Vercel deployment.
  Bundled Resources: Includes the 'vercel.json' configuration template and the CORSMiddleware Python boilerplate to allow requests from GitHub Pages.
version: 1.0.0
inputs:
  github_pages_url:
    description: Your GitHub Pages URL for CORS configuration
    required: false
    default: "https://faiqahm.github.io"
    example: "https://username.github.io"
  api_entry:
    description: Path to FastAPI main.py file
    required: false
    default: "api/main.py"
  project_name:
    description: Project name for API title
    required: false
    default: "Physical AI Book API"
---

# Vercel FastAPI Link

Configure FastAPI for Vercel serverless deployment with CORS support for GitHub Pages frontend.

## Quick Setup

**Full automated setup (recommended):**

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh --github-pages https://faiqahm.github.io
```

**Basic setup:**

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh
```

**Custom configuration:**

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh \
  --github-pages https://your-username.github.io \
  --api-entry api/main.py \
  --project-name "My API"
```

## Command Options

| Option | Description | Default |
|--------|-------------|---------|
| `--github-pages URL` | GitHub Pages URL for CORS | `https://faiqahm.github.io` |
| `--api-entry PATH` | Path to FastAPI main.py | `api/main.py` |
| `--project-name NAME` | API project name | `Physical AI Book API` |
| `--skip-vercel-json` | Don't create vercel.json | off |
| `--skip-main` | Don't create main.py template | off |
| `-h, --help` | Show help message | - |

## What It Does

### 1. Creates `vercel.json`
Configures Vercel to:
- Use `@vercel/python` runtime
- Route `/api/*` requests to FastAPI
- Expose `/docs`, `/health`, and `/openapi.json`

### 2. Creates `api/main.py`
FastAPI application with:
- CORS middleware configured for GitHub Pages
- Health check endpoint at `/health`
- OpenAPI docs at `/docs`
- Example API routes

### 3. CORS Configuration
Allows cross-origin requests from:
- `http://localhost:3000` (local Docusaurus)
- `http://localhost:8000` (local FastAPI)
- Your GitHub Pages URL (production)

---

## Bundled Resources

### 1. Vercel Configuration

**File**: `vercel.json` (project root)

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
    {
      "src": "/api/(.*)",
      "dest": "api/main.py"
    },
    {
      "src": "/health",
      "dest": "api/main.py"
    },
    {
      "src": "/docs",
      "dest": "api/main.py"
    },
    {
      "src": "/openapi.json",
      "dest": "api/main.py"
    }
  ]
}
```

### 2. CORS Middleware

**File**: `api/main.py` (CORS section)

```python
from fastapi.middleware.cors import CORSMiddleware

# GitHub Pages URL from environment
GITHUB_PAGES_URL = os.getenv("GITHUB_PAGES_URL", "https://faiqahm.github.io")

allowed_origins = [
    "http://localhost:3000",      # Local Docusaurus dev
    "http://localhost:8000",      # Local FastAPI dev
    GITHUB_PAGES_URL,             # Production GitHub Pages
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
    allow_headers=["Authorization", "Content-Type", "Accept", "Origin"],
    max_age=600,
)
```

### 3. Input Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `github_pages_url` | No | `https://faiqahm.github.io` | Frontend URL for CORS |
| `api_entry` | No | `api/main.py` | FastAPI entry point |
| `project_name` | No | `Physical AI Book API` | API title |

## Usage Instructions

### Step 1: Run Setup

```bash
.claude/skills/vercel-fastapi-link/scripts/setup.sh --github-pages https://your-username.github.io
```

### Step 2: Install Dependencies

Create `requirements.txt`:

```txt
fastapi>=0.100.0
uvicorn>=0.23.0
```

### Step 3: Test Locally

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

# Set environment variable (optional)
vercel env add GITHUB_PAGES_URL
```

### Step 5: Update Frontend

In your Docusaurus site, configure the API URL:

```javascript
// src/config.js
export const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-project.vercel.app'
  : 'http://localhost:8000';
```

## Project Structure

After setup:

```
project/
├── api/
│   ├── __init__.py
│   └── main.py          # FastAPI application
├── vercel.json          # Vercel configuration
├── requirements.txt     # Python dependencies
└── ...
```

## Verification Checklist

- [ ] `vercel.json` exists in project root
- [ ] `api/main.py` contains FastAPI app with CORS
- [ ] `requirements.txt` includes fastapi and uvicorn
- [ ] Local test: `curl http://localhost:8000/health` returns `{"status": "healthy"}`
- [ ] CORS headers present in response
- [ ] Vercel deployment successful
- [ ] Frontend can call API without CORS errors

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **CORS error in browser** | Verify `allowed_origins` includes your GitHub Pages URL |
| **404 on Vercel** | Check `vercel.json` routes match your endpoints |
| **Module not found** | Ensure `requirements.txt` is in project root |
| **Cold start slow** | Normal for serverless; first request takes longer |
| **Environment variable not set** | Use `vercel env add GITHUB_PAGES_URL` |
| **API not accessible** | Check Vercel deployment logs for errors |

### Common CORS Issues

**Problem**: `Access-Control-Allow-Origin` header missing

**Solution**: Ensure the exact origin (including protocol and port) is in `allowed_origins`:
```python
allowed_origins = [
    "https://username.github.io",  # Must match exactly
    "https://username.github.io/", # With trailing slash too
]
```

### Vercel Deployment Logs

```bash
# View deployment logs
vercel logs <deployment-url>

# View function logs
vercel logs <deployment-url> --follow
```

## Requirements

- Python 3.9+
- FastAPI 0.100.0+
- Vercel account
- Vercel CLI (`npm i -g vercel`)

## Environment Variables

| Variable | Description | Where to Set |
|----------|-------------|--------------|
| `GITHUB_PAGES_URL` | Frontend URL for CORS | Vercel Dashboard → Settings → Environment Variables |

## Related

- [Vercel Python Runtime Docs](https://vercel.com/docs/functions/runtimes/python)
- [FastAPI CORS Docs](https://fastapi.tiangolo.com/tutorial/cors/)
- [GitHub Pages Deployment](https://docs.github.com/en/pages)
- Skill: `github-pages-deploy` (for frontend deployment)
- ADR-001: Deployment Infrastructure Stack
