---
id: 004
title: Apply vercel-fastapi-link Skill for Backend Setup
stage: general
date: 2026-01-01
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: developer
command: skill application
labels: ["skill", "fastapi", "vercel", "backend", "api"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - api/main.py
  - api/__init__.py
  - vercel.json
  - requirements.txt
  - runtime.txt
  - .env.example
  - .gitignore
tests:
  - curl http://127.0.0.1:8000/health -> {"status":"healthy",...}
  - curl http://127.0.0.1:8000/api/ping -> {"message":"pong"}
---

## Prompt

Apply the vercel-fastapi-link skill to create a backend for my project. Specifically:

Use the skill to generate the vercel.json in the root and the api/main.py file.

Ensure GITHUB_PAGES_URL is set to https://faiqahm.github.io in the CORS settings.

Create a requirements.txt file with fastapi and uvicorn.

Add a new test route in api/main.py at @app.get('/api/ping') that returns {'message': 'pong'}.

After creating the files, run a local test using uvicorn to verify that http://localhost:8000/health and http://localhost:8000/api/ping work correctly.

## Response snapshot

Applied the vercel-fastapi-link v1.2.0 skill to set up the backend:

1. **Generated Files**:
   - `vercel.json` - Vercel deployment config with Python runtime
   - `api/main.py` - FastAPI app with logging, Pydantic models, CORS
   - `api/__init__.py` - Python package marker
   - `requirements.txt` - fastapi, uvicorn, pydantic dependencies
   - `runtime.txt` - Python 3.11 specification
   - `.env.example` - Environment variable template

2. **CORS Configuration**: Set to allow https://faiqahm.github.io

3. **Added `/api/ping` endpoint** returning `{"message": "pong"}`

4. **Local Testing Results**:
   - `/health` -> `{"status":"healthy","service":"physical-ai-book-api","timestamp":"2026-01-01T07:43:39Z"}`
   - `/api/ping` -> `{"message":"pong"}`

5. **Updated `.gitignore`** to exclude `.venv/` and Python artifacts

## Outcome

- ✅ Impact: Backend API ready for Vercel deployment with CORS for GitHub Pages frontend
- 🧪 Tests: Both /health and /api/ping endpoints verified working
- 📁 Files: 7 files created/modified
- 🔁 Next prompts: Deploy to Vercel, connect frontend to backend API
- 🧠 Reflection: The v1.2.0 skill with logging and Pydantic models provides good debugging capabilities

## Evaluation notes (flywheel)

- Failure modes observed: None - skill executed cleanly
- Graders run and results (PASS/FAIL): Manual curl tests PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add /api/ping route to vercel.json explicitly
