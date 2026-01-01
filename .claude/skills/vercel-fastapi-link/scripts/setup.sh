#!/bin/bash
# =============================================================================
# Vercel FastAPI Link Setup Script
# Configures FastAPI for Vercel deployment with CORS for GitHub Pages
# Version: 1.1.0
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
GITHUB_PAGES_URL="https://faiqahm.github.io"
API_ENTRY="api/main.py"
PROJECT_NAME="Physical AI Book API"
PYTHON_VERSION="3.11"
EXTRA_ORIGINS=""
SKIP_VERCEL_JSON=false
SKIP_MAIN=false
RUN_TEST=false

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"
ASSETS_DIR="$SKILL_DIR/assets"

# =============================================================================
# Functions
# =============================================================================

show_help() {
    cat << EOF
Vercel FastAPI Link - Configure FastAPI for Vercel deployment

Usage: $(basename "$0") [OPTIONS]

Options:
  --github-pages URL     GitHub Pages URL for CORS (default: $GITHUB_PAGES_URL)
  --extra-origins URLS   Additional CORS origins, comma-separated
                         Example: "https://staging.example.com,https://app.example.com"
  --api-entry PATH       Path to FastAPI main.py (default: $API_ENTRY)
  --project-name NAME    API project name (default: $PROJECT_NAME)
  --python-version VER   Python version for Vercel (default: $PYTHON_VERSION)
  --skip-vercel-json     Don't create vercel.json
  --skip-main            Don't create main.py template
  --test                 Test the setup after creation (starts server, hits /health)
  -h, --help             Show this help message

Examples:
  $(basename "$0")
  $(basename "$0") --github-pages https://myuser.github.io --test
  $(basename "$0") --github-pages https://myuser.github.io \\
    --extra-origins "https://staging.mysite.com,https://preview.mysite.com"

EOF
}

log_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

log_success() {
    echo -e "${GREEN}✓${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

log_error() {
    echo -e "${RED}✗${NC} $1" >&2
}

# =============================================================================
# Parse Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --github-pages)
            GITHUB_PAGES_URL="$2"
            shift 2
            ;;
        --extra-origins)
            EXTRA_ORIGINS="$2"
            shift 2
            ;;
        --api-entry)
            API_ENTRY="$2"
            shift 2
            ;;
        --project-name)
            PROJECT_NAME="$2"
            shift 2
            ;;
        --python-version)
            PYTHON_VERSION="$2"
            shift 2
            ;;
        --skip-vercel-json)
            SKIP_VERCEL_JSON=true
            shift
            ;;
        --skip-main)
            SKIP_MAIN=true
            shift
            ;;
        --test)
            RUN_TEST=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# =============================================================================
# Main Setup
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Vercel FastAPI Link Setup v1.1.0"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
log_info "GitHub Pages URL: $GITHUB_PAGES_URL"
if [[ -n "$EXTRA_ORIGINS" ]]; then
    log_info "Extra Origins: $EXTRA_ORIGINS"
fi
log_info "API Entry: $API_ENTRY"
log_info "Project Name: $PROJECT_NAME"
log_info "Python Version: $PYTHON_VERSION"
echo ""

# Get API directory from entry path
API_DIR=$(dirname "$API_ENTRY")

# =============================================================================
# Build CORS origins list for Python
# =============================================================================

# Start with the base origins
CORS_ORIGINS_PYTHON="    \"http://localhost:3000\",      # Local Docusaurus dev
    \"http://localhost:8000\",      # Local FastAPI dev
    \"http://127.0.0.1:3000\",
    \"http://127.0.0.1:8000\",
    GITHUB_PAGES_URL,             # Production GitHub Pages
    f\"{GITHUB_PAGES_URL}/\",       # With trailing slash"

# Add extra origins if provided
EXTRA_ORIGINS_PYTHON=""
if [[ -n "$EXTRA_ORIGINS" ]]; then
    IFS=',' read -ra ORIGINS_ARRAY <<< "$EXTRA_ORIGINS"
    for origin in "${ORIGINS_ARRAY[@]}"; do
        # Trim whitespace
        origin=$(echo "$origin" | xargs)
        EXTRA_ORIGINS_PYTHON="$EXTRA_ORIGINS_PYTHON
    \"$origin\",              # Extra origin"
    done
fi

# =============================================================================
# Create runtime.txt (Python version for Vercel)
# =============================================================================

echo "python-$PYTHON_VERSION" > runtime.txt
log_success "Created runtime.txt (Python $PYTHON_VERSION)"

# =============================================================================
# Create .env.example
# =============================================================================

cat > .env.example << EOF
# =============================================================================
# Environment Variables for FastAPI + Vercel
# Generated by vercel-fastapi-link skill
# =============================================================================

# GitHub Pages URL for CORS (required for production)
GITHUB_PAGES_URL=$GITHUB_PAGES_URL

# Additional allowed origins (comma-separated, optional)
# EXTRA_CORS_ORIGINS=https://staging.example.com,https://preview.example.com

# API Configuration
# API_VERSION=1.0.0
# DEBUG=false

# Database (if needed)
# DATABASE_URL=postgresql://user:pass@host:5432/dbname

# Authentication (if needed)
# JWT_SECRET=your-secret-key
# JWT_ALGORITHM=HS256
EOF
log_success "Created .env.example"

# Create .env if it doesn't exist
if [[ ! -f ".env" ]]; then
    cp .env.example .env
    log_success "Created .env from .env.example"
else
    log_info ".env already exists, skipping"
fi

# =============================================================================
# Create vercel.json
# =============================================================================

if [[ "$SKIP_VERCEL_JSON" != true ]]; then
    if [[ -f "vercel.json" ]]; then
        log_warning "vercel.json already exists, backing up to vercel.json.bak"
        cp vercel.json vercel.json.bak
    fi

    cat > vercel.json << EOF
{
  "version": 2,
  "builds": [
    {
      "src": "$API_ENTRY",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "$API_ENTRY"
    },
    {
      "src": "/health",
      "dest": "$API_ENTRY"
    },
    {
      "src": "/docs",
      "dest": "$API_ENTRY"
    },
    {
      "src": "/openapi.json",
      "dest": "$API_ENTRY"
    }
  ]
}
EOF
    log_success "Created vercel.json"
fi

# =============================================================================
# Create API directory and main.py
# =============================================================================

if [[ "$SKIP_MAIN" != true ]]; then
    # Create API directory
    mkdir -p "$API_DIR"
    log_success "Created $API_DIR/ directory"

    # Create __init__.py
    touch "$API_DIR/__init__.py"

    # Create main.py with CORS configured
    cat > "$API_ENTRY" << EOF
"""
FastAPI Application for Vercel Deployment
Generated by vercel-fastapi-link skill v1.1.0

Project: $PROJECT_NAME
GitHub Pages: $GITHUB_PAGES_URL
"""

import os
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

# =============================================================================
# App Configuration
# =============================================================================

app = FastAPI(
    title="$PROJECT_NAME",
    description="Backend API for the Physical AI Educational Book",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

# =============================================================================
# CORS Configuration - Allow GitHub Pages frontend
# =============================================================================

# Get GitHub Pages URL from environment or use default
GITHUB_PAGES_URL = os.getenv("GITHUB_PAGES_URL", "$GITHUB_PAGES_URL")

# Base allowed origins
allowed_origins = [
$CORS_ORIGINS_PYTHON$EXTRA_ORIGINS_PYTHON
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

# =============================================================================
# Health Check Endpoint
# =============================================================================

@app.get("/health")
async def health_check():
    """Health check endpoint for monitoring."""
    return {"status": "healthy", "service": "physical-ai-book-api"}


@app.get("/")
async def root():
    """Root endpoint with API info."""
    return {
        "message": "$PROJECT_NAME",
        "docs": "/docs",
        "health": "/health",
    }

# =============================================================================
# API Routes - Add your endpoints below
# =============================================================================

@app.get("/api/v1/chapters")
async def list_chapters():
    """List all available chapters."""
    return {
        "chapters": [
            {"id": 1, "title": "Introduction to Physical AI", "slug": "intro"},
            {"id": 2, "title": "Sensors and Perception", "slug": "chapter-1"},
            {"id": 3, "title": "Actuators and Control", "slug": "chapter-2"},
        ]
    }


@app.get("/api/v1/chapters/{chapter_id}")
async def get_chapter(chapter_id: int):
    """Get a specific chapter by ID."""
    chapters = {
        1: {"id": 1, "title": "Introduction to Physical AI", "content": "..."},
        2: {"id": 2, "title": "Sensors and Perception", "content": "..."},
        3: {"id": 3, "title": "Actuators and Control", "content": "..."},
    }
    if chapter_id not in chapters:
        return JSONResponse(status_code=404, content={"detail": "Chapter not found"})
    return chapters[chapter_id]


# =============================================================================
# Error Handlers
# =============================================================================

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for unhandled errors."""
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error", "type": type(exc).__name__},
    )


# =============================================================================
# Vercel Serverless Handler
# =============================================================================

# The app object is automatically detected by @vercel/python
# No additional handler needed for Vercel deployment
EOF
    log_success "Created $API_ENTRY with CORS middleware"
fi

# =============================================================================
# Create requirements.txt if not exists
# =============================================================================

if [[ ! -f "requirements.txt" ]]; then
    cat > requirements.txt << EOF
# FastAPI and dependencies
fastapi>=0.100.0
uvicorn>=0.23.0

# Optional: Add more dependencies as needed
# pydantic>=2.0.0
# python-dotenv>=1.0.0
EOF
    log_success "Created requirements.txt"
else
    # Check if fastapi is in requirements.txt
    if ! grep -q "fastapi" requirements.txt; then
        echo "" >> requirements.txt
        echo "# FastAPI (added by vercel-fastapi-link)" >> requirements.txt
        echo "fastapi>=0.100.0" >> requirements.txt
        echo "uvicorn>=0.23.0" >> requirements.txt
        log_success "Added fastapi to requirements.txt"
    else
        log_info "fastapi already in requirements.txt"
    fi
fi

# =============================================================================
# Add to .gitignore
# =============================================================================

if [[ -f ".gitignore" ]]; then
    if ! grep -q "^\.env$" .gitignore; then
        echo "" >> .gitignore
        echo "# Environment variables (added by vercel-fastapi-link)" >> .gitignore
        echo ".env" >> .gitignore
        echo "__pycache__/" >> .gitignore
        echo "*.pyc" >> .gitignore
        log_success "Added .env to .gitignore"
    fi
else
    cat > .gitignore << EOF
# Environment variables
.env

# Python
__pycache__/
*.pyc
*.pyo
.Python
venv/
.venv/

# IDE
.vscode/
.idea/

# Vercel
.vercel/
EOF
    log_success "Created .gitignore"
fi

# =============================================================================
# Run Test (if --test flag provided)
# =============================================================================

if [[ "$RUN_TEST" == true ]]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🧪 Running Setup Test"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    # Check if uvicorn is installed
    if ! command -v uvicorn &> /dev/null; then
        log_warning "uvicorn not found. Installing dependencies..."
        pip install -q fastapi uvicorn
    fi

    # Check if port 8765 is available (use non-standard port for testing)
    TEST_PORT=8765

    log_info "Starting test server on port $TEST_PORT..."

    # Start uvicorn in background
    uvicorn "${API_ENTRY%.py}:app" --host 127.0.0.1 --port $TEST_PORT &
    SERVER_PID=$!

    # Wait for server to start
    sleep 3

    # Test health endpoint
    log_info "Testing /health endpoint..."
    HEALTH_RESPONSE=$(curl -s "http://127.0.0.1:$TEST_PORT/health" 2>/dev/null || echo "FAILED")

    # Kill the server
    kill $SERVER_PID 2>/dev/null || true
    wait $SERVER_PID 2>/dev/null || true

    # Check response
    if echo "$HEALTH_RESPONSE" | grep -q '"status"'; then
        log_success "Health check passed: $HEALTH_RESPONSE"
        echo ""
        log_success "All tests passed! Your API is ready."
    else
        log_error "Health check failed!"
        log_error "Response: $HEALTH_RESPONSE"
        echo ""
        log_warning "Check for import errors or missing dependencies."
        exit 1
    fi
fi

# =============================================================================
# Summary
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✨ Vercel FastAPI Link setup complete!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📁 Files created:"
echo "   ├── runtime.txt           (Python $PYTHON_VERSION)"
echo "   ├── .env.example          (environment template)"
echo "   ├── .env                  (local config)"
if [[ "$SKIP_VERCEL_JSON" != true ]]; then
echo "   ├── vercel.json"
fi
if [[ "$SKIP_MAIN" != true ]]; then
echo "   ├── $API_DIR/"
echo "   │   ├── __init__.py"
echo "   │   └── main.py"
fi
echo "   └── requirements.txt"
echo ""
echo "📋 Next steps:"
echo "   1. Install dependencies:"
echo "      pip install -r requirements.txt"
echo ""
echo "   2. Test locally:"
echo "      uvicorn ${API_ENTRY%.py}:app --reload --port 8000"
echo "      curl http://localhost:8000/health"
echo ""
echo "   3. Deploy to Vercel:"
echo "      vercel"
echo ""
echo "   4. Set environment variables in Vercel:"
echo "      vercel env add GITHUB_PAGES_URL"
echo ""
echo "🔗 CORS configured for:"
echo "   • http://localhost:3000 (local dev)"
echo "   • http://localhost:8000 (local API)"
echo "   • $GITHUB_PAGES_URL (production)"
if [[ -n "$EXTRA_ORIGINS" ]]; then
    IFS=',' read -ra ORIGINS_ARRAY <<< "$EXTRA_ORIGINS"
    for origin in "${ORIGINS_ARRAY[@]}"; do
        origin=$(echo "$origin" | xargs)
        echo "   • $origin (extra origin)"
    done
fi
echo ""
