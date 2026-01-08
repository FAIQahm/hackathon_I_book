#!/bin/bash
# =============================================================================
# Vercel FastAPI Link Setup Script
# Configures FastAPI for Vercel deployment with CORS for GitHub Pages
# Version: 1.4.0
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
RUN_DEPLOY=false
DEPLOY_PROD=false

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
  --deploy               Deploy to Vercel after setup (requires vercel CLI)
  --prod                 Deploy to production (use with --deploy)
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
        --deploy)
            RUN_DEPLOY=true
            shift
            ;;
        --prod)
            DEPLOY_PROD=true
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
echo "🚀 Vercel FastAPI Link Setup v1.4.0"
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
# Generated by vercel-fastapi-link skill v1.4.0
# =============================================================================

# GitHub Pages URL for CORS (required for production)
GITHUB_PAGES_URL=$GITHUB_PAGES_URL

# Additional allowed origins (comma-separated, optional)
# EXTRA_CORS_ORIGINS=https://staging.example.com,https://preview.example.com

# Logging Configuration
# LOG_LEVEL=INFO  # Options: DEBUG, INFO, WARNING, ERROR
# Set to DEBUG for verbose output when debugging with Vercel logs

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
Generated by vercel-fastapi-link skill v1.4.0

Project: $PROJECT_NAME
GitHub Pages: $GITHUB_PAGES_URL
"""

import os
import logging
from datetime import datetime
from typing import Optional, List

from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

# =============================================================================
# Logging Configuration
# =============================================================================
# Configure logging so agents can read Vercel logs for debugging
# Vercel captures stdout/stderr, so we use StreamHandler

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("$PROJECT_NAME".lower().replace(" ", "-"))

# Adjust log level from environment (DEBUG, INFO, WARNING, ERROR)
log_level = os.getenv("LOG_LEVEL", "INFO").upper()
logger.setLevel(getattr(logging, log_level, logging.INFO))

# =============================================================================
# Pydantic Models - Define your request/response schemas here
# =============================================================================

class HealthResponse(BaseModel):
    """Health check response model."""
    status: str = Field(..., example="healthy")
    service: str = Field(..., example="physical-ai-book-api")
    timestamp: str = Field(..., example="2024-01-01T12:00:00Z")


class ChapterSummary(BaseModel):
    """Summary of a chapter for listing."""
    id: int = Field(..., example=1)
    title: str = Field(..., example="Introduction to Physical AI")
    slug: str = Field(..., example="intro")


class ChapterDetail(BaseModel):
    """Full chapter details."""
    id: int
    title: str
    content: str = Field(..., example="Chapter content goes here...")
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


class ChapterListResponse(BaseModel):
    """Response model for chapter listing."""
    chapters: List[ChapterSummary]
    total: int = Field(..., example=3)


class ErrorResponse(BaseModel):
    """Standard error response model."""
    detail: str = Field(..., example="Resource not found")
    error_code: Optional[str] = Field(None, example="NOT_FOUND")


# =============================================================================
# Personalization Models
# =============================================================================

class UserPreferences(BaseModel):
    """User learning preferences."""
    preferred_language: str = Field(default="en", example="en")
    difficulty_level: str = Field(default="beginner", example="intermediate")
    topics_of_interest: List[str] = Field(default=[], example=["robotics", "ai"])
    learning_style: str = Field(default="visual", example="hands-on")
    session_duration_minutes: int = Field(default=30, example=45)


class UserProfile(BaseModel):
    """User profile for personalization."""
    user_id: str = Field(..., example="user-123")
    email: Optional[str] = Field(None, example="user@example.com")
    display_name: Optional[str] = Field(None, example="John Doe")
    preferences: UserPreferences = Field(default_factory=UserPreferences)
    created_at: str = Field(default="", example="2024-01-01T12:00:00Z")
    updated_at: str = Field(default="", example="2024-01-01T12:00:00Z")


class Recommendation(BaseModel):
    """Content recommendation."""
    id: str = Field(..., example="rec-001")
    title: str = Field(..., example="Introduction to ROS 2")
    description: str = Field(..., example="Learn the basics of ROS 2")
    chapter_id: int = Field(..., example=1)
    relevance_score: float = Field(..., ge=0, le=1, example=0.95)
    reason: str = Field(..., example="Based on your interest in robotics")


class RecommendationsResponse(BaseModel):
    """Response with personalized recommendations."""
    user_id: str
    recommendations: List[Recommendation]
    generated_at: str


class LearningPathItem(BaseModel):
    """Single item in learning path."""
    order: int = Field(..., example=1)
    chapter_id: int = Field(..., example=1)
    title: str = Field(..., example="Getting Started")
    status: str = Field(default="not_started", example="in_progress")
    progress_percent: int = Field(default=0, ge=0, le=100, example=50)
    estimated_duration_minutes: int = Field(default=30, example=45)


class LearningPath(BaseModel):
    """User's personalized learning path."""
    user_id: str
    path_id: str = Field(..., example="path-001")
    title: str = Field(default="My Learning Journey", example="Physical AI Mastery")
    items: List[LearningPathItem]
    overall_progress_percent: int = Field(default=0, ge=0, le=100)
    created_at: str
    updated_at: str


class PersonalizationSettings(BaseModel):
    """Settings to apply for personalization."""
    user_id: str = Field(..., example="user-123")
    preferences: UserPreferences
    notifications_enabled: bool = Field(default=True)
    theme: str = Field(default="auto", example="dark")


class PersonalizationApplyResponse(BaseModel):
    """Response after applying personalization settings."""
    success: bool
    message: str
    applied_at: str


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

logger.info("FastAPI application initialized")

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

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint for monitoring."""
    logger.debug("Health check requested")
    return HealthResponse(
        status="healthy",
        service="physical-ai-book-api",
        timestamp=datetime.utcnow().isoformat() + "Z",
    )


@app.get("/")
async def root():
    """Root endpoint with API info."""
    logger.info("Root endpoint accessed")
    return {
        "message": "$PROJECT_NAME",
        "docs": "/docs",
        "health": "/health",
    }

# =============================================================================
# API Routes - Add your endpoints below
# =============================================================================

@app.get("/api/v1/chapters", response_model=ChapterListResponse)
async def list_chapters():
    """List all available chapters."""
    logger.info("Listing all chapters")
    chapters = [
        ChapterSummary(id=1, title="Introduction to Physical AI", slug="intro"),
        ChapterSummary(id=2, title="Sensors and Perception", slug="chapter-1"),
        ChapterSummary(id=3, title="Actuators and Control", slug="chapter-2"),
    ]
    logger.debug(f"Returning {len(chapters)} chapters")
    return ChapterListResponse(chapters=chapters, total=len(chapters))


@app.get(
    "/api/v1/chapters/{chapter_id}",
    response_model=ChapterDetail,
    responses={404: {"model": ErrorResponse}},
)
async def get_chapter(chapter_id: int):
    """Get a specific chapter by ID."""
    logger.info(f"Fetching chapter with id={chapter_id}")

    chapters = {
        1: ChapterDetail(id=1, title="Introduction to Physical AI", content="..."),
        2: ChapterDetail(id=2, title="Sensors and Perception", content="..."),
        3: ChapterDetail(id=3, title="Actuators and Control", content="..."),
    }

    if chapter_id not in chapters:
        logger.warning(f"Chapter not found: id={chapter_id}")
        raise HTTPException(status_code=404, detail="Chapter not found")

    logger.debug(f"Returning chapter: {chapters[chapter_id].title}")
    return chapters[chapter_id]


# =============================================================================
# Personalization Endpoints
# =============================================================================

# In-memory storage for demo (replace with database in production)
_user_profiles: dict = {}
_user_learning_paths: dict = {}


@app.get(
    "/api/personalization/profile",
    response_model=UserProfile,
    responses={404: {"model": ErrorResponse}},
)
async def get_user_profile(user_id: str):
    """Get user profile for personalization."""
    logger.info(f"Fetching profile for user_id={user_id}")

    if user_id in _user_profiles:
        logger.debug(f"Found existing profile for {user_id}")
        return _user_profiles[user_id]

    # Return default profile for new users
    logger.info(f"Creating default profile for new user {user_id}")
    default_profile = UserProfile(
        user_id=user_id,
        preferences=UserPreferences(),
        created_at=datetime.utcnow().isoformat() + "Z",
        updated_at=datetime.utcnow().isoformat() + "Z",
    )
    _user_profiles[user_id] = default_profile
    return default_profile


@app.post("/api/personalization/profile", response_model=UserProfile)
async def update_user_profile(profile: UserProfile):
    """Create or update user profile."""
    logger.info(f"Updating profile for user_id={profile.user_id}")

    profile.updated_at = datetime.utcnow().isoformat() + "Z"
    if profile.user_id not in _user_profiles:
        profile.created_at = profile.updated_at

    _user_profiles[profile.user_id] = profile
    logger.debug(f"Profile saved for {profile.user_id}")
    return profile


@app.get("/api/personalization/recommendations", response_model=RecommendationsResponse)
async def get_recommendations(user_id: str):
    """Get AI-driven content recommendations for user."""
    logger.info(f"Generating recommendations for user_id={user_id}")

    # Get user preferences for personalization
    profile = _user_profiles.get(user_id)
    topics = profile.preferences.topics_of_interest if profile else []

    # Generate recommendations based on preferences (demo logic)
    recommendations = [
        Recommendation(
            id="rec-001",
            title="Introduction to Physical AI",
            description="Start your journey into Physical AI and robotics",
            chapter_id=1,
            relevance_score=0.95,
            reason="Recommended starting point for all learners",
        ),
        Recommendation(
            id="rec-002",
            title="ROS 2 Fundamentals",
            description="Learn the Robot Operating System",
            chapter_id=1,
            relevance_score=0.88,
            reason="Essential foundation for robotics" + (f" - matches your interest in {topics[0]}" if topics else ""),
        ),
        Recommendation(
            id="rec-003",
            title="Simulation with Gazebo",
            description="Practice in a virtual environment",
            chapter_id=2,
            relevance_score=0.82,
            reason="Hands-on learning without hardware",
        ),
    ]

    logger.debug(f"Generated {len(recommendations)} recommendations for {user_id}")
    return RecommendationsResponse(
        user_id=user_id,
        recommendations=recommendations,
        generated_at=datetime.utcnow().isoformat() + "Z",
    )


@app.get("/api/personalization/learning-path", response_model=LearningPath)
async def get_learning_path(user_id: str):
    """Get personalized learning path for user."""
    logger.info(f"Fetching learning path for user_id={user_id}")

    if user_id in _user_learning_paths:
        logger.debug(f"Found existing learning path for {user_id}")
        return _user_learning_paths[user_id]

    # Generate default learning path
    logger.info(f"Creating default learning path for {user_id}")
    now = datetime.utcnow().isoformat() + "Z"

    learning_path = LearningPath(
        user_id=user_id,
        path_id=f"path-{user_id}",
        title="Physical AI Learning Journey",
        items=[
            LearningPathItem(
                order=1,
                chapter_id=1,
                title="Introduction to Physical AI & ROS 2",
                status="not_started",
                progress_percent=0,
                estimated_duration_minutes=60,
            ),
            LearningPathItem(
                order=2,
                chapter_id=2,
                title="Simulation with Gazebo",
                status="not_started",
                progress_percent=0,
                estimated_duration_minutes=90,
            ),
            LearningPathItem(
                order=3,
                chapter_id=3,
                title="Vision-Language-Action Models",
                status="not_started",
                progress_percent=0,
                estimated_duration_minutes=120,
            ),
        ],
        overall_progress_percent=0,
        created_at=now,
        updated_at=now,
    )

    _user_learning_paths[user_id] = learning_path
    return learning_path


@app.post("/api/personalization/apply", response_model=PersonalizationApplyResponse)
async def apply_personalization(settings: PersonalizationSettings):
    """Apply personalization settings for user."""
    logger.info(f"Applying personalization for user_id={settings.user_id}")

    # Update or create profile with new preferences
    profile = _user_profiles.get(settings.user_id)
    now = datetime.utcnow().isoformat() + "Z"

    if profile:
        profile.preferences = settings.preferences
        profile.updated_at = now
    else:
        profile = UserProfile(
            user_id=settings.user_id,
            preferences=settings.preferences,
            created_at=now,
            updated_at=now,
        )

    _user_profiles[settings.user_id] = profile

    logger.info(f"Personalization applied for {settings.user_id}: theme={settings.theme}, notifications={settings.notifications_enabled}")

    return PersonalizationApplyResponse(
        success=True,
        message=f"Personalization settings applied successfully for user {settings.user_id}",
        applied_at=now,
    )


# =============================================================================
# Error Handlers
# =============================================================================

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for unhandled errors."""
    logger.error(f"Unhandled exception: {type(exc).__name__}: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error", "error_code": type(exc).__name__},
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
pydantic>=2.0.0

# Optional: Add more dependencies as needed
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
        echo "pydantic>=2.0.0" >> requirements.txt
        log_success "Added fastapi to requirements.txt"
    else
        log_info "fastapi already in requirements.txt"
        # Ensure pydantic is also present
        if ! grep -q "pydantic" requirements.txt; then
            echo "pydantic>=2.0.0" >> requirements.txt
            log_success "Added pydantic to requirements.txt"
        fi
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
# Deploy to Vercel (if --deploy flag provided)
# =============================================================================

if [[ "$RUN_DEPLOY" == true ]]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🚀 Deploying to Vercel"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    # Check if vercel CLI is installed
    if ! command -v vercel &> /dev/null; then
        log_warning "Vercel CLI not found. Installing..."
        npm install -g vercel
        if ! command -v vercel &> /dev/null; then
            log_error "Failed to install Vercel CLI. Please install manually: npm i -g vercel"
            exit 1
        fi
        log_success "Vercel CLI installed"
    else
        log_success "Vercel CLI found: $(vercel --version)"
    fi

    # Check for VERCEL_TOKEN environment variable for non-interactive deployment
    if [[ -n "$VERCEL_TOKEN" ]]; then
        log_info "Using VERCEL_TOKEN for authentication"
        VERCEL_AUTH="--token $VERCEL_TOKEN"
    else
        VERCEL_AUTH=""
        log_info "No VERCEL_TOKEN found - will use interactive login if needed"
    fi

    # Build deployment command
    DEPLOY_CMD="vercel"

    if [[ "$DEPLOY_PROD" == true ]]; then
        DEPLOY_CMD="$DEPLOY_CMD --prod"
        log_info "Deploying to PRODUCTION"
    else
        log_info "Deploying to preview environment"
    fi

    # Add --yes flag for non-interactive mode
    DEPLOY_CMD="$DEPLOY_CMD --yes"

    # Add token if available
    if [[ -n "$VERCEL_AUTH" ]]; then
        DEPLOY_CMD="$DEPLOY_CMD $VERCEL_AUTH"
    fi

    log_info "Running: vercel $([ "$DEPLOY_PROD" == true ] && echo "--prod") --yes"

    # Run deployment
    DEPLOY_OUTPUT=$($DEPLOY_CMD 2>&1)
    DEPLOY_STATUS=$?

    if [[ $DEPLOY_STATUS -eq 0 ]]; then
        # Extract deployment URL from output
        DEPLOY_URL=$(echo "$DEPLOY_OUTPUT" | grep -oE 'https://[a-zA-Z0-9.-]+\.vercel\.app' | head -1)

        log_success "Deployment successful!"
        echo ""
        if [[ -n "$DEPLOY_URL" ]]; then
            echo "   🌐 Deployment URL: $DEPLOY_URL"
            echo "   📚 API Docs: $DEPLOY_URL/docs"
            echo "   ❤️  Health Check: $DEPLOY_URL/health"
        fi
        echo ""

        # Test the deployed endpoint
        if [[ -n "$DEPLOY_URL" ]]; then
            log_info "Testing deployed /health endpoint..."
            sleep 3  # Wait for deployment to propagate
            HEALTH_CHECK=$(curl -s "$DEPLOY_URL/health" 2>/dev/null || echo "TIMEOUT")
            if echo "$HEALTH_CHECK" | grep -q '"status"'; then
                log_success "Health check passed: $HEALTH_CHECK"
            else
                log_warning "Health check pending - deployment may still be propagating"
                log_info "Try: curl $DEPLOY_URL/health"
            fi
        fi
    else
        log_error "Deployment failed!"
        echo "$DEPLOY_OUTPUT"
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
