#!/bin/bash
# =============================================================================
# Auth Connect Setup Script
# JWT-based authentication and authorization for FastAPI
# Version: 1.0.0
# Agent: SecurityLead
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Actions
ACTION=""
EMAIL=""
PASSWORD=""
ROLE="student"
TOKEN=""
API_KEY_NAME=""
API_KEY_PERMISSIONS="read"

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

log_success() {
    echo -e "${GREEN}✓${NC} $1"
}

log_error() {
    echo -e "${RED}✗${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

show_help() {
    cat << EOF
Auth Connect v1.0.0 - JWT Authentication for FastAPI

Usage: setup.sh [ACTION] [OPTIONS]

Actions:
  --init                 Initialize auth system (create tables)
  --generate-secret      Generate a new JWT secret
  --create-user          Create a new user
  --list-roles           List all available roles
  --verify-token TOKEN   Verify a JWT token
  --generate-api-key     Generate a new API key
  --hash-password PASS   Hash a password
  --validate-password    Validate password strength
  --show-config          Show current configuration
  --health               Check system health
  -h, --help             Show this help message

Options:
  --email EMAIL          User email (for --create-user)
  --password PASS        User password (for --create-user)
  --role ROLE            User role (default: student)
  --name NAME            API key name (for --generate-api-key)
  --permissions PERMS    API key permissions (default: read)

Environment Variables:
  JWT_SECRET             Secret key for JWT tokens (required)
  DATABASE_URL           PostgreSQL connection string (for --init)

Examples:
  # Generate a new JWT secret
  setup.sh --generate-secret

  # Create a user
  setup.sh --create-user --email user@example.com --password secret123 --role student

  # List all roles
  setup.sh --list-roles

  # Verify a token
  setup.sh --verify-token eyJhbGciOiJIUzI1NiIs...

  # Generate an API key
  setup.sh --generate-api-key --name "My App" --permissions "read,write"

  # Hash a password
  setup.sh --hash-password "mysecretpassword"

  # Check configuration
  setup.sh --show-config
EOF
}

check_jwt_secret() {
    if [[ -z "$JWT_SECRET" ]]; then
        log_error "JWT_SECRET environment variable not set"
        echo ""
        log_info "Generate a secret with:"
        echo "  ./setup.sh --generate-secret"
        exit 1
    fi
}

check_python_deps() {
    local missing=()

    python3 -c "import jwt" 2>/dev/null || missing+=("PyJWT")
    python3 -c "import bcrypt" 2>/dev/null || missing+=("bcrypt")

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_warn "Missing Python packages: ${missing[*]}"
        log_info "Installing dependencies..."
        pip install "${missing[@]}" --quiet 2>/dev/null || pip install "${missing[@]}" --user --quiet
        log_success "Dependencies installed"
    fi
}

# =============================================================================
# Parse Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --init)
            ACTION="init"
            shift
            ;;
        --generate-secret)
            ACTION="generate-secret"
            shift
            ;;
        --create-user)
            ACTION="create-user"
            shift
            ;;
        --list-roles)
            ACTION="list-roles"
            shift
            ;;
        --verify-token)
            ACTION="verify-token"
            TOKEN="$2"
            shift 2
            ;;
        --generate-api-key)
            ACTION="generate-api-key"
            shift
            ;;
        --hash-password)
            ACTION="hash-password"
            PASSWORD="$2"
            shift 2
            ;;
        --validate-password)
            ACTION="validate-password"
            PASSWORD="$2"
            shift 2
            ;;
        --show-config)
            ACTION="show-config"
            shift
            ;;
        --health)
            ACTION="health"
            shift
            ;;
        --email)
            EMAIL="$2"
            shift 2
            ;;
        --password)
            PASSWORD="$2"
            shift 2
            ;;
        --role)
            ROLE="$2"
            shift 2
            ;;
        --name)
            API_KEY_NAME="$2"
            shift 2
            ;;
        --permissions)
            API_KEY_PERMISSIONS="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
done

# =============================================================================
# Validate
# =============================================================================

if [[ -z "$ACTION" ]]; then
    log_error "No action specified"
    echo ""
    show_help
    exit 1
fi

# Load .env if exists
if [[ -f "$PROJECT_ROOT/.env" ]]; then
    set -a
    source "$PROJECT_ROOT/.env"
    set +a
fi

# =============================================================================
# Banner
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔐 Auth Connect v1.0.0"
echo "   Agent: SecurityLead"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# =============================================================================
# Execute Action
# =============================================================================

case $ACTION in
    init)
        log_info "Initializing auth system..."

        if [[ -z "$DATABASE_URL" ]]; then
            log_error "DATABASE_URL environment variable not set"
            exit 1
        fi

        # SQL to create tables
        SQL=$(cat << 'EOSQL'
-- Users table
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    role VARCHAR(50) DEFAULT 'student',
    is_active BOOLEAN DEFAULT true,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Sessions table
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    refresh_token VARCHAR(500) NOT NULL,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    ip_address VARCHAR(45),
    user_agent TEXT
);

-- API Keys table
CREATE TABLE IF NOT EXISTS api_keys (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    name VARCHAR(100) NOT NULL,
    key_hash VARCHAR(255) NOT NULL,
    prefix VARCHAR(20) NOT NULL,
    permissions JSONB DEFAULT '["read"]',
    is_active BOOLEAN DEFAULT true,
    last_used_at TIMESTAMP,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP
);

-- Indexes
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_sessions_refresh_token ON sessions(refresh_token);
CREATE INDEX IF NOT EXISTS idx_api_keys_prefix ON api_keys(prefix);
EOSQL
)

        echo "$SQL" | psql "$DATABASE_URL"
        log_success "Auth tables created"
        ;;

    generate-secret)
        log_info "Generating JWT secret..."
        check_python_deps

        python3 "$SCRIPT_DIR/auth.py" generate-secret
        echo ""
        log_info "Add this to your .env file"
        ;;

    create-user)
        if [[ -z "$EMAIL" ]]; then
            log_error "--email is required"
            exit 1
        fi

        if [[ -z "$PASSWORD" ]]; then
            log_error "--password is required"
            exit 1
        fi

        log_info "Creating user: $EMAIL"
        log_info "Role: $ROLE"

        check_jwt_secret
        check_python_deps

        # Validate password
        VALID=$(python3 "$SCRIPT_DIR/auth.py" validate-password "$PASSWORD" 2>&1)
        if [[ "$VALID" == Invalid* ]]; then
            log_error "$VALID"
            exit 1
        fi

        # Hash password
        HASH=$(python3 "$SCRIPT_DIR/auth.py" hash-password "$PASSWORD" | grep "Hashed:" | cut -d' ' -f2)

        if [[ -n "$DATABASE_URL" ]]; then
            # Insert into database
            psql "$DATABASE_URL" -c "INSERT INTO users (email, password_hash, role) VALUES ('$EMAIL', '$HASH', '$ROLE');"
            log_success "User created in database"
        else
            log_warn "DATABASE_URL not set - showing hash only"
            echo "Password hash: $HASH"
        fi
        ;;

    list-roles)
        log_info "Available roles:"
        echo ""
        check_python_deps

        python3 "$SCRIPT_DIR/auth.py" list-roles
        ;;

    verify-token)
        if [[ -z "$TOKEN" ]]; then
            log_error "Token required"
            exit 1
        fi

        log_info "Verifying token..."

        check_jwt_secret
        check_python_deps

        python3 "$SCRIPT_DIR/auth.py" verify-token "$TOKEN"
        ;;

    generate-api-key)
        if [[ -z "$API_KEY_NAME" ]]; then
            log_error "--name is required"
            exit 1
        fi

        log_info "Generating API key..."
        log_info "Name: $API_KEY_NAME"
        log_info "Permissions: $API_KEY_PERMISSIONS"

        check_python_deps

        # Convert comma-separated to space-separated for argparse
        PERMS=$(echo "$API_KEY_PERMISSIONS" | tr ',' ' ')

        python3 "$SCRIPT_DIR/auth.py" generate-api-key --name "$API_KEY_NAME" --permissions $PERMS
        ;;

    hash-password)
        if [[ -z "$PASSWORD" ]]; then
            log_error "Password required"
            exit 1
        fi

        check_python_deps

        python3 "$SCRIPT_DIR/auth.py" hash-password "$PASSWORD"
        ;;

    validate-password)
        if [[ -z "$PASSWORD" ]]; then
            log_error "Password required"
            exit 1
        fi

        check_python_deps

        python3 "$SCRIPT_DIR/auth.py" validate-password "$PASSWORD"
        ;;

    show-config)
        log_info "Current configuration:"
        echo ""
        check_python_deps

        python3 "$SCRIPT_DIR/auth.py" show-config
        ;;

    health)
        log_info "Running health check..."
        echo ""

        # Check Python
        if python3 --version &>/dev/null; then
            log_success "Python: $(python3 --version)"
        else
            log_error "Python not found"
        fi

        # Check dependencies
        if python3 -c "import jwt" 2>/dev/null; then
            log_success "PyJWT: installed"
        else
            log_warn "PyJWT: not installed"
        fi

        if python3 -c "import bcrypt" 2>/dev/null; then
            log_success "bcrypt: installed"
        else
            log_warn "bcrypt: not installed"
        fi

        # Check env vars
        if [[ -n "$JWT_SECRET" ]]; then
            log_success "JWT_SECRET: configured"
        else
            log_warn "JWT_SECRET: not set"
        fi

        if [[ -n "$DATABASE_URL" ]]; then
            log_success "DATABASE_URL: configured"
        else
            log_warn "DATABASE_URL: not set"
        fi

        # Check config files
        if [[ -f "$SKILL_DIR/assets/auth_config.json" ]]; then
            log_success "auth_config.json: found"
        else
            log_error "auth_config.json: missing"
        fi

        if [[ -f "$SKILL_DIR/assets/roles.json" ]]; then
            log_success "roles.json: found"
        else
            log_error "roles.json: missing"
        fi
        ;;
esac

echo ""
