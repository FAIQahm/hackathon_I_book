#!/bin/bash
# =============================================================================
# Vercel Environment Variable Manager
# Manages environment variables in Vercel for FastAPI deployment
# Version: 1.0.0
# Agent: BackendIntegrator (deploys) + SecurityLead (validates)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"
# Go up: scripts -> vercel-fastapi-link -> skills -> .claude -> PROJECT_ROOT
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

# Path to env contract from SecurityLead
ENV_CONTRACT="$PROJECT_ROOT/.claude/skills/auth-connect/assets/env_contract.json"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Defaults
ACTION=""
VAR_NAME=""
VAR_VALUE=""
ENVIRONMENT="production"  # production, preview, development
SHOW_VALUES=false

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
Vercel Environment Manager v1.0.0 - Manage Vercel env vars for FastAPI

Usage: env_manager.sh [ACTION] [OPTIONS]

Actions:
  --set NAME VALUE       Set an environment variable in Vercel
  --get NAME             Get an environment variable value
  --list                 List all environment variables
  --remove NAME          Remove an environment variable
  --sync                 Sync from .env file to Vercel
  --validate             Validate env vars match contract (calls SecurityLead)
  --show-contract        Show required/optional vars from contract
  --generate-secrets     Generate required secrets (JWT_SECRET, etc.)
  -h, --help             Show this help message

Options:
  --env ENVIRONMENT      Target environment: production, preview, development
                         (default: production)
  --show-values          Show actual values (careful with secrets!)

Examples:
  # Set a variable
  env_manager.sh --set JWT_SECRET "my-secret-key" --env production

  # List all variables
  env_manager.sh --list

  # Sync from .env to Vercel
  env_manager.sh --sync --env production

  # Validate environment matches contract
  env_manager.sh --validate

  # Generate and set required secrets
  env_manager.sh --generate-secrets

Environment:
  VERCEL_TOKEN           Vercel API token (required for non-interactive)

Contract Location:
  $ENV_CONTRACT
EOF
}

check_vercel_cli() {
    if ! command -v vercel &> /dev/null; then
        log_error "Vercel CLI not found"
        log_info "Install with: npm install -g vercel"
        exit 1
    fi
}

mask_value() {
    local value="$1"
    local len=${#value}
    if [[ $len -le 8 ]]; then
        echo "****"
    else
        echo "${value:0:4}...${value: -4}"
    fi
}

# =============================================================================
# Parse Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --set)
            ACTION="set"
            VAR_NAME="$2"
            VAR_VALUE="$3"
            shift 3
            ;;
        --get)
            ACTION="get"
            VAR_NAME="$2"
            shift 2
            ;;
        --list)
            ACTION="list"
            shift
            ;;
        --remove)
            ACTION="remove"
            VAR_NAME="$2"
            shift 2
            ;;
        --sync)
            ACTION="sync"
            shift
            ;;
        --validate)
            ACTION="validate"
            shift
            ;;
        --show-contract)
            ACTION="show-contract"
            shift
            ;;
        --generate-secrets)
            ACTION="generate-secrets"
            shift
            ;;
        --env)
            ENVIRONMENT="$2"
            shift 2
            ;;
        --show-values)
            SHOW_VALUES=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
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

# =============================================================================
# Banner
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔧 Vercel Environment Manager v1.0.0"
echo "   Agent: BackendIntegrator"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# =============================================================================
# Execute Action
# =============================================================================

case $ACTION in
    set)
        if [[ -z "$VAR_NAME" || -z "$VAR_VALUE" ]]; then
            log_error "Variable name and value required"
            exit 1
        fi

        check_vercel_cli

        log_info "Setting $VAR_NAME in $ENVIRONMENT environment"

        # Vercel env add requires input or --yes flag
        echo "$VAR_VALUE" | vercel env add "$VAR_NAME" "$ENVIRONMENT" --yes 2>/dev/null || \
            vercel env add "$VAR_NAME" "$ENVIRONMENT" <<< "$VAR_VALUE"

        log_success "$VAR_NAME set in $ENVIRONMENT"
        log_info "Value: $(mask_value "$VAR_VALUE")"
        ;;

    get)
        if [[ -z "$VAR_NAME" ]]; then
            log_error "Variable name required"
            exit 1
        fi

        check_vercel_cli

        log_info "Getting $VAR_NAME..."

        VALUE=$(vercel env pull --yes 2>/dev/null && grep "^$VAR_NAME=" .env.local 2>/dev/null | cut -d'=' -f2-)

        if [[ -n "$VALUE" ]]; then
            if [[ "$SHOW_VALUES" == "true" ]]; then
                echo "$VAR_NAME=$VALUE"
            else
                echo "$VAR_NAME=$(mask_value "$VALUE")"
            fi
        else
            log_warn "$VAR_NAME not found"
        fi

        # Cleanup
        rm -f .env.local 2>/dev/null
        ;;

    list)
        check_vercel_cli

        log_info "Listing environment variables for $ENVIRONMENT..."
        echo ""

        vercel env ls "$ENVIRONMENT" 2>/dev/null || vercel env ls
        ;;

    remove)
        if [[ -z "$VAR_NAME" ]]; then
            log_error "Variable name required"
            exit 1
        fi

        check_vercel_cli

        log_warn "Removing $VAR_NAME from $ENVIRONMENT..."

        vercel env rm "$VAR_NAME" "$ENVIRONMENT" --yes

        log_success "$VAR_NAME removed from $ENVIRONMENT"
        ;;

    sync)
        check_vercel_cli

        if [[ ! -f "$PROJECT_ROOT/.env" ]]; then
            log_error ".env file not found at $PROJECT_ROOT/.env"
            exit 1
        fi

        log_info "Syncing .env to Vercel $ENVIRONMENT..."
        echo ""

        # Read .env and set each variable
        while IFS='=' read -r name value; do
            # Skip comments and empty lines
            [[ -z "$name" || "$name" =~ ^# ]] && continue

            # Remove quotes from value
            value="${value%\"}"
            value="${value#\"}"
            value="${value%\'}"
            value="${value#\'}"

            log_info "Setting $name..."
            echo "$value" | vercel env add "$name" "$ENVIRONMENT" --yes 2>/dev/null || true

        done < "$PROJECT_ROOT/.env"

        log_success "Sync complete"
        ;;

    validate)
        log_info "Validating environment against contract..."
        log_info "Contract: $ENV_CONTRACT"
        echo ""

        # Check if SecurityLead's validator exists
        VALIDATOR="$PROJECT_ROOT/.claude/skills/auth-connect/scripts/env_validator.py"

        if [[ ! -f "$VALIDATOR" ]]; then
            log_error "SecurityLead validator not found at $VALIDATOR"
            exit 1
        fi

        # Pull env vars from Vercel first
        check_vercel_cli
        log_info "Pulling environment from Vercel..."

        vercel env pull .env.vercel --yes 2>/dev/null || true

        if [[ -f ".env.vercel" ]]; then
            # Source the vars for validation
            set -a
            source .env.vercel
            set +a
            rm -f .env.vercel
        fi

        # Run SecurityLead's validator
        log_info "Running SecurityLead validation..."
        echo ""

        python3 "$VALIDATOR" --check

        RESULT=$?

        if [[ $RESULT -eq 0 ]]; then
            log_success "All required environment variables are valid!"
        else
            log_error "Environment validation failed!"
            log_info "Check missing variables and set them with:"
            echo "  env_manager.sh --set VAR_NAME \"value\" --env $ENVIRONMENT"
        fi
        ;;

    show-contract)
        if [[ ! -f "$ENV_CONTRACT" ]]; then
            log_error "Contract not found at $ENV_CONTRACT"
            exit 1
        fi

        log_info "Environment Variable Contract"
        echo ""

        echo -e "${CYAN}Required Variables:${NC}"
        python3 -c "
import json
with open('$ENV_CONTRACT') as f:
    contract = json.load(f)
for name, config in contract['variables']['required'].items():
    print(f\"  {name}:\")
    print(f\"    Description: {config['description']}\")
    print(f\"    Owner: {config['owner']}\")
    print(f\"    Deployer: {config['deployer']}\")
    if 'generate_command' in config:
        print(f\"    Generate: {config['generate_command']}\")
    print()
"

        echo -e "${CYAN}Optional Variables:${NC}"
        python3 -c "
import json
with open('$ENV_CONTRACT') as f:
    contract = json.load(f)
for name, config in contract['variables']['optional'].items():
    print(f\"  {name}:\")
    print(f\"    Description: {config['description']}\")
    print(f\"    Owner: {config['owner']}\")
    if 'default' in config:
        print(f\"    Default: {config['default']}\")
    print()
"
        ;;

    generate-secrets)
        log_info "Generating required secrets..."
        echo ""

        # Generate JWT_SECRET
        JWT_SECRET=$(python3 -c "import secrets; print(secrets.token_hex(32))")
        log_success "Generated JWT_SECRET: $(mask_value "$JWT_SECRET")"

        echo ""
        log_info "To set in Vercel:"
        echo "  ./env_manager.sh --set JWT_SECRET \"$JWT_SECRET\" --env production"
        echo ""

        # Check if other secrets are needed
        if [[ -f "$ENV_CONTRACT" ]]; then
            log_info "Other required variables from contract:"
            python3 -c "
import json
with open('$ENV_CONTRACT') as f:
    contract = json.load(f)
for name, config in contract['variables']['required'].items():
    if name != 'JWT_SECRET':
        print(f\"  {name}: {config['description']}\")
"
        fi
        ;;
esac

echo ""
