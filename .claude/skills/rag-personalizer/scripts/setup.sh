#!/bin/bash
# =============================================================================
# RAG Personalizer Setup Script
# Transform textbook content based on 10-dimension user profiles
# Version: 1.0.0
# Agent: AIEngineer
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

# Defaults
PROFILE_ID=""
DIMENSION=""
VALUE=""
CONTENT=""
QUERY=""
MODEL="gpt-4o-mini"
OUTPUT_FORMAT="markdown"

# Actions
ACTION=""

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
RAG Personalizer v1.0.0 - Personalize content based on 10-dimension user profiles

Usage: setup.sh [ACTION] [OPTIONS]

Actions:
  --create-profile ID    Create a new user profile
  --get-profile ID       Get profile details
  --update-profile ID    Update a profile dimension
  --delete-profile ID    Delete a profile
  --list-profiles        List all profiles
  --personalize QUERY    Personalize content for a query

Options:
  --profile ID           Profile ID to use (default: default)
  --dimension DIM        Dimension to update
  --value VAL            Value for dimension
  --content TEXT         Direct content to personalize
  --model MODEL          OpenAI model (default: gpt-4o-mini)
  --output FORMAT        Output format: text, markdown, json (default: markdown)
  -h, --help             Show this help message

Dimensions (10):
  learning_style         visual | auditory | kinesthetic | reading
  knowledge_level        beginner | intermediate | advanced
  learning_pace          slow | moderate | fast
  language               en | ur
  content_depth          overview | standard | deep-dive
  example_preference     theoretical | practical | code-heavy
  difficulty_tolerance   easy | moderate | challenging
  interaction_style      passive | interactive | hands-on
  time_availability      limited | moderate | extensive
  goal_orientation       certification | understanding | application

Environment Variables:
  OPENAI_API_KEY         OpenAI API key (required)

Examples:
  setup.sh --create-profile student1
  setup.sh --update-profile student1 --dimension learning_style --value visual
  setup.sh --personalize "What is ROS 2?" --profile student1
  setup.sh --content "ROS 2 uses DDS..." --profile student1 --output markdown
EOF
}

check_env() {
    if [[ -z "$OPENAI_API_KEY" ]]; then
        log_error "OPENAI_API_KEY environment variable not set"
        echo ""
        log_info "Set environment variable in .env file:"
        echo "  OPENAI_API_KEY=sk-..."
        exit 1
    fi
}

check_python_deps() {
    local missing=()

    python3 -c "import openai" 2>/dev/null || missing+=("openai")
    python3 -c "import pydantic" 2>/dev/null || missing+=("pydantic")

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_warn "Missing Python packages: ${missing[*]}"
        log_info "Installing dependencies..."
        pip install "${missing[@]}" --quiet
        log_success "Dependencies installed"
    fi
}

# =============================================================================
# Parse Arguments
# =============================================================================

DIMENSIONS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        --create-profile)
            ACTION="create-profile"
            PROFILE_ID="$2"
            shift 2
            ;;
        --get-profile)
            ACTION="get-profile"
            PROFILE_ID="$2"
            shift 2
            ;;
        --update-profile)
            ACTION="update-profile"
            PROFILE_ID="$2"
            shift 2
            ;;
        --delete-profile)
            ACTION="delete-profile"
            PROFILE_ID="$2"
            shift 2
            ;;
        --list-profiles)
            ACTION="list-profiles"
            shift
            ;;
        --personalize)
            ACTION="personalize"
            QUERY="$2"
            shift 2
            ;;
        --profile)
            PROFILE_ID="$2"
            shift 2
            ;;
        --dimension)
            DIMENSION="$2"
            shift 2
            ;;
        --value)
            VALUE="$2"
            shift 2
            ;;
        --content)
            CONTENT="$2"
            shift 2
            ;;
        --model)
            MODEL="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FORMAT="$2"
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

check_env
check_python_deps

# =============================================================================
# Banner
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎯 RAG Personalizer v1.0.0"
echo "   Agent: AIEngineer"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# =============================================================================
# Execute Action
# =============================================================================

case $ACTION in
    create-profile)
        if [[ -z "$PROFILE_ID" ]]; then
            log_error "Profile ID required"
            exit 1
        fi

        log_info "Creating profile: $PROFILE_ID"

        DIMS_ARG=""
        if [[ -n "$DIMENSION" ]] && [[ -n "$VALUE" ]]; then
            DIMS_ARG="--dimension $DIMENSION $VALUE"
        fi

        python3 "$SCRIPT_DIR/personalizer.py" create-profile "$PROFILE_ID" $DIMS_ARG

        log_success "Profile created"
        ;;

    get-profile)
        if [[ -z "$PROFILE_ID" ]]; then
            log_error "Profile ID required"
            exit 1
        fi

        log_info "Getting profile: $PROFILE_ID"
        echo ""

        python3 "$SCRIPT_DIR/personalizer.py" get-profile "$PROFILE_ID"
        ;;

    update-profile)
        if [[ -z "$PROFILE_ID" ]]; then
            log_error "Profile ID required"
            exit 1
        fi

        if [[ -z "$DIMENSION" ]] || [[ -z "$VALUE" ]]; then
            log_error "--dimension and --value are required"
            exit 1
        fi

        log_info "Updating profile: $PROFILE_ID"
        log_info "Setting $DIMENSION = $VALUE"

        python3 "$SCRIPT_DIR/personalizer.py" update-profile "$PROFILE_ID" \
            --dimension "$DIMENSION" --value "$VALUE"

        log_success "Profile updated"
        ;;

    delete-profile)
        if [[ -z "$PROFILE_ID" ]]; then
            log_error "Profile ID required"
            exit 1
        fi

        log_warn "Deleting profile: $PROFILE_ID"
        read -p "Are you sure? (y/N) " confirm

        if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
            python3 "$SCRIPT_DIR/personalizer.py" delete-profile "$PROFILE_ID"
            log_success "Profile deleted"
        else
            log_info "Cancelled"
        fi
        ;;

    list-profiles)
        log_info "Listing all profiles"
        echo ""

        python3 "$SCRIPT_DIR/personalizer.py" list-profiles
        ;;

    personalize)
        PROFILE_ID="${PROFILE_ID:-default}"

        if [[ -z "$CONTENT" ]] && [[ -z "$QUERY" ]]; then
            log_error "Either --content or --personalize QUERY required"
            exit 1
        fi

        log_info "Profile: $PROFILE_ID"
        log_info "Model: $MODEL"
        log_info "Output: $OUTPUT_FORMAT"

        if [[ -n "$QUERY" ]]; then
            log_info "Query: $QUERY"
        fi
        echo ""

        # Use content if provided, otherwise use query as content
        PERSONALIZE_CONTENT="${CONTENT:-$QUERY}"

        python3 "$SCRIPT_DIR/personalizer.py" personalize \
            --content "$PERSONALIZE_CONTENT" \
            --profile "$PROFILE_ID" \
            --model "$MODEL" \
            --output "$OUTPUT_FORMAT" \
            ${QUERY:+--query "$QUERY"}
        ;;
esac

echo ""
