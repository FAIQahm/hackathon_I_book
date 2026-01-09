#!/bin/bash
# =============================================================================
# Translation Sync Setup Script
# Synchronize English MDX files with Urdu translations for Docusaurus i18n
# Version: 1.1.0
# Agent: Linguist
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
SOURCE_DIR="docs"
TARGET_DIR="i18n/ur/docusaurus-plugin-content-docs/current"
MODEL="gpt-4o-mini"
DRY_RUN=false
PRESERVE_TERMS=true
INCREMENTAL=false
USE_TM=true

# Actions
ACTION=""
FILE_PATH=""
TERM=""
URDU=""
KEEP_ENGLISH=false
CONTEXT=""
TM_OUTPUT=""
TM_FORMAT="json"
TM_INPUT=""

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
Translation Sync v1.1.0 - Synchronize English MDX files with Urdu translations

Usage: setup.sh [ACTION] [OPTIONS]

Actions:
  --status             Show sync status for all files
  --sync               Sync all changed files
  --translate FILE     Translate a specific file
  --diff FILE          Show changes since last translation (v1.1.0)
  --validate           Validate all translations
  --add-term TERM      Add term to glossary
  --list-terms         List all glossary terms

Translation Memory (v1.1.0):
  --tm-stats           Show translation memory statistics
  --tm-export FILE     Export translation memory (--format json|tmx)
  --tm-import FILE     Import translation memory from JSON
  --tm-clear           Clear translation memory

Options:
  --urdu TEXT          Urdu translation/transliteration for term
  --keep-english       Keep English term alongside Urdu
  --context TEXT       Context for the term
  --preserve-terms     Keep technical terms in English (default: true)
  --incremental        Only translate changed sections (v1.1.0)
  --no-tm              Disable translation memory (v1.1.0)
  --source DIR         Source directory (default: docs)
  --target DIR         Target directory (default: i18n/ur/.../current)
  --model MODEL        OpenAI model (default: gpt-4o-mini)
  --format FORMAT      Export format for TM: json, tmx (default: json)
  --dry-run            Show what would be translated
  -h, --help           Show this help message

Environment Variables (required):
  OPENAI_API_KEY       OpenAI API key for translation

Examples:
  # Check status
  setup.sh --status

  # Sync with incremental mode (only changed sections)
  setup.sh --sync --incremental

  # Show what changed before re-translating
  setup.sh --diff docs/chapter-1/index.md

  # Translate a single file
  setup.sh --translate docs/intro.md

  # Translate incrementally (reuse unchanged sections)
  setup.sh --translate docs/chapter-1/index.md --incremental

  # Manage glossary terms
  setup.sh --add-term "ROS 2" --urdu "آر او ایس ٹو" --keep-english
  setup.sh --list-terms

  # Translation memory commands
  setup.sh --tm-stats
  setup.sh --tm-export translations.json
  setup.sh --tm-export translations.tmx --format tmx
  setup.sh --tm-import external_tm.json
  setup.sh --tm-clear

  # Validate translations
  setup.sh --validate

New in v1.1.0:
  • --diff: Preview changes before re-translating
  • --incremental: Only translate changed sections (saves ~60-80% API cost)
  • Translation Memory: Reuse previous translations for consistency
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
    python3 -c "import rich" 2>/dev/null || missing+=("rich")
    python3 -c "import yaml" 2>/dev/null || missing+=("pyyaml")

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
        --status)
            ACTION="status"
            shift
            ;;
        --sync)
            ACTION="sync"
            shift
            ;;
        --translate)
            ACTION="translate"
            FILE_PATH="$2"
            shift 2
            ;;
        --diff)
            ACTION="diff"
            FILE_PATH="$2"
            shift 2
            ;;
        --validate)
            ACTION="validate"
            shift
            ;;
        --add-term)
            ACTION="add-term"
            TERM="$2"
            shift 2
            ;;
        --list-terms)
            ACTION="list-terms"
            shift
            ;;
        --tm-stats)
            ACTION="tm-stats"
            shift
            ;;
        --tm-export)
            ACTION="tm-export"
            TM_OUTPUT="$2"
            shift 2
            ;;
        --tm-import)
            ACTION="tm-import"
            TM_INPUT="$2"
            shift 2
            ;;
        --tm-clear)
            ACTION="tm-clear"
            shift
            ;;
        --urdu)
            URDU="$2"
            shift 2
            ;;
        --keep-english)
            KEEP_ENGLISH=true
            shift
            ;;
        --context)
            CONTEXT="$2"
            shift 2
            ;;
        --preserve-terms)
            PRESERVE_TERMS=true
            shift
            ;;
        --incremental)
            INCREMENTAL=true
            shift
            ;;
        --no-tm)
            USE_TM=false
            shift
            ;;
        --source)
            SOURCE_DIR="$2"
            shift 2
            ;;
        --target)
            TARGET_DIR="$2"
            shift 2
            ;;
        --model)
            MODEL="$2"
            shift 2
            ;;
        --format)
            TM_FORMAT="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
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

# Check env for actions that need OpenAI
if [[ "$ACTION" == "sync" || "$ACTION" == "translate" ]]; then
    check_env
    check_python_deps
fi

# =============================================================================
# Banner
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🌐 Translation Sync v1.1.0"
echo "   Agent: Linguist"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# =============================================================================
# Execute Action
# =============================================================================

case $ACTION in
    status)
        log_info "Checking translation status..."
        echo ""

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" status
        ;;

    sync)
        log_info "Source: $SOURCE_DIR"
        log_info "Target: $TARGET_DIR"
        log_info "Model: $MODEL"
        log_info "Incremental: $INCREMENTAL"
        log_info "Translation Memory: $USE_TM"

        if [[ "$DRY_RUN" == "true" ]]; then
            log_warn "DRY RUN - No files will be written"
        fi
        echo ""

        cd "$PROJECT_ROOT"

        ARGS="--model $MODEL"
        [[ "$DRY_RUN" == "true" ]] && ARGS="$ARGS --dry-run"
        [[ "$INCREMENTAL" == "true" ]] && ARGS="$ARGS --incremental"
        [[ "$USE_TM" == "false" ]] && ARGS="$ARGS --no-tm"

        python3 "$SCRIPT_DIR/translator.py" sync $ARGS

        log_success "Sync complete"
        ;;

    translate)
        if [[ -z "$FILE_PATH" ]]; then
            log_error "File path required"
            exit 1
        fi

        log_info "Translating: $FILE_PATH"
        log_info "Model: $MODEL"
        log_info "Incremental: $INCREMENTAL"
        log_info "Translation Memory: $USE_TM"

        if [[ "$DRY_RUN" == "true" ]]; then
            log_warn "DRY RUN - No files will be written"
        fi
        echo ""

        cd "$PROJECT_ROOT"

        ARGS="--model $MODEL"
        [[ "$DRY_RUN" == "true" ]] && ARGS="$ARGS --dry-run"
        [[ "$INCREMENTAL" == "true" ]] && ARGS="$ARGS --incremental"
        [[ "$USE_TM" == "false" ]] && ARGS="$ARGS --no-tm"

        python3 "$SCRIPT_DIR/translator.py" translate "$FILE_PATH" $ARGS

        log_success "Translation complete"
        ;;

    diff)
        if [[ -z "$FILE_PATH" ]]; then
            log_error "File path required"
            exit 1
        fi

        log_info "Showing changes for: $FILE_PATH"
        echo ""

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" diff "$FILE_PATH"
        ;;

    validate)
        log_info "Validating translations..."
        echo ""

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" validate
        ;;

    add-term)
        if [[ -z "$TERM" ]]; then
            log_error "Term required"
            exit 1
        fi

        if [[ -z "$URDU" ]]; then
            log_error "--urdu is required when adding a term"
            exit 1
        fi

        log_info "Adding term: $TERM"
        log_info "Urdu: $URDU"
        log_info "Keep English: $KEEP_ENGLISH"

        cd "$PROJECT_ROOT"

        KEEP_FLAG=""
        if [[ "$KEEP_ENGLISH" == "true" ]]; then
            KEEP_FLAG="--keep-english"
        fi

        CONTEXT_FLAG=""
        if [[ -n "$CONTEXT" ]]; then
            CONTEXT_FLAG="--context \"$CONTEXT\""
        fi

        python3 "$SCRIPT_DIR/translator.py" add-term "$TERM" --urdu "$URDU" $KEEP_FLAG $CONTEXT_FLAG

        log_success "Term added to glossary"
        ;;

    list-terms)
        log_info "Glossary terms:"
        echo ""

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" list-terms
        ;;

    tm-stats)
        log_info "Translation Memory Statistics"
        echo ""

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" tm-stats
        ;;

    tm-export)
        if [[ -z "$TM_OUTPUT" ]]; then
            log_error "Output file path required"
            exit 1
        fi

        log_info "Exporting translation memory to: $TM_OUTPUT"
        log_info "Format: $TM_FORMAT"

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" tm-export "$TM_OUTPUT" --format "$TM_FORMAT"

        log_success "Translation memory exported"
        ;;

    tm-import)
        if [[ -z "$TM_INPUT" ]]; then
            log_error "Input file path required"
            exit 1
        fi

        log_info "Importing translation memory from: $TM_INPUT"

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" tm-import "$TM_INPUT"

        log_success "Translation memory imported"
        ;;

    tm-clear)
        log_warn "Clearing translation memory..."

        cd "$PROJECT_ROOT"
        python3 "$SCRIPT_DIR/translator.py" tm-clear

        log_success "Translation memory cleared"
        ;;
esac

echo ""
