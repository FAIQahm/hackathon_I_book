#!/bin/bash
# =============================================================================
# Skill Creator Script
# Creates a new skill with standard directory structure and files
# =============================================================================
# Usage: ./create-skill.sh --name <skill-name> --description "<description>"
#        ./create-skill.sh -n my-skill -d "What this skill does"
#
# Options:
#   -n, --name         Skill name (kebab-case, required)
#   -d, --description  Skill description (required)
#   -v, --version      Initial version (default: 1.0.0)
#   -t, --title        Display title (default: generated from name)
#   --no-script        Skip creating setup.sh script
#   --no-assets        Skip creating assets directory
#   -h, --help         Show this help message
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
SKILL_VERSION="1.0.0"
CREATE_SCRIPT=true
CREATE_ASSETS=true
SKILL_NAME=""
SKILL_DESCRIPTION=""
SKILL_TITLE=""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
TEMPLATE_FILE="$SCRIPT_DIR/../../templates/skill-template.md"
SKILLS_DIR="$PROJECT_ROOT/.claude/skills"

# =============================================================================
# Functions
# =============================================================================

show_help() {
    cat << EOF
Skill Creator - Generate new skills with standard structure

Usage: $(basename "$0") [OPTIONS]

Required:
  -n, --name NAME           Skill name in kebab-case (e.g., my-awesome-skill)
  -d, --description DESC    Short description of what the skill does

Optional:
  -v, --version VERSION     Initial version (default: 1.0.0)
  -t, --title TITLE         Display title (default: auto-generated from name)
  --no-script               Don't create setup.sh script
  --no-assets               Don't create assets directory
  -h, --help                Show this help message

Examples:
  $(basename "$0") -n github-pages-deploy -d "Deploy to GitHub Pages"
  $(basename "$0") --name urdu-rtl-styler --description "Apply RTL styles" -v 1.0.0
  $(basename "$0") -n simple-skill -d "A simple skill" --no-assets

Output Structure:
  .claude/skills/<name>/
  ├── SKILL.md              # Main documentation
  ├── assets/               # Static files (CSS, configs, etc.)
  └── scripts/
      └── setup.sh          # Setup automation script

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

# Convert kebab-case to Title Case
kebab_to_title() {
    echo "$1" | sed 's/-/ /g' | awk '{for(i=1;i<=NF;i++) $i=toupper(substr($i,1,1)) tolower(substr($i,2))}1'
}

# Validate skill name (kebab-case)
validate_name() {
    local name="$1"
    if [[ ! "$name" =~ ^[a-z][a-z0-9]*(-[a-z0-9]+)*$ ]]; then
        log_error "Invalid skill name: '$name'"
        log_error "Name must be kebab-case (e.g., my-skill-name)"
        exit 1
    fi
}

# =============================================================================
# Parse Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--name)
            SKILL_NAME="$2"
            shift 2
            ;;
        -d|--description)
            SKILL_DESCRIPTION="$2"
            shift 2
            ;;
        -v|--version)
            SKILL_VERSION="$2"
            shift 2
            ;;
        -t|--title)
            SKILL_TITLE="$2"
            shift 2
            ;;
        --no-script)
            CREATE_SCRIPT=false
            shift
            ;;
        --no-assets)
            CREATE_ASSETS=false
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
# Validation
# =============================================================================

if [[ -z "$SKILL_NAME" ]]; then
    log_error "Skill name is required"
    echo "Use: $(basename "$0") --name <skill-name> --description \"<description>\""
    exit 1
fi

if [[ -z "$SKILL_DESCRIPTION" ]]; then
    log_error "Skill description is required"
    echo "Use: $(basename "$0") --name <skill-name> --description \"<description>\""
    exit 1
fi

validate_name "$SKILL_NAME"

# Generate title if not provided
if [[ -z "$SKILL_TITLE" ]]; then
    SKILL_TITLE=$(kebab_to_title "$SKILL_NAME")
fi

# Check if skill already exists
SKILL_DIR="$SKILLS_DIR/$SKILL_NAME"
if [[ -d "$SKILL_DIR" ]]; then
    log_error "Skill '$SKILL_NAME' already exists at: $SKILL_DIR"
    exit 1
fi

# =============================================================================
# Create Skill Structure
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔧 Creating skill: $SKILL_NAME"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
log_info "Name: $SKILL_NAME"
log_info "Title: $SKILL_TITLE"
log_info "Version: $SKILL_VERSION"
log_info "Description: $SKILL_DESCRIPTION"
echo ""

# Create directories
log_info "Creating directory structure..."
mkdir -p "$SKILL_DIR/scripts"

if [[ "$CREATE_ASSETS" == true ]]; then
    mkdir -p "$SKILL_DIR/assets"
    log_success "Created assets/ directory"
fi

log_success "Created scripts/ directory"

# =============================================================================
# Create SKILL.md
# =============================================================================

log_info "Creating SKILL.md..."

if [[ -f "$TEMPLATE_FILE" ]]; then
    # Use template file
    sed -e "s/{{SKILL_NAME}}/$SKILL_NAME/g" \
        -e "s/{{SKILL_TITLE}}/$SKILL_TITLE/g" \
        -e "s/{{SKILL_VERSION}}/$SKILL_VERSION/g" \
        -e "s/{{SKILL_DESCRIPTION}}/$SKILL_DESCRIPTION/g" \
        "$TEMPLATE_FILE" > "$SKILL_DIR/SKILL.md"
else
    # Create minimal SKILL.md if template not found
    cat > "$SKILL_DIR/SKILL.md" << EOF
---
name: $SKILL_NAME
description: |
  $SKILL_DESCRIPTION
version: $SKILL_VERSION
inputs: {}
---

# $SKILL_TITLE

$SKILL_DESCRIPTION

## Quick Setup

\`\`\`bash
.claude/skills/$SKILL_NAME/scripts/setup.sh
\`\`\`

## Bundled Resources

(Add your resources here)

## Usage Instructions

1. Run setup script
2. Configure as needed
3. Verify installation

## Verification Checklist

- [ ] Setup completed
- [ ] Configuration applied
- [ ] Functionality verified

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Problem | Solution |

EOF
fi

log_success "Created SKILL.md"

# =============================================================================
# Create setup.sh
# =============================================================================

if [[ "$CREATE_SCRIPT" == true ]]; then
    log_info "Creating setup.sh..."

    cat > "$SKILL_DIR/scripts/setup.sh" << 'SETUP_EOF'
#!/bin/bash
# =============================================================================
# SKILL_TITLE Setup Script
# Generated by create-skill.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"

echo "🚀 Setting up SKILL_NAME..."
echo ""

# =============================================================================
# Setup Logic (customize this section)
# =============================================================================

# Example: Copy assets
# if [[ -d "$SKILL_DIR/assets" ]]; then
#     cp -r "$SKILL_DIR/assets/"* ./target/directory/
# fi

# Example: Append to config file
# cat "$SKILL_DIR/assets/config-snippet.txt" >> ./config.file

echo "✅ Setup complete!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📋 Next steps:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  1. (Add your next steps here)"
echo "  2. ..."
echo ""
SETUP_EOF

    # Replace placeholders in setup.sh
    sed -i "s/SKILL_TITLE/$SKILL_TITLE/g" "$SKILL_DIR/scripts/setup.sh"
    sed -i "s/SKILL_NAME/$SKILL_NAME/g" "$SKILL_DIR/scripts/setup.sh"

    # Make executable
    chmod +x "$SKILL_DIR/scripts/setup.sh"

    log_success "Created setup.sh (executable)"
fi

# =============================================================================
# Create placeholder asset (optional)
# =============================================================================

if [[ "$CREATE_ASSETS" == true ]]; then
    cat > "$SKILL_DIR/assets/.gitkeep" << EOF
# Placeholder to keep assets directory in git
# Add your static files here (CSS, configs, templates, etc.)
EOF
    log_success "Created assets/.gitkeep"
fi

# =============================================================================
# Summary
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✨ Skill created successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📁 Location: $SKILL_DIR"
echo ""
echo "📂 Structure:"
echo "   .claude/skills/$SKILL_NAME/"
echo "   ├── SKILL.md"
if [[ "$CREATE_ASSETS" == true ]]; then
echo "   ├── assets/"
echo "   │   └── .gitkeep"
fi
if [[ "$CREATE_SCRIPT" == true ]]; then
echo "   └── scripts/"
echo "       └── setup.sh"
fi
echo ""
echo "📋 Next steps:"
echo "   1. Edit SKILL.md to add documentation"
if [[ "$CREATE_ASSETS" == true ]]; then
echo "   2. Add static files to assets/"
fi
if [[ "$CREATE_SCRIPT" == true ]]; then
echo "   3. Customize scripts/setup.sh"
fi
echo "   4. Test the skill"
echo "   5. Commit: git add .claude/skills/$SKILL_NAME && git commit -m \"feat(skill): add $SKILL_NAME\""
echo ""
