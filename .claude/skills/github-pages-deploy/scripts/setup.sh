#!/bin/bash
# GitHub Pages Deploy Setup Script
# Usage: ./setup.sh <organization> <repository>

set -e

ORG="${1:?Usage: $0 <organization> <repository>}"
REPO="${2:?Usage: $0 <organization> <repository>}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"

echo "Setting up GitHub Pages deployment..."
echo "  Organization: $ORG"
echo "  Repository: $REPO"

# Create workflow directory
mkdir -p .github/workflows

# Copy and customize deploy.yml
sed -e "s/<organization>/$ORG/g" -e "s/<repository>/$REPO/g" \
    "$SKILL_DIR/assets/deploy.yml" > .github/workflows/deploy.yml

echo "✅ Created .github/workflows/deploy.yml"

# Check if docusaurus.config.js exists
if [ -f "docusaurus.config.js" ]; then
    echo ""
    echo "📝 Update docusaurus.config.js with:"
    echo ""
    echo "  url: 'https://$ORG.github.io',"
    echo "  baseUrl: '/$REPO/',"
    echo "  organizationName: '$ORG',"
    echo "  projectName: '$REPO',"
    echo ""
else
    echo "⚠️  docusaurus.config.js not found - create it first"
fi

echo ""
echo "Next steps:"
echo "  1. Update docusaurus.config.js with the values above"
echo "  2. Enable GitHub Pages: Settings → Pages → Source: GitHub Actions"
echo "  3. Push to main branch to trigger deployment"
