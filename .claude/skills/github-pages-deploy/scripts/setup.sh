#!/bin/bash
# GitHub Pages Deploy Setup Script
# Usage: ./setup.sh <organization> <repository> [branch]

set -e

ORG="${1:?Usage: $0 <organization> <repository> [branch]}"
REPO="${2:?Usage: $0 <organization> <repository> [branch]}"

# Auto-detect branch if not provided
if [ -n "$3" ]; then
    BRANCH="$3"
elif git rev-parse --git-dir > /dev/null 2>&1; then
    BRANCH=$(git branch --show-current 2>/dev/null || echo "main")
    if [ -z "$BRANCH" ]; then
        BRANCH="main"
    fi
else
    BRANCH="main"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"

echo "🚀 Setting up GitHub Pages deployment..."
echo "   Organization: $ORG"
echo "   Repository: $REPO"
echo "   Branch: $BRANCH"
echo ""

# Create workflow directory and copy deploy.yml with branch substitution
mkdir -p .github/workflows
sed -e "s/<organization>/$ORG/g" -e "s/<repository>/$REPO/g" -e "s/branches: \[main\]/branches: [$BRANCH]/g" \
    "$SKILL_DIR/assets/deploy.yml" > .github/workflows/deploy.yml
echo "✅ Created .github/workflows/deploy.yml (Node.js 20, branch: $BRANCH)"

# Create homepage redirect with useBaseUrl
mkdir -p src/pages
cat > src/pages/index.js << 'EOF'
import React from 'react';
import {Redirect} from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function Home() {
  return <Redirect to={useBaseUrl('/docs')} />;
}
EOF
echo "✅ Created src/pages/index.js (homepage redirect)"

# Create i18n scaffolding for Urdu
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
mkdir -p i18n/ur/docusaurus-theme-classic

cat > i18n/ur/docusaurus-theme-classic/navbar.json << EOF
{
  "title": {
    "message": "عنوان",
    "description": "The title in the navbar"
  }
}
EOF

cat > i18n/ur/docusaurus-theme-classic/footer.json << EOF
{
  "copyright": {
    "message": "کاپی رائٹ © $(date +%Y)",
    "description": "The footer copyright"
  }
}
EOF

cat > i18n/ur/code.json << 'EOF'
{
  "theme.docs.paginator.previous": {
    "message": "پچھلا",
    "description": "The label used to navigate to the previous doc"
  },
  "theme.docs.paginator.next": {
    "message": "اگلا",
    "description": "The label used to navigate to the next doc"
  }
}
EOF

cat > i18n/ur/docusaurus-plugin-content-docs/current/intro.md << 'EOF'
---
sidebar_position: 1
---

# خوش آمدید

یہ اردو ترجمہ ہے۔
EOF
echo "✅ Created i18n/ur/ scaffolding (Urdu locale)"

# Check if docusaurus.config.js exists and show config
if [ -f "docusaurus.config.js" ]; then
    echo ""
    echo "📝 Update docusaurus.config.js with:"
    echo ""
    echo "  url: 'https://$ORG.github.io',"
    echo "  baseUrl: '/$REPO/',"
    echo "  organizationName: '$ORG',"
    echo "  projectName: '$REPO',"
    echo "  trailingSlash: false,"
    echo ""
    echo "  // Use markdown.hooks (not root-level)"
    echo "  markdown: {"
    echo "    hooks: {"
    echo "      onBrokenMarkdownLinks: 'warn',"
    echo "    },"
    echo "  },"
    echo ""
else
    echo "⚠️  docusaurus.config.js not found - create it first"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📋 Next steps:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  1. Update docusaurus.config.js with the values above"
echo "  2. ⚠️  CRITICAL: baseUrl MUST have BOTH leading AND trailing slashes: '/$REPO/'"
echo "  3. ⚠️  Verify deploy.yml branches: [$BRANCH] matches your default branch"
echo "  4. Enable GitHub Pages: Settings → Pages → Source: GitHub Actions"
echo "  5. Push to $BRANCH branch to trigger deployment"
echo ""
echo "🔗 Your site will be at: https://$ORG.github.io/$REPO/"
echo "🔗 Urdu version at: https://$ORG.github.io/$REPO/ur/"
echo ""
