#!/bin/bash
# =============================================================================
# Docusaurus Scaffold Setup Script
# Scaffold a Docusaurus project for the Physical AI textbook
# Version: 1.2.0
# Agent: BookArchitect
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
SITE_TITLE="Physical AI Book"
GITHUB_ORG="faiqahm"
REPO_NAME="hackathon_I_book"
LOCALES="en,ur"
NUM_CHAPTERS=3
RUN_INSTALL=false
RUN_START=false
RUN_BUILD=false
RUN_DEPLOY=false
ADD_WORKFLOW=true
ADD_MERMAID=true

# =============================================================================
# Functions
# =============================================================================

show_help() {
    cat << EOF
Docusaurus Scaffold - Create Physical AI textbook structure

Usage: $(basename "$0") [OPTIONS]

Options:
  --title TITLE          Site title (default: $SITE_TITLE)
  --github-org ORG       GitHub organization (default: $GITHUB_ORG)
  --repo NAME            Repository name (default: $REPO_NAME)
  --locales LOCALES      Comma-separated locales (default: $LOCALES)
  --chapters N           Number of chapters (default: $NUM_CHAPTERS)
  --install              Run npm install after setup
  --build                Run npm run build after install
  --deploy               Deploy to GitHub Pages after build
  --start                Start dev server (requires --install)
  --no-workflow          Skip GitHub Actions workflow creation
  --no-mermaid           Skip Mermaid diagram plugin
  -h, --help             Show this help message

Examples:
  $(basename "$0")
  $(basename "$0") --chapters 5 --install
  $(basename "$0") --title "My Book" --github-org myuser --install --start

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
        --title)
            SITE_TITLE="$2"
            shift 2
            ;;
        --github-org)
            GITHUB_ORG="$2"
            shift 2
            ;;
        --repo)
            REPO_NAME="$2"
            shift 2
            ;;
        --locales)
            LOCALES="$2"
            shift 2
            ;;
        --chapters)
            NUM_CHAPTERS="$2"
            shift 2
            ;;
        --install)
            RUN_INSTALL=true
            shift
            ;;
        --start)
            RUN_START=true
            shift
            ;;
        --build)
            RUN_BUILD=true
            shift
            ;;
        --deploy)
            RUN_DEPLOY=true
            RUN_BUILD=true
            RUN_INSTALL=true
            shift
            ;;
        --no-workflow)
            ADD_WORKFLOW=false
            shift
            ;;
        --no-mermaid)
            ADD_MERMAID=false
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
echo "📚 Docusaurus Scaffold v1.2.0"
echo "   Agent: BookArchitect"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
log_info "Site Title: $SITE_TITLE"
log_info "GitHub: $GITHUB_ORG/$REPO_NAME"
log_info "Locales: $LOCALES"
log_info "Chapters: $NUM_CHAPTERS"
echo ""

# Parse locales into array
IFS=',' read -ra LOCALE_ARRAY <<< "$LOCALES"

# =============================================================================
# Create package.json if not exists
# =============================================================================

if [[ ! -f "package.json" ]]; then
    log_info "Creating package.json..."
    cat > package.json << EOF
{
  "name": "$REPO_NAME",
  "version": "1.0.0",
  "private": true,
  "scripts": {
    "docusaurus": "docusaurus",
    "start": "docusaurus start",
    "build": "docusaurus build",
    "swizzle": "docusaurus swizzle",
    "deploy": "docusaurus deploy",
    "clear": "docusaurus clear",
    "serve": "docusaurus serve",
    "write-translations": "docusaurus write-translations",
    "write-heading-ids": "docusaurus write-heading-ids"
  },
  "dependencies": {
    "@docusaurus/core": "^3.6.0",
    "@docusaurus/preset-classic": "^3.6.0",
    "@docusaurus/theme-mermaid": "^3.6.0",
    "@mdx-js/react": "^3.0.0",
    "clsx": "^2.0.0",
    "prism-react-renderer": "^2.3.0",
    "react": "^18.0.0",
    "react-dom": "^18.0.0"
  },
  "devDependencies": {
    "@docusaurus/module-type-aliases": "^3.6.0",
    "@docusaurus/types": "^3.6.0"
  },
  "browserslist": {
    "production": [">0.5%", "not dead", "not op_mini all"],
    "development": ["last 3 chrome version", "last 3 firefox version", "last 5 safari version"]
  },
  "engines": {
    "node": ">=18.0"
  }
}
EOF
    log_success "Created package.json"
else
    log_info "package.json already exists"
fi

# =============================================================================
# Create docusaurus.config.js
# =============================================================================

log_info "Creating docusaurus.config.js..."

# Build locale configs
LOCALE_CONFIGS=""
for locale in "${LOCALE_ARRAY[@]}"; do
    locale=$(echo "$locale" | xargs)  # Trim whitespace
    if [[ "$locale" == "en" ]]; then
        LOCALE_CONFIGS="$LOCALE_CONFIGS
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },"
    elif [[ "$locale" == "ur" ]]; then
        LOCALE_CONFIGS="$LOCALE_CONFIGS
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },"
    else
        LOCALE_CONFIGS="$LOCALE_CONFIGS
      $locale: {
        label: '$locale',
        direction: 'ltr',
      },"
    fi
done

# Build locales array string
LOCALES_JS=$(echo "$LOCALES" | sed "s/,/', '/g")

cat > docusaurus.config.js << EOF
// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: '$SITE_TITLE',
  tagline: 'Learn Physical AI, ROS 2, and Robotics',
  favicon: 'img/favicon.svg',

  // GitHub Pages deployment
  url: 'https://$GITHUB_ORG.github.io',
  baseUrl: '/$REPO_NAME/',
  organizationName: '$GITHUB_ORG',
  projectName: '$REPO_NAME',
  trailingSlash: false,

  // Broken link detection
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Mermaid diagrams
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  // i18n configuration
  i18n: {
    defaultLocale: 'en',
    locales: ['$LOCALES_JS'],
    localeConfigs: {$LOCALE_CONFIGS
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      navbar: {
        title: '$SITE_TITLE',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/$GITHUB_ORG/$REPO_NAME',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: \`Copyright © \${new Date().getFullYear()} $SITE_TITLE.\`,
      },
    }),
};

module.exports = config;
EOF
log_success "Created docusaurus.config.js"

# =============================================================================
# Create sidebars.js
# =============================================================================

log_info "Creating sidebars.js..."
cat > sidebars.js << 'EOF'
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'autogenerated',
      dirName: '.',
    },
  ],
};

module.exports = sidebars;
EOF
log_success "Created sidebars.js"

# =============================================================================
# Create directory structure
# =============================================================================

log_info "Creating directory structure..."

mkdir -p docs
mkdir -p src/css
mkdir -p src/pages
mkdir -p static/img
mkdir -p i18n

log_success "Created base directories"

# =============================================================================
# Create intro.md
# =============================================================================

log_info "Creating docs/intro.md..."
cat > docs/intro.md << 'EOF'
---
sidebar_position: 1
slug: /
---

# Welcome to Physical AI

Learn how to build intelligent robots that perceive, reason, and act in the physical world.

## What You'll Learn

- **Physical AI Fundamentals** - Understanding embodied intelligence
- **ROS 2 Development** - Robot Operating System essentials
- **Simulation** - Testing in Gazebo before real hardware
- **Vision-Language-Action** - Modern AI models for robotics

## Prerequisites

- Basic Python programming
- Linux command line familiarity
- Enthusiasm for robotics!

## How to Use This Book

Each chapter includes:
- **Learning Objectives** - What you'll achieve
- **Hands-On Exercises** - Practical coding tasks
- **Summary** - Key takeaways
- **Further Reading** - Deep dive resources

Let's begin your journey into Physical AI!
EOF
log_success "Created docs/intro.md"

# =============================================================================
# Create chapters
# =============================================================================

CHAPTER_TITLES=(
    "Introduction to Physical AI & ROS 2"
    "Simulation with Gazebo"
    "Vision-Language-Action Models"
    "Perception and Sensors"
    "Motion Planning and Control"
    "Navigation and SLAM"
    "Manipulation and Grasping"
    "Multi-Robot Systems"
    "Deployment and Real Hardware"
    "Advanced Topics"
)

for i in $(seq 1 $NUM_CHAPTERS); do
    CHAPTER_DIR="docs/chapter-$i"
    mkdir -p "$CHAPTER_DIR"

    # Get chapter title (or generate one)
    if [[ $i -le ${#CHAPTER_TITLES[@]} ]]; then
        CHAPTER_TITLE="${CHAPTER_TITLES[$i-1]}"
    else
        CHAPTER_TITLE="Chapter $i"
    fi

    log_info "Creating chapter $i: $CHAPTER_TITLE"

    # Create _category_.json
    cat > "$CHAPTER_DIR/_category_.json" << EOF
{
  "label": "Chapter $i: $CHAPTER_TITLE",
  "position": $((i + 1)),
  "collapsible": true,
  "collapsed": false
}
EOF

    # Create index.md
    cat > "$CHAPTER_DIR/index.md" << EOF
---
sidebar_position: 1
---

# Chapter $i: $CHAPTER_TITLE

## Learning Objectives

By the end of this chapter, you will be able to:
- [ ] Understand the core concepts of this topic
- [ ] Implement basic examples
- [ ] Apply knowledge to real-world scenarios

## Prerequisites

Before starting this chapter, ensure you understand:
- Previous chapter concepts
- Basic programming skills

## Introduction

This chapter covers $CHAPTER_TITLE. We will explore the fundamental concepts and practical applications.

## Key Concepts

### Concept 1

Explanation of the first key concept.

### Concept 2

Explanation of the second key concept.

## Architecture Diagram

\`\`\`mermaid
graph TD
    A[Input] --> B[Processing]
    B --> C[Output]
    B --> D[Feedback Loop]
    D --> B
\`\`\`

## Hands-On Exercise

\`\`\`python
# Example code for this chapter
print("Welcome to Chapter $i!")
\`\`\`

## Summary

In this chapter, we covered:
- Key concept 1
- Key concept 2
- Practical applications

## Further Reading

- [Documentation Link](https://example.com)
- [Tutorial Link](https://example.com)
EOF

    log_success "Created chapter $i"
done

# =============================================================================
# Create custom.css with RTL support
# =============================================================================

log_info "Creating src/css/custom.css..."
cat > src/css/custom.css << 'EOF'
/**
 * Physical AI Book - Custom Styles
 * Includes RTL support for Urdu
 */

:root {
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
  --ifm-color-primary-light: #33925d;
  --ifm-color-primary-lighter: #359962;
  --ifm-color-primary-lightest: #3cad6e;
  --ifm-code-font-size: 95%;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
}

[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #21af90;
  --ifm-color-primary-darker: #1fa588;
  --ifm-color-primary-darkest: #1a8870;
  --ifm-color-primary-light: #29d5b0;
  --ifm-color-primary-lighter: #32d8b4;
  --ifm-color-primary-lightest: #4fddbf;
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.3);
}

/* RTL Support for Urdu */
html[dir='rtl'] {
  direction: rtl;
}

html[dir='rtl'] .navbar__items {
  flex-direction: row-reverse;
}

html[dir='rtl'] .navbar__items--right {
  flex-direction: row-reverse;
}

html[dir='rtl'] .markdown {
  text-align: right;
}

html[dir='rtl'] .markdown ul,
html[dir='rtl'] .markdown ol {
  padding-right: 2rem;
  padding-left: 0;
}

html[dir='rtl'] .table-of-contents {
  text-align: right;
}

html[dir='rtl'] .menu {
  text-align: right;
}

html[dir='rtl'] .menu__link {
  padding-right: var(--ifm-menu-link-padding-horizontal);
  padding-left: 0;
}

/* Urdu font family */
html[lang='ur'],
html[lang='ur'] body {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', 'Nafees', serif;
}

/* Code blocks should stay LTR */
html[dir='rtl'] pre,
html[dir='rtl'] code {
  direction: ltr;
  text-align: left;
}
EOF
log_success "Created src/css/custom.css"

# =============================================================================
# Create homepage redirect
# =============================================================================

log_info "Creating src/pages/index.js..."
cat > src/pages/index.js << 'EOF'
import React from 'react';
import {Redirect} from '@docusaurus/router';

export default function Home() {
  return <Redirect to="/docs/intro" />;
}
EOF
log_success "Created src/pages/index.js"

# =============================================================================
# Create favicon
# =============================================================================

log_info "Creating static/img/favicon.svg..."
cat > static/img/favicon.svg << 'EOF'
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100">
  <circle cx="50" cy="50" r="45" fill="#2e8555"/>
  <text x="50" y="65" font-size="40" text-anchor="middle" fill="white" font-family="Arial, sans-serif" font-weight="bold">AI</text>
</svg>
EOF
log_success "Created favicon"

# =============================================================================
# Create i18n structure for Urdu
# =============================================================================

if [[ "$LOCALES" == *"ur"* ]]; then
    log_info "Creating Urdu i18n structure..."

    mkdir -p i18n/ur/docusaurus-theme-classic
    mkdir -p i18n/ur/docusaurus-plugin-content-docs/current

    # Navbar translations
    cat > i18n/ur/docusaurus-theme-classic/navbar.json << 'EOF'
{
  "title": {
    "message": "فزیکل اے آئی کتاب",
    "description": "The title in the navbar"
  },
  "item.label.Chapters": {
    "message": "ابواب",
    "description": "Navbar item with label Chapters"
  },
  "item.label.GitHub": {
    "message": "گٹ ہب",
    "description": "Navbar item with label GitHub"
  }
}
EOF

    # Footer translations
    cat > i18n/ur/docusaurus-theme-classic/footer.json << 'EOF'
{
  "copyright": {
    "message": "کاپی رائٹ © 2026 فزیکل اے آئی کتاب۔",
    "description": "The footer copyright"
  }
}
EOF

    # Create Urdu intro
    cat > i18n/ur/docusaurus-plugin-content-docs/current/intro.md << 'EOF'
---
sidebar_position: 1
slug: /
---

# فزیکل اے آئی میں خوش آمدید

ذہین روبوٹ بنانا سیکھیں جو جسمانی دنیا میں سوچتے، سمجھتے اور عمل کرتے ہیں۔

## آپ کیا سیکھیں گے

- **فزیکل اے آئی کی بنیادیں** - مجسم ذہانت کو سمجھنا
- **ROS 2 ڈویلپمنٹ** - روبوٹ آپریٹنگ سسٹم کی بنیادی باتیں
- **سمولیشن** - حقیقی ہارڈویئر سے پہلے Gazebo میں ٹیسٹنگ
- **ویژن-لینگویج-ایکشن** - روبوٹکس کے لیے جدید AI ماڈلز

## پہلے سے ضروری معلومات

- بنیادی Python پروگرامنگ
- Linux کمانڈ لائن سے واقفیت
- روبوٹکس کے لیے جوش!

## یہ کتاب کیسے استعمال کریں

ہر باب میں شامل ہے:
- **سیکھنے کے مقاصد** - آپ کیا حاصل کریں گے
- **عملی مشقیں** - کوڈنگ کے عملی کام
- **خلاصہ** - اہم نکات
- **مزید مطالعہ** - گہرائی میں جانے کے وسائل

آئیے فزیکل اے آئی کا سفر شروع کریں!
EOF

    # Create Urdu chapters
    for i in $(seq 1 $NUM_CHAPTERS); do
        CHAPTER_DIR="i18n/ur/docusaurus-plugin-content-docs/current/chapter-$i"
        mkdir -p "$CHAPTER_DIR"

        cat > "$CHAPTER_DIR/index.md" << EOF
---
sidebar_position: 1
---

# باب $i

## سیکھنے کے مقاصد

اس باب کے اختتام تک، آپ قابل ہوں گے:
- [ ] اس موضوع کے بنیادی تصورات کو سمجھنا
- [ ] بنیادی مثالیں نافذ کرنا
- [ ] حقیقی دنیا کے منظرناموں پر علم کا اطلاق

## خلاصہ

اس باب میں، ہم نے احاطہ کیا:
- اہم تصور 1
- اہم تصور 2
- عملی ایپلی کیشنز
EOF
    done

    log_success "Created Urdu translations"
fi

# =============================================================================
# Create GitHub Actions Workflow
# =============================================================================

if [[ "$ADD_WORKFLOW" == true ]]; then
    log_info "Creating GitHub Actions workflow..."
    mkdir -p .github/workflows

    cat > .github/workflows/deploy.yml << EOF
name: Deploy to GitHub Pages

on:
  push:
    branches: [master, main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 20
          cache: npm
      - run: npm ci
      - run: npm run build
        env:
          NODE_OPTIONS: --max-old-space-size=4096
      - uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    environment:
      name: github-pages
      url: \${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - uses: actions/deploy-pages@v4
        id: deployment
EOF
    log_success "Created .github/workflows/deploy.yml"
fi

# =============================================================================
# Install dependencies
# =============================================================================

if [[ "$RUN_INSTALL" == true ]]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "📦 Installing Dependencies"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    npm install
    log_success "Dependencies installed"
fi

# =============================================================================
# Start dev server
# =============================================================================

if [[ "$RUN_START" == true ]]; then
    if [[ "$RUN_INSTALL" != true ]]; then
        log_warning "--start requires --install flag"
    else
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "🚀 Starting Development Server"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        npm start
    fi
fi

# =============================================================================
# Build
# =============================================================================

if [[ "$RUN_BUILD" == true ]]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🔨 Building Site"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    npm run build
    log_success "Build complete"
fi

# =============================================================================
# Deploy to GitHub Pages
# =============================================================================

if [[ "$RUN_DEPLOY" == true ]]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🚀 Deploying to GitHub Pages"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""

    # Use docusaurus deploy or gh-pages
    if command -v gh &> /dev/null; then
        log_info "Deploying via GitHub Actions..."
        git add -A
        git commit -m "chore: deploy to GitHub Pages" || true
        git push origin master || git push origin main
        log_success "Pushed to GitHub - Actions will deploy automatically"
        echo ""
        echo "🔗 Check deployment: https://github.com/$GITHUB_ORG/$REPO_NAME/actions"
    else
        npm run deploy
        log_success "Deployed to GitHub Pages"
    fi
    echo ""
    echo "🌐 Site URL: https://$GITHUB_ORG.github.io/$REPO_NAME/"
fi

# =============================================================================
# Summary
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✨ Docusaurus Scaffold complete!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📁 Structure created:"
echo "   ├── docs/"
echo "   │   ├── intro.md"
for i in $(seq 1 $NUM_CHAPTERS); do
echo "   │   └── chapter-$i/"
done
echo "   ├── i18n/"
if [[ "$LOCALES" == *"ur"* ]]; then
echo "   │   └── ur/"
fi
echo "   ├── src/"
echo "   │   ├── css/custom.css"
echo "   │   └── pages/index.js"
echo "   ├── static/img/"
echo "   ├── docusaurus.config.js"
echo "   ├── sidebars.js"
echo "   └── package.json"
echo ""
echo "📋 Next steps:"
if [[ "$RUN_INSTALL" != true ]]; then
echo "   1. npm install"
echo "   2. npm start"
else
echo "   1. npm start (if not already running)"
fi
echo "   3. Edit docs/ to add content"
echo "   4. npm run build"
echo "   5. Deploy to GitHub Pages"
echo ""
echo "🔗 URLs after deployment:"
echo "   • https://$GITHUB_ORG.github.io/$REPO_NAME/"
echo ""
