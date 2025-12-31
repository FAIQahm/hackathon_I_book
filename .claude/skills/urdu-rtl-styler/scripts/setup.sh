#!/bin/bash
# Urdu RTL Styler Setup Script
# Usage: ./setup.sh [--local-fonts]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"
USE_LOCAL_FONTS=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --local-fonts)
            USE_LOCAL_FONTS=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--local-fonts]"
            exit 1
            ;;
    esac
done

echo "🎨 Setting up Urdu RTL styles..."
echo ""

# Create CSS directory if not exists
mkdir -p src/css

# Check if custom.css exists
if [ -f "src/css/custom.css" ]; then
    echo "📝 Appending RTL styles to existing src/css/custom.css"
    echo "" >> src/css/custom.css
    echo "/* Urdu RTL Styles - Added by urdu-rtl-styler */" >> src/css/custom.css
    cat "$SKILL_DIR/assets/urdu-rtl.css" >> src/css/custom.css
else
    echo "📝 Creating src/css/custom.css with RTL styles"
    cp "$SKILL_DIR/assets/urdu-rtl.css" src/css/custom.css
fi

echo "✅ Added Urdu RTL styles to src/css/custom.css"

# Setup local fonts if requested
if [ "$USE_LOCAL_FONTS" = true ]; then
    echo ""
    echo "📦 Setting up local fonts..."
    mkdir -p static/fonts

    echo "⚠️  Please download Noto Nastaliq Urdu fonts from:"
    echo "    https://fonts.google.com/specimen/Noto+Nastaliq+Urdu"
    echo ""
    echo "    Place the following files in static/fonts/:"
    echo "    - NotoNastaliqUrdu-Regular.woff2"
    echo "    - NotoNastaliqUrdu-Bold.woff2"
    echo ""

    # Create local font CSS
    cat > static/fonts/local-fonts.css << 'EOF'
/* Local Noto Nastaliq Urdu Fonts */
@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('/fonts/NotoNastaliqUrdu-Regular.woff2') format('woff2');
  font-weight: 400;
  font-style: normal;
  font-display: swap;
}

@font-face {
  font-family: 'Noto Nastaliq Urdu';
  src: url('/fonts/NotoNastaliqUrdu-Bold.woff2') format('woff2');
  font-weight: 700;
  font-style: normal;
  font-display: swap;
}
EOF

    echo "✅ Created static/fonts/local-fonts.css"
    echo "📝 Remember to import this in your custom.css:"
    echo '    @import url("/fonts/local-fonts.css");'
fi

# Check docusaurus.config.js for i18n configuration
echo ""
if [ -f "docusaurus.config.js" ]; then
    if grep -q "direction: 'rtl'" docusaurus.config.js; then
        echo "✅ RTL direction already configured in docusaurus.config.js"
    else
        echo "⚠️  Please add RTL direction to your Urdu locale config:"
        echo ""
        echo "    ur: {"
        echo "      label: 'اردو',"
        echo "      direction: 'rtl',  // ⚠️ CRITICAL for RTL styles"
        echo "      htmlLang: 'ur-PK',"
        echo "    },"
        echo ""
    fi
else
    echo "⚠️  docusaurus.config.js not found"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📋 Next steps:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  1. Ensure i18n.localeConfigs.ur has direction: 'rtl'"
echo "  2. Create Urdu content in i18n/ur/docusaurus-plugin-content-docs/current/"
echo "  3. Test locally: npm run build && npm run serve"
echo "  4. Visit: http://localhost:3000/<repo>/ur/"
echo ""
echo "✨ Urdu RTL styling setup complete!"
echo ""
