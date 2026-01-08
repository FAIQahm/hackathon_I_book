#!/bin/bash
# =============================================================================
# Docusaurus Scaffold Test Script
# Tests the skill functionality in an isolated directory
# Version: 1.2.0
# =============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="/tmp/docusaurus-scaffold-test-$$"
PASSED=0
FAILED=0

log_test() {
    echo -e "${BLUE}[TEST]${NC} $1"
}

log_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    PASSED=$((PASSED + 1))
}

log_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    FAILED=$((FAILED + 1))
}

cleanup() {
    if [[ -d "$TEST_DIR" ]]; then
        rm -rf "$TEST_DIR"
    fi
}

trap cleanup EXIT

# =============================================================================
# Setup
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🧪 Docusaurus Scaffold Test Suite"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Test directory: $TEST_DIR"
echo ""

mkdir -p "$TEST_DIR"
cd "$TEST_DIR"

# =============================================================================
# Test 1: Help flag works
# =============================================================================

log_test "Test 1: --help flag works"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "Docusaurus Scaffold"; then
    log_pass "--help displays usage information"
else
    log_fail "--help did not display expected output"
fi

# =============================================================================
# Test 2: Basic scaffold creates required files
# =============================================================================

log_test "Test 2: Basic scaffold creates required files"
bash "$SCRIPT_DIR/setup.sh" --no-workflow --no-mermaid > /dev/null 2>&1

# Check package.json
if [[ -f "package.json" ]]; then
    log_pass "package.json created"
else
    log_fail "package.json not created"
fi

# Check docusaurus.config.js
if [[ -f "docusaurus.config.js" ]]; then
    log_pass "docusaurus.config.js created"
else
    log_fail "docusaurus.config.js not created"
fi

# Check sidebars.js
if [[ -f "sidebars.js" ]]; then
    log_pass "sidebars.js created"
else
    log_fail "sidebars.js not created"
fi

# Check docs/intro.md
if [[ -f "docs/intro.md" ]]; then
    log_pass "docs/intro.md created"
else
    log_fail "docs/intro.md not created"
fi

# =============================================================================
# Test 3: Default 3 chapters created
# =============================================================================

log_test "Test 3: Default 3 chapters created"
for i in 1 2 3; do
    if [[ -d "docs/chapter-$i" ]] && [[ -f "docs/chapter-$i/index.md" ]]; then
        log_pass "Chapter $i created with index.md"
    else
        log_fail "Chapter $i missing or incomplete"
    fi
done

# =============================================================================
# Test 4: i18n Urdu structure created
# =============================================================================

log_test "Test 4: i18n Urdu structure created"
if [[ -f "i18n/ur/docusaurus-theme-classic/navbar.json" ]]; then
    log_pass "Urdu navbar.json created"
else
    log_fail "Urdu navbar.json not created"
fi

if [[ -f "i18n/ur/docusaurus-plugin-content-docs/current/intro.md" ]]; then
    log_pass "Urdu intro.md created"
else
    log_fail "Urdu intro.md not created"
fi

# =============================================================================
# Test 5: RTL CSS exists
# =============================================================================

log_test "Test 5: RTL CSS configuration"
if [[ -f "src/css/custom.css" ]] && grep -q "dir='rtl'" "src/css/custom.css"; then
    log_pass "RTL styles present in custom.css"
else
    log_fail "RTL styles missing from custom.css"
fi

# =============================================================================
# Test 6: Homepage redirect exists
# =============================================================================

log_test "Test 6: Homepage redirect"
if [[ -f "src/pages/index.js" ]] && grep -q "Redirect" "src/pages/index.js"; then
    log_pass "Homepage redirect configured"
else
    log_fail "Homepage redirect missing"
fi

# =============================================================================
# Test 7: Favicon created
# =============================================================================

log_test "Test 7: Favicon created"
if [[ -f "static/img/favicon.svg" ]]; then
    log_pass "Favicon created"
else
    log_fail "Favicon not created"
fi

# =============================================================================
# Test 8: Custom chapters flag
# =============================================================================

log_test "Test 8: Custom chapters (--chapters 5)"
rm -rf "$TEST_DIR"/*
bash "$SCRIPT_DIR/setup.sh" --chapters 5 --no-workflow > /dev/null 2>&1

CHAPTER_COUNT=$(ls -d docs/chapter-* 2>/dev/null | wc -l)
if [[ "$CHAPTER_COUNT" -eq 5 ]]; then
    log_pass "--chapters 5 created 5 chapters"
else
    log_fail "--chapters 5 created $CHAPTER_COUNT chapters instead of 5"
fi

# =============================================================================
# Test 9: GitHub Actions workflow created by default
# =============================================================================

log_test "Test 9: GitHub Actions workflow (default)"
rm -rf "$TEST_DIR"/*
bash "$SCRIPT_DIR/setup.sh" > /dev/null 2>&1

if [[ -f ".github/workflows/deploy.yml" ]]; then
    log_pass "GitHub Actions workflow created"
else
    log_fail "GitHub Actions workflow not created"
fi

# =============================================================================
# Test 10: Mermaid configuration
# =============================================================================

log_test "Test 10: Mermaid diagram support"
if grep -q "mermaid" "docusaurus.config.js" && grep -q "theme-mermaid" "package.json"; then
    log_pass "Mermaid configured in config and package.json"
else
    log_fail "Mermaid configuration missing"
fi

# =============================================================================
# Test 11: Chapter template has Mermaid example
# =============================================================================

log_test "Test 11: Chapter template has Mermaid diagram"
if grep -q "mermaid" "docs/chapter-1/index.md"; then
    log_pass "Chapter template includes Mermaid example"
else
    log_fail "Chapter template missing Mermaid example"
fi

# =============================================================================
# Test 12: --no-workflow flag works
# =============================================================================

log_test "Test 12: --no-workflow flag"
rm -rf "$TEST_DIR"
mkdir -p "$TEST_DIR"
cd "$TEST_DIR"
bash "$SCRIPT_DIR/setup.sh" --no-workflow > /dev/null 2>&1

if [[ ! -f "$TEST_DIR/.github/workflows/deploy.yml" ]]; then
    log_pass "--no-workflow skipped workflow creation"
else
    log_fail "--no-workflow did not skip workflow"
fi

# =============================================================================
# Summary
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📊 Test Results"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo -e "  ${GREEN}Passed:${NC} $PASSED"
echo -e "  ${RED}Failed:${NC} $FAILED"
echo ""

if [[ $FAILED -eq 0 ]]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}✗ Some tests failed${NC}"
    exit 1
fi
