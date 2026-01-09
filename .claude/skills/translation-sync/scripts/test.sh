#!/bin/bash
# =============================================================================
# Translation Sync Test Script
# Tests the skill functionality
# Version: 1.1.0
# =============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"
TEST_DIR="/tmp/translation-sync-test-$$"
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

log_skip() {
    echo -e "${YELLOW}[SKIP]${NC} $1"
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
echo "🧪 Translation Sync Test Suite v1.1.0"
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
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "Translation Sync"; then
    log_pass "--help displays usage information"
else
    log_fail "--help did not display expected output"
fi

# =============================================================================
# Test 2: Version is 1.1.0
# =============================================================================

log_test "Test 2: Version is 1.1.0"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "v1.1.0"; then
    log_pass "Version 1.1.0 displayed"
else
    log_fail "Version not displayed correctly"
fi

# =============================================================================
# Test 3: Python module exists
# =============================================================================

log_test "Test 3: translator.py module exists"
if [[ -f "$SCRIPT_DIR/translator.py" ]]; then
    log_pass "translator.py exists"
else
    log_fail "translator.py not found"
fi

# =============================================================================
# Test 4: TranslationSync class exists
# =============================================================================

log_test "Test 4: TranslationSync class exists"
if grep -q "class TranslationSync" "$SCRIPT_DIR/translator.py"; then
    log_pass "TranslationSync class defined"
else
    log_fail "TranslationSync class not found"
fi

# =============================================================================
# Test 5: FileStatus class exists
# =============================================================================

log_test "Test 5: FileStatus class exists"
if grep -q "class FileStatus" "$SCRIPT_DIR/translator.py"; then
    log_pass "FileStatus class defined"
else
    log_fail "FileStatus class not found"
fi

# =============================================================================
# Test 6: Python syntax validation
# =============================================================================

log_test "Test 6: Python syntax validation"
if python3 -m py_compile "$SCRIPT_DIR/translator.py" 2>/dev/null; then
    log_pass "translator.py has valid Python syntax"
else
    log_fail "translator.py has syntax errors"
fi

# =============================================================================
# Test 7: SKILL.md exists
# =============================================================================

log_test "Test 7: SKILL.md exists"
if [[ -f "$SKILL_DIR/SKILL.md" ]]; then
    log_pass "SKILL.md exists"
else
    log_fail "SKILL.md not found"
fi

# =============================================================================
# Test 8: requirements.txt exists
# =============================================================================

log_test "Test 8: requirements.txt exists"
if [[ -f "$SKILL_DIR/requirements.txt" ]]; then
    log_pass "requirements.txt exists"
else
    log_fail "requirements.txt not found"
fi

# =============================================================================
# Test 9: glossary.json exists
# =============================================================================

log_test "Test 9: glossary.json exists"
if [[ -f "$SKILL_DIR/assets/glossary.json" ]]; then
    log_pass "glossary.json exists"
else
    log_fail "glossary.json not found"
fi

# =============================================================================
# Test 10: translation_config.json exists
# =============================================================================

log_test "Test 10: translation_config.json exists"
if [[ -f "$SKILL_DIR/assets/translation_config.json" ]]; then
    log_pass "translation_config.json exists"
else
    log_fail "translation_config.json not found"
fi

# =============================================================================
# Test 11: Glossary has 30+ terms
# =============================================================================

log_test "Test 11: Glossary has 30+ terms"
TERM_COUNT=$(python3 -c "import json; print(len(json.load(open('$SKILL_DIR/assets/glossary.json'))['terms']))" 2>/dev/null || echo 0)
if [[ $TERM_COUNT -ge 30 ]]; then
    log_pass "Glossary has $TERM_COUNT terms (30+ required)"
else
    log_fail "Glossary has only $TERM_COUNT terms (30+ required)"
fi

# =============================================================================
# Test 12: Core methods exist
# =============================================================================

log_test "Test 12: Core methods exist in TranslationSync"
METHODS=("get_status" "translate_file" "sync_all" "validate_translations" "add_term" "list_terms")
MISSING_METHODS=0

for method in "${METHODS[@]}"; do
    if ! grep -q "def $method" "$SCRIPT_DIR/translator.py"; then
        MISSING_METHODS=$((MISSING_METHODS + 1))
    fi
done

if [[ $MISSING_METHODS -eq 0 ]]; then
    log_pass "All core methods exist"
else
    log_fail "$MISSING_METHODS methods missing"
fi

# =============================================================================
# Test 13: Code block preservation methods exist
# =============================================================================

log_test "Test 13: Code block preservation methods exist"
if grep -q "_preserve_code_blocks" "$SCRIPT_DIR/translator.py" && \
   grep -q "_restore_code_blocks" "$SCRIPT_DIR/translator.py"; then
    log_pass "Code block preservation methods exist"
else
    log_fail "Code block preservation methods missing"
fi

# =============================================================================
# Test 14: RTL wrapper method exists
# =============================================================================

log_test "Test 14: RTL wrapper method exists"
if grep -q "_apply_rtl_wrapper" "$SCRIPT_DIR/translator.py"; then
    log_pass "RTL wrapper method exists"
else
    log_fail "RTL wrapper method missing"
fi

# =============================================================================
# Test 15: v1.1.0 - DiffResult class exists
# =============================================================================

log_test "Test 15: v1.1.0 - DiffResult class exists"
if grep -q "class DiffResult" "$SCRIPT_DIR/translator.py"; then
    log_pass "DiffResult class exists"
else
    log_fail "DiffResult class not found"
fi

# =============================================================================
# Test 16: v1.1.0 - TMEntry class exists
# =============================================================================

log_test "Test 16: v1.1.0 - TMEntry class exists"
if grep -q "class TMEntry" "$SCRIPT_DIR/translator.py"; then
    log_pass "TMEntry class exists"
else
    log_fail "TMEntry class not found"
fi

# =============================================================================
# Test 17: v1.1.0 - SectionInfo class exists
# =============================================================================

log_test "Test 17: v1.1.0 - SectionInfo class exists"
if grep -q "class SectionInfo" "$SCRIPT_DIR/translator.py"; then
    log_pass "SectionInfo class exists"
else
    log_fail "SectionInfo class not found"
fi

# =============================================================================
# Test 18: v1.1.0 - show_diff method exists
# =============================================================================

log_test "Test 18: v1.1.0 - show_diff method exists"
if grep -q "def show_diff" "$SCRIPT_DIR/translator.py"; then
    log_pass "show_diff method exists"
else
    log_fail "show_diff method not found"
fi

# =============================================================================
# Test 19: v1.1.0 - translate_file_incremental method exists
# =============================================================================

log_test "Test 19: v1.1.0 - translate_file_incremental method exists"
if grep -q "def translate_file_incremental" "$SCRIPT_DIR/translator.py"; then
    log_pass "translate_file_incremental method exists"
else
    log_fail "translate_file_incremental method not found"
fi

# =============================================================================
# Test 20: v1.1.0 - TM methods exist
# =============================================================================

log_test "Test 20: v1.1.0 - Translation Memory methods exist"
TM_METHODS=("get_tm_stats" "export_tm" "import_tm" "clear_tm" "_lookup_tm" "_store_tm")
MISSING_TM=0

for method in "${TM_METHODS[@]}"; do
    if ! grep -q "def $method" "$SCRIPT_DIR/translator.py"; then
        MISSING_TM=$((MISSING_TM + 1))
    fi
done

if [[ $MISSING_TM -eq 0 ]]; then
    log_pass "All TM methods exist"
else
    log_fail "$MISSING_TM TM methods missing"
fi

# =============================================================================
# Test 21: v1.1.0 - --diff documented in help
# =============================================================================

log_test "Test 21: v1.1.0 - --diff documented in help"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "\-\-diff"; then
    log_pass "--diff documented in help"
else
    log_fail "--diff not documented"
fi

# =============================================================================
# Test 22: v1.1.0 - --incremental documented in help
# =============================================================================

log_test "Test 22: v1.1.0 - --incremental documented in help"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "\-\-incremental"; then
    log_pass "--incremental documented in help"
else
    log_fail "--incremental not documented"
fi

# =============================================================================
# Test 23: v1.1.0 - --tm-stats documented in help
# =============================================================================

log_test "Test 23: v1.1.0 - --tm-stats documented in help"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "\-\-tm-stats"; then
    log_pass "--tm-stats documented in help"
else
    log_fail "--tm-stats not documented"
fi

# =============================================================================
# Test 24: Run Python unit tests
# =============================================================================

log_test "Test 24: Python unit tests"
if python3 "$SCRIPT_DIR/test_translator.py" 2>&1; then
    log_pass "Python unit tests passed"
else
    log_fail "Python unit tests failed"
fi

# =============================================================================
# Test 25: Error on missing action
# =============================================================================

log_test "Test 25: Error on missing action"
export OPENAI_API_KEY="test-key"

if bash "$SCRIPT_DIR/setup.sh" 2>&1 | grep -q "No action specified"; then
    log_pass "Error shown when no action specified"
else
    log_fail "No error for missing action"
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
