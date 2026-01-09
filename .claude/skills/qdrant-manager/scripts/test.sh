#!/bin/bash
# =============================================================================
# Qdrant Manager Test Script
# Tests the skill functionality with mocked dependencies
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
TEST_DIR="/tmp/qdrant-manager-test-$$"
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
echo "🧪 Qdrant Manager Test Suite v1.1.0"
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
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "Qdrant Manager"; then
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
# Test 3: New --language option in help
# =============================================================================

log_test "Test 3: --language option documented"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "\-\-language"; then
    log_pass "--language option documented in help"
else
    log_fail "--language option not in help"
fi

# =============================================================================
# Test 4: New --chapter option in help
# =============================================================================

log_test "Test 4: --chapter option documented"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "\-\-chapter"; then
    log_pass "--chapter option documented in help"
else
    log_fail "--chapter option not in help"
fi

# =============================================================================
# Test 5: New --update option in help
# =============================================================================

log_test "Test 5: --update option documented"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "\-\-update"; then
    log_pass "--update option documented in help"
else
    log_fail "--update option not in help"
fi

# =============================================================================
# Test 6: Python module exists
# =============================================================================

log_test "Test 6: vectorize.py module exists"
if [[ -f "$SCRIPT_DIR/vectorize.py" ]]; then
    log_pass "vectorize.py exists"
else
    log_fail "vectorize.py not found"
fi

# =============================================================================
# Test 7: Python module has QdrantManager class
# =============================================================================

log_test "Test 7: QdrantManager class exists"
if grep -q "class QdrantManager" "$SCRIPT_DIR/vectorize.py"; then
    log_pass "QdrantManager class defined"
else
    log_fail "QdrantManager class not found"
fi

# =============================================================================
# Test 8: Python module has query method with language filter
# =============================================================================

log_test "Test 8: query() method has language parameter"
# Multi-line function signature, check for language parameter near query def
if grep -A 10 "def query(" "$SCRIPT_DIR/vectorize.py" | grep -q "language"; then
    log_pass "query() method has language parameter"
else
    log_fail "query() method missing language parameter"
fi

# =============================================================================
# Test 9: Python module has vectorize_directory with update_only
# =============================================================================

log_test "Test 9: vectorize_directory() has update_only parameter"
# Multi-line function signature, check for update_only parameter near def
if grep -A 10 "def vectorize_directory(" "$SCRIPT_DIR/vectorize.py" | grep -q "update_only"; then
    log_pass "vectorize_directory() has update_only parameter"
else
    log_fail "vectorize_directory() missing update_only parameter"
fi

# =============================================================================
# Test 10: Python module syntax check
# =============================================================================

log_test "Test 10: Python syntax validation"
if python3 -m py_compile "$SCRIPT_DIR/vectorize.py" 2>/dev/null; then
    log_pass "vectorize.py has valid Python syntax"
else
    log_fail "vectorize.py has syntax errors"
fi

# =============================================================================
# Test 11: Requirements file exists
# =============================================================================

log_test "Test 11: requirements.txt exists"
if [[ -f "$SKILL_DIR/requirements.txt" ]]; then
    log_pass "requirements.txt exists"
else
    log_fail "requirements.txt not found"
fi

# =============================================================================
# Test 12: SKILL.md exists and has v1.1.0
# =============================================================================

log_test "Test 12: SKILL.md updated to v1.1.0"
if grep -q "version: 1.1.0" "$SKILL_DIR/SKILL.md"; then
    log_pass "SKILL.md has version 1.1.0"
else
    log_fail "SKILL.md version not updated"
fi

# =============================================================================
# Test 13: Run Python unit tests
# =============================================================================

log_test "Test 13: Python unit tests"
if python3 "$SCRIPT_DIR/test_vectorize.py" 2>&1; then
    log_pass "Python unit tests passed"
else
    log_fail "Python unit tests failed"
fi

# =============================================================================
# Test 14: Error on missing action
# =============================================================================

log_test "Test 14: Error on missing action"
# Set dummy env vars to avoid env check failure
export QDRANT_URL="https://test.qdrant.io"
export QDRANT_API_KEY="test-key"
export OPENAI_API_KEY="sk-test"

if bash "$SCRIPT_DIR/setup.sh" 2>&1 | grep -q "No action specified"; then
    log_pass "Error shown when no action specified"
else
    log_fail "No error for missing action"
fi

# =============================================================================
# Test 15: Unknown option error
# =============================================================================

log_test "Test 15: Error on unknown option"
if bash "$SCRIPT_DIR/setup.sh" --invalid-option 2>&1 | grep -q "Unknown option"; then
    log_pass "Error shown for unknown option"
else
    log_fail "No error for unknown option"
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
