#!/bin/bash
# =============================================================================
# RAG Personalizer Test Script
# Tests the skill functionality
# Version: 1.0.0
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
TEST_DIR="/tmp/rag-personalizer-test-$$"
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
echo "🧪 RAG Personalizer Test Suite v1.0.0"
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
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "RAG Personalizer"; then
    log_pass "--help displays usage information"
else
    log_fail "--help did not display expected output"
fi

# =============================================================================
# Test 2: Version is 1.0.0
# =============================================================================

log_test "Test 2: Version is 1.0.0"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "v1.0.0"; then
    log_pass "Version 1.0.0 displayed"
else
    log_fail "Version not displayed correctly"
fi

# =============================================================================
# Test 3: All 10 dimensions documented
# =============================================================================

log_test "Test 3: All 10 dimensions documented in help"
DIMENSIONS=("learning_style" "knowledge_level" "learning_pace" "language"
            "content_depth" "example_preference" "difficulty_tolerance"
            "interaction_style" "time_availability" "goal_orientation")

HELP_OUTPUT=$(bash "$SCRIPT_DIR/setup.sh" --help 2>&1)
MISSING=0
for dim in "${DIMENSIONS[@]}"; do
    if ! echo "$HELP_OUTPUT" | grep -q "$dim"; then
        MISSING=$((MISSING + 1))
    fi
done

if [[ $MISSING -eq 0 ]]; then
    log_pass "All 10 dimensions documented"
else
    log_fail "$MISSING dimensions missing from help"
fi

# =============================================================================
# Test 4: Python module exists
# =============================================================================

log_test "Test 4: personalizer.py module exists"
if [[ -f "$SCRIPT_DIR/personalizer.py" ]]; then
    log_pass "personalizer.py exists"
else
    log_fail "personalizer.py not found"
fi

# =============================================================================
# Test 5: Personalizer class exists
# =============================================================================

log_test "Test 5: Personalizer class exists"
if grep -q "class Personalizer" "$SCRIPT_DIR/personalizer.py"; then
    log_pass "Personalizer class defined"
else
    log_fail "Personalizer class not found"
fi

# =============================================================================
# Test 6: Profile class exists
# =============================================================================

log_test "Test 6: Profile class exists"
if grep -q "class Profile" "$SCRIPT_DIR/personalizer.py"; then
    log_pass "Profile class defined"
else
    log_fail "Profile class not found"
fi

# =============================================================================
# Test 7: ProfileDimensions class with 10 dimensions
# =============================================================================

log_test "Test 7: ProfileDimensions has 10 dimensions"
DIM_COUNT=$(grep -c "learning_style\|knowledge_level\|learning_pace\|language\|content_depth\|example_preference\|difficulty_tolerance\|interaction_style\|time_availability\|goal_orientation" "$SCRIPT_DIR/personalizer.py" | head -1)
if [[ $DIM_COUNT -ge 10 ]]; then
    log_pass "ProfileDimensions has all 10 dimensions"
else
    log_fail "ProfileDimensions missing dimensions (found $DIM_COUNT references)"
fi

# =============================================================================
# Test 8: Python syntax validation
# =============================================================================

log_test "Test 8: Python syntax validation"
if python3 -m py_compile "$SCRIPT_DIR/personalizer.py" 2>/dev/null; then
    log_pass "personalizer.py has valid Python syntax"
else
    log_fail "personalizer.py has syntax errors"
fi

# =============================================================================
# Test 9: SKILL.md exists
# =============================================================================

log_test "Test 9: SKILL.md exists"
if [[ -f "$SKILL_DIR/SKILL.md" ]]; then
    log_pass "SKILL.md exists"
else
    log_fail "SKILL.md not found"
fi

# =============================================================================
# Test 10: requirements.txt exists
# =============================================================================

log_test "Test 10: requirements.txt exists"
if [[ -f "$SKILL_DIR/requirements.txt" ]]; then
    log_pass "requirements.txt exists"
else
    log_fail "requirements.txt not found"
fi

# =============================================================================
# Test 11: prompts.json exists
# =============================================================================

log_test "Test 11: prompts.json exists"
if [[ -f "$SKILL_DIR/assets/prompts.json" ]]; then
    log_pass "prompts.json exists"
else
    log_fail "prompts.json not found"
fi

# =============================================================================
# Test 12: profile_schema.json exists
# =============================================================================

log_test "Test 12: profile_schema.json exists"
if [[ -f "$SKILL_DIR/assets/profile_schema.json" ]]; then
    log_pass "profile_schema.json exists"
else
    log_fail "profile_schema.json not found"
fi

# =============================================================================
# Test 13: CRUD methods exist
# =============================================================================

log_test "Test 13: CRUD methods exist in Personalizer"
METHODS=("create_profile" "get_profile" "update_profile" "delete_profile" "list_profiles" "personalize")
MISSING_METHODS=0

for method in "${METHODS[@]}"; do
    if ! grep -q "def $method" "$SCRIPT_DIR/personalizer.py"; then
        MISSING_METHODS=$((MISSING_METHODS + 1))
    fi
done

if [[ $MISSING_METHODS -eq 0 ]]; then
    log_pass "All CRUD + personalize methods exist"
else
    log_fail "$MISSING_METHODS methods missing"
fi

# =============================================================================
# Test 14: Run Python unit tests
# =============================================================================

log_test "Test 14: Python unit tests"
if python3 "$SCRIPT_DIR/test_personalizer.py" 2>&1; then
    log_pass "Python unit tests passed"
else
    log_fail "Python unit tests failed"
fi

# =============================================================================
# Test 15: Error on missing action
# =============================================================================

log_test "Test 15: Error on missing action"
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
