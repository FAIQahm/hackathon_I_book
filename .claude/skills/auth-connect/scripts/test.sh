#!/bin/bash
# =============================================================================
# Auth Connect Test Script
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

# =============================================================================
# Setup
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🧪 Auth Connect Test Suite v1.0.0"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Set test JWT secret
export JWT_SECRET="test-secret-key-for-unit-tests-only-12345678"

# =============================================================================
# Test 1: Help flag works
# =============================================================================

log_test "Test 1: --help flag works"
if bash "$SCRIPT_DIR/setup.sh" --help 2>&1 | grep -q "Auth Connect"; then
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
# Test 3: Python module exists
# =============================================================================

log_test "Test 3: auth.py module exists"
if [[ -f "$SCRIPT_DIR/auth.py" ]]; then
    log_pass "auth.py exists"
else
    log_fail "auth.py not found"
fi

# =============================================================================
# Test 4: AuthManager class exists
# =============================================================================

log_test "Test 4: AuthManager class exists"
if grep -q "class AuthManager" "$SCRIPT_DIR/auth.py"; then
    log_pass "AuthManager class defined"
else
    log_fail "AuthManager class not found"
fi

# =============================================================================
# Test 5: TokenPayload class exists
# =============================================================================

log_test "Test 5: TokenPayload class exists"
if grep -q "class TokenPayload" "$SCRIPT_DIR/auth.py"; then
    log_pass "TokenPayload class defined"
else
    log_fail "TokenPayload class not found"
fi

# =============================================================================
# Test 6: Python syntax validation
# =============================================================================

log_test "Test 6: Python syntax validation"
if python3 -m py_compile "$SCRIPT_DIR/auth.py" 2>/dev/null; then
    log_pass "auth.py has valid Python syntax"
else
    log_fail "auth.py has syntax errors"
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
# Test 9: auth_config.json exists
# =============================================================================

log_test "Test 9: auth_config.json exists"
if [[ -f "$SKILL_DIR/assets/auth_config.json" ]]; then
    log_pass "auth_config.json exists"
else
    log_fail "auth_config.json not found"
fi

# =============================================================================
# Test 10: roles.json exists
# =============================================================================

log_test "Test 10: roles.json exists"
if [[ -f "$SKILL_DIR/assets/roles.json" ]]; then
    log_pass "roles.json exists"
else
    log_fail "roles.json not found"
fi

# =============================================================================
# Test 11: roles.json has required roles
# =============================================================================

log_test "Test 11: roles.json has required roles"
ROLES_COUNT=$(python3 -c "import json; print(len(json.load(open('$SKILL_DIR/assets/roles.json'))['roles']))" 2>/dev/null || echo 0)
if [[ $ROLES_COUNT -ge 4 ]]; then
    log_pass "roles.json has $ROLES_COUNT roles (4+ required)"
else
    log_fail "roles.json has only $ROLES_COUNT roles (4+ required)"
fi

# =============================================================================
# Test 12: Core functions exist
# =============================================================================

log_test "Test 12: Core functions exist"
FUNCTIONS=("hash_password" "verify_password" "validate_password" "generate_jwt_secret" "generate_api_key")
MISSING_FUNCS=0

for func in "${FUNCTIONS[@]}"; do
    if ! grep -q "def $func" "$SCRIPT_DIR/auth.py"; then
        MISSING_FUNCS=$((MISSING_FUNCS + 1))
    fi
done

if [[ $MISSING_FUNCS -eq 0 ]]; then
    log_pass "All core functions exist"
else
    log_fail "$MISSING_FUNCS functions missing"
fi

# =============================================================================
# Test 13: AuthManager methods exist
# =============================================================================

log_test "Test 13: AuthManager methods exist"
METHODS=("create_access_token" "create_refresh_token" "verify_token" "revoke_token" "is_token_valid")
MISSING_METHODS=0

for method in "${METHODS[@]}"; do
    if ! grep -q "def $method" "$SCRIPT_DIR/auth.py"; then
        MISSING_METHODS=$((MISSING_METHODS + 1))
    fi
done

if [[ $MISSING_METHODS -eq 0 ]]; then
    log_pass "All AuthManager methods exist"
else
    log_fail "$MISSING_METHODS methods missing"
fi

# =============================================================================
# Test 14: FastAPI dependencies exist
# =============================================================================

log_test "Test 14: FastAPI dependencies exist"
if grep -q "get_current_user" "$SCRIPT_DIR/auth.py" && \
   grep -q "require_role" "$SCRIPT_DIR/auth.py" && \
   grep -q "require_permission" "$SCRIPT_DIR/auth.py"; then
    log_pass "FastAPI dependencies defined"
else
    log_fail "FastAPI dependencies missing"
fi

# =============================================================================
# Test 15: Generate secret command works
# =============================================================================

log_test "Test 15: Generate secret command works"
if bash "$SCRIPT_DIR/setup.sh" --generate-secret 2>&1 | grep -q "JWT_SECRET="; then
    log_pass "--generate-secret produces valid output"
else
    log_fail "--generate-secret failed"
fi

# =============================================================================
# Test 16: List roles command works
# =============================================================================

log_test "Test 16: List roles command works"
if bash "$SCRIPT_DIR/setup.sh" --list-roles 2>&1 | grep -q "admin"; then
    log_pass "--list-roles shows admin role"
else
    log_fail "--list-roles failed"
fi

# =============================================================================
# Test 17: Show config command works
# =============================================================================

log_test "Test 17: Show config command works"
if bash "$SCRIPT_DIR/setup.sh" --show-config 2>&1 | grep -q "jwt"; then
    log_pass "--show-config shows JWT configuration"
else
    log_fail "--show-config failed"
fi

# =============================================================================
# Test 18: Health check command works
# =============================================================================

log_test "Test 18: Health check command works"
if bash "$SCRIPT_DIR/setup.sh" --health 2>&1 | grep -q "Python"; then
    log_pass "--health shows Python status"
else
    log_fail "--health failed"
fi

# =============================================================================
# Test 19: Hash password command works
# =============================================================================

log_test "Test 19: Hash password command works"
if bash "$SCRIPT_DIR/setup.sh" --hash-password "TestPass123" 2>&1 | grep -q "Hashed:"; then
    log_pass "--hash-password produces hash"
else
    log_fail "--hash-password failed"
fi

# =============================================================================
# Test 20: Validate password - valid
# =============================================================================

log_test "Test 20: Validate password - valid password"
if bash "$SCRIPT_DIR/setup.sh" --validate-password "SecurePass123" 2>&1 | grep -q "meets requirements"; then
    log_pass "Valid password accepted"
else
    log_fail "Valid password rejected"
fi

# =============================================================================
# Test 21: Validate password - invalid
# =============================================================================

log_test "Test 21: Validate password - invalid password"
if bash "$SCRIPT_DIR/setup.sh" --validate-password "weak" 2>&1 | grep -q "Invalid"; then
    log_pass "Weak password rejected"
else
    log_fail "Weak password accepted"
fi

# =============================================================================
# Test 22: Error on missing action
# =============================================================================

log_test "Test 22: Error on missing action"
if bash "$SCRIPT_DIR/setup.sh" 2>&1 | grep -q "No action specified"; then
    log_pass "Error shown when no action specified"
else
    log_fail "No error for missing action"
fi

# =============================================================================
# Test 23: Python unit tests
# =============================================================================

log_test "Test 23: Python unit tests"
if python3 "$SCRIPT_DIR/test_auth.py" 2>&1; then
    log_pass "Python unit tests passed"
else
    log_fail "Python unit tests failed"
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
