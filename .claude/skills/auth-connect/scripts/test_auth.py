#!/usr/bin/env python3
"""
Auth Connect - Unit Tests
Version: 1.0.0
"""

import os
import sys
import unittest
import json
from pathlib import Path
from datetime import datetime, timedelta, timezone

# Add scripts directory to path
SCRIPT_DIR = Path(__file__).parent
sys.path.insert(0, str(SCRIPT_DIR))


class TestImports(unittest.TestCase):
    """Test that modules can be imported."""

    def test_auth_module_imports(self):
        """Test auth module can be imported."""
        import auth
        self.assertTrue(hasattr(auth, 'AuthManager'))

    def test_token_payload_class_exists(self):
        """Test TokenPayload class exists."""
        from auth import TokenPayload
        self.assertTrue(TokenPayload is not None)

    def test_user_create_class_exists(self):
        """Test UserCreate class exists."""
        from auth import UserCreate
        self.assertTrue(UserCreate is not None)

    def test_api_key_class_exists(self):
        """Test APIKey class exists."""
        from auth import APIKey
        self.assertTrue(APIKey is not None)


class TestPasswordHashing(unittest.TestCase):
    """Test password hashing functions."""

    def test_hash_password(self):
        """Test password hashing."""
        from auth import hash_password
        hashed = hash_password("testpassword123")
        self.assertIsInstance(hashed, str)
        self.assertTrue(hashed.startswith("$2"))  # bcrypt prefix

    def test_verify_password_correct(self):
        """Test verifying correct password."""
        from auth import hash_password, verify_password
        password = "testpassword123"
        hashed = hash_password(password)
        self.assertTrue(verify_password(password, hashed))

    def test_verify_password_incorrect(self):
        """Test verifying incorrect password."""
        from auth import hash_password, verify_password
        hashed = hash_password("testpassword123")
        self.assertFalse(verify_password("wrongpassword", hashed))


class TestPasswordValidation(unittest.TestCase):
    """Test password validation."""

    def test_valid_password(self):
        """Test valid password passes validation."""
        from auth import validate_password
        is_valid, msg = validate_password("SecurePass123")
        self.assertTrue(is_valid)
        self.assertEqual(msg, "")

    def test_password_too_short(self):
        """Test short password fails."""
        from auth import validate_password
        is_valid, msg = validate_password("Short1")
        self.assertFalse(is_valid)
        self.assertIn("at least", msg.lower())

    def test_password_no_uppercase(self):
        """Test password without uppercase fails."""
        from auth import validate_password
        is_valid, msg = validate_password("lowercase123")
        self.assertFalse(is_valid)
        self.assertIn("uppercase", msg.lower())

    def test_password_no_lowercase(self):
        """Test password without lowercase fails."""
        from auth import validate_password
        is_valid, msg = validate_password("UPPERCASE123")
        self.assertFalse(is_valid)
        self.assertIn("lowercase", msg.lower())

    def test_password_no_digit(self):
        """Test password without digit fails."""
        from auth import validate_password
        is_valid, msg = validate_password("NoDigitsHere")
        self.assertFalse(is_valid)
        self.assertIn("digit", msg.lower())


class TestTokenPayload(unittest.TestCase):
    """Test TokenPayload dataclass."""

    def test_token_payload_creation(self):
        """Test creating a TokenPayload."""
        from auth import TokenPayload
        payload = TokenPayload(
            user_id="user123",
            email="test@example.com",
            role="student"
        )
        self.assertEqual(payload.user_id, "user123")
        self.assertEqual(payload.email, "test@example.com")
        self.assertEqual(payload.role, "student")

    def test_token_payload_with_permissions(self):
        """Test TokenPayload with permissions."""
        from auth import TokenPayload
        payload = TokenPayload(
            user_id="user123",
            email="test@example.com",
            role="admin",
            permissions=["read", "write", "delete"]
        )
        self.assertEqual(len(payload.permissions), 3)
        self.assertIn("write", payload.permissions)


class TestAuthManager(unittest.TestCase):
    """Test AuthManager class."""

    @classmethod
    def setUpClass(cls):
        """Set up test environment."""
        os.environ["JWT_SECRET"] = "test-secret-key-for-unit-tests-only-12345678"

    def test_auth_manager_creation(self):
        """Test creating AuthManager."""
        from auth import AuthManager
        auth = AuthManager()
        self.assertIsNotNone(auth)

    def test_create_access_token(self):
        """Test creating access token."""
        from auth import AuthManager
        auth = AuthManager()
        token = auth.create_access_token(
            user_id="user123",
            email="test@example.com",
            role="student"
        )
        self.assertIsInstance(token, str)
        self.assertTrue(len(token) > 50)

    def test_create_refresh_token(self):
        """Test creating refresh token."""
        from auth import AuthManager
        auth = AuthManager()
        token = auth.create_refresh_token(user_id="user123")
        self.assertIsInstance(token, str)
        self.assertTrue(len(token) > 50)

    def test_verify_valid_token(self):
        """Test verifying a valid token."""
        from auth import AuthManager
        auth = AuthManager()
        token = auth.create_access_token(
            user_id="user123",
            email="test@example.com",
            role="student"
        )
        payload = auth.verify_token(token)
        self.assertIsNotNone(payload)
        self.assertEqual(payload.user_id, "user123")
        self.assertEqual(payload.email, "test@example.com")

    def test_verify_invalid_token(self):
        """Test verifying an invalid token."""
        from auth import AuthManager
        auth = AuthManager()
        payload = auth.verify_token("invalid-token")
        self.assertIsNone(payload)

    def test_is_token_valid(self):
        """Test is_token_valid method."""
        from auth import AuthManager
        auth = AuthManager()
        token = auth.create_access_token(
            user_id="user123",
            email="test@example.com",
            role="student"
        )
        self.assertTrue(auth.is_token_valid(token))
        self.assertFalse(auth.is_token_valid("invalid-token"))

    def test_revoke_token(self):
        """Test revoking a token."""
        from auth import AuthManager
        auth = AuthManager()
        token = auth.create_access_token(
            user_id="user123",
            email="test@example.com",
            role="student"
        )
        # Token should be valid before revocation
        self.assertTrue(auth.is_token_valid(token))

        # Revoke token
        auth.revoke_token(token)

        # Token should be invalid after revocation
        self.assertFalse(auth.is_token_valid(token))


class TestAPIKeyGeneration(unittest.TestCase):
    """Test API key generation."""

    def test_generate_api_key(self):
        """Test generating an API key."""
        from auth import generate_api_key
        key = generate_api_key("Test App")
        self.assertIsNotNone(key)
        self.assertEqual(key.name, "Test App")
        self.assertTrue(key.key.startswith("pak_"))

    def test_generate_api_key_with_permissions(self):
        """Test generating API key with permissions."""
        from auth import generate_api_key
        key = generate_api_key("Test App", ["read", "write"])
        self.assertEqual(key.permissions, ["read", "write"])

    def test_verify_api_key(self):
        """Test verifying an API key."""
        from auth import generate_api_key, verify_api_key
        import hashlib

        key = generate_api_key("Test App")
        stored_hash = hashlib.sha256(key.key.encode()).hexdigest()

        self.assertTrue(verify_api_key(key.key, stored_hash))
        self.assertFalse(verify_api_key("wrong-key", stored_hash))


class TestUtilityFunctions(unittest.TestCase):
    """Test utility functions."""

    def test_generate_jwt_secret(self):
        """Test generating JWT secret."""
        from auth import generate_jwt_secret
        secret = generate_jwt_secret()
        self.assertIsInstance(secret, str)
        self.assertEqual(len(secret), 64)  # 64 hex chars = 32 bytes

    def test_generate_jwt_secret_custom_length(self):
        """Test generating JWT secret with custom length."""
        from auth import generate_jwt_secret
        secret = generate_jwt_secret(length=128)
        self.assertEqual(len(secret), 128)

    def test_list_roles(self):
        """Test listing roles."""
        from auth import list_roles
        roles = list_roles()
        self.assertIsInstance(roles, list)
        self.assertTrue(len(roles) > 0)

    def test_get_role_permissions(self):
        """Test getting role permissions."""
        from auth import get_role_permissions
        permissions = get_role_permissions("admin")
        self.assertIsInstance(permissions, list)
        self.assertIn("read", permissions)

    def test_has_permission(self):
        """Test has_permission function."""
        from auth import has_permission
        permissions = ["read", "write"]
        self.assertTrue(has_permission(permissions, "read"))
        self.assertFalse(has_permission(permissions, "delete"))


class TestConfiguration(unittest.TestCase):
    """Test configuration loading."""

    def test_load_config(self):
        """Test loading configuration."""
        from auth import load_config
        config = load_config()
        self.assertIsInstance(config, dict)
        self.assertIn("jwt", config)
        self.assertIn("password", config)

    def test_load_roles(self):
        """Test loading roles."""
        from auth import load_roles
        roles = load_roles()
        self.assertIsInstance(roles, dict)
        self.assertIn("roles", roles)
        self.assertIn("admin", roles["roles"])
        self.assertIn("student", roles["roles"])


class TestCLI(unittest.TestCase):
    """Test CLI functionality."""

    def test_main_function_exists(self):
        """Test main function exists."""
        from auth import main
        self.assertTrue(callable(main))


if __name__ == "__main__":
    # Set up test environment
    os.environ["JWT_SECRET"] = "test-secret-key-for-unit-tests-only-12345678"

    # Run tests
    unittest.main(verbosity=2)
