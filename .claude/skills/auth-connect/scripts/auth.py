#!/usr/bin/env python3
"""
Auth Connect - Authentication Module
JWT-based authentication and authorization for FastAPI
Version: 1.0.0
Agent: SecurityLead
"""

import os
import json
import secrets
import hashlib
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional, List, Dict, Any, Union
from dataclasses import dataclass, field

# JWT handling
try:
    import jwt
    from jwt.exceptions import InvalidTokenError, ExpiredSignatureError
except ImportError:
    jwt = None

# Password hashing
try:
    import bcrypt
except ImportError:
    bcrypt = None

# FastAPI integration
try:
    from fastapi import Depends, HTTPException, status
    from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
except ImportError:
    HTTPBearer = None
    Depends = None

# Pydantic models
try:
    from pydantic import BaseModel, EmailStr, field_validator
except ImportError:
    BaseModel = object
    EmailStr = str


# =============================================================================
# Configuration
# =============================================================================

SCRIPT_DIR = Path(__file__).parent
SKILL_DIR = SCRIPT_DIR.parent
ASSETS_DIR = SKILL_DIR / "assets"

# Load configuration
def load_config() -> Dict[str, Any]:
    """Load auth configuration from JSON file."""
    config_path = ASSETS_DIR / "auth_config.json"
    if config_path.exists():
        with open(config_path, "r") as f:
            return json.load(f)
    return {}

def load_roles() -> Dict[str, Any]:
    """Load role definitions from JSON file."""
    roles_path = ASSETS_DIR / "roles.json"
    if roles_path.exists():
        with open(roles_path, "r") as f:
            return json.load(f)
    return {"roles": {}, "default_role": "student"}

CONFIG = load_config()
ROLES = load_roles()


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class TokenPayload:
    """JWT token payload."""
    user_id: str
    email: str
    role: str
    permissions: List[str] = field(default_factory=list)
    exp: Optional[datetime] = None
    iat: Optional[datetime] = None
    jti: Optional[str] = None  # JWT ID for blacklisting


@dataclass
class UserCreate:
    """User creation data."""
    email: str
    password: str
    role: str = "student"


@dataclass
class TokenResponse:
    """Token response data."""
    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_in: int = 1800  # 30 minutes


@dataclass
class APIKey:
    """API key data."""
    id: str
    name: str
    prefix: str
    key: str  # Only available at creation
    permissions: List[str] = field(default_factory=list)
    created_at: str = ""
    expires_at: Optional[str] = None


# =============================================================================
# Password Hashing
# =============================================================================

def hash_password(password: str) -> str:
    """Hash a password using bcrypt."""
    if bcrypt is None:
        raise ImportError("bcrypt is required for password hashing")

    rounds = CONFIG.get("password", {}).get("bcrypt_rounds", 12)
    salt = bcrypt.gensalt(rounds=rounds)
    hashed = bcrypt.hashpw(password.encode("utf-8"), salt)
    return hashed.decode("utf-8")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash."""
    if bcrypt is None:
        raise ImportError("bcrypt is required for password verification")

    return bcrypt.checkpw(
        plain_password.encode("utf-8"),
        hashed_password.encode("utf-8")
    )


def validate_password(password: str) -> tuple[bool, str]:
    """
    Validate password against security requirements.
    Returns (is_valid, error_message).
    """
    config = CONFIG.get("password", {})
    min_length = config.get("min_length", 8)
    max_length = config.get("max_length", 128)
    require_upper = config.get("require_uppercase", True)
    require_lower = config.get("require_lowercase", True)
    require_digit = config.get("require_digit", True)
    require_special = config.get("require_special", False)

    if len(password) < min_length:
        return False, f"Password must be at least {min_length} characters"

    if len(password) > max_length:
        return False, f"Password must be at most {max_length} characters"

    if require_upper and not any(c.isupper() for c in password):
        return False, "Password must contain at least one uppercase letter"

    if require_lower and not any(c.islower() for c in password):
        return False, "Password must contain at least one lowercase letter"

    if require_digit and not any(c.isdigit() for c in password):
        return False, "Password must contain at least one digit"

    if require_special:
        special_chars = "!@#$%^&*()_+-=[]{}|;:,.<>?"
        if not any(c in special_chars for c in password):
            return False, "Password must contain at least one special character"

    return True, ""


# =============================================================================
# JWT Token Management
# =============================================================================

class AuthManager:
    """
    Authentication manager for JWT token operations.
    """

    def __init__(
        self,
        secret_key: Optional[str] = None,
        algorithm: Optional[str] = None
    ):
        """
        Initialize AuthManager.

        Args:
            secret_key: JWT secret key (defaults to JWT_SECRET env var)
            algorithm: JWT algorithm (defaults to config or HS256)
        """
        if jwt is None:
            raise ImportError("PyJWT is required for token management")

        self.secret_key = secret_key or os.environ.get("JWT_SECRET", "")
        if not self.secret_key:
            raise ValueError("JWT_SECRET environment variable is required")

        jwt_config = CONFIG.get("jwt", {})
        self.algorithm = algorithm or jwt_config.get("algorithm", "HS256")
        self.access_token_expire = jwt_config.get("access_token_expire_minutes", 30)
        self.refresh_token_expire = jwt_config.get("refresh_token_expire_days", 7)
        self.issuer = jwt_config.get("issuer", "physical-ai-book")
        self.audience = jwt_config.get("audience", "physical-ai-book-users")

        # Token blacklist (in-memory, use Redis in production)
        self._blacklist: set = set()

    def create_access_token(
        self,
        user_id: str,
        email: str,
        role: str,
        additional_claims: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Create a new access token.

        Args:
            user_id: User identifier
            email: User email
            role: User role
            additional_claims: Extra claims to include

        Returns:
            JWT access token string
        """
        now = datetime.now(timezone.utc)
        expire = now + timedelta(minutes=self.access_token_expire)

        # Get permissions for role
        role_data = ROLES.get("roles", {}).get(role, {})
        permissions = role_data.get("permissions", [])

        payload = {
            "sub": user_id,
            "email": email,
            "role": role,
            "permissions": permissions,
            "type": "access",
            "iat": now,
            "exp": expire,
            "iss": self.issuer,
            "aud": self.audience,
            "jti": secrets.token_hex(16)
        }

        if additional_claims:
            payload.update(additional_claims)

        return jwt.encode(payload, self.secret_key, algorithm=self.algorithm)

    def create_refresh_token(self, user_id: str) -> str:
        """
        Create a new refresh token.

        Args:
            user_id: User identifier

        Returns:
            JWT refresh token string
        """
        now = datetime.now(timezone.utc)
        expire = now + timedelta(days=self.refresh_token_expire)

        payload = {
            "sub": user_id,
            "type": "refresh",
            "iat": now,
            "exp": expire,
            "iss": self.issuer,
            "jti": secrets.token_hex(16)
        }

        return jwt.encode(payload, self.secret_key, algorithm=self.algorithm)

    def verify_token(self, token: str) -> Optional[TokenPayload]:
        """
        Verify and decode a JWT token.

        Args:
            token: JWT token string

        Returns:
            TokenPayload if valid, None otherwise
        """
        try:
            payload = jwt.decode(
                token,
                self.secret_key,
                algorithms=[self.algorithm],
                issuer=self.issuer,
                audience=self.audience
            )

            # Check blacklist
            jti = payload.get("jti")
            if jti and jti in self._blacklist:
                return None

            return TokenPayload(
                user_id=payload.get("sub", ""),
                email=payload.get("email", ""),
                role=payload.get("role", ""),
                permissions=payload.get("permissions", []),
                exp=datetime.fromtimestamp(payload.get("exp", 0), timezone.utc),
                iat=datetime.fromtimestamp(payload.get("iat", 0), timezone.utc),
                jti=jti
            )

        except ExpiredSignatureError:
            return None
        except InvalidTokenError:
            return None

    def refresh_access_token(self, refresh_token: str) -> Optional[str]:
        """
        Generate new access token from refresh token.

        Args:
            refresh_token: Valid refresh token

        Returns:
            New access token or None if refresh token is invalid
        """
        try:
            payload = jwt.decode(
                refresh_token,
                self.secret_key,
                algorithms=[self.algorithm],
                issuer=self.issuer
            )

            if payload.get("type") != "refresh":
                return None

            # Check blacklist
            jti = payload.get("jti")
            if jti and jti in self._blacklist:
                return None

            user_id = payload.get("sub")
            # In production, fetch user from database to get current role/email
            # For now, we just return a basic token
            return self.create_access_token(
                user_id=user_id,
                email="",  # Should be fetched from DB
                role="student"  # Should be fetched from DB
            )

        except (ExpiredSignatureError, InvalidTokenError):
            return None

    def revoke_token(self, token: str) -> bool:
        """
        Revoke a token by adding it to blacklist.

        Args:
            token: Token to revoke

        Returns:
            True if revoked successfully
        """
        try:
            payload = jwt.decode(
                token,
                self.secret_key,
                algorithms=[self.algorithm],
                options={
                    "verify_exp": False,
                    "verify_aud": False
                }
            )
            jti = payload.get("jti")
            if jti:
                self._blacklist.add(jti)
                return True
        except InvalidTokenError:
            pass
        return False

    def is_token_valid(self, token: str) -> bool:
        """Check if token is valid without returning payload."""
        return self.verify_token(token) is not None


# =============================================================================
# API Key Management
# =============================================================================

def generate_api_key(name: str, permissions: Optional[List[str]] = None) -> APIKey:
    """
    Generate a new API key.

    Args:
        name: Name/description for the key
        permissions: List of permissions for this key

    Returns:
        APIKey with the plaintext key (only available at creation)
    """
    config = CONFIG.get("api_key", {})
    prefix_length = config.get("prefix_length", 8)
    key_length = config.get("key_length", 32)

    # Generate key parts
    prefix = secrets.token_hex(prefix_length // 2)
    key_secret = secrets.token_hex(key_length // 2)
    full_key = f"pak_{prefix}_{key_secret}"  # pak = Physical AI Key

    # Hash for storage
    key_hash = hashlib.sha256(full_key.encode()).hexdigest()

    return APIKey(
        id=secrets.token_hex(8),
        name=name,
        prefix=f"pak_{prefix}",
        key=full_key,
        permissions=permissions or ["read"],
        created_at=datetime.now(timezone.utc).isoformat()
    )


def verify_api_key(key: str, stored_hash: str) -> bool:
    """
    Verify an API key against its stored hash.

    Args:
        key: Plaintext API key
        stored_hash: SHA256 hash of the key

    Returns:
        True if key matches hash
    """
    key_hash = hashlib.sha256(key.encode()).hexdigest()
    return secrets.compare_digest(key_hash, stored_hash)


# =============================================================================
# FastAPI Dependencies
# =============================================================================

if HTTPBearer is not None:
    security = HTTPBearer()

    async def get_current_user(
        credentials: HTTPAuthorizationCredentials = Depends(security)
    ) -> TokenPayload:
        """
        FastAPI dependency to get current authenticated user.

        Usage:
            @app.get("/protected")
            async def protected(user: TokenPayload = Depends(get_current_user)):
                return {"user": user.user_id}
        """
        token = credentials.credentials

        try:
            auth = AuthManager()
            payload = auth.verify_token(token)

            if payload is None:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="Invalid or expired token",
                    headers={"WWW-Authenticate": "Bearer"}
                )

            return payload

        except ValueError as e:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=str(e)
            )

    def require_role(allowed_roles: Union[str, List[str]]):
        """
        FastAPI dependency factory to require specific role(s).

        Usage:
            @app.get("/admin")
            async def admin(user: TokenPayload = Depends(require_role("admin"))):
                return {"admin": True}

            @app.get("/staff")
            async def staff(user: TokenPayload = Depends(require_role(["admin", "instructor"]))):
                return {"staff": True}
        """
        if isinstance(allowed_roles, str):
            allowed_roles = [allowed_roles]

        async def role_checker(
            credentials: HTTPAuthorizationCredentials = Depends(security)
        ) -> TokenPayload:
            token = credentials.credentials

            try:
                auth = AuthManager()
                payload = auth.verify_token(token)

                if payload is None:
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="Invalid or expired token",
                        headers={"WWW-Authenticate": "Bearer"}
                    )

                # Check role hierarchy
                user_role = payload.role
                role_hierarchy = ROLES.get("role_hierarchy", {})

                # User's effective roles include their role and all roles they inherit
                effective_roles = {user_role}
                roles_to_check = [user_role]
                while roles_to_check:
                    current = roles_to_check.pop()
                    inherited = role_hierarchy.get(current, [])
                    for r in inherited:
                        if r not in effective_roles:
                            effective_roles.add(r)
                            roles_to_check.append(r)

                # Check if any allowed role is in effective roles
                if not any(role in effective_roles for role in allowed_roles):
                    raise HTTPException(
                        status_code=status.HTTP_403_FORBIDDEN,
                        detail=f"Insufficient permissions. Required: {allowed_roles}"
                    )

                return payload

            except ValueError as e:
                raise HTTPException(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    detail=str(e)
                )

        return role_checker

    def require_permission(permission: str):
        """
        FastAPI dependency factory to require specific permission.

        Usage:
            @app.delete("/content/{id}")
            async def delete(user: TokenPayload = Depends(require_permission("delete"))):
                return {"deleted": True}
        """
        async def permission_checker(
            credentials: HTTPAuthorizationCredentials = Depends(security)
        ) -> TokenPayload:
            token = credentials.credentials

            try:
                auth = AuthManager()
                payload = auth.verify_token(token)

                if payload is None:
                    raise HTTPException(
                        status_code=status.HTTP_401_UNAUTHORIZED,
                        detail="Invalid or expired token",
                        headers={"WWW-Authenticate": "Bearer"}
                    )

                if permission not in payload.permissions:
                    raise HTTPException(
                        status_code=status.HTTP_403_FORBIDDEN,
                        detail=f"Missing permission: {permission}"
                    )

                return payload

            except ValueError as e:
                raise HTTPException(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    detail=str(e)
                )

        return permission_checker


# =============================================================================
# Utility Functions
# =============================================================================

def generate_jwt_secret(length: int = 64) -> str:
    """Generate a secure random JWT secret."""
    return secrets.token_hex(length // 2)


def get_role_permissions(role: str) -> List[str]:
    """Get permissions for a given role."""
    role_data = ROLES.get("roles", {}).get(role, {})
    return role_data.get("permissions", [])


def list_roles() -> List[Dict[str, Any]]:
    """List all available roles."""
    roles = ROLES.get("roles", {})
    return [
        {
            "name": name,
            "display_name": data.get("display_name", name),
            "description": data.get("description", ""),
            "permissions": data.get("permissions", []),
            "can_be_assigned": data.get("can_be_assigned", True)
        }
        for name, data in roles.items()
    ]


def has_permission(user_permissions: List[str], required: str) -> bool:
    """Check if user has a specific permission."""
    return required in user_permissions


# =============================================================================
# CLI Entry Point
# =============================================================================

def main():
    """CLI entry point for auth module."""
    import argparse

    parser = argparse.ArgumentParser(description="Auth Connect CLI")
    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # Generate secret
    subparsers.add_parser("generate-secret", help="Generate JWT secret")

    # List roles
    subparsers.add_parser("list-roles", help="List all roles")

    # Verify token
    verify_parser = subparsers.add_parser("verify-token", help="Verify a token")
    verify_parser.add_argument("token", help="Token to verify")

    # Generate API key
    apikey_parser = subparsers.add_parser("generate-api-key", help="Generate API key")
    apikey_parser.add_argument("--name", required=True, help="Key name")
    apikey_parser.add_argument("--permissions", nargs="+", default=["read"], help="Permissions")

    # Hash password
    hash_parser = subparsers.add_parser("hash-password", help="Hash a password")
    hash_parser.add_argument("password", help="Password to hash")

    # Validate password
    validate_parser = subparsers.add_parser("validate-password", help="Validate password strength")
    validate_parser.add_argument("password", help="Password to validate")

    # Show config
    subparsers.add_parser("show-config", help="Show configuration")

    args = parser.parse_args()

    if args.command == "generate-secret":
        secret = generate_jwt_secret()
        print(f"JWT_SECRET={secret}")

    elif args.command == "list-roles":
        roles = list_roles()
        for role in roles:
            print(f"\n{role['display_name']} ({role['name']})")
            print(f"  {role['description']}")
            print(f"  Permissions: {', '.join(role['permissions'])}")

    elif args.command == "verify-token":
        try:
            auth = AuthManager()
            payload = auth.verify_token(args.token)
            if payload:
                print(f"Valid token")
                print(f"  User ID: {payload.user_id}")
                print(f"  Email: {payload.email}")
                print(f"  Role: {payload.role}")
                print(f"  Expires: {payload.exp}")
            else:
                print("Invalid or expired token")
        except ValueError as e:
            print(f"Error: {e}")

    elif args.command == "generate-api-key":
        key = generate_api_key(args.name, args.permissions)
        print(f"API Key generated:")
        print(f"  Name: {key.name}")
        print(f"  Key: {key.key}")
        print(f"  Prefix: {key.prefix}")
        print(f"  Permissions: {', '.join(key.permissions)}")
        print(f"\nSave the key now - it cannot be recovered!")

    elif args.command == "hash-password":
        hashed = hash_password(args.password)
        print(f"Hashed: {hashed}")

    elif args.command == "validate-password":
        is_valid, message = validate_password(args.password)
        if is_valid:
            print("Password meets requirements")
        else:
            print(f"Invalid: {message}")

    elif args.command == "show-config":
        print(json.dumps(CONFIG, indent=2))

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
