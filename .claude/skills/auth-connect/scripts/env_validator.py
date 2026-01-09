#!/usr/bin/env python3
"""
Environment Variable Validator
Ensures environment variables set by BackendIntegrator are properly loaded.
Version: 1.0.0
Agent: SecurityLead (validates) + BackendIntegrator (deploys)
"""

import os
import re
import json
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from datetime import datetime, timezone

# =============================================================================
# Configuration
# =============================================================================

SCRIPT_DIR = Path(__file__).parent
SKILL_DIR = SCRIPT_DIR.parent
ASSETS_DIR = SKILL_DIR / "assets"
CONTRACT_PATH = ASSETS_DIR / "env_contract.json"


@dataclass
class ValidationResult:
    """Result of validating a single environment variable."""
    name: str
    is_set: bool
    is_valid: bool
    is_required: bool
    error: Optional[str] = None
    masked_value: Optional[str] = None  # e.g., "sk-...abc" for security


@dataclass
class EnvValidationReport:
    """Complete environment validation report."""
    timestamp: str
    all_valid: bool
    required_valid: bool
    total_checked: int
    required_count: int
    optional_count: int
    missing_required: List[str] = field(default_factory=list)
    missing_optional: List[str] = field(default_factory=list)
    invalid: List[str] = field(default_factory=list)
    results: List[ValidationResult] = field(default_factory=list)


def load_contract() -> Dict[str, Any]:
    """Load the environment variable contract."""
    if CONTRACT_PATH.exists():
        with open(CONTRACT_PATH, "r") as f:
            return json.load(f)
    return {"variables": {"required": {}, "optional": {}}}


def mask_value(value: str, show_chars: int = 4) -> str:
    """Mask a sensitive value for logging."""
    if not value:
        return ""
    if len(value) <= show_chars * 2:
        return "*" * len(value)
    return f"{value[:show_chars]}...{value[-show_chars:]}"


def validate_pattern(value: str, pattern: str) -> bool:
    """Validate value against a regex pattern."""
    try:
        return bool(re.match(pattern, value))
    except re.error:
        return False


def validate_enum(value: str, allowed: List[str]) -> bool:
    """Validate value is in allowed list."""
    return value.upper() in [v.upper() for v in allowed]


def validate_variable(
    name: str,
    config: Dict[str, Any],
    is_required: bool
) -> ValidationResult:
    """
    Validate a single environment variable.

    Args:
        name: Variable name
        config: Variable configuration from contract
        is_required: Whether this variable is required

    Returns:
        ValidationResult with status
    """
    value = os.environ.get(name, "")

    result = ValidationResult(
        name=name,
        is_set=bool(value),
        is_valid=False,
        is_required=is_required
    )

    if not value:
        if is_required:
            result.error = "Required variable not set"
        else:
            # Optional and not set is valid
            result.is_valid = True
        return result

    # Mask the value for reporting
    result.masked_value = mask_value(value)

    # Get validation rules
    validation = config.get("validation", {})

    # Check minimum length
    min_length = validation.get("min_length")
    if min_length and len(value) < min_length:
        result.error = f"Too short (min {min_length} chars, got {len(value)})"
        return result

    # Check pattern
    pattern = validation.get("pattern")
    if pattern and not validate_pattern(value, pattern):
        result.error = f"Doesn't match expected pattern"
        return result

    # Check enum
    allowed_values = validation.get("enum")
    if allowed_values and not validate_enum(value, allowed_values):
        result.error = f"Must be one of: {', '.join(allowed_values)}"
        return result

    # All validations passed
    result.is_valid = True
    return result


def validate_all() -> EnvValidationReport:
    """
    Validate all environment variables from the contract.

    Returns:
        EnvValidationReport with complete results
    """
    contract = load_contract()
    variables = contract.get("variables", {})
    required_vars = variables.get("required", {})
    optional_vars = variables.get("optional", {})

    report = EnvValidationReport(
        timestamp=datetime.now(timezone.utc).isoformat(),
        all_valid=True,
        required_valid=True,
        total_checked=len(required_vars) + len(optional_vars),
        required_count=len(required_vars),
        optional_count=len(optional_vars)
    )

    # Validate required variables
    for name, config in required_vars.items():
        result = validate_variable(name, config, is_required=True)
        report.results.append(result)

        if not result.is_set:
            report.missing_required.append(name)
            report.required_valid = False
            report.all_valid = False
        elif not result.is_valid:
            report.invalid.append(name)
            report.required_valid = False
            report.all_valid = False

    # Validate optional variables
    for name, config in optional_vars.items():
        result = validate_variable(name, config, is_required=False)
        report.results.append(result)

        if not result.is_set:
            report.missing_optional.append(name)
        elif not result.is_valid:
            report.invalid.append(name)
            report.all_valid = False

    return report


def get_status_dict() -> Dict[str, Any]:
    """
    Get environment status as a dictionary (for API endpoints).

    Returns:
        Dictionary with validation status
    """
    report = validate_all()

    return {
        "timestamp": report.timestamp,
        "status": "healthy" if report.required_valid else "unhealthy",
        "all_valid": report.all_valid,
        "required_valid": report.required_valid,
        "summary": {
            "total": report.total_checked,
            "required": report.required_count,
            "optional": report.optional_count,
            "missing_required": len(report.missing_required),
            "missing_optional": len(report.missing_optional),
            "invalid": len(report.invalid)
        },
        "missing_required": report.missing_required,
        "missing_optional": report.missing_optional,
        "invalid": report.invalid,
        "variables": {
            r.name: {
                "set": r.is_set,
                "valid": r.is_valid,
                "required": r.is_required,
                "masked_value": r.masked_value,
                "error": r.error
            }
            for r in report.results
        }
    }


def check_required() -> bool:
    """
    Quick check if all required variables are valid.

    Returns:
        True if all required variables are set and valid
    """
    report = validate_all()
    return report.required_valid


def get_missing_required() -> List[str]:
    """
    Get list of missing required variables.

    Returns:
        List of missing variable names
    """
    report = validate_all()
    return report.missing_required


# =============================================================================
# FastAPI Integration
# =============================================================================

def create_env_health_endpoint():
    """
    Create a FastAPI endpoint for environment health checks.

    Usage in main.py:
        from env_validator import create_env_health_endpoint
        app.get("/api/health/env")(create_env_health_endpoint())
    """
    async def env_health():
        """Check environment variable health."""
        status = get_status_dict()
        return status

    return env_health


def require_env_vars(*var_names: str):
    """
    Decorator/dependency to require specific environment variables.

    Usage:
        @app.get("/api/protected")
        @require_env_vars("OPENAI_API_KEY", "DATABASE_URL")
        async def protected_endpoint():
            ...
    """
    def decorator(func):
        async def wrapper(*args, **kwargs):
            missing = []
            for var in var_names:
                if not os.environ.get(var):
                    missing.append(var)

            if missing:
                # Import here to avoid circular dependency
                from fastapi import HTTPException
                raise HTTPException(
                    status_code=503,
                    detail=f"Service unavailable: missing environment variables: {', '.join(missing)}"
                )

            return await func(*args, **kwargs)

        wrapper.__name__ = func.__name__
        wrapper.__doc__ = func.__doc__
        return wrapper

    return decorator


# =============================================================================
# CLI Entry Point
# =============================================================================

def print_report(report: EnvValidationReport) -> None:
    """Print validation report to console."""
    try:
        from rich.console import Console
        from rich.table import Table
        from rich.panel import Panel

        console = Console()

        # Header
        status = "[green]HEALTHY[/green]" if report.required_valid else "[red]UNHEALTHY[/red]"
        console.print(Panel(f"Environment Validation: {status}", style="bold"))

        # Summary
        console.print(f"\nTimestamp: {report.timestamp}")
        console.print(f"Required: {report.required_count}, Optional: {report.optional_count}")

        # Table
        table = Table(show_header=True, header_style="bold")
        table.add_column("Variable", style="cyan")
        table.add_column("Required")
        table.add_column("Set")
        table.add_column("Valid")
        table.add_column("Value")
        table.add_column("Error")

        for r in report.results:
            req = "[yellow]Yes[/yellow]" if r.is_required else "No"
            is_set = "[green]Yes[/green]" if r.is_set else "[red]No[/red]"
            valid = "[green]Yes[/green]" if r.is_valid else "[red]No[/red]"
            value = r.masked_value or "-"
            error = r.error or "-"

            table.add_row(r.name, req, is_set, valid, value, error)

        console.print(table)

        # Warnings
        if report.missing_required:
            console.print(f"\n[red]Missing required:[/red] {', '.join(report.missing_required)}")

        if report.invalid:
            console.print(f"[yellow]Invalid:[/yellow] {', '.join(report.invalid)}")

    except ImportError:
        # Fallback without rich
        print(f"\n=== Environment Validation ===")
        print(f"Status: {'HEALTHY' if report.required_valid else 'UNHEALTHY'}")
        print(f"Timestamp: {report.timestamp}")
        print(f"\nVariables:")

        for r in report.results:
            status = "OK" if r.is_valid else "FAIL"
            req = "(required)" if r.is_required else "(optional)"
            print(f"  {r.name} {req}: {status}")
            if r.error:
                print(f"    Error: {r.error}")

        if report.missing_required:
            print(f"\nMissing required: {', '.join(report.missing_required)}")


def main():
    """CLI entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="Environment Variable Validator")
    parser.add_argument("--check", action="store_true", help="Check all variables")
    parser.add_argument("--json", action="store_true", help="Output as JSON")
    parser.add_argument("--required-only", action="store_true", help="Only check required vars")
    parser.add_argument("--var", type=str, help="Check specific variable")

    args = parser.parse_args()

    if args.var:
        # Check specific variable
        value = os.environ.get(args.var, "")
        if value:
            print(f"{args.var}: SET ({mask_value(value)})")
        else:
            print(f"{args.var}: NOT SET")
        return

    # Full validation
    report = validate_all()

    if args.json:
        print(json.dumps(get_status_dict(), indent=2))
    else:
        print_report(report)

    # Exit with error code if required vars are missing
    if not report.required_valid:
        exit(1)


if __name__ == "__main__":
    main()
