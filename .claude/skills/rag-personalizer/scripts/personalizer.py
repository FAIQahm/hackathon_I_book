#!/usr/bin/env python3
"""
RAG Personalizer Module
Transform textbook content based on 10-dimension user profiles.

Usage:
    from personalizer import Personalizer

    p = Personalizer()
    profile = p.create_profile("user123", {"learning_style": "visual"})
    result = p.personalize("Content here...", profile, "What is ROS 2?")
"""

import os
import json
import hashlib
from pathlib import Path
from datetime import datetime, timezone
from typing import Optional, Literal, List, Dict, Any
from dataclasses import dataclass, field, asdict

try:
    from openai import OpenAI
    from pydantic import BaseModel, Field
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Run: pip install openai pydantic")
    raise


# =============================================================================
# Type Definitions
# =============================================================================

LearningStyle = Literal["visual", "auditory", "kinesthetic", "reading"]
KnowledgeLevel = Literal["beginner", "intermediate", "advanced"]
LearningPace = Literal["slow", "moderate", "fast"]
Language = Literal["en", "ur"]
ContentDepth = Literal["overview", "standard", "deep-dive"]
ExamplePreference = Literal["theoretical", "practical", "code-heavy"]
DifficultyTolerance = Literal["easy", "moderate", "challenging"]
InteractionStyle = Literal["passive", "interactive", "hands-on"]
TimeAvailability = Literal["limited", "moderate", "extensive"]
GoalOrientation = Literal["certification", "understanding", "application"]


# =============================================================================
# Models
# =============================================================================

class ProfileDimensions(BaseModel):
    """10-dimension learner profile."""
    learning_style: LearningStyle = "reading"
    knowledge_level: KnowledgeLevel = "beginner"
    learning_pace: LearningPace = "moderate"
    language: Language = "en"
    content_depth: ContentDepth = "standard"
    example_preference: ExamplePreference = "practical"
    difficulty_tolerance: DifficultyTolerance = "moderate"
    interaction_style: InteractionStyle = "interactive"
    time_availability: TimeAvailability = "moderate"
    goal_orientation: GoalOrientation = "understanding"


class Profile(BaseModel):
    """User profile with dimensions."""
    id: str
    created_at: str = Field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    updated_at: str = Field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    dimensions: ProfileDimensions = Field(default_factory=ProfileDimensions)


@dataclass
class PersonalizedContent:
    """Result of content personalization."""
    original_content: str
    personalized_content: str
    profile_id: str
    adaptations_applied: List[str] = field(default_factory=list)
    language: str = "en"
    model_used: str = "gpt-4o-mini"
    tokens_used: int = 0


# =============================================================================
# Dimension Validators
# =============================================================================

VALID_DIMENSIONS = {
    "learning_style": ["visual", "auditory", "kinesthetic", "reading"],
    "knowledge_level": ["beginner", "intermediate", "advanced"],
    "learning_pace": ["slow", "moderate", "fast"],
    "language": ["en", "ur"],
    "content_depth": ["overview", "standard", "deep-dive"],
    "example_preference": ["theoretical", "practical", "code-heavy"],
    "difficulty_tolerance": ["easy", "moderate", "challenging"],
    "interaction_style": ["passive", "interactive", "hands-on"],
    "time_availability": ["limited", "moderate", "extensive"],
    "goal_orientation": ["certification", "understanding", "application"]
}

DEFAULT_DIMENSIONS = {
    "learning_style": "reading",
    "knowledge_level": "beginner",
    "learning_pace": "moderate",
    "language": "en",
    "content_depth": "standard",
    "example_preference": "practical",
    "difficulty_tolerance": "moderate",
    "interaction_style": "interactive",
    "time_availability": "moderate",
    "goal_orientation": "understanding"
}


# =============================================================================
# Personalizer Class
# =============================================================================

class Personalizer:
    """Manage user profiles and personalize content."""

    PROFILES_DIR = ".profiles"
    DEFAULT_MODEL = "gpt-4o-mini"

    def __init__(
        self,
        openai_api_key: Optional[str] = None,
        profiles_dir: Optional[str] = None,
        prompts_path: Optional[str] = None
    ):
        """Initialize personalizer with OpenAI client."""
        self.api_key = openai_api_key or os.environ.get("OPENAI_API_KEY")

        if not self.api_key:
            raise ValueError("OPENAI_API_KEY not set")

        self.client = OpenAI(api_key=self.api_key)

        # Set profiles directory
        if profiles_dir:
            self.profiles_dir = Path(profiles_dir)
        else:
            # Default to project root .profiles
            script_dir = Path(__file__).parent
            project_root = script_dir.parent.parent.parent.parent
            self.profiles_dir = project_root / self.PROFILES_DIR

        self.profiles_dir.mkdir(parents=True, exist_ok=True)

        # Load prompts
        if prompts_path:
            self.prompts_path = Path(prompts_path)
        else:
            self.prompts_path = Path(__file__).parent.parent / "assets" / "prompts.json"

        self.prompts = self._load_prompts()

    def _load_prompts(self) -> dict:
        """Load personalization prompts from JSON."""
        if self.prompts_path.exists():
            return json.loads(self.prompts_path.read_text())
        return {
            "base_system": "You are an expert educational content adapter.",
            "dimension_instructions": {}
        }

    def _profile_path(self, profile_id: str) -> Path:
        """Get path to profile JSON file."""
        safe_id = hashlib.md5(profile_id.encode()).hexdigest()[:12]
        return self.profiles_dir / f"{safe_id}_{profile_id}.json"

    # =========================================================================
    # Profile Management
    # =========================================================================

    def create_profile(
        self,
        profile_id: str,
        dimensions: Optional[Dict[str, str]] = None
    ) -> Profile:
        """Create a new user profile."""
        # Check if exists
        path = self._profile_path(profile_id)
        if path.exists():
            raise ValueError(f"Profile '{profile_id}' already exists")

        # Build dimensions
        dims = ProfileDimensions(**{
            **DEFAULT_DIMENSIONS,
            **(dimensions or {})
        })

        profile = Profile(id=profile_id, dimensions=dims)

        # Save
        path.write_text(profile.model_dump_json(indent=2))
        return profile

    def get_profile(self, profile_id: str) -> Profile:
        """Get a user profile by ID."""
        path = self._profile_path(profile_id)
        if not path.exists():
            raise ValueError(f"Profile '{profile_id}' not found")

        data = json.loads(path.read_text())
        return Profile(**data)

    def update_profile(
        self,
        profile_id: str,
        dimension: str,
        value: str
    ) -> Profile:
        """Update a single dimension of a profile."""
        # Validate dimension
        if dimension not in VALID_DIMENSIONS:
            raise ValueError(f"Invalid dimension: {dimension}")

        if value not in VALID_DIMENSIONS[dimension]:
            raise ValueError(
                f"Invalid value '{value}' for dimension '{dimension}'. "
                f"Valid options: {VALID_DIMENSIONS[dimension]}"
            )

        # Get existing profile
        profile = self.get_profile(profile_id)

        # Update dimension
        setattr(profile.dimensions, dimension, value)
        profile.updated_at = datetime.now(timezone.utc).isoformat()

        # Save
        path = self._profile_path(profile_id)
        path.write_text(profile.model_dump_json(indent=2))

        return profile

    def delete_profile(self, profile_id: str) -> bool:
        """Delete a user profile."""
        path = self._profile_path(profile_id)
        if path.exists():
            path.unlink()
            return True
        return False

    def list_profiles(self) -> List[Profile]:
        """List all profiles."""
        profiles = []
        for path in self.profiles_dir.glob("*.json"):
            try:
                data = json.loads(path.read_text())
                profiles.append(Profile(**data))
            except Exception:
                continue
        return profiles

    # =========================================================================
    # Content Personalization
    # =========================================================================

    def _build_adaptation_instructions(self, profile: Profile) -> str:
        """Build personalization instructions from profile dimensions."""
        instructions = []
        dim_instructions = self.prompts.get("dimension_instructions", {})

        dims = profile.dimensions.model_dump()
        for dim_name, dim_value in dims.items():
            if dim_name in dim_instructions:
                if dim_value in dim_instructions[dim_name]:
                    instructions.append(
                        f"**{dim_name.replace('_', ' ').title()}** ({dim_value}): "
                        f"{dim_instructions[dim_name][dim_value]}"
                    )

        return "\n".join(instructions)

    def _build_profile_summary(self, profile: Profile) -> str:
        """Build a human-readable profile summary."""
        dims = profile.dimensions.model_dump()
        lines = [f"- **{k.replace('_', ' ').title()}**: {v}" for k, v in dims.items()]
        return "\n".join(lines)

    def personalize(
        self,
        content: str,
        profile: Profile,
        query: Optional[str] = None,
        model: str = DEFAULT_MODEL,
        output_format: str = "markdown"
    ) -> PersonalizedContent:
        """
        Personalize content based on user profile.

        Args:
            content: The content to personalize
            profile: User profile with dimensions
            query: Optional query context
            model: OpenAI model to use
            output_format: Output format (text, markdown, json)

        Returns:
            PersonalizedContent with adapted content
        """
        # Build prompt components
        profile_summary = self._build_profile_summary(profile)
        adaptation_instructions = self._build_adaptation_instructions(profile)

        # Build personalization prompt
        template = self.prompts.get(
            "personalization_template",
            "Adapt this content for the learner:\n\n{content}"
        )

        user_prompt = template.format(
            profile_summary=profile_summary,
            content=content,
            query=query or "General learning",
            adaptation_instructions=adaptation_instructions,
            output_format=output_format
        )

        system_prompt = self.prompts.get("base_system", "You are an educational content adapter.")

        # Check if translation is needed
        if profile.dimensions.language == "ur":
            system_prompt += "\n\nIMPORTANT: Provide the final output in Urdu (اردو). Keep technical terms in English with Urdu transliteration."

        # Call OpenAI
        response = self.client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=4000
        )

        personalized_text = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0

        # Build list of adaptations applied
        adaptations = [
            f"{k}: {v}" for k, v in profile.dimensions.model_dump().items()
        ]

        return PersonalizedContent(
            original_content=content,
            personalized_content=personalized_text,
            profile_id=profile.id,
            adaptations_applied=adaptations,
            language=profile.dimensions.language,
            model_used=model,
            tokens_used=tokens_used
        )

    def personalize_with_rag(
        self,
        query: str,
        profile: Profile,
        qdrant_results: List[Dict[str, Any]],
        model: str = DEFAULT_MODEL
    ) -> PersonalizedContent:
        """
        Personalize RAG-retrieved content.

        Args:
            query: User's query
            profile: User profile
            qdrant_results: Results from QdrantManager.query()
            model: OpenAI model to use

        Returns:
            PersonalizedContent with adapted RAG results
        """
        # Combine RAG results into context
        context_parts = []
        for i, result in enumerate(qdrant_results[:5], 1):
            source = result.get('source', 'Unknown')
            section = result.get('section', '')
            content = result.get('content', '')
            context_parts.append(f"### Source {i}: {section}\n{content}")

        combined_content = "\n\n".join(context_parts)

        return self.personalize(
            content=combined_content,
            profile=profile,
            query=query,
            model=model
        )


# =============================================================================
# CLI Interface
# =============================================================================

def main():
    """Command-line interface for Personalizer."""
    import argparse

    parser = argparse.ArgumentParser(
        description="RAG Personalizer - Personalize content based on user profiles"
    )

    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # create-profile
    create_parser = subparsers.add_parser("create-profile", help="Create user profile")
    create_parser.add_argument("id", help="Profile ID")
    create_parser.add_argument("--dimension", action="append", nargs=2,
                               metavar=("DIM", "VALUE"), help="Set dimension value")

    # get-profile
    get_parser = subparsers.add_parser("get-profile", help="Get profile details")
    get_parser.add_argument("id", help="Profile ID")

    # update-profile
    update_parser = subparsers.add_parser("update-profile", help="Update profile")
    update_parser.add_argument("id", help="Profile ID")
    update_parser.add_argument("--dimension", required=True, help="Dimension to update")
    update_parser.add_argument("--value", required=True, help="New value")

    # delete-profile
    delete_parser = subparsers.add_parser("delete-profile", help="Delete profile")
    delete_parser.add_argument("id", help="Profile ID")

    # list-profiles
    subparsers.add_parser("list-profiles", help="List all profiles")

    # personalize
    pers_parser = subparsers.add_parser("personalize", help="Personalize content")
    pers_parser.add_argument("--content", required=True, help="Content to personalize")
    pers_parser.add_argument("--profile", default="default", help="Profile ID")
    pers_parser.add_argument("--query", help="Query context")
    pers_parser.add_argument("--model", default="gpt-4o-mini", help="OpenAI model")
    pers_parser.add_argument("--output", default="markdown",
                            choices=["text", "markdown", "json"], help="Output format")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    try:
        personalizer = Personalizer()

        if args.command == "create-profile":
            dimensions = {}
            if args.dimension:
                for dim, val in args.dimension:
                    dimensions[dim] = val

            profile = personalizer.create_profile(args.id, dimensions)
            print(f"Created profile: {profile.id}")
            print(profile.model_dump_json(indent=2))

        elif args.command == "get-profile":
            profile = personalizer.get_profile(args.id)
            print(profile.model_dump_json(indent=2))

        elif args.command == "update-profile":
            profile = personalizer.update_profile(args.id, args.dimension, args.value)
            print(f"Updated {args.dimension} = {args.value}")
            print(profile.model_dump_json(indent=2))

        elif args.command == "delete-profile":
            if personalizer.delete_profile(args.id):
                print(f"Deleted profile: {args.id}")
            else:
                print(f"Profile not found: {args.id}")

        elif args.command == "list-profiles":
            profiles = personalizer.list_profiles()
            print(f"Found {len(profiles)} profiles:")
            for p in profiles:
                print(f"  - {p.id} (lang: {p.dimensions.language}, level: {p.dimensions.knowledge_level})")

        elif args.command == "personalize":
            profile = personalizer.get_profile(args.profile)
            result = personalizer.personalize(
                content=args.content,
                profile=profile,
                query=args.query,
                model=args.model,
                output_format=args.output
            )

            print("=" * 50)
            print("Personalized Content")
            print("=" * 50)
            print(result.personalized_content)
            print("\n" + "=" * 50)
            print(f"Tokens used: {result.tokens_used}")

    except Exception as e:
        print(f"Error: {e}")
        raise


if __name__ == "__main__":
    main()
