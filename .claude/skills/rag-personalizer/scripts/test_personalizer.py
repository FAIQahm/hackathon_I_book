#!/usr/bin/env python3
"""
Unit tests for RAG Personalizer module.
Tests core functionality without requiring actual OpenAI connections.
"""

import sys
import os
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

# Add script directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class TestPersonalizerImports(unittest.TestCase):
    """Test that module can be imported and has expected structure."""

    def test_module_imports(self):
        """Test that personalizer module can be imported."""
        try:
            import personalizer
            self.assertTrue(hasattr(personalizer, 'Personalizer'))
        except ImportError as e:
            self.skipTest(f"Dependencies not installed: {e}")

    def test_profile_class_exists(self):
        """Test Profile class exists."""
        try:
            from personalizer import Profile
            self.assertTrue(Profile is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_profile_dimensions_class_exists(self):
        """Test ProfileDimensions class exists."""
        try:
            from personalizer import ProfileDimensions
            self.assertTrue(ProfileDimensions is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_personalized_content_class_exists(self):
        """Test PersonalizedContent class exists."""
        try:
            from personalizer import PersonalizedContent
            self.assertTrue(PersonalizedContent is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")


class TestProfileDimensions(unittest.TestCase):
    """Test profile dimensions model."""

    def setUp(self):
        try:
            from personalizer import ProfileDimensions, VALID_DIMENSIONS, DEFAULT_DIMENSIONS
            self.ProfileDimensions = ProfileDimensions
            self.VALID_DIMENSIONS = VALID_DIMENSIONS
            self.DEFAULT_DIMENSIONS = DEFAULT_DIMENSIONS
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_has_10_dimensions(self):
        """Test that ProfileDimensions has exactly 10 fields."""
        dims = self.ProfileDimensions()
        fields = dims.model_dump()
        self.assertEqual(len(fields), 10)

    def test_default_values(self):
        """Test default dimension values."""
        dims = self.ProfileDimensions()
        self.assertEqual(dims.learning_style, "reading")
        self.assertEqual(dims.knowledge_level, "beginner")
        self.assertEqual(dims.language, "en")

    def test_all_dimension_names(self):
        """Test all 10 dimension names are present."""
        expected = [
            "learning_style", "knowledge_level", "learning_pace",
            "language", "content_depth", "example_preference",
            "difficulty_tolerance", "interaction_style",
            "time_availability", "goal_orientation"
        ]
        dims = self.ProfileDimensions()
        fields = list(dims.model_dump().keys())
        self.assertEqual(sorted(fields), sorted(expected))

    def test_valid_dimensions_dict(self):
        """Test VALID_DIMENSIONS has entries for all 10 dimensions."""
        self.assertEqual(len(self.VALID_DIMENSIONS), 10)

    def test_learning_style_options(self):
        """Test learning_style has correct options."""
        expected = ["visual", "auditory", "kinesthetic", "reading"]
        self.assertEqual(
            sorted(self.VALID_DIMENSIONS["learning_style"]),
            sorted(expected)
        )

    def test_language_options(self):
        """Test language has en and ur."""
        self.assertIn("en", self.VALID_DIMENSIONS["language"])
        self.assertIn("ur", self.VALID_DIMENSIONS["language"])


class TestProfile(unittest.TestCase):
    """Test Profile model."""

    def setUp(self):
        try:
            from personalizer import Profile, ProfileDimensions
            self.Profile = Profile
            self.ProfileDimensions = ProfileDimensions
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_profile_creation(self):
        """Test creating a profile."""
        profile = self.Profile(id="test123")
        self.assertEqual(profile.id, "test123")
        self.assertIsNotNone(profile.created_at)
        self.assertIsInstance(profile.dimensions, self.ProfileDimensions)

    def test_profile_with_custom_dimensions(self):
        """Test profile with custom dimensions."""
        dims = self.ProfileDimensions(
            learning_style="visual",
            knowledge_level="advanced"
        )
        profile = self.Profile(id="test", dimensions=dims)
        self.assertEqual(profile.dimensions.learning_style, "visual")
        self.assertEqual(profile.dimensions.knowledge_level, "advanced")

    def test_profile_serialization(self):
        """Test profile can be serialized to JSON."""
        profile = self.Profile(id="test")
        json_str = profile.model_dump_json()
        self.assertIn("test", json_str)
        self.assertIn("dimensions", json_str)


class TestPersonalizerMethods(unittest.TestCase):
    """Test Personalizer class methods exist."""

    def test_class_has_required_methods(self):
        """Test Personalizer has all required methods."""
        try:
            from personalizer import Personalizer
            required_methods = [
                'create_profile',
                'get_profile',
                'update_profile',
                'delete_profile',
                'list_profiles',
                'personalize',
                'personalize_with_rag'
            ]
            for method in required_methods:
                self.assertTrue(
                    hasattr(Personalizer, method),
                    f"Missing method: {method}"
                )
        except ImportError:
            self.skipTest("Dependencies not installed")


class TestProfileManagement(unittest.TestCase):
    """Test profile CRUD operations."""

    def setUp(self):
        """Set up test environment with mocked OpenAI."""
        try:
            from personalizer import Personalizer, Profile
            self.Personalizer = Personalizer
            self.Profile = Profile
        except ImportError:
            self.skipTest("Dependencies not installed")

        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        """Clean up temp directory."""
        import shutil
        if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_create_and_get_profile(self):
        """Test creating and retrieving a profile."""
        with patch.object(self.Personalizer, '__init__', lambda x, **kwargs: None):
            p = self.Personalizer()
            p.profiles_dir = Path(self.temp_dir)
            p.prompts = {}

            # Mock _profile_path
            def mock_path(profile_id):
                return Path(self.temp_dir) / f"{profile_id}.json"
            p._profile_path = mock_path

            # Create
            profile = p.create_profile("user1", {"learning_style": "visual"})
            self.assertEqual(profile.id, "user1")
            self.assertEqual(profile.dimensions.learning_style, "visual")

            # Get
            retrieved = p.get_profile("user1")
            self.assertEqual(retrieved.id, "user1")

    def test_update_profile_dimension(self):
        """Test updating a profile dimension."""
        with patch.object(self.Personalizer, '__init__', lambda x, **kwargs: None):
            p = self.Personalizer()
            p.profiles_dir = Path(self.temp_dir)
            p.prompts = {}

            def mock_path(profile_id):
                return Path(self.temp_dir) / f"{profile_id}.json"
            p._profile_path = mock_path

            # Create
            p.create_profile("user2")

            # Update
            updated = p.update_profile("user2", "knowledge_level", "advanced")
            self.assertEqual(updated.dimensions.knowledge_level, "advanced")

    def test_delete_profile(self):
        """Test deleting a profile."""
        with patch.object(self.Personalizer, '__init__', lambda x, **kwargs: None):
            p = self.Personalizer()
            p.profiles_dir = Path(self.temp_dir)
            p.prompts = {}

            def mock_path(profile_id):
                return Path(self.temp_dir) / f"{profile_id}.json"
            p._profile_path = mock_path

            # Create
            p.create_profile("user3")

            # Delete
            result = p.delete_profile("user3")
            self.assertTrue(result)

            # Verify deleted
            self.assertFalse(mock_path("user3").exists())


class TestAdaptationInstructions(unittest.TestCase):
    """Test adaptation instruction building."""

    def test_prompts_json_structure(self):
        """Test prompts.json has correct structure."""
        prompts_path = Path(__file__).parent.parent / "assets" / "prompts.json"
        if not prompts_path.exists():
            self.skipTest("prompts.json not found")

        prompts = json.loads(prompts_path.read_text())

        self.assertIn("base_system", prompts)
        self.assertIn("dimension_instructions", prompts)
        self.assertIn("learning_style", prompts["dimension_instructions"])
        self.assertIn("visual", prompts["dimension_instructions"]["learning_style"])


class TestCLI(unittest.TestCase):
    """Test CLI interface."""

    def test_main_function_exists(self):
        """Test main() function exists for CLI."""
        try:
            from personalizer import main
            self.assertTrue(callable(main))
        except ImportError:
            self.skipTest("Dependencies not installed")


if __name__ == '__main__':
    unittest.main(verbosity=2)
