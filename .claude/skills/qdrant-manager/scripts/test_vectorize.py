#!/usr/bin/env python3
"""
Unit tests for QdrantManager vectorize module.
Tests core functionality without requiring actual Qdrant/OpenAI connections.
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


class TestQdrantManagerImports(unittest.TestCase):
    """Test that module can be imported and has expected structure."""

    def test_module_imports(self):
        """Test that vectorize module can be imported."""
        try:
            import vectorize
            self.assertTrue(hasattr(vectorize, 'QdrantManager'))
        except ImportError as e:
            # Skip if dependencies not installed
            self.skipTest(f"Dependencies not installed: {e}")

    def test_class_has_required_methods(self):
        """Test QdrantManager has all required methods."""
        try:
            from vectorize import QdrantManager
            required_methods = [
                'init_collection',
                'delete_collection',
                'get_collection_info',
                'list_collections',
                'embed',
                'chunk_markdown',
                'extract_metadata',
                'vectorize_file',
                'vectorize_directory',
                'query'
            ]
            for method in required_methods:
                self.assertTrue(
                    hasattr(QdrantManager, method),
                    f"Missing method: {method}"
                )
        except ImportError:
            self.skipTest("Dependencies not installed")


class TestChunkMarkdown(unittest.TestCase):
    """Test markdown chunking functionality."""

    def setUp(self):
        """Set up mock QdrantManager for testing."""
        try:
            from vectorize import QdrantManager
            # Create a mock instance without real connections
            with patch.object(QdrantManager, '__init__', lambda x: None):
                self.manager = QdrantManager()
                # Set up mock encoder
                self.manager.encoder = Mock()
                self.manager.encoder.encode = Mock(return_value=[1] * 100)  # 100 tokens
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_chunk_by_h2_headers(self):
        """Test that content is split on ## headers."""
        content = """# Title

Introduction text.

## Section One

Content for section one.

## Section Two

Content for section two.
"""
        chunks = self.manager.chunk_markdown(content)
        self.assertGreaterEqual(len(chunks), 2)

        # Check sections are captured
        sections = [c['section'] for c in chunks]
        self.assertIn('Section One', sections)
        self.assertIn('Section Two', sections)

    def test_chunk_preserves_content(self):
        """Test that chunking preserves all content."""
        content = "## Test\n\nSome content here."
        chunks = self.manager.chunk_markdown(content)

        # Verify content is preserved
        all_content = ' '.join(c['content'] for c in chunks)
        self.assertIn('Some content here', all_content)


class TestExtractMetadata(unittest.TestCase):
    """Test metadata extraction from markdown files."""

    def setUp(self):
        """Set up mock QdrantManager for testing."""
        try:
            from vectorize import QdrantManager
            with patch.object(QdrantManager, '__init__', lambda x: None):
                self.manager = QdrantManager()
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_extract_language_english(self):
        """Test English language detection."""
        filepath = Path("docs/chapter-1/index.md")
        content = "# Chapter 1\n\nContent"

        metadata = self.manager.extract_metadata(filepath, content)
        self.assertEqual(metadata['language'], 'en')

    def test_extract_language_urdu(self):
        """Test Urdu language detection from path."""
        filepath = Path("i18n/ur/docusaurus-plugin-content-docs/current/intro.md")
        content = "# تعارف\n\nمواد"

        metadata = self.manager.extract_metadata(filepath, content)
        self.assertEqual(metadata['language'], 'ur')

    def test_extract_chapter_number(self):
        """Test chapter number extraction."""
        filepath = Path("docs/chapter-3/index.md")
        content = "# Chapter 3"

        metadata = self.manager.extract_metadata(filepath, content)
        self.assertEqual(metadata['chapter'], 3)

    def test_extract_title(self):
        """Test title extraction from H1."""
        filepath = Path("docs/intro.md")
        content = "# Introduction to ROS 2\n\nContent here"

        metadata = self.manager.extract_metadata(filepath, content)
        self.assertEqual(metadata['title'], 'Introduction to ROS 2')

    def test_extract_keywords(self):
        """Test keyword extraction."""
        filepath = Path("docs/chapter-1/index.md")
        content = "# ROS 2 and Gazebo\n\nLearn about ROS 2 robotics."

        metadata = self.manager.extract_metadata(filepath, content)
        self.assertIn('ros2', metadata['keywords'])
        self.assertIn('gazebo', metadata['keywords'])
        self.assertIn('robotics', metadata['keywords'])


class TestStateTracking(unittest.TestCase):
    """Test incremental update state tracking."""

    def setUp(self):
        """Set up mock QdrantManager and temp directory."""
        try:
            from vectorize import QdrantManager
            with patch.object(QdrantManager, '__init__', lambda x: None):
                self.manager = QdrantManager()
                self.manager.STATE_FILE = ".vectorize-state.json"
        except ImportError:
            self.skipTest("Dependencies not installed")

        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        """Clean up temp directory."""
        import shutil
        if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_get_file_hash(self):
        """Test file hash calculation."""
        # Create a test file
        test_file = Path(self.temp_dir) / "test.md"
        test_file.write_text("# Test Content")

        hash1 = self.manager._get_file_hash(test_file)
        self.assertIsInstance(hash1, str)
        self.assertEqual(len(hash1), 32)  # MD5 hex length

        # Same content = same hash
        hash2 = self.manager._get_file_hash(test_file)
        self.assertEqual(hash1, hash2)

        # Different content = different hash
        test_file.write_text("# Different Content")
        hash3 = self.manager._get_file_hash(test_file)
        self.assertNotEqual(hash1, hash3)

    def test_load_save_state(self):
        """Test state file load/save."""
        state_path = Path(self.temp_dir) / ".vectorize-state.json"

        # Initially empty
        state = self.manager._load_state(state_path)
        self.assertEqual(state['files'], {})

        # Save state
        state['files']['test.md'] = 'abc123'
        self.manager._save_state(state_path, state)

        # Reload and verify
        loaded = self.manager._load_state(state_path)
        self.assertEqual(loaded['files']['test.md'], 'abc123')
        self.assertIn('last_run', loaded)


class TestQueryFilters(unittest.TestCase):
    """Test query filtering functionality."""

    def test_query_method_signature(self):
        """Test query method accepts filter parameters."""
        try:
            from vectorize import QdrantManager
            import inspect

            sig = inspect.signature(QdrantManager.query)
            params = list(sig.parameters.keys())

            self.assertIn('language', params)
            self.assertIn('chapter', params)
            self.assertIn('top_k', params)
        except ImportError:
            self.skipTest("Dependencies not installed")


class TestCLI(unittest.TestCase):
    """Test CLI interface."""

    def test_main_function_exists(self):
        """Test main() function exists for CLI."""
        try:
            from vectorize import main
            self.assertTrue(callable(main))
        except ImportError:
            self.skipTest("Dependencies not installed")


if __name__ == '__main__':
    # Run with verbosity
    unittest.main(verbosity=2)
