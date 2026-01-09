#!/usr/bin/env python3
"""
Unit tests for Translation Sync module.
Tests core functionality without requiring actual OpenAI connections.
Version: 1.1.0
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


class TestTranslatorImports(unittest.TestCase):
    """Test that module can be imported and has expected structure."""

    def test_module_imports(self):
        """Test that translator module can be imported."""
        try:
            import translator
            self.assertTrue(hasattr(translator, 'TranslationSync'))
        except ImportError as e:
            self.skipTest(f"Dependencies not installed: {e}")

    def test_file_status_class_exists(self):
        """Test FileStatus class exists."""
        try:
            from translator import FileStatus
            self.assertTrue(FileStatus is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_translation_result_class_exists(self):
        """Test TranslationResult class exists."""
        try:
            from translator import TranslationResult
            self.assertTrue(TranslationResult is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_sync_status_class_exists(self):
        """Test SyncStatus class exists."""
        try:
            from translator import SyncStatus
            self.assertTrue(SyncStatus is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_diff_result_class_exists(self):
        """Test DiffResult class exists (v1.1.0)."""
        try:
            from translator import DiffResult
            self.assertTrue(DiffResult is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_tm_entry_class_exists(self):
        """Test TMEntry class exists (v1.1.0)."""
        try:
            from translator import TMEntry
            self.assertTrue(TMEntry is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_section_info_class_exists(self):
        """Test SectionInfo class exists (v1.1.0)."""
        try:
            from translator import SectionInfo
            self.assertTrue(SectionInfo is not None)
        except ImportError:
            self.skipTest("Dependencies not installed")


class TestFileStatus(unittest.TestCase):
    """Test FileStatus data class."""

    def setUp(self):
        try:
            from translator import FileStatus
            self.FileStatus = FileStatus
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_file_status_creation(self):
        """Test creating a FileStatus."""
        fs = self.FileStatus(
            source_path="docs/intro.md",
            source_hash="abc123"
        )
        self.assertEqual(fs.source_path, "docs/intro.md")
        self.assertEqual(fs.source_hash, "abc123")
        self.assertEqual(fs.status, "pending")

    def test_file_status_synced(self):
        """Test FileStatus with synced status."""
        fs = self.FileStatus(
            source_path="docs/intro.md",
            source_hash="abc123",
            translated_hash="def456",
            status="synced"
        )
        self.assertEqual(fs.status, "synced")

    def test_file_status_with_sections(self):
        """Test FileStatus with sections (v1.1.0)."""
        fs = self.FileStatus(
            source_path="docs/intro.md",
            source_hash="abc123",
            sections={"# Intro": {"hash": "xyz", "translated": "..."}}
        )
        self.assertIn("# Intro", fs.sections)


class TestTranslationResult(unittest.TestCase):
    """Test TranslationResult data class."""

    def setUp(self):
        try:
            from translator import TranslationResult
            self.TranslationResult = TranslationResult
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_translation_result_success(self):
        """Test successful translation result."""
        result = self.TranslationResult(
            source_path="docs/intro.md",
            target_path="i18n/ur/docs/intro.md",
            original_content="Hello",
            translated_content="ہیلو",
            success=True
        )
        self.assertTrue(result.success)
        self.assertIsNone(result.error)

    def test_translation_result_failure(self):
        """Test failed translation result."""
        result = self.TranslationResult(
            source_path="docs/intro.md",
            target_path="",
            original_content="Hello",
            translated_content="",
            success=False,
            error="File not found"
        )
        self.assertFalse(result.success)
        self.assertEqual(result.error, "File not found")

    def test_translation_result_incremental_fields(self):
        """Test TranslationResult with incremental fields (v1.1.0)."""
        result = self.TranslationResult(
            source_path="docs/intro.md",
            target_path="i18n/ur/docs/intro.md",
            original_content="Hello",
            translated_content="ہیلو",
            success=True,
            sections_translated=2,
            sections_cached=5,
            tm_hits=1
        )
        self.assertEqual(result.sections_translated, 2)
        self.assertEqual(result.sections_cached, 5)
        self.assertEqual(result.tm_hits, 1)


class TestDiffResult(unittest.TestCase):
    """Test DiffResult data class (v1.1.0)."""

    def setUp(self):
        try:
            from translator import DiffResult
            self.DiffResult = DiffResult
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_diff_result_no_changes(self):
        """Test DiffResult with no changes."""
        result = self.DiffResult(
            file_path="docs/intro.md",
            has_changes=False
        )
        self.assertFalse(result.has_changes)

    def test_diff_result_with_changes(self):
        """Test DiffResult with changes."""
        result = self.DiffResult(
            file_path="docs/intro.md",
            has_changes=True,
            changed_sections=["## Introduction", "## Getting Started"]
        )
        self.assertTrue(result.has_changes)
        self.assertEqual(len(result.changed_sections), 2)


class TestTMEntry(unittest.TestCase):
    """Test TMEntry data class (v1.1.0)."""

    def setUp(self):
        try:
            from translator import TMEntry
            self.TMEntry = TMEntry
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_tm_entry_creation(self):
        """Test creating a TMEntry."""
        entry = self.TMEntry(
            source="Hello World",
            target="ہیلو ورلڈ",
            count=5
        )
        self.assertEqual(entry.source, "Hello World")
        self.assertEqual(entry.target, "ہیلو ورلڈ")
        self.assertEqual(entry.count, 5)


class TestTranslationSyncMethods(unittest.TestCase):
    """Test TranslationSync class methods exist."""

    def test_class_has_required_methods(self):
        """Test TranslationSync has all required methods."""
        try:
            from translator import TranslationSync
            required_methods = [
                'get_status',
                'translate_file',
                'sync_all',
                'validate_translations',
                'add_term',
                'list_terms',
                # v1.1.0 methods
                'show_diff',
                'translate_file_incremental',
                'get_tm_stats',
                'export_tm',
                'import_tm',
                'clear_tm'
            ]
            for method in required_methods:
                self.assertTrue(
                    hasattr(TranslationSync, method),
                    f"Missing method: {method}"
                )
        except ImportError:
            self.skipTest("Dependencies not installed")


class TestFrontmatterExtraction(unittest.TestCase):
    """Test frontmatter extraction and rebuilding."""

    def setUp(self):
        try:
            from translator import TranslationSync
            self.TranslationSync = TranslationSync
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_extract_frontmatter(self):
        """Test extracting frontmatter from content."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.config = {"frontmatter_keys_to_translate": ["title"]}

            content = """---
title: Introduction
description: Getting started
---

# Hello World
"""
            frontmatter, body = sync._extract_frontmatter(content)

            self.assertEqual(frontmatter.get("title"), "Introduction")
            self.assertEqual(frontmatter.get("description"), "Getting started")
            self.assertIn("# Hello World", body)

    def test_extract_no_frontmatter(self):
        """Test extracting from content without frontmatter."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.config = {}

            content = "# Hello World\n\nSome content"
            frontmatter, body = sync._extract_frontmatter(content)

            self.assertEqual(frontmatter, {})
            self.assertEqual(body, content)


class TestCodeBlockPreservation(unittest.TestCase):
    """Test code block preservation."""

    def setUp(self):
        try:
            from translator import TranslationSync
            self.TranslationSync = TranslationSync
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_preserve_fenced_code_blocks(self):
        """Test preserving fenced code blocks."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()

            content = """Some text

```python
import rclpy
```

More text"""

            preserved, placeholders = sync._preserve_code_blocks(content)

            self.assertIn("__CODE_BLOCK_0__", preserved)
            self.assertIn("```python", placeholders.get("__CODE_BLOCK_0__", ""))

    def test_preserve_inline_code(self):
        """Test preserving inline code."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()

            content = "Run the `ros2 run` command"

            preserved, placeholders = sync._preserve_code_blocks(content)

            self.assertIn("__CODE_BLOCK_0__", preserved)
            self.assertEqual(placeholders.get("__CODE_BLOCK_0__"), "`ros2 run`")

    def test_restore_code_blocks(self):
        """Test restoring code blocks from placeholders."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()

            content = "Text __CODE_BLOCK_0__ more text"
            placeholders = {"__CODE_BLOCK_0__": "`code`"}

            restored = sync._restore_code_blocks(content, placeholders)

            self.assertEqual(restored, "Text `code` more text")


class TestSectionSplitting(unittest.TestCase):
    """Test section splitting (v1.1.0)."""

    def setUp(self):
        try:
            from translator import TranslationSync
            self.TranslationSync = TranslationSync
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_split_into_sections(self):
        """Test splitting content into sections."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()

            content = """# Introduction

Some intro text.

## Getting Started

Getting started content.

## Advanced Topics

Advanced content here.
"""
            sections = sync._split_into_sections(content)

            self.assertGreaterEqual(len(sections), 3)
            headers = [s.header for s in sections]
            self.assertIn("# Introduction", headers)
            self.assertIn("## Getting Started", headers)
            self.assertIn("## Advanced Topics", headers)


class TestGlossaryManagement(unittest.TestCase):
    """Test glossary management."""

    def setUp(self):
        try:
            from translator import TranslationSync
            self.TranslationSync = TranslationSync
        except ImportError:
            self.skipTest("Dependencies not installed")

        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_add_term(self):
        """Test adding a term to glossary."""
        glossary_path = Path(self.temp_dir) / "glossary.json"
        glossary_path.write_text('{"meta": {"version": "1.0.0"}, "terms": {}}')

        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.glossary_path = glossary_path
            sync.glossary = {}

            result = sync.add_term("ROS 2", "آر او ایس ٹو", keep_english=True)

            self.assertTrue(result)
            self.assertIn("ROS 2", sync.glossary)

    def test_list_terms(self):
        """Test listing glossary terms."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.glossary = {
                "ROS 2": {"urdu": "آر او ایس ٹو", "keep_english": True},
                "robot": {"urdu": "روبوٹ", "keep_english": False}
            }

            terms = sync.list_terms()

            self.assertEqual(len(terms), 2)
            self.assertIn("ROS 2", terms)
            self.assertIn("robot", terms)


class TestTranslationMemory(unittest.TestCase):
    """Test translation memory (v1.1.0)."""

    def setUp(self):
        try:
            from translator import TranslationSync, TMEntry
            self.TranslationSync = TranslationSync
            self.TMEntry = TMEntry
        except ImportError:
            self.skipTest("Dependencies not installed")

        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        import shutil
        if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_store_and_lookup_tm(self):
        """Test storing and looking up in translation memory."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.tm = {}

            # Store
            sync._store_tm("Hello World", "ہیلو ورلڈ")

            # Lookup
            result = sync._lookup_tm("Hello World")

            self.assertEqual(result, "ہیلو ورلڈ")

    def test_tm_stats(self):
        """Test getting TM statistics."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.tm = {
                "hash1": self.TMEntry(source="Hello", target="ہیلو", count=5),
                "hash2": self.TMEntry(source="World", target="ورلڈ", count=3)
            }

            stats = sync.get_tm_stats()

            self.assertEqual(stats["total_segments"], 2)
            self.assertEqual(stats["total_uses"], 8)

    def test_export_tm_json(self):
        """Test exporting TM to JSON."""
        output_path = Path(self.temp_dir) / "tm_export.json"

        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.tm = {
                "hash1": self.TMEntry(source="Hello", target="ہیلو", count=1)
            }

            result = sync.export_tm(str(output_path), format="json")

            self.assertTrue(result)
            self.assertTrue(output_path.exists())

            data = json.loads(output_path.read_text())
            self.assertEqual(len(data["segments"]), 1)

    def test_clear_tm(self):
        """Test clearing translation memory."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.tm = {"hash1": self.TMEntry(source="Hello", target="ہیلو", count=1)}
            sync.tm_file = Path(self.temp_dir) / "tm.json"

            count = sync.clear_tm()

            self.assertEqual(count, 1)
            self.assertEqual(len(sync.tm), 0)


class TestHashContent(unittest.TestCase):
    """Test content hashing."""

    def setUp(self):
        try:
            from translator import TranslationSync
            self.TranslationSync = TranslationSync
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_hash_content(self):
        """Test hashing content."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()

            hash1 = sync._hash_content("Hello World")
            hash2 = sync._hash_content("Hello World")
            hash3 = sync._hash_content("Different content")

            self.assertEqual(hash1, hash2)
            self.assertNotEqual(hash1, hash3)
            self.assertEqual(len(hash1), 12)  # Truncated hash

    def test_hash_segment(self):
        """Test hashing segment for TM (v1.1.0)."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()

            # Same content with different whitespace should hash the same
            hash1 = sync._hash_segment("Hello World")
            hash2 = sync._hash_segment("Hello  World")  # Extra space
            hash3 = sync._hash_segment("Hello\nWorld")  # Newline

            self.assertEqual(hash1, hash2)
            self.assertEqual(hash1, hash3)


class TestGlossaryPromptBuilding(unittest.TestCase):
    """Test glossary prompt building."""

    def setUp(self):
        try:
            from translator import TranslationSync
            self.TranslationSync = TranslationSync
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_build_glossary_prompt_empty(self):
        """Test building prompt with empty glossary."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.glossary = {}

            prompt = sync._build_glossary_prompt()

            self.assertEqual(prompt, "")

    def test_build_glossary_prompt_with_terms(self):
        """Test building prompt with glossary terms."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.glossary = {
                "ROS 2": {"urdu": "آر او ایس ٹو", "keep_english": True},
                "robot": {"urdu": "روبوٹ", "keep_english": False}
            }

            prompt = sync._build_glossary_prompt()

            self.assertIn("ROS 2", prompt)
            self.assertIn("robot", prompt)
            self.assertIn("Keep as", prompt)
            self.assertIn("Translate to", prompt)


class TestRTLWrapper(unittest.TestCase):
    """Test RTL wrapper application."""

    def setUp(self):
        try:
            from translator import TranslationSync
            self.TranslationSync = TranslationSync
        except ImportError:
            self.skipTest("Dependencies not installed")

    def test_apply_rtl_wrapper(self):
        """Test applying RTL wrapper."""
        with patch.object(self.TranslationSync, '__init__', lambda x, **kwargs: None):
            sync = self.TranslationSync()
            sync.config = {"rtl_wrapper": '<div dir="rtl">\n\n{content}\n\n</div>'}

            content = "ہیلو ورلڈ"
            wrapped = sync._apply_rtl_wrapper(content)

            self.assertIn('dir="rtl"', wrapped)
            self.assertIn(content, wrapped)


class TestCLI(unittest.TestCase):
    """Test CLI interface."""

    def test_main_function_exists(self):
        """Test main() function exists for CLI."""
        try:
            from translator import main
            self.assertTrue(callable(main))
        except ImportError:
            self.skipTest("Dependencies not installed")


if __name__ == '__main__':
    unittest.main(verbosity=2)
