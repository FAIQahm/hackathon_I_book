#!/usr/bin/env python3
"""
Translation Sync Module
Synchronize English MDX files with Urdu translations for Docusaurus i18n.
Version: 1.1.0
Agent: Linguist
"""

import os
import re
import json
import hashlib
import argparse
import difflib
from pathlib import Path
from datetime import datetime, timezone
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Any, Tuple

try:
    from openai import OpenAI
    from rich.console import Console
    from rich.table import Table
    from rich.progress import Progress, SpinnerColumn, TextColumn
    from rich.syntax import Syntax
    from rich.panel import Panel
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    Console = None


# =============================================================================
# Constants
# =============================================================================

SCRIPT_DIR = Path(__file__).parent
SKILL_DIR = SCRIPT_DIR.parent
ASSETS_DIR = SKILL_DIR / "assets"

DEFAULT_SOURCE_DIR = "docs"
DEFAULT_TARGET_DIR = "i18n/ur/docusaurus-plugin-content-docs/current"
DEFAULT_GLOSSARY_PATH = ASSETS_DIR / "glossary.json"
DEFAULT_CONFIG_PATH = ASSETS_DIR / "translation_config.json"
DEFAULT_STATUS_FILE = ".translation-status.json"
DEFAULT_TM_FILE = ".translation-memory.json"
DEFAULT_MODEL = "gpt-4o-mini"


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class SectionInfo:
    """Information about a document section."""
    header: str
    content: str
    hash: str
    start_line: int
    end_line: int


@dataclass
class FileStatus:
    """Status of a single file's translation."""
    source_path: str
    source_hash: str
    translated_hash: Optional[str] = None
    last_synced: Optional[str] = None
    status: str = "pending"  # pending, synced, outdated
    sections: Dict[str, Dict[str, str]] = field(default_factory=dict)


@dataclass
class TranslationResult:
    """Result of a translation operation."""
    source_path: str
    target_path: str
    original_content: str
    translated_content: str
    success: bool
    error: Optional[str] = None
    terms_preserved: List[str] = field(default_factory=list)
    sections_translated: int = 0
    sections_cached: int = 0
    tm_hits: int = 0


@dataclass
class SyncStatus:
    """Overall sync status."""
    total: int = 0
    synced: int = 0
    pending: int = 0
    outdated: int = 0
    files: Dict[str, FileStatus] = field(default_factory=dict)


@dataclass
class DiffResult:
    """Result of comparing source changes."""
    file_path: str
    has_changes: bool
    added_lines: List[str] = field(default_factory=list)
    removed_lines: List[str] = field(default_factory=list)
    changed_sections: List[str] = field(default_factory=list)
    unified_diff: str = ""


@dataclass
class TMEntry:
    """Translation memory entry."""
    source: str
    target: str
    count: int = 1
    last_used: str = ""
    context: str = ""


# =============================================================================
# Translation Sync Class
# =============================================================================

class TranslationSync:
    """
    Synchronize English MDX files with Urdu translations.

    Features:
    - File synchronization tracking
    - Technical terms glossary preservation
    - Code block and frontmatter preservation
    - RTL wrapper injection
    - OpenAI-powered translation
    - Section-level incremental translation (v1.1.0)
    - Translation memory cache (v1.1.0)
    - Diff mode for change preview (v1.1.0)
    """

    def __init__(
        self,
        openai_api_key: Optional[str] = None,
        source_dir: str = DEFAULT_SOURCE_DIR,
        target_dir: str = DEFAULT_TARGET_DIR,
        glossary_path: Optional[Path] = None,
        config_path: Optional[Path] = None,
        project_root: Optional[Path] = None
    ):
        """Initialize the translation sync manager."""
        self.openai_api_key = openai_api_key or os.getenv("OPENAI_API_KEY")
        self.project_root = project_root or self._find_project_root()
        self.source_dir = self.project_root / source_dir
        self.target_dir = self.project_root / target_dir
        self.glossary_path = glossary_path or DEFAULT_GLOSSARY_PATH
        self.config_path = config_path or DEFAULT_CONFIG_PATH
        self.status_file = self.project_root / DEFAULT_STATUS_FILE
        self.tm_file = self.project_root / DEFAULT_TM_FILE

        # Load configuration
        self.config = self._load_config()
        self.glossary = self._load_glossary()
        self.status = self._load_status()
        self.tm = self._load_tm()

        # Initialize OpenAI client
        self.client = None
        if self.openai_api_key:
            self.client = OpenAI(api_key=self.openai_api_key)

        # Rich console for output
        self.console = Console() if RICH_AVAILABLE else None

    def _find_project_root(self) -> Path:
        """Find project root by looking for common markers."""
        current = Path.cwd()
        markers = ["package.json", "docusaurus.config.js", ".git", "CLAUDE.md"]

        for _ in range(10):  # Max 10 levels up
            for marker in markers:
                if (current / marker).exists():
                    return current
            if current.parent == current:
                break
            current = current.parent

        return Path.cwd()

    def _load_config(self) -> Dict[str, Any]:
        """Load translation configuration."""
        if self.config_path.exists():
            return json.loads(self.config_path.read_text())
        return {
            "preserve_patterns": [],
            "frontmatter_keys_to_translate": ["title", "description"],
            "rtl_wrapper": '<div dir="rtl">\n\n{content}\n\n</div>',
            "file_extensions": [".md", ".mdx"]
        }

    def _load_glossary(self) -> Dict[str, Dict[str, Any]]:
        """Load technical terms glossary."""
        if self.glossary_path.exists():
            data = json.loads(self.glossary_path.read_text())
            return data.get("terms", {})
        return {}

    def _load_status(self) -> SyncStatus:
        """Load sync status from file."""
        if self.status_file.exists():
            data = json.loads(self.status_file.read_text())
            status = SyncStatus()
            for path, file_data in data.get("files", {}).items():
                sections = file_data.pop("sections", {})
                status.files[path] = FileStatus(**file_data, sections=sections)
            return status
        return SyncStatus()

    def _save_status(self):
        """Save sync status to file."""
        data = {
            "files": {
                path: {
                    "source_path": fs.source_path,
                    "source_hash": fs.source_hash,
                    "translated_hash": fs.translated_hash,
                    "last_synced": fs.last_synced,
                    "status": fs.status,
                    "sections": fs.sections
                }
                for path, fs in self.status.files.items()
            },
            "last_updated": datetime.now(timezone.utc).isoformat()
        }
        self.status_file.write_text(json.dumps(data, indent=2, ensure_ascii=False))

    def _load_tm(self) -> Dict[str, TMEntry]:
        """Load translation memory from file."""
        if self.tm_file.exists():
            data = json.loads(self.tm_file.read_text())
            tm = {}
            for hash_key, entry_data in data.get("segments", {}).items():
                tm[hash_key] = TMEntry(**entry_data)
            return tm
        return {}

    def _save_tm(self):
        """Save translation memory to file."""
        data = {
            "meta": {
                "version": "1.0.0",
                "total_segments": len(self.tm),
                "last_updated": datetime.now(timezone.utc).isoformat()
            },
            "segments": {
                hash_key: {
                    "source": entry.source,
                    "target": entry.target,
                    "count": entry.count,
                    "last_used": entry.last_used,
                    "context": entry.context
                }
                for hash_key, entry in self.tm.items()
            }
        }
        self.tm_file.write_text(json.dumps(data, indent=2, ensure_ascii=False))

    def _hash_content(self, content: str) -> str:
        """Generate hash for content."""
        return hashlib.md5(content.encode()).hexdigest()[:12]

    def _hash_segment(self, text: str) -> str:
        """Generate hash for a text segment (for TM lookup)."""
        # Normalize whitespace for consistent hashing
        normalized = ' '.join(text.split())
        return hashlib.md5(normalized.encode()).hexdigest()

    def _get_source_files(self) -> List[Path]:
        """Get all source files to translate."""
        extensions = self.config.get("file_extensions", [".md", ".mdx"])
        skip_dirs = self.config.get("skip_directories", [])
        files = []

        if not self.source_dir.exists():
            return files

        for ext in extensions:
            for file in self.source_dir.rglob(f"*{ext}"):
                # Skip files in excluded directories
                if any(skip in file.parts for skip in skip_dirs):
                    continue
                files.append(file)

        return sorted(files)

    def _get_target_path(self, source_path: Path) -> Path:
        """Get target path for a source file."""
        relative = source_path.relative_to(self.source_dir)
        return self.target_dir / relative

    def _extract_frontmatter(self, content: str) -> Tuple[Dict[str, str], str]:
        """Extract YAML frontmatter from content."""
        frontmatter = {}
        body = content

        match = re.match(r'^---\s*\n(.*?)\n---\s*\n(.*)$', content, re.DOTALL)
        if match:
            fm_text = match.group(1)
            body = match.group(2)

            # Simple YAML parsing for key: value pairs
            for line in fm_text.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip().strip('"\'')

        return frontmatter, body

    def _rebuild_frontmatter(self, frontmatter: Dict[str, str], body: str) -> str:
        """Rebuild content with frontmatter."""
        if not frontmatter:
            return body

        fm_lines = ["---"]
        for key, value in frontmatter.items():
            # Quote values with special characters
            if any(c in str(value) for c in [':', '#', '"', "'"]):
                value = f'"{value}"'
            fm_lines.append(f"{key}: {value}")
        fm_lines.append("---")
        fm_lines.append("")

        return '\n'.join(fm_lines) + body

    def _split_into_sections(self, content: str) -> List[SectionInfo]:
        """Split content into sections based on headers."""
        sections = []
        lines = content.split('\n')

        current_header = "__intro__"
        current_content = []
        current_start = 0

        for i, line in enumerate(lines):
            # Check for markdown headers (## or ###)
            if re.match(r'^#{1,3}\s+', line):
                # Save previous section
                if current_content or current_header == "__intro__":
                    section_content = '\n'.join(current_content)
                    sections.append(SectionInfo(
                        header=current_header,
                        content=section_content,
                        hash=self._hash_content(section_content),
                        start_line=current_start,
                        end_line=i - 1
                    ))

                current_header = line.strip()
                current_content = [line]
                current_start = i
            else:
                current_content.append(line)

        # Don't forget the last section
        if current_content:
            section_content = '\n'.join(current_content)
            sections.append(SectionInfo(
                header=current_header,
                content=section_content,
                hash=self._hash_content(section_content),
                start_line=current_start,
                end_line=len(lines) - 1
            ))

        return sections

    def _preserve_code_blocks(self, content: str) -> Tuple[str, Dict[str, str]]:
        """Replace code blocks with placeholders."""
        placeholders = {}
        counter = [0]

        def replace(match):
            key = f"__CODE_BLOCK_{counter[0]}__"
            placeholders[key] = match.group(0)
            counter[0] += 1
            return key

        # Replace fenced code blocks
        content = re.sub(r'```[\s\S]*?```', replace, content)
        # Replace inline code
        content = re.sub(r'`[^`]+`', replace, content)

        return content, placeholders

    def _restore_code_blocks(self, content: str, placeholders: Dict[str, str]) -> str:
        """Restore code blocks from placeholders."""
        for key, value in placeholders.items():
            content = content.replace(key, value)
        return content

    def _build_glossary_prompt(self) -> str:
        """Build glossary instructions for the prompt."""
        if not self.glossary:
            return ""

        lines = ["\n\nTechnical Terms Glossary (preserve these terms):"]
        for term, data in self.glossary.items():
            urdu = data.get("urdu", "")
            keep_en = data.get("keep_english", False)
            if keep_en:
                lines.append(f"- {term}: Keep as '{term}' (transliterate as {urdu} if needed)")
            else:
                lines.append(f"- {term}: Translate to '{urdu}'")

        return '\n'.join(lines)

    def _lookup_tm(self, text: str) -> Optional[str]:
        """Look up text in translation memory."""
        hash_key = self._hash_segment(text)
        if hash_key in self.tm:
            entry = self.tm[hash_key]
            entry.count += 1
            entry.last_used = datetime.now(timezone.utc).isoformat()
            return entry.target
        return None

    def _store_tm(self, source: str, target: str, context: str = ""):
        """Store a translation in translation memory."""
        hash_key = self._hash_segment(source)
        self.tm[hash_key] = TMEntry(
            source=source,
            target=target,
            count=1,
            last_used=datetime.now(timezone.utc).isoformat(),
            context=context
        )

    def _translate_text(self, text: str, model: str = DEFAULT_MODEL, use_tm: bool = True) -> Tuple[str, bool]:
        """
        Translate text using OpenAI or TM cache.
        Returns (translated_text, was_from_tm).
        """
        # Try TM lookup first
        if use_tm:
            tm_result = self._lookup_tm(text)
            if tm_result:
                return tm_result, True

        if not self.client:
            raise ValueError("OpenAI client not initialized. Set OPENAI_API_KEY.")

        glossary_prompt = self._build_glossary_prompt()

        system_prompt = f"""You are a professional translator specializing in technical documentation.
Translate the following English text to Urdu.

Rules:
1. Preserve all markdown formatting (headers, lists, links, etc.)
2. Do NOT translate content inside code blocks or inline code (already removed)
3. Preserve all placeholders like __CODE_BLOCK_0__
4. Use proper Urdu technical terminology
5. Maintain the original structure and formatting
6. For technical terms, follow the glossary instructions
{glossary_prompt}

Return ONLY the translated text, no explanations."""

        response = self.client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text}
            ],
            temperature=0.3
        )

        translated = response.choices[0].message.content.strip()

        # Store in TM
        if use_tm:
            self._store_tm(text, translated)

        return translated, False

    def _apply_rtl_wrapper(self, content: str) -> str:
        """Apply RTL wrapper to content."""
        wrapper = self.config.get("rtl_wrapper", '<div dir="rtl">\n\n{content}\n\n</div>')
        return wrapper.format(content=content)

    # =========================================================================
    # Diff Mode (Enhancement 1)
    # =========================================================================

    def show_diff(self, source_path: str) -> DiffResult:
        """Show changes between current source and last translated version."""
        source = self.project_root / source_path
        if not source.exists():
            return DiffResult(
                file_path=source_path,
                has_changes=False,
                unified_diff=f"Source file not found: {source_path}"
            )

        rel_path = str(source.relative_to(self.project_root))
        current_content = source.read_text(encoding='utf-8')

        # Get previous content from status
        existing = self.status.files.get(rel_path)
        if not existing or not existing.sections:
            return DiffResult(
                file_path=source_path,
                has_changes=True,
                unified_diff="No previous translation found. File is new."
            )

        # Compare sections
        current_sections = self._split_into_sections(current_content)
        changed_sections = []
        added_lines = []
        removed_lines = []

        # Build previous content from sections
        previous_section_hashes = existing.sections

        for section in current_sections:
            section_key = section.header
            if section_key in previous_section_hashes:
                prev_hash = previous_section_hashes[section_key].get("hash", "")
                if section.hash != prev_hash:
                    changed_sections.append(section.header)
            else:
                changed_sections.append(f"{section.header} (new)")

        # Check for removed sections
        current_headers = {s.header for s in current_sections}
        for prev_header in previous_section_hashes.keys():
            if prev_header not in current_headers:
                changed_sections.append(f"{prev_header} (removed)")

        # Generate unified diff
        target_path = self._get_target_path(source)
        if target_path.exists():
            previous_source_content = ""
            # Reconstruct previous source from stored section content
            for header, data in previous_section_hashes.items():
                if "source_content" in data:
                    previous_source_content += data["source_content"] + "\n"

            if previous_source_content:
                diff = difflib.unified_diff(
                    previous_source_content.splitlines(keepends=True),
                    current_content.splitlines(keepends=True),
                    fromfile=f"a/{source_path} (previous)",
                    tofile=f"b/{source_path} (current)",
                    lineterm=""
                )
                unified_diff = ''.join(diff)
            else:
                unified_diff = "Section content not stored. Run --sync to update."
        else:
            unified_diff = "No previous translation exists."

        return DiffResult(
            file_path=source_path,
            has_changes=len(changed_sections) > 0,
            changed_sections=changed_sections,
            unified_diff=unified_diff if unified_diff else "No changes detected."
        )

    # =========================================================================
    # Section-Level Incremental Translation (Enhancement 2)
    # =========================================================================

    def translate_file_incremental(
        self,
        source_path: str,
        model: str = DEFAULT_MODEL,
        use_tm: bool = True,
        dry_run: bool = False
    ) -> TranslationResult:
        """Translate a file incrementally, only translating changed sections."""
        source = self.project_root / source_path
        if not source.exists():
            return TranslationResult(
                source_path=source_path,
                target_path="",
                original_content="",
                translated_content="",
                success=False,
                error=f"Source file not found: {source_path}"
            )

        target = self._get_target_path(source)
        original_content = source.read_text(encoding='utf-8')
        rel_path = str(source.relative_to(self.project_root))

        try:
            # Extract frontmatter
            frontmatter, body = self._extract_frontmatter(original_content)

            # Split into sections
            sections = self._split_into_sections(body)

            # Get existing section data
            existing = self.status.files.get(rel_path)
            existing_sections = existing.sections if existing else {}

            translated_sections = []
            sections_translated = 0
            sections_cached = 0
            tm_hits = 0
            new_section_data = {}

            for section in sections:
                section_key = section.header
                prev_data = existing_sections.get(section_key, {})

                # Check if section changed
                if prev_data.get("hash") == section.hash and "translated" in prev_data:
                    # Use cached translation
                    translated_sections.append(prev_data["translated"])
                    sections_cached += 1
                    new_section_data[section_key] = prev_data
                else:
                    # Translate section
                    section_content, placeholders = self._preserve_code_blocks(section.content)
                    translated, from_tm = self._translate_text(section_content, model, use_tm)
                    translated = self._restore_code_blocks(translated, placeholders)
                    translated_sections.append(translated)

                    if from_tm:
                        tm_hits += 1
                    sections_translated += 1

                    new_section_data[section_key] = {
                        "hash": section.hash,
                        "translated": translated,
                        "source_content": section.content
                    }

            # Combine translated sections
            translated_body = '\n\n'.join(translated_sections)

            # Translate frontmatter keys
            keys_to_translate = self.config.get("frontmatter_keys_to_translate", [])
            for key in keys_to_translate:
                if key in frontmatter:
                    translated_fm, _ = self._translate_text(frontmatter[key], model, use_tm)
                    frontmatter[key] = translated_fm

            # Rebuild content
            translated_content = self._rebuild_frontmatter(frontmatter, translated_body)

            # Apply RTL wrapper
            translated_content = self._apply_rtl_wrapper(translated_content)

            # Write to target (unless dry run)
            if not dry_run:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(translated_content, encoding='utf-8')

                # Update status with section data
                self.status.files[rel_path] = FileStatus(
                    source_path=rel_path,
                    source_hash=self._hash_content(original_content),
                    translated_hash=self._hash_content(translated_content),
                    last_synced=datetime.now(timezone.utc).isoformat(),
                    status="synced",
                    sections=new_section_data
                )
                self._save_status()
                self._save_tm()

            # Find preserved terms
            terms_preserved = [
                term for term in self.glossary.keys()
                if term.lower() in translated_content.lower()
            ]

            return TranslationResult(
                source_path=source_path,
                target_path=str(target.relative_to(self.project_root)),
                original_content=original_content,
                translated_content=translated_content,
                success=True,
                terms_preserved=terms_preserved,
                sections_translated=sections_translated,
                sections_cached=sections_cached,
                tm_hits=tm_hits
            )

        except Exception as e:
            return TranslationResult(
                source_path=source_path,
                target_path=str(target.relative_to(self.project_root)),
                original_content=original_content,
                translated_content="",
                success=False,
                error=str(e)
            )

    # =========================================================================
    # Translation Memory Management (Enhancement 3)
    # =========================================================================

    def get_tm_stats(self) -> Dict[str, Any]:
        """Get translation memory statistics."""
        if not self.tm:
            return {
                "total_segments": 0,
                "total_uses": 0,
                "avg_reuse": 0,
                "most_used": []
            }

        total_uses = sum(entry.count for entry in self.tm.values())
        return {
            "total_segments": len(self.tm),
            "total_uses": total_uses,
            "avg_reuse": round(total_uses / len(self.tm), 2) if self.tm else 0,
            "most_used": sorted(
                [(e.source[:50], e.count) for e in self.tm.values()],
                key=lambda x: x[1],
                reverse=True
            )[:5]
        }

    def export_tm(self, output_path: str, format: str = "json") -> bool:
        """Export translation memory to file."""
        output = Path(output_path)

        if format == "json":
            data = {
                "meta": {
                    "exported": datetime.now(timezone.utc).isoformat(),
                    "total_segments": len(self.tm)
                },
                "segments": [
                    {
                        "source": entry.source,
                        "target": entry.target,
                        "count": entry.count
                    }
                    for entry in self.tm.values()
                ]
            }
            output.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        elif format == "tmx":
            # TMX (Translation Memory eXchange) format
            tmx_lines = [
                '<?xml version="1.0" encoding="UTF-8"?>',
                '<!DOCTYPE tmx SYSTEM "tmx14.dtd">',
                '<tmx version="1.4">',
                '  <header creationtool="translation-sync" srclang="en" adminlang="en" datatype="plaintext"/>',
                '  <body>'
            ]
            for entry in self.tm.values():
                source_escaped = entry.source.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
                target_escaped = entry.target.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
                tmx_lines.extend([
                    '    <tu>',
                    f'      <tuv xml:lang="en"><seg>{source_escaped}</seg></tuv>',
                    f'      <tuv xml:lang="ur"><seg>{target_escaped}</seg></tuv>',
                    '    </tu>'
                ])
            tmx_lines.extend([
                '  </body>',
                '</tmx>'
            ])
            output.write_text('\n'.join(tmx_lines), encoding='utf-8')
        else:
            return False

        return True

    def import_tm(self, input_path: str) -> int:
        """Import translation memory from file. Returns number of segments imported."""
        input_file = Path(input_path)
        if not input_file.exists():
            return 0

        data = json.loads(input_file.read_text())
        imported = 0

        for segment in data.get("segments", []):
            source = segment.get("source", "")
            target = segment.get("target", "")
            if source and target:
                self._store_tm(source, target)
                imported += 1

        self._save_tm()
        return imported

    def clear_tm(self) -> int:
        """Clear translation memory. Returns number of segments cleared."""
        count = len(self.tm)
        self.tm = {}
        self._save_tm()
        return count

    # =========================================================================
    # Original Methods (Updated)
    # =========================================================================

    def get_status(self) -> SyncStatus:
        """Get current sync status for all files."""
        source_files = self._get_source_files()

        status = SyncStatus(total=len(source_files))

        for source_path in source_files:
            rel_path = str(source_path.relative_to(self.project_root))
            target_path = self._get_target_path(source_path)

            source_content = source_path.read_text(encoding='utf-8')
            source_hash = self._hash_content(source_content)

            # Check existing status
            existing = self.status.files.get(rel_path)

            if target_path.exists():
                target_content = target_path.read_text(encoding='utf-8')
                target_hash = self._hash_content(target_content)

                if existing and existing.source_hash == source_hash:
                    file_status = "synced"
                    status.synced += 1
                else:
                    file_status = "outdated"
                    status.outdated += 1
            else:
                target_hash = None
                file_status = "pending"
                status.pending += 1

            status.files[rel_path] = FileStatus(
                source_path=rel_path,
                source_hash=source_hash,
                translated_hash=target_hash,
                last_synced=existing.last_synced if existing else None,
                status=file_status,
                sections=existing.sections if existing else {}
            )

        return status

    def translate_file(
        self,
        source_path: str,
        model: str = DEFAULT_MODEL,
        preserve_terms: bool = True,
        dry_run: bool = False,
        incremental: bool = False,
        use_tm: bool = True
    ) -> TranslationResult:
        """Translate a single file."""
        # Use incremental translation if requested
        if incremental:
            return self.translate_file_incremental(source_path, model, use_tm, dry_run)

        source = self.project_root / source_path
        if not source.exists():
            return TranslationResult(
                source_path=source_path,
                target_path="",
                original_content="",
                translated_content="",
                success=False,
                error=f"Source file not found: {source_path}"
            )

        target = self._get_target_path(source)
        original_content = source.read_text(encoding='utf-8')

        try:
            # Extract frontmatter
            frontmatter, body = self._extract_frontmatter(original_content)

            # Preserve code blocks
            body, placeholders = self._preserve_code_blocks(body)

            # Translate body
            translated_body, from_tm = self._translate_text(body, model, use_tm)
            tm_hits = 1 if from_tm else 0

            # Restore code blocks
            translated_body = self._restore_code_blocks(translated_body, placeholders)

            # Translate frontmatter keys
            keys_to_translate = self.config.get("frontmatter_keys_to_translate", [])
            for key in keys_to_translate:
                if key in frontmatter:
                    translated_fm, _ = self._translate_text(frontmatter[key], model, use_tm)
                    frontmatter[key] = translated_fm

            # Rebuild content
            translated_content = self._rebuild_frontmatter(frontmatter, translated_body)

            # Apply RTL wrapper
            translated_content = self._apply_rtl_wrapper(translated_content)

            # Write to target (unless dry run)
            if not dry_run:
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(translated_content, encoding='utf-8')

                # Update status
                rel_path = str(source.relative_to(self.project_root))

                # Store section data for incremental updates
                sections = self._split_into_sections(body)
                section_data = {}
                for section in sections:
                    section_data[section.header] = {
                        "hash": section.hash,
                        "source_content": section.content
                    }

                self.status.files[rel_path] = FileStatus(
                    source_path=rel_path,
                    source_hash=self._hash_content(original_content),
                    translated_hash=self._hash_content(translated_content),
                    last_synced=datetime.now(timezone.utc).isoformat(),
                    status="synced",
                    sections=section_data
                )
                self._save_status()
                self._save_tm()

            # Find preserved terms
            terms_preserved = [
                term for term in self.glossary.keys()
                if term.lower() in translated_content.lower()
            ]

            return TranslationResult(
                source_path=source_path,
                target_path=str(target.relative_to(self.project_root)),
                original_content=original_content,
                translated_content=translated_content,
                success=True,
                terms_preserved=terms_preserved,
                tm_hits=tm_hits
            )

        except Exception as e:
            return TranslationResult(
                source_path=source_path,
                target_path=str(target.relative_to(self.project_root)),
                original_content=original_content,
                translated_content="",
                success=False,
                error=str(e)
            )

    def sync_all(
        self,
        model: str = DEFAULT_MODEL,
        dry_run: bool = False,
        incremental: bool = False,
        use_tm: bool = True
    ) -> List[TranslationResult]:
        """Sync all pending and outdated files."""
        status = self.get_status()
        results = []

        files_to_sync = [
            path for path, fs in status.files.items()
            if fs.status in ("pending", "outdated")
        ]

        for path in files_to_sync:
            result = self.translate_file(
                path,
                model=model,
                dry_run=dry_run,
                incremental=incremental,
                use_tm=use_tm
            )
            results.append(result)

        return results

    def validate_translations(self) -> List[Dict[str, Any]]:
        """Validate all translations for issues."""
        issues = []
        status = self.get_status()

        for path, fs in status.files.items():
            if fs.status != "synced":
                continue

            target_path = self._get_target_path(self.project_root / path)
            if not target_path.exists():
                issues.append({
                    "file": path,
                    "issue": "Translation file missing",
                    "severity": "error"
                })
                continue

            content = target_path.read_text(encoding='utf-8')

            # Check for RTL wrapper
            if 'dir="rtl"' not in content:
                issues.append({
                    "file": path,
                    "issue": "Missing RTL wrapper",
                    "severity": "warning"
                })

            # Check for unreplaced placeholders
            if re.search(r'__CODE_BLOCK_\d+__', content):
                issues.append({
                    "file": path,
                    "issue": "Unreplaced code block placeholders",
                    "severity": "error"
                })

            # Check for translated code blocks (shouldn't happen)
            code_blocks = re.findall(r'```[\s\S]*?```', content)
            for block in code_blocks:
                if re.search(r'[\u0600-\u06FF]', block):
                    issues.append({
                        "file": path,
                        "issue": "Code block appears to contain Urdu text",
                        "severity": "warning"
                    })
                    break

        return issues

    def add_term(self, term: str, urdu: str, keep_english: bool = False, context: str = "") -> bool:
        """Add a term to the glossary."""
        if not self.glossary_path.exists():
            data = {"meta": {"version": "1.0.0"}, "terms": {}}
        else:
            data = json.loads(self.glossary_path.read_text())

        data["terms"][term] = {
            "urdu": urdu,
            "keep_english": keep_english,
            "context": context
        }
        data["meta"]["last_updated"] = datetime.now(timezone.utc).strftime("%Y-%m-%d")

        self.glossary_path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        self.glossary = data["terms"]
        return True

    def list_terms(self) -> Dict[str, Dict[str, Any]]:
        """List all glossary terms."""
        return self.glossary


# =============================================================================
# CLI
# =============================================================================

def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Translation Sync v1.1.0 - Synchronize English MDX files with Urdu translations"
    )

    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # Status command
    status_parser = subparsers.add_parser("status", help="Show sync status")

    # Sync command
    sync_parser = subparsers.add_parser("sync", help="Sync all pending files")
    sync_parser.add_argument("--model", default=DEFAULT_MODEL, help="OpenAI model")
    sync_parser.add_argument("--dry-run", action="store_true", help="Preview only")
    sync_parser.add_argument("--incremental", action="store_true", help="Only translate changed sections")
    sync_parser.add_argument("--no-tm", action="store_true", help="Disable translation memory")

    # Translate command
    translate_parser = subparsers.add_parser("translate", help="Translate a file")
    translate_parser.add_argument("file", help="File to translate")
    translate_parser.add_argument("--model", default=DEFAULT_MODEL, help="OpenAI model")
    translate_parser.add_argument("--dry-run", action="store_true", help="Preview only")
    translate_parser.add_argument("--incremental", action="store_true", help="Only translate changed sections")
    translate_parser.add_argument("--no-tm", action="store_true", help="Disable translation memory")

    # Diff command (NEW)
    diff_parser = subparsers.add_parser("diff", help="Show changes since last translation")
    diff_parser.add_argument("file", help="File to diff")

    # Validate command
    validate_parser = subparsers.add_parser("validate", help="Validate translations")

    # Add-term command
    term_parser = subparsers.add_parser("add-term", help="Add glossary term")
    term_parser.add_argument("term", help="English term")
    term_parser.add_argument("--urdu", required=True, help="Urdu translation")
    term_parser.add_argument("--keep-english", action="store_true", help="Keep English")
    term_parser.add_argument("--context", default="", help="Context")

    # List-terms command
    list_parser = subparsers.add_parser("list-terms", help="List glossary terms")

    # TM commands (NEW)
    tm_stats_parser = subparsers.add_parser("tm-stats", help="Show translation memory statistics")

    tm_export_parser = subparsers.add_parser("tm-export", help="Export translation memory")
    tm_export_parser.add_argument("output", help="Output file path")
    tm_export_parser.add_argument("--format", choices=["json", "tmx"], default="json", help="Export format")

    tm_import_parser = subparsers.add_parser("tm-import", help="Import translation memory")
    tm_import_parser.add_argument("input", help="Input file path (JSON)")

    tm_clear_parser = subparsers.add_parser("tm-clear", help="Clear translation memory")

    args = parser.parse_args()

    sync = TranslationSync()
    console = Console() if RICH_AVAILABLE else None

    if args.command == "status":
        status = sync.get_status()

        if console:
            table = Table(title="Translation Status")
            table.add_column("File", style="cyan")
            table.add_column("Status", style="green")

            for path, fs in status.files.items():
                status_icon = {"synced": "[green]✓ synced", "pending": "[yellow]○ pending", "outdated": "[red]⚠ outdated"}
                table.add_row(path, status_icon.get(fs.status, fs.status))

            console.print(table)
            console.print(f"\nTotal: {status.total}, Synced: {status.synced}, Pending: {status.pending}, Outdated: {status.outdated}")
        else:
            for path, fs in status.files.items():
                print(f"{fs.status}: {path}")

    elif args.command == "sync":
        use_tm = not args.no_tm
        results = sync.sync_all(
            model=args.model,
            dry_run=args.dry_run,
            incremental=args.incremental,
            use_tm=use_tm
        )

        total_translated = 0
        total_cached = 0
        total_tm_hits = 0

        for result in results:
            if result.success:
                print(f"[OK] {result.source_path} -> {result.target_path}")
                if args.incremental:
                    print(f"     Sections: {result.sections_translated} translated, {result.sections_cached} cached, {result.tm_hits} TM hits")
                total_translated += result.sections_translated
                total_cached += result.sections_cached
                total_tm_hits += result.tm_hits
            else:
                print(f"[ERR] {result.source_path}: {result.error}")

        if args.incremental and results:
            print(f"\nSummary: {total_translated} sections translated, {total_cached} cached, {total_tm_hits} TM hits")

    elif args.command == "translate":
        use_tm = not args.no_tm
        result = sync.translate_file(
            args.file,
            model=args.model,
            dry_run=args.dry_run,
            incremental=args.incremental,
            use_tm=use_tm
        )

        if result.success:
            print(f"Translated: {result.source_path} -> {result.target_path}")
            if args.incremental:
                print(f"Sections: {result.sections_translated} translated, {result.sections_cached} cached, {result.tm_hits} TM hits")
            if args.dry_run:
                print("\n--- Preview ---")
                print(result.translated_content[:500] + "..." if len(result.translated_content) > 500 else result.translated_content)
        else:
            print(f"Error: {result.error}")

    elif args.command == "diff":
        diff_result = sync.show_diff(args.file)

        if console:
            if diff_result.has_changes:
                console.print(Panel(f"[yellow]Changes detected in {diff_result.file_path}[/yellow]"))
                if diff_result.changed_sections:
                    console.print("\n[bold]Changed sections:[/bold]")
                    for section in diff_result.changed_sections:
                        console.print(f"  • {section}")
                if diff_result.unified_diff:
                    console.print("\n[bold]Diff:[/bold]")
                    console.print(diff_result.unified_diff)
            else:
                console.print(f"[green]No changes in {diff_result.file_path}[/green]")
        else:
            print(f"File: {diff_result.file_path}")
            print(f"Has changes: {diff_result.has_changes}")
            if diff_result.changed_sections:
                print("Changed sections:")
                for section in diff_result.changed_sections:
                    print(f"  - {section}")
            if diff_result.unified_diff:
                print("\nDiff:")
                print(diff_result.unified_diff)

    elif args.command == "validate":
        issues = sync.validate_translations()

        if not issues:
            print("All translations valid!")
        else:
            for issue in issues:
                print(f"[{issue['severity'].upper()}] {issue['file']}: {issue['issue']}")

    elif args.command == "add-term":
        sync.add_term(args.term, args.urdu, args.keep_english, args.context)
        print(f"Added term: {args.term}")

    elif args.command == "list-terms":
        terms = sync.list_terms()

        if console:
            table = Table(title="Glossary Terms")
            table.add_column("Term", style="cyan")
            table.add_column("Urdu", style="green")
            table.add_column("Keep English", style="yellow")

            for term, data in terms.items():
                table.add_row(term, data.get("urdu", ""), str(data.get("keep_english", False)))

            console.print(table)
        else:
            for term, data in terms.items():
                print(f"{term}: {data.get('urdu', '')} (keep_en={data.get('keep_english', False)})")

    elif args.command == "tm-stats":
        stats = sync.get_tm_stats()

        if console:
            console.print(Panel("[bold]Translation Memory Statistics[/bold]"))
            console.print(f"Total segments: {stats['total_segments']}")
            console.print(f"Total uses: {stats['total_uses']}")
            console.print(f"Average reuse: {stats['avg_reuse']}x")
            if stats.get('most_used'):
                console.print("\n[bold]Most used segments:[/bold]")
                for text, count in stats['most_used']:
                    console.print(f"  {count}x: {text}...")
        else:
            print(f"Total segments: {stats['total_segments']}")
            print(f"Total uses: {stats['total_uses']}")
            print(f"Average reuse: {stats['avg_reuse']}x")

    elif args.command == "tm-export":
        success = sync.export_tm(args.output, args.format)
        if success:
            print(f"Exported TM to {args.output} ({args.format} format)")
        else:
            print(f"Failed to export TM")

    elif args.command == "tm-import":
        count = sync.import_tm(args.input)
        print(f"Imported {count} segments")

    elif args.command == "tm-clear":
        count = sync.clear_tm()
        print(f"Cleared {count} segments from translation memory")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
