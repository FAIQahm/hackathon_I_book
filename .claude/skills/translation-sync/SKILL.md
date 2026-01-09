---
name: translation-sync
description: |
  Synchronize English MDX files with the i18n/ur folder, ensuring technical terms are handled correctly during the Urdu translation process.
  Agent: Linguist
version: 1.1.0
inputs:
  openai_api_key:
    description: OpenAI API key for translation
    required: true
    example: "sk-..."
  source_dir:
    description: Source directory with English MDX files
    required: false
    default: "docs"
    example: "docs"
  target_dir:
    description: Target directory for Urdu translations
    required: false
    default: "i18n/ur/docusaurus-plugin-content-docs/current"
    example: "i18n/ur/docusaurus-plugin-content-docs/current"
  glossary_path:
    description: Path to technical terms glossary
    required: false
    default: ".claude/skills/translation-sync/assets/glossary.json"
---

# Translation Sync

**Agent:** Linguist

Synchronize English MDX files with the i18n/ur folder for Docusaurus, ensuring technical terms are handled correctly during the Urdu translation process. This skill maintains translation consistency, tracks sync status, and preserves code blocks and technical terminology.

## Quick Setup

```bash
# Set environment variables
export OPENAI_API_KEY="sk-..."

# Check sync status
.claude/skills/translation-sync/scripts/setup.sh --status

# Sync all files (detect changes)
.claude/skills/translation-sync/scripts/setup.sh --sync

# Sync with incremental mode (only changed sections) - v1.1.0
.claude/skills/translation-sync/scripts/setup.sh --sync --incremental

# Show changes before re-translating - v1.1.0
.claude/skills/translation-sync/scripts/setup.sh --diff docs/chapter-1/index.md

# Translate a specific file
.claude/skills/translation-sync/scripts/setup.sh --translate docs/intro.md

# Translate incrementally - v1.1.0
.claude/skills/translation-sync/scripts/setup.sh --translate docs/chapter-1/index.md --incremental

# Add term to glossary
.claude/skills/translation-sync/scripts/setup.sh --add-term "ROS 2" --urdu "آر او ایس ٹو"

# Translation Memory commands - v1.1.0
.claude/skills/translation-sync/scripts/setup.sh --tm-stats
.claude/skills/translation-sync/scripts/setup.sh --tm-export translations.tmx --format tmx

# Validate translations
.claude/skills/translation-sync/scripts/setup.sh --validate
```

## Command Options

| Option | Description | Default |
|--------|-------------|---------|
| `--status` | Show sync status for all files | - |
| `--sync` | Sync all changed files | - |
| `--translate FILE` | Translate a specific file | - |
| `--diff FILE` | Show changes since last translation (v1.1.0) | - |
| `--validate` | Validate all translations | - |
| `--add-term TERM` | Add term to glossary | - |
| `--urdu TEXT` | Urdu translation/transliteration for term | - |
| `--list-terms` | List all glossary terms | - |
| `--incremental` | Only translate changed sections (v1.1.0) | `false` |
| `--no-tm` | Disable translation memory (v1.1.0) | `false` |
| `--preserve-terms` | Keep technical terms in English | `true` |
| `--source DIR` | Source directory | `docs` |
| `--target DIR` | Target directory | `i18n/ur/.../current` |
| `--model MODEL` | OpenAI model | `gpt-4o-mini` |
| `--dry-run` | Show what would be translated | `false` |
| `-h, --help` | Show help message | - |

### Translation Memory Commands (v1.1.0)

| Option | Description |
|--------|-------------|
| `--tm-stats` | Show translation memory statistics |
| `--tm-export FILE` | Export TM to file (JSON or TMX format) |
| `--tm-import FILE` | Import TM from JSON file |
| `--tm-clear` | Clear all translation memory |
| `--format FORMAT` | Export format: `json` or `tmx` |

## What It Does

### 1. File Synchronization

Tracks English source files and their Urdu translations:

```
docs/                          i18n/ur/.../current/
├── intro.md          ──►      ├── intro.md (translated)
├── chapter-1/                 ├── chapter-1/
│   └── index.md      ──►      │   └── index.md (translated)
└── chapter-2/                 └── chapter-2/
    └── index.md      ──►          └── index.md (needs update)
```

### 2. Translation Status Tracking

Maintains `.translation-status.json` to track:

```json
{
  "files": {
    "docs/intro.md": {
      "source_hash": "abc123",
      "translated_hash": "def456",
      "last_synced": "2026-01-02T12:00:00Z",
      "status": "synced",
      "sections": {
        "# Introduction": {"hash": "xyz789", "translated": "..."}
      }
    }
  }
}
```

### 3. Diff Mode (v1.1.0)

Preview changes before re-translating:

```bash
.claude/skills/translation-sync/scripts/setup.sh --diff docs/chapter-1/index.md
```

Output:
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Changes detected in docs/chapter-1/index.md

Changed sections:
  • ## Getting Started (modified)
  • ## Advanced Topics (new)

Diff:
--- a/docs/chapter-1/index.md (previous)
+++ b/docs/chapter-1/index.md (current)
@@ -15,6 +15,10 @@
 This is the original content.
+
+## Advanced Topics
+
+New content added here.
```

### 4. Section-Level Incremental Translation (v1.1.0)

Only translate sections that changed, preserving cached translations for unchanged sections:

```bash
.claude/skills/translation-sync/scripts/setup.sh --translate docs/chapter-1/index.md --incremental
```

Output:
```
Translated: docs/chapter-1/index.md -> i18n/ur/.../chapter-1/index.md
Sections: 2 translated, 5 cached, 1 TM hits
```

**Benefits:**
- 60-80% reduction in API costs for updates
- Faster translation times
- Preserves human edits to unchanged sections

### 5. Translation Memory (v1.1.0)

Stores and reuses previously translated segments:

```bash
# View TM statistics
.claude/skills/translation-sync/scripts/setup.sh --tm-stats

# Output:
# Total segments: 145
# Total uses: 312
# Average reuse: 2.15x
#
# Most used segments:
#   15x: ROS 2 is a robotics middleware that provides...
#   12x: This chapter covers the basics of...
#   8x: For more information, see the official...
```

**TM Storage (`.translation-memory.json`):**
```json
{
  "meta": {
    "version": "1.0.0",
    "total_segments": 145,
    "last_updated": "2026-01-02T21:00:00Z"
  },
  "segments": {
    "abc123hash": {
      "source": "ROS 2 is a robotics middleware.",
      "target": "ROS 2 ایک روبوٹکس مڈل ویئر ہے۔",
      "count": 15,
      "last_used": "2026-01-02T21:00:00Z"
    }
  }
}
```

**Export to TMX (industry standard):**
```bash
.claude/skills/translation-sync/scripts/setup.sh --tm-export translations.tmx --format tmx
```

### 6. Technical Terms Glossary

Maintains consistent translation of technical terms:

```json
{
  "ROS 2": {
    "urdu": "آر او ایس ٹو",
    "keep_english": true,
    "context": "Robot Operating System 2"
  },
  "Gazebo": {
    "urdu": "گیزیبو",
    "keep_english": true,
    "context": "Simulation software"
  },
  "node": {
    "urdu": "نوڈ",
    "keep_english": false,
    "context": "ROS node"
  }
}
```

### 7. Smart Translation

Preserves code blocks, frontmatter, and technical elements:

**Before (English):**
```markdown
# Introduction to ROS 2

ROS 2 is a robotics middleware.

\`\`\`python
import rclpy
\`\`\`
```

**After (Urdu):**
```markdown
# آر او ایس ٹو کا تعارف

ROS 2 ایک روبوٹکس مڈل ویئر ہے۔

\`\`\`python
import rclpy
\`\`\`
```

### 8. RTL Formatting

Automatically applies RTL markers for proper Urdu display:

```markdown
<div dir="rtl">

# اردو عنوان

یہ اردو متن ہے۔

</div>
```

## Bundled Resources

### 1. Python Dependencies

**File**: `requirements.txt`

```
openai>=1.0.0
python-dotenv>=1.0.0
rich>=13.0.0
pyyaml>=6.0.0
```

### 2. Technical Terms Glossary

**File**: `assets/glossary.json`

```json
{
  "meta": {
    "version": "1.0.0",
    "description": "Technical terms glossary for Physical AI textbook"
  },
  "terms": {
    "ROS 2": {"urdu": "آر او ایس ٹو", "keep_english": true},
    "Gazebo": {"urdu": "گیزیبو", "keep_english": true},
    "robot": {"urdu": "روبوٹ", "keep_english": false},
    "sensor": {"urdu": "سینسر", "keep_english": true},
    "actuator": {"urdu": "ایکچویٹر", "keep_english": true},
    "node": {"urdu": "نوڈ", "keep_english": false},
    "topic": {"urdu": "ٹاپک", "keep_english": false},
    "publisher": {"urdu": "پبلشر", "keep_english": false},
    "subscriber": {"urdu": "سبسکرائبر", "keep_english": false},
    "message": {"urdu": "پیغام", "keep_english": false},
    "service": {"urdu": "سروس", "keep_english": false},
    "action": {"urdu": "ایکشن", "keep_english": false},
    "package": {"urdu": "پیکج", "keep_english": false},
    "workspace": {"urdu": "ورک سپیس", "keep_english": false},
    "launch file": {"urdu": "لانچ فائل", "keep_english": false},
    "URDF": {"urdu": "یو آر ڈی ایف", "keep_english": true},
    "TF": {"urdu": "ٹی ایف", "keep_english": true},
    "SLAM": {"urdu": "سلیم", "keep_english": true},
    "navigation": {"urdu": "نیویگیشن", "keep_english": false},
    "localization": {"urdu": "لوکلائزیشن", "keep_english": false},
    "simulation": {"urdu": "سمولیشن", "keep_english": false},
    "physical AI": {"urdu": "فزیکل اے آئی", "keep_english": true},
    "machine learning": {"urdu": "مشین لرننگ", "keep_english": false},
    "deep learning": {"urdu": "ڈیپ لرننگ", "keep_english": false},
    "neural network": {"urdu": "نیورل نیٹ ورک", "keep_english": false},
    "API": {"urdu": "اے پی آئی", "keep_english": true},
    "SDK": {"urdu": "ایس ڈی کے", "keep_english": true},
    "CLI": {"urdu": "سی ایل آئی", "keep_english": true},
    "GUI": {"urdu": "جی یو آئی", "keep_english": true}
  }
}
```

### 3. Translation Configuration

**File**: `assets/translation_config.json`

```json
{
  "source_language": "en",
  "target_language": "ur",
  "preserve_patterns": [
    "```[\\s\\S]*?```",
    "`[^`]+`",
    "\\{[^}]+\\}",
    "\\[[^\\]]*\\]\\([^)]*\\)",
    "^---[\\s\\S]*?---",
    "<[^>]+>",
    "\\$\\$[\\s\\S]*?\\$\\$",
    "\\$[^$]+\\$"
  ],
  "rtl_wrapper": "<div dir=\"rtl\">\n\n{content}\n\n</div>",
  "frontmatter_keys_to_translate": ["title", "description", "sidebar_label"],
  "skip_directories": ["node_modules", ".git", "build", ".docusaurus"],
  "file_extensions": [".md", ".mdx"]
}
```

### 4. Translation Module

**File**: `scripts/translator.py`

```python
from translator import TranslationSync

sync = TranslationSync()

# Check status
status = sync.get_status()
print(f"Pending: {status.pending}, Synced: {status.synced}")

# Translate file (with incremental and TM)
result = sync.translate_file("docs/intro.md", incremental=True, use_tm=True)
print(f"Sections: {result.sections_translated} new, {result.sections_cached} cached")

# Show diff
diff = sync.show_diff("docs/chapter-1/index.md")
print(f"Changed sections: {diff.changed_sections}")

# Translation memory
stats = sync.get_tm_stats()
print(f"TM segments: {stats['total_segments']}, reuse: {stats['avg_reuse']}x")

# Export TM to TMX
sync.export_tm("translations.tmx", format="tmx")
```

### 5. Test Suite

**File**: `scripts/test.sh` - Bash test runner (16+ tests)
**File**: `scripts/test_translator.py` - Python unit tests (21+ tests)

```bash
# Run tests
.claude/skills/translation-sync/scripts/test.sh
```

## Usage Instructions

### Step 1: Set Environment Variables

```bash
# Add to .env file
OPENAI_API_KEY=sk-...
```

### Step 2: Check Sync Status

```bash
# See which files need translation
.claude/skills/translation-sync/scripts/setup.sh --status
```

Output:
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📊 Translation Status
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✓ docs/intro.md                    [synced]
✓ docs/chapter-1/index.md          [synced]
○ docs/chapter-2/index.md          [pending]
⚠ docs/chapter-3/index.md          [outdated]

Summary: 2 synced, 1 pending, 1 outdated
```

### Step 3: Preview Changes (v1.1.0)

```bash
# See what changed before re-translating
.claude/skills/translation-sync/scripts/setup.sh --diff docs/chapter-3/index.md
```

### Step 4: Translate Files

```bash
# Translate single file (full)
.claude/skills/translation-sync/scripts/setup.sh --translate docs/chapter-2/index.md

# Translate incrementally (only changed sections)
.claude/skills/translation-sync/scripts/setup.sh --translate docs/chapter-3/index.md --incremental

# Sync all pending/outdated files incrementally
.claude/skills/translation-sync/scripts/setup.sh --sync --incremental

# Dry run to preview changes
.claude/skills/translation-sync/scripts/setup.sh --sync --dry-run
```

### Step 5: Manage Glossary

```bash
# List all terms
.claude/skills/translation-sync/scripts/setup.sh --list-terms

# Add new term
.claude/skills/translation-sync/scripts/setup.sh --add-term "lidar" --urdu "لائیڈار"

# Add term that should stay in English
.claude/skills/translation-sync/scripts/setup.sh --add-term "RViz" --urdu "آر ویز" --keep-english
```

### Step 6: Manage Translation Memory (v1.1.0)

```bash
# View TM statistics
.claude/skills/translation-sync/scripts/setup.sh --tm-stats

# Export to JSON
.claude/skills/translation-sync/scripts/setup.sh --tm-export backup.json

# Export to TMX (industry standard)
.claude/skills/translation-sync/scripts/setup.sh --tm-export translations.tmx --format tmx

# Import from another project
.claude/skills/translation-sync/scripts/setup.sh --tm-import external_tm.json

# Clear TM (if needed)
.claude/skills/translation-sync/scripts/setup.sh --tm-clear
```

### Step 7: Validate Translations

```bash
# Check all translations for issues
.claude/skills/translation-sync/scripts/setup.sh --validate
```

## Translation Rules

### Terms Handling

| Category | Rule | Example |
|----------|------|---------|
| **Acronyms** | Keep English, add transliteration | ROS 2 (آر او ایس ٹو) |
| **Product Names** | Keep English | Gazebo, RViz |
| **Technical Verbs** | Translate | publish → شائع کریں |
| **Code Elements** | Never translate | `rclpy`, `ros2 run` |
| **File Paths** | Never translate | `/opt/ros/humble` |

### Preservation Rules

1. **Code Blocks**: Never translate content inside ``` or `
2. **Frontmatter**: Only translate specified keys (title, description)
3. **Links**: Keep URLs intact, translate link text
4. **Components**: Keep JSX/MDX components unchanged
5. **Math**: Keep LaTeX expressions unchanged

## Verification Checklist

- [ ] `OPENAI_API_KEY` environment variable set
- [ ] Source directory exists with MDX files
- [ ] Target directory structure created
- [ ] Glossary loaded correctly
- [ ] Translation preserves code blocks
- [ ] RTL formatting applied correctly
- [ ] Technical terms handled per glossary
- [ ] Translation memory working (v1.1.0)
- [ ] Incremental mode caching sections (v1.1.0)

## Integration with Docusaurus

This skill is designed for Docusaurus i18n:

```javascript
// docusaurus.config.js
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },
};
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **RTL not displaying** | Check `dir="rtl"` wrapper is present |
| **Code blocks translated** | Verify preserve_patterns in config |
| **Terms inconsistent** | Add terms to glossary.json |
| **API rate limit** | Reduce batch size or add delays |
| **Frontmatter broken** | Check YAML syntax in source file |
| **TM not matching** | Segments must be exact matches |
| **Incremental not caching** | Run full sync first to populate section hashes |

## Cost Considerations

| Resource | Cost |
|----------|------|
| **GPT-4o-mini** | $0.15 / 1M input tokens |
| **GPT-4o** | $2.50 / 1M input tokens |
| **Estimated per chapter (full)** | ~$0.02 (gpt-4o-mini) |
| **Estimated per chapter (incremental)** | ~$0.005 (60-80% savings) |

## Related

- Skill: `urdu-rtl-styler` - RTL CSS styling for Docusaurus
- Skill: `docusaurus-scaffold` - Docusaurus site scaffolding
- Feature: `005-translation` - Translation specification

## Changelog

### v1.1.0 (2026-01-02)
**Feature Enhancements**

- **NEW**: `--diff FILE` - Preview changes since last translation
- **NEW**: `--incremental` - Section-level incremental translation (60-80% cost savings)
- **NEW**: Translation Memory (TM) cache with reuse tracking
  - `--tm-stats` - View TM statistics
  - `--tm-export FILE` - Export to JSON or TMX format
  - `--tm-import FILE` - Import from JSON
  - `--tm-clear` - Clear TM cache
- **NEW**: `--no-tm` - Disable translation memory
- Section-level hashing for precise change detection
- Status file now includes section data for incremental updates
- TranslationResult includes `sections_translated`, `sections_cached`, `tm_hits`

### v1.0.0 (2026-01-02)
**Initial Release**

- File synchronization between en and ur
- Translation status tracking
- Technical terms glossary with 30+ terms
- Code block and frontmatter preservation
- RTL wrapper injection
- OpenAI-powered translation
- Dry-run mode for previewing changes
- Validation for translation quality
