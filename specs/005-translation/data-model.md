# Data Model: Urdu Translation & RTL Support

**Feature Branch**: `005-translation`
**Date**: 2025-12-31
**Status**: Complete

## Overview

This feature uses **static files** rather than a database for translation storage. The "data model" describes the structure of translation files and the conceptual entities involved.

## Entity Relationship Diagram

```
┌─────────────────────┐       ┌─────────────────────┐
│   SourceContent     │       │   TranslatedContent │
│   (English MDX)     │──────▶│   (Urdu MDX)        │
└─────────────────────┘       └─────────────────────┘
         │                             │
         │                             │
         ▼                             ▼
┌─────────────────────┐       ┌─────────────────────┐
│   UIStrings         │       │   UIStrings         │
│   (English JSON)    │──────▶│   (Urdu JSON)       │
└─────────────────────┘       └─────────────────────┘
         │
         │
         ▼
┌─────────────────────┐
│  LanguagePreference │
│  (localStorage)     │
└─────────────────────┘
```

## Entities

### 1. Translation Unit

A segment of content with source (English) and target (Urdu) versions.

**Storage**: File-based (MDX for content, JSON for UI strings)

| Attribute | Type | Description |
|-----------|------|-------------|
| `source_path` | string | Path to English source file |
| `target_path` | string | Path to Urdu translation file |
| `content_type` | enum | `chapter` \| `ui_string` \| `sidebar` |
| `translation_status` | enum | `complete` \| `in_progress` \| `pending` |
| `quality_score` | float | 0.0-1.0 (85% = 0.85 target) |
| `last_updated` | datetime | ISO timestamp of last edit |

**Example - Chapter Content (MDX)**:
```
Source: docs/chapter-1.mdx
Target: i18n/ur/docusaurus-plugin-content-docs/current/chapter-1.mdx
```

**Example - UI String (JSON)**:
```json
// i18n/ur/docusaurus-theme-classic/navbar.json
{
  "title": {
    "message": "فزیکل AI کتاب",
    "description": "The title in the navbar"
  }
}
```

---

### 2. Language Preference

User's selected display language, stored client-side.

**Storage**: localStorage (primary), user profile (if authenticated)

| Attribute | Type | Description |
|-----------|------|-------------|
| `locale` | string | `'en'` \| `'ur'` |
| `updated_at` | datetime | Last preference change |
| `source` | enum | `localStorage` \| `user_profile` |

**localStorage Key**: `preferredLocale`

**TypeScript Interface**:
```typescript
interface LanguagePreference {
  locale: 'en' | 'ur';
  updatedAt: string; // ISO datetime
}

// Get preference
const getLanguagePreference = (): LanguagePreference => {
  const stored = localStorage.getItem('preferredLocale');
  return {
    locale: stored === 'ur' ? 'ur' : 'en',
    updatedAt: localStorage.getItem('preferredLocaleUpdatedAt') || new Date().toISOString()
  };
};

// Set preference
const setLanguagePreference = (locale: 'en' | 'ur') => {
  localStorage.setItem('preferredLocale', locale);
  localStorage.setItem('preferredLocaleUpdatedAt', new Date().toISOString());
};
```

---

### 3. Bilingual Content Block

A content section that renders differently based on language selection.

**Storage**: Separate MDX files per locale

| Attribute | Type | Description |
|-----------|------|-------------|
| `english_content` | MDX | English version |
| `urdu_content` | MDX | Urdu version |
| `content_type` | enum | `text` \| `heading` \| `caption` \| `code_explanation` |
| `direction` | enum | `ltr` \| `rtl` |
| `has_code` | boolean | Contains code block |

**File Structure**:
```
docs/
└── chapter-1.mdx           # English content

i18n/ur/docusaurus-plugin-content-docs/current/
└── chapter-1.mdx           # Urdu content (mirrored structure)
```

---

### 4. Technical Term Entry

Glossary entry for consistent translation of technical terms.

**Storage**: `i18n/glossary.md` (Markdown table)

| Attribute | Type | Description |
|-----------|------|-------------|
| `english_term` | string | Original English term |
| `urdu_term` | string | Urdu translation/transliteration |
| `transliteration` | string | Roman letter representation |
| `keep_english` | boolean | Whether to keep English in parentheses |
| `context` | string | Usage context notes |

**Example**:
```markdown
| English | Urdu | Keep English | Context |
|---------|------|--------------|---------|
| ROS 2 | ROS 2 | yes | Always keep as-is |
| Robotics | روبوٹکس | yes | Add (Robotics) on first use |
| Neural Network | نیورل نیٹ ورک | no | Common enough |
```

---

### 5. Translation Quality Record

Quality assessment for random sample paragraphs.

**Storage**: `specs/005-translation/quality-review.md` (during QA)

| Attribute | Type | Description |
|-----------|------|-------------|
| `paragraph_id` | string | Reference to paragraph |
| `source_text` | string | Original English |
| `translated_text` | string | Urdu translation |
| `fluency_score` | int | 1-5 rating |
| `accuracy_score` | int | 1-5 rating |
| `naturalness_score` | int | 1-5 rating |
| `average_score` | float | (fluency + accuracy + naturalness) / 3 |
| `reviewer` | string | Reviewer identifier |
| `review_date` | date | Date of review |

**Quality Calculation**:
```typescript
interface QualityReview {
  paragraphId: string;
  fluency: 1 | 2 | 3 | 4 | 5;
  accuracy: 1 | 2 | 3 | 4 | 5;
  naturalness: 1 | 2 | 3 | 4 | 5;
}

const calculateQuality = (reviews: QualityReview[]): number => {
  const total = reviews.reduce((sum, r) =>
    sum + (r.fluency + r.accuracy + r.naturalness) / 3, 0);
  return (total / reviews.length) / 5; // Returns 0.0-1.0
};

// SC-048: Target >= 0.85 (85%)
```

---

## File Structure Schema

### Docusaurus i18n Directory

```
i18n/
└── ur/
    ├── docusaurus-theme-classic/
    │   ├── navbar.json           # Schema: NavbarStrings
    │   └── footer.json           # Schema: FooterStrings
    ├── docusaurus-plugin-content-docs/
    │   └── current/
    │       ├── chapter-1.mdx     # Schema: ChapterContent
    │       ├── chapter-2.mdx
    │       ├── chapter-3.mdx
    │       └── sidebar.json      # Schema: SidebarStrings
    └── code.json                 # Schema: CustomStrings
```

### JSON Schemas

**NavbarStrings**:
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "patternProperties": {
    "^.*$": {
      "type": "object",
      "properties": {
        "message": { "type": "string" },
        "description": { "type": "string" }
      },
      "required": ["message"]
    }
  }
}
```

**Example navbar.json**:
```json
{
  "title": {
    "message": "فزیکل AI کتاب",
    "description": "The title in the navbar"
  },
  "item.label.Docs": {
    "message": "دستاویزات",
    "description": "Navbar item with label Docs"
  },
  "item.label.Chapter 1": {
    "message": "باب ۱",
    "description": "Chapter 1 navigation"
  },
  "item.label.Chapter 2": {
    "message": "باب ۲",
    "description": "Chapter 2 navigation"
  },
  "item.label.Chapter 3": {
    "message": "باب ۳",
    "description": "Chapter 3 navigation"
  }
}
```

---

## TypeScript Interfaces

```typescript
// src/types/translation.ts

/**
 * Supported locales
 */
export type Locale = 'en' | 'ur';

/**
 * Text direction
 */
export type Direction = 'ltr' | 'rtl';

/**
 * Locale configuration
 */
export interface LocaleConfig {
  label: string;
  direction: Direction;
  htmlLang: string;
}

/**
 * i18n configuration for Docusaurus
 */
export interface I18nConfig {
  defaultLocale: Locale;
  locales: Locale[];
  localeConfigs: Record<Locale, LocaleConfig>;
}

/**
 * Translation status for a content unit
 */
export type TranslationStatus = 'complete' | 'in_progress' | 'pending';

/**
 * Translation unit metadata
 */
export interface TranslationUnit {
  sourcePath: string;
  targetPath: string;
  contentType: 'chapter' | 'ui_string' | 'sidebar';
  status: TranslationStatus;
  qualityScore?: number;
  lastUpdated: string;
}

/**
 * Language preference stored client-side
 */
export interface LanguagePreference {
  locale: Locale;
  updatedAt: string;
}

/**
 * Technical term glossary entry
 */
export interface GlossaryEntry {
  english: string;
  urdu: string;
  transliteration?: string;
  keepEnglishInParens: boolean;
  context?: string;
}

/**
 * Quality review for a paragraph
 */
export interface QualityReview {
  paragraphId: string;
  sourceText: string;
  translatedText: string;
  fluencyScore: 1 | 2 | 3 | 4 | 5;
  accuracyScore: 1 | 2 | 3 | 4 | 5;
  naturalnessScore: 1 | 2 | 3 | 4 | 5;
  averageScore: number;
  reviewer: string;
  reviewDate: string;
}

/**
 * Scroll preservation state
 */
export interface ScrollState {
  scrollY: number;
  scrollPercent: number;
  path: string;
}
```

---

## Docusaurus Configuration Schema

```javascript
// docusaurus.config.js - i18n section
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },
};
```

---

## Data Flow

### Language Switch Flow

```
User clicks language toggle
         │
         ▼
┌─────────────────────┐
│ Save scroll position │
│ to sessionStorage    │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Update localStorage  │
│ preferredLocale     │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Navigate to new URL │
│ /{locale}/path      │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Docusaurus loads    │
│ locale-specific     │
│ static files        │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Restore scroll      │
│ from sessionStorage │
└─────────────────────┘
```

### Content Resolution Flow

```
Request: /ur/chapter-1
         │
         ▼
┌─────────────────────┐
│ Docusaurus checks   │
│ i18n/ur/docs/       │
│ chapter-1.mdx       │
└──────────┬──────────┘
           │
    ┌──────┴──────┐
    │             │
    ▼             ▼
 Found?        Not Found?
    │             │
    ▼             ▼
┌─────────┐  ┌─────────────┐
│ Render  │  │ Fallback to │
│ Urdu    │  │ English +   │
│ content │  │ notice      │
└─────────┘  └─────────────┘
```

---

## Notes

### Why No Database?

This feature uses static files instead of a database because:

1. **Static hosting**: GitHub Pages cannot run a backend
2. **Performance**: No API calls, instant content delivery
3. **Version control**: Translations tracked with source code
4. **Build-time validation**: Missing translations caught at build
5. **Simplicity**: No migration scripts, no connection strings

### Integration with 003-Authentication

When user is authenticated, their `language_preference` from the user profile (003-authentication) should sync with localStorage:

```typescript
// On login, sync preference
const syncLanguagePreference = async (userProfile: UserProfile) => {
  if (userProfile.preferences.language_preference) {
    localStorage.setItem('preferredLocale', userProfile.preferences.language_preference);
  }
};

// On preference change, update profile if authenticated
const updateLanguagePreference = async (locale: Locale) => {
  localStorage.setItem('preferredLocale', locale);

  if (isAuthenticated()) {
    await api.patch('/api/preferences', {
      preferences: [{ preference_key: 'language_preference', preference_value: locale }]
    });
  }
};
```
