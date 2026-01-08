# Implementation Plan: Urdu Translation & RTL Support

**Branch**: `005-translation` | **Date**: 2025-12-31 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-translation/spec.md`

## Summary

Implement complete Urdu translation for all 3 book chapters with Right-to-Left (RTL) rendering support. The system uses Docusaurus's built-in i18n framework for language switching, stores translations as static files, and preserves code examples in their original programming language while translating surrounding context.

## Technical Context

**Language/Version**: TypeScript 5.x (Docusaurus), MDX for content
**Primary Dependencies**: Docusaurus i18n, React, TailwindCSS (RTL utilities)
**Storage**: Static files (MDX for content, JSON for UI strings)
**Testing**: Vitest (components), Manual QA (translation quality)
**Target Platform**: GitHub Pages (static site hosting)
**Project Type**: Static site with i18n (Docusaurus documentation site)
**Performance Goals**:
- <2s language switch (FR-077)
- Scroll position preserved within 100px (SC-051)
**Constraints**:
- 85%+ translation quality (SC-048)
- RTL rendering on all browsers (SC-049)
- Code examples unchanged (FR-082)
**Scale/Scope**: 3 chapters (~15,000 words), full UI translation

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Requirement | Status |
|-----------|-------------|--------|
| Educational Excellence | Accurate, clear content | ✅ 85%+ translation quality requirement |
| Technical Accuracy | No placeholder content | ✅ Complete translations before deploy |
| AI-Native Development | 4+ skills, 3+ agents | ✅ AI-assisted translation with human review |
| User-Centric Design | RTL support, accessible | ✅ FR-075 RTL, proper Urdu typography |
| Quality Over Quantity | Quality translations | ✅ SC-048 native speaker review |

**Tech Stack Compliance**:
- ✅ Docusaurus i18n (native framework feature)
- ✅ GitHub Pages (static deployment)
- ✅ RTL support (per constitution requirement)
- ✅ 85%+ quality (per constitution requirement)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         DOCUSAURUS FRONTEND                              │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ Language     │  │ RTL Layout   │  │ Chapter      │  │ Navigation   │ │
│  │ Toggle       │  │ Provider     │  │ Content      │  │ (i18n)       │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
└─────────┼──────────────────┼──────────────────┼──────────────────┼───────┘
          │                  │                  │                  │
          ▼                  ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         DOCUSAURUS i18n LAYER                            │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    TranslationProvider                            │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ │   │
│  │  │ Locale      │ │ Content     │ │ UI String   │ │ Direction   │ │   │
│  │  │ Detection   │ │ Loader      │ │ Resolver    │ │ Manager     │ │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          STATIC FILES                                    │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                   │
│  │ /docs/       │  │ /i18n/ur/    │  │ /i18n/ur/    │                   │
│  │ (English)    │  │ docusaurus-  │  │ docs/        │                   │
│  │              │  │ theme-*/     │  │ (Urdu MDX)   │                   │
│  └──────────────┘  └──────────────┘  └──────────────┘                   │
└─────────────────────────────────────────────────────────────────────────┘
```

## Implementation Phases

### Phase 1: Docusaurus i18n Configuration
**Goal**: Set up Docusaurus i18n infrastructure for Urdu

| Task | Description | Files |
|------|-------------|-------|
| 1.1 | Configure Docusaurus i18n for Urdu locale | `docusaurus.config.js` |
| 1.2 | Set up i18n directory structure | `i18n/ur/` directory |
| 1.3 | Configure RTL direction for Urdu locale | `docusaurus.config.js` |
| 1.4 | Add Urdu web fonts (Noto Nastaliq Urdu) | `src/css/custom.css` |
| 1.5 | Create locale-specific theme overrides | `i18n/ur/docusaurus-theme-classic/` |

**Docusaurus i18n Configuration**:
```javascript
// docusaurus.config.js
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

### Phase 2: RTL Styling & Typography
**Goal**: Implement proper RTL rendering and Urdu typography

| Task | Description | Files |
|------|-------------|-------|
| 2.1 | Add RTL CSS utilities | `src/css/rtl.css` |
| 2.2 | Configure Urdu font stack | `src/css/fonts.css` |
| 2.3 | Create RTL layout overrides | `src/css/rtl-overrides.css` |
| 2.4 | Handle mixed LTR/RTL content (bidi) | `src/css/bidi.css` |
| 2.5 | Test RTL rendering across browsers | Manual QA |

**RTL CSS Configuration**:
```css
/* src/css/rtl.css */
[dir="rtl"] {
  /* Base RTL styles */
  text-align: right;
}

[dir="rtl"] .navbar__items {
  flex-direction: row-reverse;
}

[dir="rtl"] .pagination-nav {
  flex-direction: row-reverse;
}

/* Preserve LTR for code blocks */
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}

/* Urdu typography */
[lang="ur"] {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
  line-height: 2;
  font-size: 1.1em;
}
```

### Phase 3: UI String Translation
**Goal**: Translate all Docusaurus theme UI strings

| Task | Description | Files |
|------|-------------|-------|
| 3.1 | Extract theme strings to JSON | `i18n/ur/docusaurus-theme-classic/navbar.json` |
| 3.2 | Translate navigation labels | `i18n/ur/docusaurus-theme-classic/navbar.json` |
| 3.3 | Translate footer content | `i18n/ur/docusaurus-theme-classic/footer.json` |
| 3.4 | Translate common UI elements | `i18n/ur/docusaurus-plugin-content-docs/current.json` |
| 3.5 | Translate error messages | `i18n/ur/code.json` |

**UI Strings Structure**:
```
i18n/ur/
├── docusaurus-theme-classic/
│   ├── navbar.json          # Navigation labels
│   └── footer.json          # Footer content
├── docusaurus-plugin-content-docs/
│   └── current.json         # Docs sidebar labels
└── code.json                 # Custom strings
```

**Example Translation**:
```json
// i18n/ur/docusaurus-theme-classic/navbar.json
{
  "title": {
    "message": "فزیکل AI کتاب",
    "description": "The title in the navbar"
  },
  "item.label.Docs": {
    "message": "دستاویزات",
    "description": "Navbar item with label Docs"
  },
  "item.label.Next Chapter": {
    "message": "اگلا باب",
    "description": "Next chapter navigation"
  }
}
```

### Phase 4: Chapter Content Translation
**Goal**: Translate all 3 chapters to Urdu

| Task | Description | Files |
|------|-------------|-------|
| 4.1 | Create Urdu docs directory structure | `i18n/ur/docusaurus-plugin-content-docs/current/` |
| 4.2 | Translate Chapter 1: Intro to Physical AI & ROS 2 | `i18n/ur/.../chapter-1.mdx` |
| 4.3 | Translate Chapter 2: Simulation with Gazebo | `i18n/ur/.../chapter-2.mdx` |
| 4.4 | Translate Chapter 3: Vision-Language-Action Models | `i18n/ur/.../chapter-3.mdx` |
| 4.5 | Translate sidebars and navigation | `i18n/ur/docusaurus-plugin-content-docs/current/sidebar.json` |

**Translation Guidelines**:
```markdown
## Translation Rules

1. **Preserve Code Blocks**: Keep all code exactly as-is
   ```python
   # This code stays in English
   import ros2
   ```

2. **Translate Surrounding Text**: All explanations in Urdu
   یہ کوڈ ROS 2 لائبریری کو امپورٹ کرتا ہے۔

3. **Technical Terms**: Keep English in parentheses
   روبوٹکس (Robotics) میں مشین لرننگ کا استعمال

4. **Image Captions**: Translate but keep alt text descriptive
   ![ROS 2 Architecture](./ros2-arch.png)
   *تصویر: ROS 2 کا فن تعمیر*
```

### Phase 5: Language Toggle Component
**Goal**: Create accessible language switching UI

| Task | Description | Files |
|------|-------------|-------|
| 5.1 | Create LanguageToggle component | `src/components/LanguageToggle.tsx` |
| 5.2 | Add scroll position preservation | `src/hooks/useScrollPreservation.ts` |
| 5.3 | Implement language preference storage | `src/utils/languagePreference.ts` |
| 5.4 | Add to navbar | `src/theme/Navbar/Content/index.tsx` |
| 5.5 | Style toggle for both LTR/RTL | `src/components/LanguageToggle.module.css` |

**Language Toggle Implementation**:
```typescript
// src/components/LanguageToggle.tsx
import React from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function LanguageToggle() {
  const { i18n } = useDocusaurusContext();
  const location = useLocation();

  const switchLocale = (locale: string) => {
    // Save scroll position
    const scrollPos = window.scrollY;
    sessionStorage.setItem('scrollPos', String(scrollPos));

    // Save preference
    localStorage.setItem('preferredLocale', locale);

    // Navigate to new locale
    const newPath = location.pathname.replace(
      /^\/(en|ur)/,
      `/${locale}`
    );
    window.location.href = newPath;
  };

  return (
    <div className="language-toggle">
      <button
        onClick={() => switchLocale(i18n.currentLocale === 'en' ? 'ur' : 'en')}
        aria-label="Switch language"
      >
        {i18n.currentLocale === 'en' ? 'اردو' : 'English'}
      </button>
    </div>
  );
}
```

### Phase 6: Translation Quality Assurance
**Goal**: Verify 85%+ translation quality

| Task | Description | Files |
|------|-------------|-------|
| 6.1 | Create translation quality rubric | `docs/translation-rubric.md` |
| 6.2 | Select 20 random paragraphs for review | QA process |
| 6.3 | Native speaker review and scoring | Manual QA |
| 6.4 | Document technical term glossary | `i18n/glossary.md` |
| 6.5 | Fix quality issues from review | Update translations |

**Quality Rubric**:
```
Translation Quality Scoring (SC-048)
====================================
Each paragraph scored 1-5 on three dimensions:

1. Fluency: Does it read naturally in Urdu?
   - 5: Native-quality, flows perfectly
   - 3: Understandable but awkward
   - 1: Difficult to read

2. Accuracy: Does it convey the original meaning?
   - 5: Perfect semantic match
   - 3: Minor meaning shifts
   - 1: Significant errors

3. Naturalness: Does it use appropriate Urdu conventions?
   - 5: Proper idioms and style
   - 3: Literal but acceptable
   - 1: Foreign-sounding

Target: Average ≥ 4.25/5.0 (85%)
```

## Project Structure

### Documentation (this feature)

```text
specs/005-translation/
├── plan.md              # This file
├── research.md          # Translation technology decisions
├── data-model.md        # Translation unit structure
├── quickstart.md        # Developer setup guide
├── contracts/           # N/A (no API for translation)
└── tasks.md             # Generated by /sp.tasks
```

### Source Code (repository root)

```text
i18n/
├── ur/
│   ├── docusaurus-theme-classic/
│   │   ├── navbar.json       # Navigation translations
│   │   └── footer.json       # Footer translations
│   ├── docusaurus-plugin-content-docs/
│   │   └── current/
│   │       ├── chapter-1.mdx # Chapter 1 Urdu
│   │       ├── chapter-2.mdx # Chapter 2 Urdu
│   │       ├── chapter-3.mdx # Chapter 3 Urdu
│   │       └── sidebar.json  # Sidebar translations
│   └── code.json             # Custom UI strings
└── glossary.md               # Technical term translations

src/
├── components/
│   ├── LanguageToggle.tsx    # Language switcher
│   └── LanguageToggle.module.css
├── hooks/
│   └── useScrollPreservation.ts
├── utils/
│   └── languagePreference.ts
├── theme/
│   └── Navbar/
│       └── Content/
│           └── index.tsx     # Navbar with toggle
└── css/
    ├── custom.css            # Base styles
    ├── rtl.css               # RTL utilities
    ├── fonts.css             # Urdu fonts
    ├── rtl-overrides.css     # Component overrides
    └── bidi.css              # Bidirectional text

docusaurus.config.js          # i18n configuration
```

**Structure Decision**: Static site with Docusaurus i18n. No backend API needed - translations stored as static files and served directly from GitHub Pages.

## Design Decisions

### D1: Docusaurus Native i18n
**Decision**: Use Docusaurus built-in i18n instead of external service
**Rationale**: Native support, static files, works with GitHub Pages, no API costs
**Alternative Rejected**: Translation API (adds latency, costs, complexity)

### D2: Static File Translations
**Decision**: Store translations as MDX/JSON files in i18n/ directory
**Rationale**: Build-time compilation, fast serving, version controlled
**Alternative Rejected**: Database storage (unnecessary for static content)

### D3: Noto Nastaliq Urdu Font
**Decision**: Use Google's Noto Nastaliq Urdu as primary font
**Rationale**: Free, widely supported, proper Nastaliq script rendering
**Fallback**: Jameel Noori Nastaleeq, system serif

### D4: CSS-based RTL
**Decision**: Use CSS direction and logical properties for RTL
**Rationale**: Native browser support, no JavaScript overhead
**Implementation**: `[dir="rtl"]` selectors with flexbox/grid adjustments

### D5: localStorage for Preference
**Decision**: Store language preference in localStorage
**Rationale**: Works without auth, persists across sessions (SC-055)
**Integration**: Check on page load, apply before render

### D6: Code Block Preservation
**Decision**: Keep all `<pre>` and `<code>` as LTR regardless of page direction
**Rationale**: Code must remain readable (FR-082)
**CSS**: `[dir="rtl"] pre, [dir="rtl"] code { direction: ltr; }`

## Success Criteria Mapping

| SC | Implementation |
|----|----------------|
| SC-047 | Phase 4 complete translations for all 3 chapters |
| SC-048 | Phase 6 quality rubric with native speaker review |
| SC-049 | Phase 2 RTL CSS + browser testing |
| SC-050 | Phase 5 LanguageToggle with <2s switch |
| SC-051 | Phase 5 useScrollPreservation hook |
| SC-052 | Phase 3 complete UI string translations |
| SC-053 | D6 CSS rule preserving code LTR |
| SC-054 | Phase 2 bidi.css for mixed content |
| SC-055 | D5 localStorage preference |

## Dependencies

```
001-textbook-generation ─────┐
  (English content to        │
   translate)                │
                             ▼
003-authentication ────► 005-translation
  (language_preference      │
   from user profile)       │
                            ▼
                     004-personalization
                     (language in profile)
```

**Blocking Dependencies**:
- Content from 001-textbook-generation MUST be complete before translation
- Stable chapter content required (changes require re-translation)

**Integration Points**:
- 003-authentication: language_preference dimension syncs with localStorage
- 004-personalization: Uses language preference for response adaptation
