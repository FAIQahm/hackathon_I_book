# Quickstart: Urdu Translation & RTL Support

**Feature Branch**: `005-translation`
**Created**: 2025-12-31

## Prerequisites

- Node.js 18+ and npm
- Git
- Text editor with UTF-8 and RTL support (VS Code recommended)
- Urdu font installed (for local preview)

## Environment Setup

### 1. Clone and Branch

```bash
git clone <repository-url>
cd hackathon-1
git checkout 005-translation
```

### 2. Install Dependencies

```bash
npm install
```

### 3. Create i18n Directory Structure

```bash
# Create Urdu locale directories
mkdir -p i18n/ur/docusaurus-theme-classic
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current

# Initialize translation files
npm run write-translations -- --locale ur
```

This creates the base translation files that need to be filled in.

## Development Workflow

### Start Development Server

```bash
# Start in English (default)
npm run start

# Start in Urdu mode
npm run start -- --locale ur
```

Development server available at:
- English: `http://localhost:3000/`
- Urdu: `http://localhost:3000/ur/`

### Build for Production

```bash
# Build all locales
npm run build

# Build specific locale only
npm run build -- --locale ur
```

## Translation Tasks

### Task 1: Configure i18n in Docusaurus

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
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

### Task 2: Add Urdu Fonts

Create `src/css/fonts.css`:

```css
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');

[lang="ur"],
[dir="rtl"] {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
  line-height: 2;
  font-size: 1.1em;
}
```

Import in `src/css/custom.css`:

```css
@import './fonts.css';
```

### Task 3: Add RTL Styles

Create `src/css/rtl.css`:

```css
/* Base RTL adjustments */
[dir="rtl"] {
  text-align: right;
}

/* Navbar items reversed */
[dir="rtl"] .navbar__items {
  flex-direction: row-reverse;
}

/* Pagination reversed */
[dir="rtl"] .pagination-nav {
  flex-direction: row-reverse;
}

/* Sidebar adjustments */
[dir="rtl"] .menu__link {
  padding-left: 0;
  padding-right: var(--ifm-menu-link-padding-horizontal);
}

/* Code blocks stay LTR */
[dir="rtl"] pre,
[dir="rtl"] code,
[dir="rtl"] .prism-code {
  direction: ltr;
  text-align: left;
  unicode-bidi: isolate;
}

/* Lists */
[dir="rtl"] ul,
[dir="rtl"] ol {
  padding-right: 2rem;
  padding-left: 0;
}
```

Import in `src/css/custom.css`:

```css
@import './rtl.css';
```

### Task 4: Translate UI Strings

Edit `i18n/ur/docusaurus-theme-classic/navbar.json`:

```json
{
  "title": {
    "message": "فزیکل AI کتاب",
    "description": "The title in the navbar"
  },
  "item.label.Docs": {
    "message": "دستاویزات",
    "description": "Navbar docs link"
  },
  "logo.alt": {
    "message": "Physical AI کتاب لوگو",
    "description": "Logo alt text"
  }
}
```

Edit `i18n/ur/docusaurus-theme-classic/footer.json`:

```json
{
  "copyright": {
    "message": "کاپی رائٹ © 2025 Physical AI کتاب۔ تمام حقوق محفوظ ہیں۔",
    "description": "Footer copyright"
  }
}
```

### Task 5: Translate Chapter Content

Copy English content and translate:

```bash
# Copy structure
cp docs/chapter-1.mdx i18n/ur/docusaurus-plugin-content-docs/current/chapter-1.mdx
```

Edit the Urdu version, following these rules:

```markdown
---
id: chapter-1
title: فزیکل AI اور ROS 2 کا تعارف
sidebar_label: باب ۱
---

# فزیکل AI اور ROS 2 کا تعارف

## سیکھنے کے مقاصد

اس باب کو مکمل کرنے کے بعد، آپ یہ کر سکیں گے:

- فزیکل AI (Physical AI) کی بنیادی تصورات کو سمجھنا
- ROS 2 کے فریم ورک سے واقفیت حاصل کرنا

## کوڈ کی مثال

یہ کوڈ ROS 2 نوڈ بناتا ہے:

```python
# Code stays in English - DO NOT TRANSLATE
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
```

اوپر دی گئی مثال میں، ہم `rclpy` لائبریری استعمال کر رہے ہیں۔
```

### Task 6: Create Language Toggle Component

Create `src/components/LanguageToggle.tsx`:

```typescript
import React from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './LanguageToggle.module.css';

export default function LanguageToggle(): JSX.Element {
  const { i18n } = useDocusaurusContext();
  const location = useLocation();
  const currentLocale = i18n.currentLocale;

  const switchLocale = (newLocale: string) => {
    // Preserve scroll position
    const scrollPercent = window.scrollY / document.body.scrollHeight;
    sessionStorage.setItem('scrollPercent', String(scrollPercent));

    // Save preference
    localStorage.setItem('preferredLocale', newLocale);

    // Navigate
    const currentPath = location.pathname;
    const pathWithoutLocale = currentPath.replace(/^\/(en|ur)/, '') || '/';
    const newPath = newLocale === 'en'
      ? pathWithoutLocale
      : `/${newLocale}${pathWithoutLocale}`;

    window.location.href = newPath;
  };

  const targetLocale = currentLocale === 'en' ? 'ur' : 'en';
  const label = currentLocale === 'en' ? 'اردو' : 'English';

  return (
    <button
      className={styles.toggle}
      onClick={() => switchLocale(targetLocale)}
      aria-label={`Switch to ${targetLocale === 'ur' ? 'Urdu' : 'English'}`}
    >
      {label}
    </button>
  );
}
```

Create `src/components/LanguageToggle.module.css`:

```css
.toggle {
  background: transparent;
  border: 1px solid var(--ifm-color-primary);
  border-radius: 4px;
  padding: 0.25rem 0.75rem;
  cursor: pointer;
  font-size: 0.9rem;
  color: var(--ifm-color-primary);
  transition: all 0.2s;
}

.toggle:hover {
  background: var(--ifm-color-primary);
  color: white;
}

[dir="rtl"] .toggle {
  font-family: 'Noto Nastaliq Urdu', serif;
}
```

## Testing

### Visual RTL Testing

1. Start server in Urdu mode: `npm run start -- --locale ur`
2. Check these elements render RTL:
   - [ ] Page text flows right-to-left
   - [ ] Navbar items are reversed
   - [ ] Sidebar is on the right
   - [ ] Pagination arrows are correct
   - [ ] Lists bullets are on the right

### Code Block Testing

1. View any page with code examples
2. Verify:
   - [ ] Code blocks remain LTR
   - [ ] Syntax highlighting works
   - [ ] Copy button is positioned correctly

### Language Switch Testing

1. Click language toggle
2. Verify:
   - [ ] Content switches to other language
   - [ ] Scroll position is approximately preserved
   - [ ] Preference persists after refresh

### Browser Testing

Test on all target browsers:

| Browser | RTL | Fonts | Toggle |
|---------|-----|-------|--------|
| Chrome | [ ] | [ ] | [ ] |
| Firefox | [ ] | [ ] | [ ] |
| Safari | [ ] | [ ] | [ ] |
| Edge | [ ] | [ ] | [ ] |

## Common Issues

### Issue: Urdu text not displaying correctly

**Cause**: Font not loaded
**Solution**: Check network tab for font loading. Ensure Google Fonts URL is correct.

```css
/* Verify this import is present */
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');
```

### Issue: RTL not applying

**Cause**: `dir="rtl"` not set on HTML
**Solution**: Verify `docusaurus.config.js` has correct locale config:

```javascript
ur: {
  direction: 'rtl',  // Must be 'rtl', not 'ltr'
  htmlLang: 'ur-PK',
}
```

### Issue: Code blocks reversed

**Cause**: RTL CSS overriding code blocks
**Solution**: Add specific override:

```css
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr !important;
  text-align: left !important;
}
```

### Issue: Build fails with missing translations

**Cause**: JSON syntax error in translation files
**Solution**: Validate JSON files:

```bash
# Check JSON syntax
npx jsonlint i18n/ur/docusaurus-theme-classic/navbar.json
```

### Issue: Language toggle not appearing

**Cause**: Component not added to navbar
**Solution**: Add to `src/theme/Navbar/Content/index.tsx` or configure in `docusaurus.config.js`:

```javascript
navbar: {
  items: [
    // ... other items
    {
      type: 'localeDropdown',
      position: 'right',
    },
  ],
},
```

## Translation Quality Checklist

Before submitting translations for review:

- [ ] All 3 chapters are fully translated
- [ ] No untranslated English text in content
- [ ] Technical terms follow glossary conventions
- [ ] Code examples are unchanged
- [ ] Images have translated captions
- [ ] All UI strings are translated
- [ ] Navigation labels are translated
- [ ] RTL renders correctly on all pages

## File Checklist

Files to create/modify for this feature:

```
✅ Created during setup:
- [ ] docusaurus.config.js (i18n section)
- [ ] src/css/fonts.css
- [ ] src/css/rtl.css
- [ ] src/components/LanguageToggle.tsx
- [ ] src/components/LanguageToggle.module.css

✅ Translation files:
- [ ] i18n/ur/docusaurus-theme-classic/navbar.json
- [ ] i18n/ur/docusaurus-theme-classic/footer.json
- [ ] i18n/ur/docusaurus-plugin-content-docs/current/chapter-1.mdx
- [ ] i18n/ur/docusaurus-plugin-content-docs/current/chapter-2.mdx
- [ ] i18n/ur/docusaurus-plugin-content-docs/current/chapter-3.mdx
- [ ] i18n/ur/docusaurus-plugin-content-docs/current/sidebar.json
- [ ] i18n/glossary.md

✅ Optional (for custom implementation):
- [ ] src/hooks/useScrollPreservation.ts
- [ ] src/utils/languagePreference.ts
```

## Next Steps

1. Complete Phase 1 (i18n Configuration)
2. Implement Phase 2 (RTL Styling)
3. Complete Phase 3 (UI String Translation)
4. Translate Phase 4 (Chapter Content)
5. Build Phase 5 (Language Toggle)
6. Conduct Phase 6 (Quality Assurance)

See `plan.md` for detailed implementation phases.
