# Research: Urdu Translation & RTL Support

**Feature Branch**: `005-translation`
**Date**: 2025-12-31
**Status**: Complete

## Research Questions

This document captures technology decisions and research findings for implementing Urdu translation with RTL support.

## Decision 1: Translation Framework

### Question
Which framework should handle multilingual content and language switching?

### Options Considered

| Option | Pros | Cons |
|--------|------|------|
| **Docusaurus i18n** | Native integration, static files, no API costs, works with GitHub Pages | Limited to supported locales (Urdu supported) |
| External Translation API | Dynamic translation, no manual work | API costs, latency, quality concerns |
| Custom i18n Implementation | Full control | Significant development effort, reinventing wheel |

### Decision
**Docusaurus i18n** - Native framework feature with built-in support for:
- Locale detection and routing
- Static file storage for translations
- Direction (RTL/LTR) configuration per locale
- Theme string translation

### Rationale
- Zero runtime cost (static files)
- Works seamlessly with GitHub Pages
- Well-documented and community-supported
- Handles locale routing automatically (`/ur/chapter-1`)

---

## Decision 2: RTL Implementation

### Question
How should Right-to-Left rendering be implemented?

### Options Considered

| Option | Pros | Cons |
|--------|------|------|
| **CSS Logical Properties + dir attribute** | Native browser support, no JS overhead | Requires careful CSS refactoring |
| JavaScript-based direction switching | Full control | Performance overhead, complexity |
| Separate RTL stylesheet | Complete isolation | Maintenance burden, duplication |

### Decision
**CSS with `dir="rtl"` attribute and logical properties**

### Implementation
```css
/* Base approach */
[dir="rtl"] {
  text-align: right;
}

/* Use logical properties where possible */
.container {
  margin-inline-start: 1rem; /* Instead of margin-left */
  padding-inline-end: 1rem;  /* Instead of padding-right */
}

/* Flexbox reversal for navigation */
[dir="rtl"] .navbar__items {
  flex-direction: row-reverse;
}
```

### Rationale
- Docusaurus automatically sets `dir="rtl"` for RTL locales
- CSS-only solution has no JavaScript performance impact
- Logical properties provide natural LTR/RTL adaptation

---

## Decision 3: Urdu Typography

### Question
Which font family should be used for Urdu text?

### Options Considered

| Font | Pros | Cons |
|------|------|------|
| **Noto Nastaliq Urdu** | Free, Google Fonts, proper Nastaliq | Large file size (~2MB) |
| Jameel Noori Nastaleeq | Popular in Pakistan | Licensing concerns, not on Google Fonts |
| Noto Sans Arabic | Smaller size, faster load | Naskh style, less authentic for Urdu |
| System fonts | Zero load time | Inconsistent rendering |

### Decision
**Noto Nastaliq Urdu** as primary, with Jameel Noori Nastaleeq and system serif as fallbacks.

### Implementation
```css
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');

[lang="ur"] {
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', 'Noto Sans Arabic', serif;
  line-height: 2;      /* Nastaliq requires more line height */
  font-size: 1.1em;    /* Slightly larger for readability */
}
```

### Rationale
- Noto Nastaliq provides authentic Nastaliq script rendering
- Free and available via Google Fonts (reliable CDN)
- Fallback chain ensures graceful degradation

---

## Decision 4: Translation Storage

### Question
Where should translation files be stored?

### Options Considered

| Option | Pros | Cons |
|--------|------|------|
| **i18n directory (Docusaurus standard)** | Native integration, version controlled | Part of repo size |
| External CMS | Non-developer editing | Additional dependency, API calls |
| Database | Dynamic updates | Requires backend, not static |

### Decision
**Standard Docusaurus i18n directory structure**

### Implementation
```
i18n/
└── ur/
    ├── docusaurus-theme-classic/
    │   ├── navbar.json       # Theme navbar strings
    │   └── footer.json       # Theme footer strings
    ├── docusaurus-plugin-content-docs/
    │   └── current/
    │       ├── chapter-1.mdx # Full chapter translations
    │       ├── chapter-2.mdx
    │       ├── chapter-3.mdx
    │       └── sidebar.json  # Sidebar label translations
    └── code.json             # Custom strings
```

### Rationale
- Version controlled with source code
- Build-time validation (broken translations fail build)
- No external dependencies
- Works with GitHub Pages static hosting

---

## Decision 5: Language Preference Persistence

### Question
How should user's language preference be remembered?

### Options Considered

| Option | Pros | Cons |
|--------|------|------|
| **localStorage** | No auth required, persists across sessions | Per-browser only |
| User profile (003-auth) | Cross-device sync | Requires login |
| Cookies | Works without JS | Size limits, complexity |
| URL-only | Shareable links | No persistence |

### Decision
**localStorage for unauthenticated users, sync with user profile when authenticated**

### Implementation
```typescript
// Save preference
const saveLanguagePreference = (locale: string) => {
  localStorage.setItem('preferredLocale', locale);

  // If authenticated, also update profile
  if (isAuthenticated()) {
    updateUserPreference('language_preference', locale);
  }
};

// Load preference on page load
const getPreferredLocale = () => {
  // Check authenticated user first
  if (isAuthenticated()) {
    return getUserPreference('language_preference');
  }
  return localStorage.getItem('preferredLocale') || 'en';
};
```

### Rationale
- Works without authentication (majority of users)
- Syncs with 003-authentication profile when logged in
- Persists across sessions (SC-055)
- Simple implementation

---

## Decision 6: Code Block Handling

### Question
How should code examples be handled in RTL context?

### Options Considered

| Option | Pros | Cons |
|--------|------|------|
| **Force LTR with CSS** | Simple, reliable | Requires CSS rule |
| Unicode bidi overrides | Standard approach | Complex, can break |
| Separate code component | Full control | Component refactoring |

### Decision
**CSS-based LTR forcing for all `<pre>` and `<code>` elements**

### Implementation
```css
/* Force LTR for all code regardless of page direction */
[dir="rtl"] pre,
[dir="rtl"] code,
[dir="rtl"] .prism-code {
  direction: ltr;
  text-align: left;
  unicode-bidi: isolate;
}
```

### Rationale
- Code must remain readable (FR-082)
- Simple CSS solution with no JavaScript
- `unicode-bidi: isolate` prevents text mixing issues

---

## Decision 7: Translation Quality Process

### Question
How should translation quality be ensured?

### Options Considered

| Option | Pros | Cons |
|--------|------|------|
| **AI translation + human review** | Fast initial translation, quality review | Requires reviewer |
| Professional translation service | High quality | Cost, timeline |
| Community translation | Free, authentic | Unpredictable quality/timeline |
| Pure AI translation | Fastest | Quality concerns for technical content |

### Decision
**AI-assisted translation with native speaker review**

### Process
1. Use AI (Claude/GPT) for initial translation draft
2. Native Urdu speaker reviews and corrects
3. Quality scoring using rubric (85% target)
4. Create glossary of technical terms for consistency

### Quality Rubric (SC-048)
- **Fluency** (1-5): Does it read naturally?
- **Accuracy** (1-5): Does it convey original meaning?
- **Naturalness** (1-5): Uses appropriate Urdu conventions?
- **Target**: Average ≥ 4.25/5.0 (85%)

### Rationale
- AI provides fast first draft
- Human review ensures quality for technical content
- Glossary maintains consistency across chapters
- Measurable quality criteria for success

---

## Decision 8: Scroll Position Preservation

### Question
How should scroll position be maintained when switching languages?

### Options Considered

| Option | Pros | Cons |
|--------|------|------|
| **sessionStorage + scroll restoration** | Simple, reliable | Requires same page structure |
| Content section anchors | Precise positioning | Requires anchor mapping |
| Percentage-based | Works with different heights | Less precise |

### Decision
**sessionStorage with percentage-based fallback**

### Implementation
```typescript
// Before language switch
const preserveScroll = () => {
  const scrollPercent = window.scrollY / document.body.scrollHeight;
  sessionStorage.setItem('scrollPercent', String(scrollPercent));
  sessionStorage.setItem('scrollY', String(window.scrollY));
};

// After page load in new language
const restoreScroll = () => {
  const scrollPercent = parseFloat(sessionStorage.getItem('scrollPercent') || '0');
  const targetY = scrollPercent * document.body.scrollHeight;
  window.scrollTo(0, targetY);
  sessionStorage.removeItem('scrollPercent');
};
```

### Rationale
- SC-051 requires within 100px accuracy
- Percentage-based handles different content heights between languages
- sessionStorage survives page navigation

---

## Technical Term Glossary

| English | Urdu | Transliteration |
|---------|------|-----------------|
| Robotics | روبوٹکس | Robotics |
| Machine Learning | مشین لرننگ | Machine Learning |
| ROS 2 | ROS 2 | (keep as-is) |
| Gazebo | Gazebo | (keep as-is) |
| Simulation | سمولیشن | Simulation |
| Sensor | سینسر | Sensor |
| Algorithm | الگورتھم | Algorithm |
| Neural Network | نیورل نیٹ ورک | Neural Network |
| Vision-Language-Action | ویژن-لینگوئج-ایکشن | Vision-Language-Action |
| API | API | (keep as-is) |
| Physical AI | فزیکل AI | Physical AI |

### Note on Technical Terms
Many technical terms in Physical AI and robotics have no established Urdu equivalents. The approach is:
1. Use English term with Urdu transliteration
2. Provide English in parentheses on first use
3. Maintain consistency using this glossary

---

## Browser Compatibility

### RTL Support Matrix

| Browser | RTL Support | Notes |
|---------|-------------|-------|
| Chrome 90+ | ✅ Full | Logical properties supported |
| Firefox 89+ | ✅ Full | Excellent RTL support |
| Safari 14.1+ | ✅ Full | Logical properties supported |
| Edge 90+ | ✅ Full | Chromium-based |
| IE 11 | ⚠️ Partial | No logical properties (unsupported) |

### Font Rendering

| Browser | Noto Nastaliq | Notes |
|---------|---------------|-------|
| Chrome | ✅ Excellent | Variable font support |
| Firefox | ✅ Excellent | Good Nastaliq rendering |
| Safari | ✅ Good | Minor ligature differences |
| Edge | ✅ Excellent | Same as Chrome |

---

## Performance Considerations

### Font Loading Strategy
```css
/* Preload critical font weight */
<link rel="preload" href="noto-nastaliq-urdu.woff2" as="font" type="font/woff2" crossorigin>

/* Font display swap for fast initial render */
@font-face {
  font-family: 'Noto Nastaliq Urdu';
  font-display: swap;
  src: url('noto-nastaliq-urdu.woff2') format('woff2');
}
```

### Language Switch Performance
- Target: <2 seconds (FR-077)
- Approach: Static file serving (no API calls)
- Optimization: Prefetch alternate locale on hover

```typescript
// Prefetch on hover
const prefetchLocale = (locale: string) => {
  const link = document.createElement('link');
  link.rel = 'prefetch';
  link.href = `/${locale}${location.pathname}`;
  document.head.appendChild(link);
};
```

---

## References

- [Docusaurus i18n Documentation](https://docusaurus.io/docs/i18n/introduction)
- [CSS Logical Properties](https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_Logical_Properties)
- [Noto Nastaliq Urdu](https://fonts.google.com/specimen/Noto+Nastaliq+Urdu)
- [Unicode Bidirectional Algorithm](https://unicode.org/reports/tr9/)
