# ADR-003: Translation & Internationalization Approach

**Status**: Accepted
**Date**: 2025-12-31
**Deciders**: Project Team

## Context

The Physical AI Educational Book requires Urdu translation with Right-to-Left (RTL) support to serve Pakistani readers. The solution must:
- Translate all 3 chapters (~15,000 words) to Urdu
- Support RTL text rendering
- Provide fast language switching (<2 seconds)
- Work with static hosting (GitHub Pages)
- Preserve code examples in original language

## Decision

We adopt a **Docusaurus native i18n approach with static files**:

| Component | Technology | Purpose |
|-----------|------------|---------|
| **i18n Framework** | Docusaurus built-in i18n | Locale routing, content loading |
| **Translation Storage** | Static MDX/JSON files | Version-controlled translations |
| **RTL Implementation** | CSS `dir="rtl"` + logical properties | Native browser rendering |
| **Typography** | Noto Nastaliq Urdu (Google Fonts) | Proper Nastaliq script |
| **Language Preference** | localStorage (syncs with auth profile) | Persistent preference |

### Implementation Details

**Docusaurus Configuration**:
```javascript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    ur: {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur-PK',
    },
  },
}
```

**File Structure**:
```
i18n/ur/
├── docusaurus-theme-classic/
│   ├── navbar.json
│   └── footer.json
└── docusaurus-plugin-content-docs/current/
    ├── chapter-1.mdx
    ├── chapter-2.mdx
    └── chapter-3.mdx
```

**RTL CSS Strategy**:
- `[dir="rtl"]` selectors for component adjustments
- CSS logical properties (`margin-inline-start`)
- Code blocks forced to LTR with `unicode-bidi: isolate`

**Quality Target**: 85% translation accuracy (SC-048)

## Consequences

### Positive

- **Zero Runtime Cost**: Static files served directly, no API calls
- **GitHub Pages Compatible**: Works with static hosting
- **Version Controlled**: Translations tracked with source code
- **Build-Time Validation**: Missing translations caught at build
- **Fast Switching**: Pre-rendered pages, <2s target achievable
- **Native RTL**: Browser handles RTL natively via `dir` attribute

### Negative

- **Manual Translation**: Requires human effort for quality translations
- **Large Font File**: Noto Nastaliq Urdu is ~2MB (mitigated by font-display: swap)
- **Duplicate Content**: Translations stored separately from source
- **Update Burden**: Content changes require re-translation

### Risks

- **Quality Variance**: Translation quality depends on human review
- **Font Loading**: Slow networks may show FOUT (mitigated by preload)

## Alternatives Considered

### Alternative 1: Translation API (Google Translate, DeepL)
- **Pros**: Automatic translation, dynamic updates
- **Cons**: API costs, latency, lower quality for technical content, not static-friendly

### Alternative 2: Custom i18n Implementation
- **Pros**: Full control
- **Cons**: Reinventing wheel, not integrated with Docusaurus

### Alternative 3: CMS-Based Translation (Crowdin, Lokalise)
- **Pros**: Collaboration tools, translation memory
- **Cons**: Additional service, sync complexity, cost

### Alternative 4: Runtime Translation Loading
- **Pros**: Smaller initial bundle
- **Cons**: Loading delay, requires API or dynamic imports, not suitable for GitHub Pages

## References

- `specs/005-translation/plan.md` - Full implementation plan
- `specs/005-translation/research.md` - Technology decisions
- `specs/005-translation/data-model.md` - Translation unit structures
- `.specify/memory/constitution.md` - RTL and Urdu requirements
