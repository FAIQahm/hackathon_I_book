---
name: github-pages-deploy
description: Deploy Docusaurus static site to GitHub Pages for automated CI/CD pipeline with i18n support. Use when (1) setting up a new Docusaurus project for GitHub Pages, (2) adding CI/CD deployment to an existing site, (3) configuring multi-locale builds (en/ur), or (4) migrating from manual to automated deployment.
---

# GitHub Pages Deploy

Deploy Docusaurus to GitHub Pages with GitHub Actions and i18n support.

## Quick Setup

Run the setup script to create all deployment files:

```bash
.claude/skills/github-pages-deploy/scripts/setup.sh <organization> <repository>
```

Example:
```bash
.claude/skills/github-pages-deploy/scripts/setup.sh my-org physical-ai-book
```

## Manual Setup

If you prefer manual setup or need to customize:

### 1. Create Workflow

Copy `assets/deploy.yml` to `.github/workflows/deploy.yml`

### 2. Update Docusaurus Config

Add to `docusaurus.config.js`:

```javascript
module.exports = {
  url: 'https://<organization>.github.io',

  // ⚠️ CRITICAL: baseUrl MUST have BOTH leading AND trailing slashes!
  // ✅ Correct: '/<repository>/'
  // ❌ Wrong:   '<repository>/'   (missing leading slash → 404 errors)
  // ❌ Wrong:   '/<repository>'   (missing trailing slash → 404 errors)
  // ❌ Wrong:   'repository'      (missing both → 404 errors)
  baseUrl: '/<repository>/',

  organizationName: '<organization>',
  projectName: '<repository>',
  trailingSlash: false,

  // Broken link detection
  onBrokenLinks: 'throw',

  // Markdown configuration (Docusaurus v3.9+)
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // i18n for multi-locale builds (requires NODE_OPTIONS for memory)
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: { label: 'English', direction: 'ltr', htmlLang: 'en-US' },
      ur: { label: 'اردو', direction: 'rtl', htmlLang: 'ur-PK' },
    },
  },
};
```

> **⚠️ Common 404 Cause:** Missing leading or trailing slash in `baseUrl`. Always use `'/<repo>/'` format.

### 3. Create Homepage Redirect

Create `src/pages/index.js` to redirect to docs:

```javascript
import React from 'react';
import {Redirect} from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function Home() {
  return <Redirect to={useBaseUrl('/docs')} />;
}
```

> **Important:** Always use `useBaseUrl()` for redirects to prevent double-slash issues with baseUrl.

### 4. Create i18n Scaffolding (for Urdu locale)

```bash
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current
mkdir -p i18n/ur/docusaurus-theme-classic
```

Create `i18n/ur/docusaurus-theme-classic/navbar.json`:
```json
{
  "title": {
    "message": "Your Title in Urdu",
    "description": "The title in the navbar"
  }
}
```

Create `i18n/ur/docusaurus-theme-classic/footer.json`:
```json
{
  "copyright": {
    "message": "کاپی رائٹ © 2025",
    "description": "The footer copyright"
  }
}
```

Copy and translate docs to `i18n/ur/docusaurus-plugin-content-docs/current/`.

### 5. Add RTL Support

Add to `src/css/custom.css`:

```css
/* RTL Support for Urdu */
[dir='rtl'] {
  text-align: right;
}

[dir='rtl'] .navbar__items {
  flex-direction: row-reverse;
}

/* Code blocks stay LTR */
[dir='rtl'] pre,
[dir='rtl'] code {
  direction: ltr;
  text-align: left;
}
```

### 6. Enable GitHub Pages

1. Repository **Settings** → **Pages**
2. Source: **GitHub Actions**
3. Save

### 7. Deploy

```bash
git add . && git commit -m "feat: add GitHub Pages deployment" && git push
```

## Verification Checklist

- [ ] `.github/workflows/deploy.yml` exists with `node-version: 20`
- [ ] `.github/workflows/deploy.yml` has `NODE_OPTIONS: --max-old-space-size=4096`
- [ ] `docusaurus.config.js` has url, baseUrl, organizationName, projectName
- [ ] `baseUrl` has BOTH leading AND trailing slashes: `'/<repo>/'`
- [ ] `src/pages/index.js` uses `useBaseUrl()` for redirect (not hardcoded path)
- [ ] `markdown.hooks.onBrokenMarkdownLinks` (not root-level config)
- [ ] GitHub Pages source set to "GitHub Actions"
- [ ] `i18n/ur/` directory exists with translated content
- [ ] Site accessible at `https://<org>.github.io/<repo>/`
- [ ] Urdu site at `https://<org>.github.io/<repo>/ur/`

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **Build fails on Node version** | Ensure `node-version: 20` in deploy.yml (Docusaurus 3.9+ dropped Node 18 support) |
| **404 on all pages** | Check `baseUrl` has BOTH leading AND trailing slashes: `'/<repo>/'` |
| **404 on homepage** | Create `src/pages/index.js` with redirect using `useBaseUrl()` |
| **Double slashes in URL** | Use `useBaseUrl()` hook, not hardcoded paths |
| **404 on /ur/ locale** | Create `i18n/ur/` folder with translated docs |
| **Deprecation warnings** | Move `onBrokenMarkdownLinks` to `markdown.hooks` |
| **Memory errors (i18n builds)** | Set `NODE_OPTIONS=--max-old-space-size=4096` (essential for multi-locale) |

## Requirements

- **Node.js 20+** (required - Docusaurus 3.9+ dropped Node 18 support)
- Docusaurus 3.9+
- GitHub repository with Pages enabled

## Environment Variables

The workflow includes essential environment variables:

```yaml
env:
  NODE_OPTIONS: --max-old-space-size=4096  # Required for i18n/Urdu builds
```

This is critical for multi-locale builds which consume significantly more memory during compilation.
