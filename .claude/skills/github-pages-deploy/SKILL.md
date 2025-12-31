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
  baseUrl: '/<repository>/',
  organizationName: '<organization>',
  projectName: '<repository>',
  trailingSlash: false,

  // i18n for multi-locale builds
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: { label: 'English', direction: 'ltr' },
      ur: { label: 'اردو', direction: 'rtl' },
    },
  },
};
```

### 3. Enable GitHub Pages

1. Repository **Settings** → **Pages**
2. Source: **GitHub Actions**
3. Save

### 4. Deploy

```bash
git add . && git commit -m "feat: add GitHub Pages deployment" && git push
```

## Verification

- [ ] `.github/workflows/deploy.yml` exists
- [ ] `docusaurus.config.js` has url, baseUrl, organizationName, projectName
- [ ] GitHub Pages source set to "GitHub Actions"
- [ ] Site accessible at `https://<org>.github.io/<repo>/`
- [ ] Urdu site at `https://<org>.github.io/<repo>/ur/`

## Troubleshooting

**Build fails**: Run `npm run build` locally first. For memory errors: `NODE_OPTIONS=--max-old-space-size=4096 npm run build`

**404 errors**: Check baseUrl matches repository name with slashes: `/<repo>/`

**i18n issues**: Verify `i18n/ur/` directory exists with translated content.
