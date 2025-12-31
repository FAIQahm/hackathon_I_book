---
name: github-pages-deploy
description: |
  Deploy Docusaurus static site to GitHub Pages for automated CI/CD pipeline.
  Bundled resources: GitHub Actions workflow, docusaurus.config.js deployment block.
  Use when setting up new Docusaurus project, adding CI/CD, or migrating to automated deployment.
version: 1.1.0
inputs:
  organization:
    description: GitHub username or organization
    required: true
    example: my-org
  repository:
    description: Repository name
    required: true
    example: my-docs
  node_version:
    description: Node.js version for build
    required: false
    default: "18"
  locales:
    description: Locales to build (comma-separated or "all")
    required: false
    default: "all"
    example: "en,ur"
---

# GitHub Pages Deploy

## Quick Setup

Run this command from your project root to set up deployment:

```bash
# Create workflow directory
mkdir -p .github/workflows

# Create deploy.yml
cat << 'EOF' > .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      - name: Build website (all locales)
        run: npm run build

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
EOF

echo "✅ Created .github/workflows/deploy.yml"
echo "📝 Now update docusaurus.config.js with your organization and repository names"
```

## Bundled Resources

### 1. GitHub Actions Workflow (with i18n support)

**File**: `.github/workflows/deploy.yml`

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

permissions:
  contents: read
  pages: write
  id-token: write

concurrency:
  group: "pages"
  cancel-in-progress: false

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci

      # Builds all configured locales (en, ur, etc.)
      # Docusaurus automatically builds all locales defined in docusaurus.config.js
      - name: Build website (all locales)
        run: npm run build
        env:
          NODE_OPTIONS: --max-old-space-size=4096

      - name: Upload artifact
        uses: actions/upload-pages-artifact@v3
        with:
          path: build

  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    needs: build
    steps:
      - name: Deploy to GitHub Pages
        id: deployment
        uses: actions/deploy-pages@v4
```

**i18n Notes**:
- Docusaurus builds ALL locales defined in `docusaurus.config.js` by default
- To build specific locale only: `npm run build -- --locale en`
- To build multiple: `npm run build -- --locale en --locale ur`
- `NODE_OPTIONS` increases memory for multi-locale builds

### 2. Docusaurus Configuration Block

**File**: `docusaurus.config.js` (merge into existing config)

```javascript
const config = {
  // ===========================================
  // GitHub Pages Deployment Configuration
  // ===========================================

  // Replace <organization> with your GitHub username or org name
  url: 'https://<organization>.github.io',

  // Replace <repository> with your repository name
  // Use '/' if deploying to <organization>.github.io (user/org site)
  baseUrl: '/<repository>/',

  // GitHub Pages deployment settings
  organizationName: '<organization>', // GitHub org/user name
  projectName: '<repository>',        // Repository name
  trailingSlash: false,
  deploymentBranch: 'gh-pages',

  // Recommended: Fail build on broken links
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // ===========================================
  // i18n Configuration (for multi-locale builds)
  // ===========================================
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

  // ... rest of your config
};

module.exports = config;
```

### 3. Input Variables

| Variable | Required | Default | Description | Example |
|----------|----------|---------|-------------|---------|
| `organization` | Yes | - | GitHub username or org | `my-org` |
| `repository` | Yes | - | Repository name | `my-docs` |
| `node_version` | No | `18` | Node.js version | `20` |
| `locales` | No | `all` | Locales to build | `en,ur` |

## Usage Instructions

### Step 1: Run Quick Setup

```bash
# Option A: Run the quick setup script above
# Option B: Manually create files from Bundled Resources section
```

### Step 2: Update Docusaurus Config

Replace placeholders in `docusaurus.config.js`:

```javascript
// Before
url: 'https://<organization>.github.io',
baseUrl: '/<repository>/',
organizationName: '<organization>',
projectName: '<repository>',

// After (example)
url: 'https://my-org.github.io',
baseUrl: '/physical-ai-book/',
organizationName: 'my-org',
projectName: 'physical-ai-book',
```

### Step 3: Enable GitHub Pages

1. Go to repository **Settings** → **Pages**
2. Under "Build and deployment", select **Source**: `GitHub Actions`
3. Save changes

### Step 4: Push and Deploy

```bash
git add .
git commit -m "feat: add GitHub Pages deployment with i18n support"
git push origin main
```

The workflow will automatically build all locales and deploy.

## Verification Checklist

- [ ] `.github/workflows/deploy.yml` exists
- [ ] `docusaurus.config.js` has correct `url`, `baseUrl`, `organizationName`, `projectName`
- [ ] `docusaurus.config.js` has `i18n` config with all locales
- [ ] GitHub Pages is enabled in repository settings
- [ ] Source is set to "GitHub Actions"
- [ ] First deployment completed successfully
- [ ] English site accessible at `https://<org>.github.io/<repo>/`
- [ ] Urdu site accessible at `https://<org>.github.io/<repo>/ur/`

## Troubleshooting

### Build Fails

```bash
# Test build locally first
npm run build

# If memory error, increase heap size
NODE_OPTIONS=--max-old-space-size=4096 npm run build
```

### 404 Errors

- Check `baseUrl` matches repository name (with leading and trailing slashes)
- Verify `url` is correct for your organization

### Assets Not Loading

- Ensure `trailingSlash: false` is set
- Check browser console for path errors

### i18n Build Issues

```bash
# Build specific locale only (for debugging)
npm run build -- --locale en

# Check locale directory exists
ls -la i18n/ur/
```

### Urdu/RTL Not Rendering

- Verify `direction: 'rtl'` in locale config
- Check font imports in `src/css/custom.css`
- Ensure `i18n/ur/` directory has translated content

## Related

- [Docusaurus Deployment Docs](https://docusaurus.io/docs/deployment#deploying-to-github-pages)
- [Docusaurus i18n Docs](https://docusaurus.io/docs/i18n/introduction)
- [GitHub Pages Documentation](https://docs.github.com/en/pages)
- ADR-001: Deployment Infrastructure Stack
- ADR-003: Translation & Internationalization Approach
