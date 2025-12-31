# Skill Templates

Reusable bundled resources for common development tasks.

## Skill Structure

Every skill has two parts:

### Part 1: YAML Frontmatter

```yaml
---
name: skill-id
description: |
  [Action verb] + [input type] + [to/for] + [output type] + [key features].
  Use when [trigger conditions].
version: 1.0.0
---
```

### Part 2: Markdown Body (Instructions)

- Bundled Resources (code templates, config files)
- Usage Instructions (step-by-step guide)
- Verification Checklist
- Troubleshooting

## Available Skills

| Name | Description | Version |
|------|-------------|---------|
| `github-pages-deploy` | Deploy Docusaurus site to GitHub Pages for automated CI/CD. Use when setting up new project or adding deployment automation. | 1.0.0 |

## Adding New Skills

1. Create file: `.specify/templates/skills/<skill-id>.md`
2. Add YAML frontmatter with `name`, `description`, `version`
3. Write markdown body with instructions
4. Update this README

### Description Formula

```
[Action verb] + [input type] + [to/for] + [output type] + [key features].
Use when [trigger conditions].
```

**Example**:
> Deploy Docusaurus static site to GitHub Pages for automated CI/CD pipeline.
> Use when setting up new Docusaurus project, adding CI/CD, or migrating to automated deployment.
