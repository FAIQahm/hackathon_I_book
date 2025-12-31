---
name: {{SKILL_NAME}}
description: |
  {{SKILL_DESCRIPTION}}
version: {{SKILL_VERSION}}
inputs:
  example_input:
    description: Example input parameter
    required: false
    default: "default_value"
    example: "example_value"
---

# {{SKILL_TITLE}}

{{SKILL_DESCRIPTION}}

## Quick Setup

Run from your project root:

```bash
# Option 1: Run setup script
.claude/skills/{{SKILL_NAME}}/scripts/setup.sh

# Option 2: Manual setup
# (Add manual steps here)
```

## Bundled Resources

### 1. Main Resource

**File**: `path/to/file`

```
# Add resource content or code here
```

### 2. Input Variables

| Variable | Required | Default | Description | Example |
|----------|----------|---------|-------------|---------|
| `example_input` | No | `default_value` | Description | `example_value` |

## Usage Instructions

### Step 1: Setup

```bash
.claude/skills/{{SKILL_NAME}}/scripts/setup.sh
```

### Step 2: Configure

Add any configuration steps here.

### Step 3: Verify

Verify the setup worked correctly.

## Verification Checklist

- [ ] Step 1 completed
- [ ] Step 2 completed
- [ ] Step 3 completed

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **Problem 1** | Solution 1 |
| **Problem 2** | Solution 2 |

## Requirements

- Requirement 1
- Requirement 2

## Related

- Link to related docs
- Link to related skills
