# Data Model: Physical AI Textbook Generation

**Feature**: 001-textbook-generation
**Date**: 2025-12-30

## Overview

This feature is primarily **content-driven** rather than data-driven. The "data model" represents the structure of MDX content files and their metadata, not database entities.

## Content Entities

### Chapter

A complete educational unit in MDX format.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | string | unique, slug format | URL-safe identifier (e.g., "chapter-1") |
| title | string | required, max 100 chars | Display title |
| description | string | required, max 200 chars | Meta description for SEO |
| sidebar_position | integer | 1-3 | Order in navigation |
| word_count | integer | 4,500-6,000 | Validated at build time |
| learning_objectives | string[] | min 3, max 7 | Bulleted list at chapter start |
| key_takeaways | string[] | min 3, max 7 | Bulleted list at chapter end |
| diagrams | Diagram[] | min 2 | Embedded illustrations |
| code_examples | CodeExample[] | min 1 | Syntax-highlighted blocks |

**Frontmatter Schema** (MDX):
```yaml
---
id: chapter-1
title: "Introduction to Physical AI & Robotics"
description: "Learn the fundamentals of embodied AI and ROS 2"
sidebar_position: 1
keywords: [physical ai, robotics, ros2, embodied intelligence]
---
```

### Diagram

A visual illustration within a chapter.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| type | enum | mermaid, image | Rendering method |
| src | string | required if image | Path to image file |
| alt | string | required, min 20 chars | Accessibility description |
| caption | string | optional | Display caption below |
| width | string | optional | CSS width value |

**Mermaid Example**:
```markdown
```mermaid
graph LR
    A[Sensor] --> B[Perception]
    B --> C[Planning]
    C --> D[Action]
    D --> E[Actuator]
```
```

**Image Example**:
```markdown
![ROS 2 Architecture showing nodes, topics, and services](/img/ros2-architecture.png)
*Figure 1.1: ROS 2 node communication architecture*
```

### CodeExample

A syntax-highlighted code block.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| language | string | python, bash, xml, yaml | Syntax highlighting language |
| source_file | string | optional | Path to tested source file |
| title | string | optional | Display title above block |
| show_line_numbers | boolean | default: false | Line number display |
| highlight_lines | integer[] | optional | Lines to emphasize |

**Example**:
```markdown
```python title="examples/hello_ros.py" showLineNumbers
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```
```

### LearningObjective

A statement of what readers will learn.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| text | string | required, action verb start | Objective statement |
| chapter_id | string | FK to Chapter | Parent chapter |

**Format Convention**: Start with action verb (Understand, Implement, Configure, etc.)

**Example**:
```markdown
## Learning Objectives

By the end of this chapter, you will be able to:

- **Understand** the core concepts of Physical AI and embodied intelligence
- **Configure** a ROS 2 development environment
- **Implement** a basic publisher-subscriber pattern
```

### KeyTakeaway

A summary point capturing essential knowledge.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| text | string | required, max 150 chars | Concise summary point |
| chapter_id | string | FK to Chapter | Parent chapter |

**Example**:
```markdown
## Key Takeaways

- Physical AI combines perception, planning, and action in the real world
- ROS 2 provides a standardized framework for robot software development
- Simulation enables safe testing before deploying to physical hardware
```

## File Structure

```
docs/
├── intro.md                    # Book introduction/landing
├── chapter-1.mdx              # Chapter 1 content
├── chapter-2.mdx              # Chapter 2 content
├── chapter-3.mdx              # Chapter 3 content
└── glossary.md                # Technical terms reference

static/
└── img/
    ├── chapter-1/
    │   ├── ros2-architecture.png
    │   └── dev-setup.png
    ├── chapter-2/
    │   ├── gazebo-architecture.png
    │   └── urdf-structure.png
    └── chapter-3/
        ├── vla-pipeline.png
        └── perception-action.png

examples/
├── chapter-1/
│   ├── hello_ros.py
│   └── publisher_subscriber.py
├── chapter-2/
│   ├── spawn_robot.py
│   └── sensor_reader.py
└── chapter-3/
    ├── image_processor.py
    └── vla_inference.py
```

## Validation Rules

### Chapter Validation
- Word count MUST be 4,500-6,000 (SC-002)
- MUST have ≥2 diagrams (FR-003)
- MUST start with learning objectives (FR-004)
- MUST end with key takeaways (FR-005)
- All internal links MUST resolve (FR-011)
- All external links MUST be valid (FR-012)

### Diagram Validation
- Alt text MUST be ≥20 characters (FR-009)
- Image files MUST exist at specified path
- Mermaid syntax MUST be valid

### Code Example Validation
- If source_file specified, file MUST exist
- Code MUST pass syntax check for declared language
- Python code MUST pass pytest (SC-004)

## State Transitions

Content follows a simple publication workflow:

```
Draft → Review → Published
  ↑       ↓
  └── Revision ←┘
```

| State | Description | Validation |
|-------|-------------|------------|
| Draft | Initial content creation | None |
| Review | Content complete, under review | Word count, diagrams present |
| Published | Live on site | All validation rules pass |
| Revision | Updates to published content | Full re-validation required |

## Relationships

```
Chapter (1) ──────< (many) Diagram
    │
    ├──────< (many) CodeExample
    │
    ├──────< (many) LearningObjective
    │
    └──────< (many) KeyTakeaway
```

All relationships are **composition** - child entities exist only within their parent chapter MDX file.
