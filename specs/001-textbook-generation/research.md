# Research: Physical AI Textbook Generation

**Feature**: 001-textbook-generation
**Date**: 2025-12-30
**Status**: Complete

## Research Questions

### 1. Documentation Framework Selection

**Decision**: Docusaurus 3.x

**Rationale**:
- Constitution mandates Docusaurus for documentation framework
- Native MDX support for rich content (diagrams, code blocks)
- Built-in syntax highlighting via Prism
- GitHub Pages deployment supported out of the box
- Excellent SEO and performance (easily achieves Lighthouse >95)
- Built-in navigation (sidebar, next/previous)

**Alternatives Considered**:
- VitePress: Good performance but constitution specifies Docusaurus
- GitBook: Proprietary, not aligned with open-source approach
- MkDocs: Python-based, less React ecosystem integration

### 2. Chapter Topic Selection (Physical AI & Robotics)

**Decision**: Three progressive chapters covering Physical AI fundamentals

**Chapter 1: Introduction to Physical AI & Robotics**
- What is Physical AI (embodied intelligence)
- ROS 2 fundamentals and architecture
- Setting up a development environment
- Word target: 5,000 words

**Chapter 2: Simulation & Digital Twins**
- Gazebo simulation environment
- URDF robot descriptions
- Sensor simulation and data
- Word target: 5,250 words

**Chapter 3: Vision-Language-Action Models**
- Computer vision in robotics
- Language models for robot control
- VLA architectures and applications
- Word target: 5,500 words

**Rationale**:
- Progressive complexity from fundamentals to advanced
- Covers simulation (required for learning without hardware)
- Includes cutting-edge VLA models (timely and relevant)
- Each chapter builds on previous knowledge

**Alternatives Considered**:
- Hardware-focused chapters: Excluded per constitution constraints
- Navigation/SLAM: Too specialized for introductory book
- Multi-robot systems: Too advanced for 3-chapter scope

### 3. Diagram Generation Approach

**Decision**: AI-generated diagrams using Mermaid + static images

**Rationale**:
- Mermaid diagrams render natively in Docusaurus
- Architecture diagrams can be version-controlled as code
- Static PNG/SVG for complex illustrations
- Alt text easily added for accessibility (FR-009)

**Diagram Plan**:
- Chapter 1: ROS 2 node communication diagram, development setup diagram
- Chapter 2: Gazebo architecture diagram, URDF structure diagram
- Chapter 3: VLA pipeline diagram, perception-action loop diagram

**Alternatives Considered**:
- Draw.io exports: Manual process, harder to maintain
- D3.js interactive: Excluded per constitution (no interactive visualizations)
- Excalidraw: Good for sketches but less professional

### 4. Code Example Testing Strategy

**Decision**: Extracted code blocks tested in CI with pytest

**Rationale**:
- Constitution requires all code be tested and working
- Python 3.11 as specified in SC-004
- pytest for consistency with Python ecosystem
- Frozen requirements.txt for reproducibility

**Implementation**:
- Code examples stored in `/examples/` directory
- MDX imports from tested source files
- CI runs pytest on all examples before build
- Dependencies pinned in requirements.txt

**Alternatives Considered**:
- Inline code only: Can't be tested automatically
- Jupyter notebooks: Heavier infrastructure, harder to embed
- Docker-based testing: Overkill for documentation examples

### 5. Content Quality Assurance

**Decision**: Automated + manual quality checks

**Tooling**:
- Word count: Custom script or docusaurus plugin
- Grammar: Grammarly CLI or LanguageTool integration
- Link checking: docusaurus build --check-links
- Accessibility: Lighthouse CI + axe-core

**Rationale**:
- SC-005 requires Grammarly check with <5 warnings
- SC-007 requires Lighthouse >95 and axe-core pass
- Automated checks catch issues before manual review

### 6. Navigation Implementation

**Decision**: Docusaurus native sidebar + pagination

**Rationale**:
- Docusaurus provides sidebar configuration
- Next/Previous navigation built-in
- Table of contents auto-generated from headings
- SC-008 requires <3 clicks to any chapter (achievable with sidebar)

**Configuration**:
```javascript
// sidebars.js
module.exports = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapters',
      items: ['chapter-1', 'chapter-2', 'chapter-3'],
    },
  ],
};
```

### 7. Responsive Design Approach

**Decision**: Docusaurus default theme + CSS customization

**Rationale**:
- Default theme is mobile-responsive
- SC-009 requires 3 viewport sizes (375px, 768px, 1440px)
- Minimal CSS needed for content-focused site
- 16px minimum font size enforced via CSS variables

**Breakpoints**:
- Mobile: 375px (iPhone SE)
- Tablet: 768px (iPad)
- Desktop: 1440px (standard laptop)

## Dependencies

### Production Dependencies
- `@docusaurus/core`: ^3.0.0
- `@docusaurus/preset-classic`: ^3.0.0
- `prism-react-renderer`: Syntax highlighting
- `@mdx-js/react`: MDX support

### Development Dependencies
- `pytest`: Code example testing
- `ruff`: Python linting (PEP 8)
- `eslint`: JavaScript linting (Airbnb)
- `lighthouse`: Performance auditing

## Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Word count exceeds 6,000 | Medium | Low | Edit ruthlessly; move details to appendix |
| Code examples outdated | Low | High | Pin all versions; test in CI |
| Diagrams don't load | Low | Medium | Alt text fallback; multiple formats |
| Lighthouse <95 | Low | Medium | Optimize images; lazy load below fold |

## Open Questions (Resolved)

All research questions resolved. No NEEDS CLARIFICATION remaining.
