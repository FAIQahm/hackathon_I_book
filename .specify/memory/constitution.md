<!--
SYNC IMPACT REPORT
==================
Version change: 0.0.0 → 1.0.0 (MAJOR: Initial constitution ratification)

Modified principles: N/A (initial version)
Added sections:
  - Core Principles (5 principles)
  - Key Standards
  - Constraints
  - Success Criteria (Required Deliverables + Definition of Done)
  - Governance
Removed sections: None

Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ Compatible (Constitution Check section exists)
  - .specify/templates/spec-template.md: ✅ Compatible (Requirements/Success Criteria align)
  - .specify/templates/tasks-template.md: ✅ Compatible (Phase structure supports user stories)

Follow-up TODOs: None
==================
-->

# Physical AI Educational Book Constitution

## Core Principles

### I. Educational Excellence

Content MUST be accurate, clear, and pedagogically sound with real-world examples.

- All technical explanations MUST include practical, runnable examples
- Concepts MUST progress from fundamentals to advanced topics logically
- Each chapter MUST include learning objectives and key takeaways
- Code examples MUST be tested and verified working before publication
- Content MUST avoid jargon without explanation; define terms on first use

**Rationale**: Educational content that confuses or misleads undermines the core mission. Real-world examples cement understanding and demonstrate practical applicability.

### II. Technical Accuracy

All code MUST be tested and working; no placeholder content is permitted.

- Every code snippet MUST be extracted from tested source files
- API integrations MUST be verified against live endpoints
- Configuration examples MUST reflect actual deployment requirements
- Error handling MUST be demonstrated, not implied
- Version numbers and dependencies MUST be pinned and current

**Rationale**: Placeholder or untested code erodes trust and wastes reader time debugging examples that should work out of the box.

### III. AI-Native Development

Leverage Claude Code skills and agents throughout the development process.

- MUST use minimum 4 skills for development workflow
- MUST use minimum 3 agents for autonomous task execution
- All skill and agent usage MUST be documented with Git evidence
- Development artifacts MUST be generated through AI-assisted workflows
- PHRs MUST capture all significant AI interactions

**Rationale**: This project demonstrates AI-native development; dogfooding our own tools validates their effectiveness and provides authentic documentation.

### IV. User-Centric Design

Interface MUST be fast, accessible, and mobile-friendly.

- Lighthouse performance score MUST exceed 95
- All pages MUST be responsive across mobile, tablet, and desktop
- RAG chatbot MUST respond within 2 seconds (p95)
- Cached responses MUST return within 1 second
- Accessibility MUST meet WCAG 2.1 AA standards minimum
- RTL rendering MUST be supported for Urdu translation

**Rationale**: Poor performance and inaccessibility exclude users. A technical book about AI should exemplify modern web standards.

### V. Quality Over Quantity

Three excellent chapters beat five mediocre ones.

- Each chapter MUST be 4,500-6,000 words
- Each chapter MUST include minimum 2 diagrams
- Zero broken links or images permitted
- All content MUST pass Grammarly or equivalent quality check
- Chatbot accuracy MUST exceed 90% on test queries
- No hallucinated responses from RAG system permitted

**Rationale**: Depth and correctness provide more value than breadth. Rushed content accumulates technical debt that costs more to fix than to prevent.

## Key Standards

### Content Standards

- **Chapters**: 4,500-6,000 words each, 2+ diagrams, learning objectives + key takeaways
- **Topic**: Physical AI & Robotics (3 chapters required)
- **Languages**: English primary, Urdu translation with RTL support (85%+ quality)

### Technical Standards

- **Framework**: Docusaurus deployed to GitHub Pages
- **Chatbot**: OpenAI GPT-4o-mini, Qdrant Cloud for vector storage
- **Database**: Neon Postgres for structured data
- **Auth**: Better-Auth with 8-10 signup questions
- **Performance**: Lighthouse >95, mobile responsive
- **Code Style**: PEP 8 (Python), Airbnb (JavaScript/TypeScript)

### Integration Standards

- Environment variables for all secrets (no hardcoded tokens)
- Structured error handling with user-friendly messages
- npm audit clean (no known vulnerabilities)
- 0 linting errors before deployment

## Constraints

### Must Have

- 3 chapters on Physical AI & Robotics
- RAG chatbot embedded in documentation
- GitHub Pages deployment
- Submission by Nov 30, 2025 6:00 PM

### Tech Stack (Mandatory)

- Docusaurus (documentation framework)
- OpenAI GPT-4o-mini (chatbot inference)
- Qdrant Cloud (vector database)
- Neon Postgres (relational database)
- Better-Auth (authentication - optional but recommended)

### Cannot Include

- Videos or multimedia beyond images/diagrams
- Hardware setup guides or physical component instructions
- Paid tier features or premium content gates
- Languages beyond English and Urdu

## Success Criteria

### Required Deliverables

| Deliverable | Specification |
|-------------|--------------|
| Book | 3 chapters, professional formatting, diagrams, working code, zero broken links |
| Chatbot | Embedded, 90%+ accuracy, <2s response, handles text-selection, no hallucinations |
| Skills & Agents | 4+ skills, 3+ agents, documented, Git evidence |
| Auth | Better-Auth, 8-10 signup questions, secure |
| Personalization | 5+ dimensions, <4s response, <1s cached |
| Translation | Full Urdu, RTL rendering, 85%+ quality |

### Definition of Done

All items MUST be checked before submission:

- [ ] 3 chapters (4,500-6,000 words each), >90% chatbot accuracy, <2s response
- [ ] Skills: 4+, Agents: 3+, Auth: Better-Auth with 8-10 questions
- [ ] Personalization: 5+ dimensions, Translation: Full Urdu with RTL
- [ ] GitHub Pages deployed, Lighthouse >95, mobile responsive
- [ ] Zero broken links/images, all code tested
- [ ] No hardcoded secrets, 0 linting errors, security scan passed
- [ ] README with setup, 90-second demo video, submission form complete

## Testing Checklist

Week 4 final verification (before submission):

### Content Verification

- [ ] Word count within range (4,500-6,000 per chapter)
- [ ] All code examples tested and working
- [ ] All links verified (internal and external)
- [ ] Grammar and style check passed

### Performance Verification

- [ ] Lighthouse score >95 (Performance, Accessibility, Best Practices, SEO)
- [ ] 3G network simulation test passed
- [ ] Mobile responsive (3 viewport sizes minimum)

### Chatbot Verification

- [ ] 20 test questions answered correctly
- [ ] Response times <2s (p95)
- [ ] Citations accurate and relevant
- [ ] No hallucinated content

### Security Verification

- [ ] npm audit clean
- [ ] No secrets in repository
- [ ] CORS properly configured

### Browser Verification

- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest)
- [ ] Edge (latest)
- [ ] iOS Safari
- [ ] Android Chrome

## Governance

This constitution supersedes all other practices and guidelines for the Physical AI Educational Book project.

### Amendment Procedure

1. Proposed amendments MUST be documented with rationale
2. Amendments MUST NOT weaken quality standards without explicit justification
3. All amendments MUST be reflected in updated version number
4. Dependent templates MUST be reviewed for consistency after amendment

### Versioning Policy

- **MAJOR**: Backward incompatible changes to principles or removal of requirements
- **MINOR**: New sections added or existing guidance materially expanded
- **PATCH**: Clarifications, typo fixes, non-semantic refinements

### Compliance Review

- All PRs MUST be verified against constitution principles before merge
- Constitution violations MUST be resolved before deployment
- Complexity additions MUST be justified against "Quality Over Quantity" principle

**Version**: 1.0.0 | **Ratified**: 2025-12-30 | **Last Amended**: 2025-12-30
