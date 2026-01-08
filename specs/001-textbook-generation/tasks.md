# Tasks: 001-textbook-generation

**Feature**: Physical AI Textbook Content
**Agent**: @BookArchitect
**Skills**: docusaurus-scaffold, github-pages-deploy
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

---

## Phase 0: Infrastructure Setup

| ID | Task | Acceptance Criteria | Est. | Status |
|----|------|---------------------|------|--------|
| T-001 | Configure Docusaurus project with i18n | `npm run build` succeeds with en/ur locales | 20m | [X] |
| T-002 | Set up GitHub Actions deploy workflow | Push to main triggers deployment | 15m | [X] |

**CHECKPOINT 0**: [ ] Docusaurus builds and deploys successfully

---

## Phase 1: Chapter Content (US-001 - P1)

| ID | Task | Acceptance Criteria | Est. | Status |
|----|------|---------------------|------|--------|
| T-010 | Create Chapter 1: Intro to Physical AI & ROS 2 | 4500-6000 words, learning objectives, 2+ diagrams | 30m | [X] |
| T-011 | Create Chapter 2: Simulation with Gazebo | 4500-6000 words, learning objectives, 2+ diagrams | 30m | [X] |
| T-012 | Create Chapter 3: Vision-Language-Action Models | 4500-6000 words, learning objectives, 2+ diagrams | 30m | [X] |
| T-013 | Add key takeaways to all chapters | Each chapter ends with summary section | 20m | [X] |
| T-014 | Add content metadata (topic, difficulty) | `content-schema.json` validation passes | 20m | [X] |
| T-015 | Add alt text to all diagrams | Every `<img>` has descriptive alt attribute | 15m | [X] N/A - mermaid diagrams |

**CHECKPOINT 1**: [X] All 3 chapters have content with learning objectives

---

## Phase 2: Navigation (US-002 - P2)

| ID | Task | Acceptance Criteria | Est. | Status |
|----|------|---------------------|------|--------|
| T-020 | Implement next/previous navigation | Links work on all chapter pages | 15m | [ ] |
| T-021 | Create table of contents sidebar | All 3 chapters accessible from any page | 15m | [ ] |
| T-022 | Verify all internal links resolve | 0 broken links in `npm run build` | 15m | [ ] |

**CHECKPOINT 2**: [ ] Navigation works across all chapters

---

## Phase 3: Code Examples (US-003 - P3)

| ID | Task | Acceptance Criteria | Est. | Status |
|----|------|---------------------|------|--------|
| T-030 | Add syntax highlighting to code blocks | Prism.js renders Python/JS code | 15m | [ ] |
| T-031 | Add context explanations to code examples | Each code block has surrounding text | 20m | [ ] |
| T-032 | Verify code examples execute | CI test passes with Python 3.11 | 20m | [ ] |

**CHECKPOINT 3**: [ ] Code examples render correctly and execute

---

## Phase 4: Quality & Deployment

| ID | Task | Acceptance Criteria | Est. | Status |
|----|------|---------------------|------|--------|
| T-040 | Run Lighthouse accessibility audit | Score > 95 on all pages | 20m | [ ] |
| T-041 | Verify responsive design | 375px, 768px, 1440px all work | 15m | [ ] |
| T-042 | Deploy to GitHub Pages | Site accessible at production URL | 15m | [ ] |

**FINAL CHECKPOINT**: [ ] All SC-001 to SC-011 verified

---

## Success Criteria Mapping

| SC | Requirement | Task |
|----|-------------|------|
| SC-001 | 3 chapters published, 0 broken links | T-010,011,012,022 |
| SC-002 | Word count 4500-6000 per chapter | T-010,011,012 |
| SC-003 | 2+ diagrams per chapter | T-010,011,012 |
| SC-004 | Code examples execute | T-032 |
| SC-005 | Grammarly 0 critical errors | Manual check |
| SC-006 | < 3s page load | T-042 |
| SC-007 | Lighthouse > 95 | T-040 |
| SC-008 | Any chapter in < 3 clicks | T-020,021 |
| SC-009 | Responsive on 3 viewports | T-041 |
| SC-010 | Metadata validation passes | T-014 |

---

*Generated for 001-textbook-generation - 2026-01-04*
