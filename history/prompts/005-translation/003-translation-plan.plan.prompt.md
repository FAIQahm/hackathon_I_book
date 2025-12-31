---
id: 003
title: Translation Technical Plan Creation
stage: plan
date: 2025-12-31
surface: agent
model: claude-opus-4-5-20250514
feature: 005-translation
branch: 005-translation
user: PMLS
command: /sp.plan 005-translation
labels: ["plan", "architecture", "translation", "i18n", "rtl", "urdu"]
links:
  spec: specs/005-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-translation/plan.md
  - specs/005-translation/research.md
  - specs/005-translation/data-model.md
  - specs/005-translation/quickstart.md
tests:
  - N/A (planning phase)
---

## Prompt

/sp.plan 005-translation

Execute the implementation planning workflow for the Urdu Translation & RTL Support feature using Docusaurus i18n framework with static file translations and Right-to-Left rendering.

## Response snapshot

Created comprehensive technical plan for 005-translation feature with complete Phase 1 artifacts.

**Architecture**: Static site with Docusaurus i18n
- Frontend: Docusaurus with i18n locale routing
- Storage: Static MDX files for content, JSON for UI strings
- No backend API needed (GitHub Pages compatible)

**Implementation Phases**: 6 phases
1. Docusaurus i18n Configuration (locale setup, directory structure)
2. RTL Styling & Typography (CSS, Noto Nastaliq Urdu font)
3. UI String Translation (navbar, footer, sidebar JSON)
4. Chapter Content Translation (3 chapters MDX)
5. Language Toggle Component (React component, scroll preservation)
6. Translation Quality Assurance (85% target, native speaker review)

**Technology Decisions** (research.md):
- Framework: Docusaurus native i18n
- RTL: CSS `dir="rtl"` with logical properties
- Font: Noto Nastaliq Urdu (Google Fonts)
- Storage: Static files in i18n/ directory
- Preference: localStorage (syncs with 003-auth when logged in)
- Code blocks: Force LTR with CSS isolation

**Data Model** (data-model.md):
- Translation Unit (file-based, MDX/JSON)
- Language Preference (localStorage)
- Bilingual Content Block
- Technical Term Glossary
- Quality Review Record

**No API Contract**: Feature uses static files, no backend API needed

**Design Decisions**: 8 key decisions
- D1: Docusaurus native i18n (no external service)
- D2: Static file translations (version controlled)
- D3: Noto Nastaliq Urdu font (free, proper Nastaliq)
- D4: CSS-based RTL (no JS overhead)
- D5: localStorage preference (works without auth)
- D6: Code block LTR preservation
- D7: AI translation + human review (quality process)
- D8: Percentage-based scroll preservation

## Outcome

- Impact: Complete technical blueprint for Urdu translation and RTL implementation
- Tests: N/A (planning phase)
- Files: plan.md, research.md, data-model.md, quickstart.md created
- Next prompts: /sp.tasks to generate actionable task list
- Reflection: Plan addresses all success criteria (SC-047 to SC-056) with clear mappings

## Evaluation notes (flywheel)

- Failure modes observed: None - straightforward static site feature
- Graders run and results (PASS/FAIL): N/A (planning)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
