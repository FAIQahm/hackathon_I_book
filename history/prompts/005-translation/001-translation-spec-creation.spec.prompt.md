---
id: 001
title: Translation Specification Creation
stage: spec
date: 2025-12-30
surface: agent
model: claude-opus-4-5-20251101
feature: 005-translation
branch: 005-translation
user: PMLS
command: /sp.specify 005-translation.md
labels: ["specification", "translation", "urdu", "rtl", "i18n", "localization"]
links:
  spec: specs/005-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-translation/spec.md
  - specs/005-translation/checklists/requirements.md
tests:
  - N/A (specification document)
---

## Prompt

/sp.specify 005-translation.md

## Response snapshot

Created feature specification for Urdu Translation & RTL Support:

**Branch**: `005-translation`
**Spec File**: `specs/005-translation/spec.md`

**User Stories (4)**:
1. P1: Read Chapter in Urdu - Full chapter translation with RTL rendering
2. P2: Switch Between Languages - Toggle between English and Urdu with scroll preservation
3. P3: View Code Examples with Translated Context - Code untranslated, explanations translated
4. P4: Navigate with Translated Interface - All UI elements in Urdu

**Functional Requirements (15)**: FR-074 through FR-088 covering:
- Complete Urdu translation for all 3 chapters
- RTL text rendering
- Language toggle with 2-second switch time
- Scroll position preservation
- Session persistence for language preference
- Navigation UI translation
- Code example handling (code unchanged, context translated)
- Mixed LTR/RTL content handling
- 85%+ translation accuracy

**Translation Scope**:
- MUST translate: Chapter content, navigation UI, metadata, supporting text, error messages
- MUST NOT translate: Code examples, technical commands, diagrams with text, chatbot responses

**Success Criteria (10)**: SC-047 through SC-056 with measurable outcomes

**Constitution Alignment**: All translation requirements verified (Full Urdu, RTL, 85%+ quality)

**Quality Checklist**: All 16 items passed validation

## Outcome

- Impact: Defined complete specification for Urdu translation and RTL support
- Tests: N/A (specification document)
- Files: spec.md, checklists/requirements.md created
- Next prompts: /sp.plan to create implementation plan
- Reflection: Clear scope boundaries for what to translate vs preserve, chatbot explicitly out of scope

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Quality checklist PASS (16/16), Constitution alignment PASS (4/4)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Confirm native Urdu speaker availability during planning phase
