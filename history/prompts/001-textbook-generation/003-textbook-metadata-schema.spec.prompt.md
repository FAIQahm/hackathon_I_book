---
id: 003
title: Add Personalization Metadata Schema
stage: spec
date: 2025-12-30
surface: agent
model: claude-opus-4-5-20251101
feature: 001-textbook-generation
branch: 001-textbook-generation
user: PMLS
command: Update spec for 004-personalization dependency
labels: ["metadata", "personalization", "content-schema", "cross-feature"]
links:
  spec: specs/001-textbook-generation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-textbook-generation/spec.md
  - specs/001-textbook-generation/contracts/content-schema.json
tests:
  - N/A (spec update)
---

## Prompt

User requested ensuring content metadata schema exists in 001-textbook-generation before continuing with 004-personalization clarification. Selected Option A: Update 001-textbook-generation spec + content-schema.json to include personalization metadata.

## Response snapshot

Updated 001-textbook-generation to provide content metadata required by 004-personalization.

**Schema Changes (content-schema.json):**
- Added `primary_topic` (required): ros2 | simulation | vision | ml
- Added `difficulty_range` (required): { min, max } with beginner/intermediate/advanced
- Added `sections` array with section-level metadata definition:
  - `difficulty`: beginner | intermediate | advanced
  - `content_types`: [theory, code, practical, math, diagram]
  - `topics`: [ros2, simulation, vision, ml, fundamentals]
  - `requires_primer`: boolean for beginner adaptation

**Spec Changes:**
- Added FR-013: Chapter personalization metadata requirement
- Added FR-014: Section tagging requirement
- Added FR-015: requires_primer marking requirement
- Added "Content Metadata Contract" section documenting schema
- Added "Content Section" entity
- Updated Chapter entity with new attributes
- Added SC-010: Schema validation success criterion
- Renumbered SC-011 (deadline criterion)

## Outcome

- Impact: Established contract between 001-textbook-generation and 004-personalization for content metadata
- Tests: N/A (spec update)
- Files: spec.md, contracts/content-schema.json updated
- Next prompts: Continue /sp.clarify 004-personalization, then /sp.plan
- Reflection: Cross-feature dependencies should be identified and documented early

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
