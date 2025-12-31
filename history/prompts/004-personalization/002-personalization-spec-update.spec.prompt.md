---
id: 002
title: Personalization Spec Update with Chapter View
stage: spec
date: 2025-12-30
surface: agent
model: claude-opus-4-5-20251101
feature: 004-personalization
branch: 004-personalization
user: PMLS
command: /sp.specify 004-personalization make sure to add these so update the specification accordingly
labels: ["specification", "personalization", "chapter-view", "beginner-profile", "onboarding-modal"]
links:
  spec: specs/004-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-personalization/spec.md
  - specs/004-personalization/checklists/requirements.md
tests:
  - N/A (specification update)
---

## Prompt

/sp.specify 004-personalization make sure to add these so update the specification accordingly :                                                                      Key Entities- Add "Personalized Chapter View": A transient state of a chapter page where content is visually modified or filtered based on the active Profile.
Acceptance Scenarios-    Add: "Given a user is on Chapter 2 and clicks 'Personalize', When the engine processes their 'Beginner' profile, Then advanced math blocks are collapsed/hidden and a 'Concept Primer' is prepended."
Edge Cases-    Add: "What happens if a user clicks 'Personalize' but hasn't set any preferences?" (Should probably open the onboarding modal).

## Response snapshot

Updated 004-personalization specification with user-requested additions:

**Key Entity Added**:
- "Personalized Chapter View": A transient state of a chapter page where content is visually modified or filtered based on the active Personalization Profile. Attributes include chapter reference, active profile, content modifications applied (collapsed sections, prepended primers), view timestamp.

**Acceptance Scenario Added** (to User Story 1):
- Scenario 5: "Given a user is on Chapter 2 and clicks 'Personalize', When the engine processes their 'Beginner' profile, Then advanced math blocks are collapsed/hidden and a 'Concept Primer' is prepended to the chapter content"

**Edge Case Added**:
- "What happens if a user clicks 'Personalize' but hasn't set any preferences?" → Open the onboarding preferences modal to guide the user through setting their preferences before applying personalization.

**New Functional Requirements (4)**:
- FR-070: "Personalize" action on chapter pages
- FR-071: Collapse/hide advanced content for "Beginner" profile
- FR-072: Prepend "Concept Primer" for beginner users
- FR-073: Open onboarding modal when no preferences set

**New Success Criteria (2)**:
- SC-044: Chapter 2 personalization with Beginner profile (collapsed math, Concept Primer visible within 2s)
- SC-045: Onboarding modal appears 100% for users without preferences
- SC-046: Deadline criterion (updated from SC-044)

**Updated Totals**:
- Functional Requirements: 15 → 19 (FR-055 to FR-073)
- Success Criteria: 10 → 12 (SC-035 to SC-046)
- Edge Cases: 6 → 7

## Outcome

- Impact: Enriched specification with interactive "Personalize" button behavior and beginner-specific content adaptations
- Tests: N/A (specification update)
- Files: spec.md updated, checklists/requirements.md updated
- Next prompts: /sp.plan to create implementation plan
- Reflection: User additions focus on explicit user-triggered personalization ("Personalize" button) vs automatic adaptation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Quality checklist PASS (16/16 maintained)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Clarify if "Personalize" button is per-chapter or global during planning
