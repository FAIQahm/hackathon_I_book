# Physical AI Textbook - Consolidated Task Breakdown

**Generated**: 2026-01-04
**Total Features**: 5
**Total User Stories**: 20
**Estimated Tasks**: 85 atomic units

---

## Dependency Graph

```
001-textbook-generation (Foundation)
         │
         ├──────────────────┐
         ▼                  ▼
002-rag-chatbot      005-translation
         │                  │
         ▼                  │
003-authentication ◄────────┘
         │
         ▼
004-personalization
```

---

## Phase 0: Infrastructure Setup
**Checkpoint**: All infrastructure configured and verified

### T-000: Project Infrastructure
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-001 | Configure Docusaurus project with i18n | `npm run build` succeeds with en/ur locales | 20m | 001/FR-010 |
| T-002 | Set up GitHub Actions deploy workflow | Push to main triggers deployment | 15m | 001/plan |
| T-003 | Configure Vercel FastAPI project | `vercel dev` runs locally | 20m | 003/plan |
| T-004 | Set up Neon Postgres database | Connection string works, tables created | 20m | 003/FR-044 |
| T-005 | Configure Qdrant Cloud collection | Vector collection created, test insert works | 20m | 002/FR-014 |
| T-006 | Set environment variables in Vercel | `env_manager.sh --validate` passes | 15m | env_contract |

**CHECKPOINT 0**: [ ] All services configured and accessible

---

## Phase 1: Textbook Content (001-textbook-generation)
**Priority**: P1 (Foundation for all other features)
**Spec**: `specs/001-textbook-generation/spec.md`

### US-001: Read Chapter Content (P1)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-010 | Create Chapter 1: Intro to Physical AI & ROS 2 | 4500-6000 words, learning objectives, 2+ diagrams | 30m | FR-001,002,003,004 |
| T-011 | Create Chapter 2: Simulation with Gazebo | 4500-6000 words, learning objectives, 2+ diagrams | 30m | FR-001,002,003,004 |
| T-012 | Create Chapter 3: Vision-Language-Action Models | 4500-6000 words, learning objectives, 2+ diagrams | 30m | FR-001,002,003,004 |
| T-013 | Add key takeaways to all chapters | Each chapter ends with summary section | 20m | FR-005 |
| T-014 | Add content metadata (topic, difficulty) | `content-schema.json` validation passes | 20m | FR-013,014,015 |
| T-015 | Add alt text to all diagrams | Every `<img>` has descriptive alt attribute | 15m | FR-009 |

### US-002: Navigate Between Chapters (P2)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-020 | Implement next/previous navigation | Links work on all chapter pages | 15m | FR-006 |
| T-021 | Create table of contents sidebar | All 3 chapters accessible from any page | 15m | FR-006 |
| T-022 | Verify all internal links resolve | 0 broken links in `npm run build` | 15m | FR-011 |

### US-003: Run Code Examples (P3)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-030 | Add syntax highlighting to code blocks | Prism.js renders Python/JS code | 15m | FR-007 |
| T-031 | Add context explanations to code examples | Each code block has surrounding text | 20m | US-003 |
| T-032 | Verify code examples execute | CI test passes with Python 3.11 | 20m | SC-004 |

**CHECKPOINT 1**: [ ] All 3 chapters published, < 3s page load (SC-006)

---

## Phase 2: RAG Chatbot (002-rag-chatbot)
**Depends on**: Phase 1 (content to index)
**Spec**: `specs/002-rag-chatbot/spec.md`

### US-004: Ask Question About Book Content (P1)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-040 | Create vector embeddings for all chapters | 3 chapters indexed in Qdrant | 25m | FR-014 |
| T-041 | Implement RAG retrieval endpoint | `/api/chat` returns relevant chunks | 25m | FR-014 |
| T-042 | Implement LLM response generation | OpenAI generates grounded answers | 25m | FR-014 |
| T-043 | Add citation references to responses | Response includes chapter/section source | 20m | FR-017 |
| T-044 | Create chatbot UI component | Embedded chat panel in docs | 25m | FR-013 |
| T-045 | Implement response caching | Cached responses < 1s (SC-014) | 20m | FR-016 |

### US-005: Select Text and Ask (P2)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-050 | Implement text selection detection | Selection triggers popup within 500ms | 20m | SC-017 |
| T-051 | Pass selected text as context | Chatbot uses selection in query | 20m | FR-018 |

### US-006: View Conversation History (P3)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-055 | Implement session-based history | History persists 30+ minutes | 20m | FR-021,SC-019 |
| T-056 | Create scrollable history UI | All Q&A pairs visible on scroll | 15m | FR-021 |

### US-007: "I Don't Know" Response (P4)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-060 | Implement out-of-scope detection | 10/10 out-of-scope questions declined | 20m | FR-020,SC-018 |
| T-061 | Add graceful degradation | Keyword fallback when Qdrant fails | 20m | FR-027 |

**CHECKPOINT 2**: [ ] 90%+ accuracy on 20 test questions (SC-012)

---

## Phase 3: Authentication (003-authentication)
**Depends on**: Phase 0 (database)
**Spec**: `specs/003-authentication/spec.md`

### US-008: New User Registration (P1)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-070 | Install and configure Better-Auth | Library integrated with FastAPI | 20m | FR-039 |
| T-071 | Create user registration endpoint | `/api/auth/register` creates user | 20m | FR-039a |
| T-072 | Implement password validation | Min 8 chars, 1 upper, 1 number | 15m | FR-041 |
| T-073 | Create signup form component | React form with validation | 20m | plan/Phase 4 |
| T-074 | Implement 10 onboarding questions | Wizard collects all preferences | 25m | FR-040 |
| T-075 | Store preferences in database | All 10 dimensions persisted | 20m | FR-050 |

### US-009: Returning User Login (P2)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-080 | Create login endpoint | `/api/auth/login` returns JWT | 20m | FR-043 |
| T-081 | Implement session management | Secure HttpOnly cookies | 20m | FR-044 |
| T-082 | Create login form component | React form with error handling | 15m | plan/Phase 4 |
| T-083 | Implement account lockout | Lock after 5 failed attempts | 20m | FR-049 |

### US-010: Update Preferences (P3)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-085 | Create preferences update endpoint | `/api/preferences` PUT works | 15m | FR-047 |
| T-086 | Create profile settings component | User can view/edit preferences | 20m | plan/Phase 4 |

### US-011: Password Reset (P4)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-090 | Implement password reset request | Email sent within 2 min | 20m | FR-045 |
| T-091 | Implement reset confirmation | New password set via link | 20m | FR-046 |

### US-012: Logout (P5)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-095 | Implement logout endpoint | Session terminated, cookie cleared | 15m | FR-048 |

**CHECKPOINT 3**: [ ] Registration completes in < 5 min (SC-024)

---

## Phase 4: Personalization (004-personalization)
**Depends on**: Phase 2 (chatbot), Phase 3 (preferences)
**Spec**: `specs/004-personalization/spec.md`

### US-013: Personalized Chatbot Responses (P1)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-100 | Create ProfileComputer service | Transforms preferences to profile | 25m | plan/Phase 2 |
| T-101 | Create complexity prompt templates | simple.txt, balanced.txt, technical.txt | 20m | plan/Phase 3 |
| T-102 | Integrate ResponseAdapter with chatbot | Responses differ by background | 25m | FR-055,056 |
| T-103 | Implement code example toggle | Include/exclude based on preference | 15m | FR-057 |

### US-014: Content Recommendations (P2)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-110 | Create RecommendationEngine | Scores content by relevance | 25m | plan/Phase 4 |
| T-111 | Create Recommendations component | Shows top 5 suggestions | 20m | plan/Phase 4 |
| T-112 | Implement focus area prioritization | Different focus = different recs | 15m | FR-059 |

### US-015: Learning Path (P3)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-115 | Create LearningPathGenerator | Condensed vs comprehensive paths | 25m | plan/Phase 5 |
| T-116 | Create LearningPath component | Shows ordered content sequence | 20m | plan/Phase 5 |

### US-016: Language Adaptation (P4)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-120 | Add language to profile | Respects language_preference | 15m | FR-061 |
| T-121 | Create LanguageContext provider | RTL flag available globally | 15m | plan/Phase 7 |

### Chapter Personalization (FR-070-073)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-125 | Create PersonalizeButton component | Button visible on chapters | 15m | FR-070 |
| T-126 | Implement ContentFilter for beginners | Advanced sections collapse | 20m | FR-071 |
| T-127 | Create ConceptPrimer component | Beginner context prepended | 20m | FR-072 |
| T-128 | Trigger onboarding modal if no prefs | Modal opens for new users | 15m | FR-073 |

**CHECKPOINT 4**: [ ] Responses differ by profile (SC-035)

---

## Phase 5: Translation (005-translation)
**Depends on**: Phase 1 (content to translate)
**Spec**: `specs/005-translation/spec.md`

### US-017: Read Chapter in Urdu (P1)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-130 | Configure Docusaurus i18n for Urdu | `ur` locale in config | 15m | plan/Phase 1 |
| T-131 | Add Urdu web fonts (Noto Nastaliq) | Font loads, renders correctly | 15m | FR-084 |
| T-132 | Translate Chapter 1 to Urdu | Complete translation in i18n/ur/ | 30m | FR-074 |
| T-133 | Translate Chapter 2 to Urdu | Complete translation in i18n/ur/ | 30m | FR-074 |
| T-134 | Translate Chapter 3 to Urdu | Complete translation in i18n/ur/ | 30m | FR-074 |

### US-018: Switch Languages (P2)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-140 | Create LanguageToggle component | Toggle visible in navbar | 20m | FR-076 |
| T-141 | Implement scroll preservation | Position within 100px on switch | 20m | FR-078,SC-051 |
| T-142 | Implement language preference storage | Preference persists in localStorage | 15m | FR-079 |

### US-019: Code with Translated Context (P3)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-145 | Preserve code blocks in LTR | Code stays English, left-aligned | 15m | FR-082 |
| T-146 | Translate code explanations | Surrounding text in Urdu | 20m | FR-083 |

### US-020: Translated Interface (P4)
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-150 | Translate navbar items | All nav labels in Urdu | 15m | FR-080 |
| T-151 | Translate footer content | Footer displays in Urdu | 15m | FR-080 |
| T-152 | Translate sidebar labels | Chapter titles in Urdu | 15m | FR-081 |

### RTL Styling
| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-155 | Create RTL CSS utilities | `[dir="rtl"]` styles work | 20m | FR-075 |
| T-156 | Handle bidi content | English terms in Urdu text | 15m | FR-085 |

**CHECKPOINT 5**: [ ] 85%+ translation quality (SC-048)

---

## Phase 6: Integration & Polish
**Depends on**: All previous phases

| ID | Task | Acceptance Criteria | Est. | Spec Ref |
|----|------|---------------------|------|----------|
| T-160 | Run Lighthouse accessibility audit | Score > 95 on all pages | 20m | SC-007 |
| T-161 | Run axe-core accessibility check | 0 critical violations | 15m | SC-007,SC-021 |
| T-162 | Verify 3 viewport responsiveness | 375px, 768px, 1440px all work | 20m | SC-009 |
| T-163 | Run k6 load test (100 users) | p95 < 4s under load | 25m | SC-027 |
| T-164 | Verify all success criteria | All SC-* criteria pass | 30m | All |

**FINAL CHECKPOINT**: [ ] All success criteria verified

---

## Summary by Priority

| Priority | User Stories | Est. Tasks | Critical Path |
|----------|--------------|------------|---------------|
| P1 | US-001,004,008,013,017 | 35 | Yes |
| P2 | US-002,005,009,014,018 | 18 | Partial |
| P3 | US-003,006,010,015,019 | 14 | No |
| P4 | US-007,011,016,020 | 12 | No |
| P5 | US-012 | 1 | No |

**Recommended Execution Order**:
1. Phase 0 (Infrastructure) - All teams
2. Phase 1 (Content) - BookArchitect
3. Phase 2 (Chatbot) + Phase 3 (Auth) - Parallel
4. Phase 5 (Translation) - Can start after Phase 1
5. Phase 4 (Personalization) - Requires Phase 2+3
6. Phase 6 (Integration) - Final pass

---

## Agent Assignments

| Agent | Primary Phases | Skills Used |
|-------|----------------|-------------|
| BookArchitect | 1, 5, 6 | docusaurus-scaffold, github-pages-deploy |
| AIEngineer | 2, 4 | qdrant-manager, rag-personalizer |
| SecurityLead | 3 | auth-connect |
| BackendIntegrator | 0, 3 | vercel-fastapi-link |
| Linguist | 5 | translation-sync |

---

*Generated by /sp.tasks - 2026-01-04*
