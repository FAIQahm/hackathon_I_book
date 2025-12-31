# Technical Plan: 004-Personalization Feature

**Feature Branch**: `004-personalization`
**Created**: 2025-12-30
**Status**: Approved

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         CLIENT LAYER                                      │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ Personalize  │  │ Chatbot UI   │  │ Recommend    │  │ Learning     │ │
│  │ Button       │  │ + Adapter    │  │ Widget       │  │ Path View    │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
└─────────┼──────────────────┼──────────────────┼──────────────────┼───────┘
          │                  │                  │                  │
          ▼                  ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      PERSONALIZATION ENGINE (Server)                     │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    PersonalizationService                         │   │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ │   │
│  │  │ Profile     │ │ Response    │ │ Recommend   │ │ Learning    │ │   │
│  │  │ Computer    │ │ Adapter     │ │ Engine      │ │ Path Gen    │ │   │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘ │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                  │                                       │
│  ┌───────────────────────────────┴───────────────────────────────────┐  │
│  │                         Cache Layer (Redis/Memory)                 │  │
│  │           TTL: Profile=5min, Response=1hr, Recommend=30min         │  │
│  └────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          DATA LAYER                                      │
├─────────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                   │
│  │ User Prefs   │  │ Content      │  │ Chatbot      │                   │
│  │ (003-auth)   │  │ Metadata     │  │ (002-rag)    │                   │
│  │ Neon PG      │  │ (001-text)   │  │ Qdrant       │                   │
│  └──────────────┘  └──────────────┘  └──────────────┘                   │
└─────────────────────────────────────────────────────────────────────────┘
```

## Implementation Phases

### Phase 1: Core Infrastructure (Foundation)
**Goal**: Set up personalization service skeleton and data access

| Task | Description | Files |
|------|-------------|-------|
| 1.1 | Create PersonalizationService class | `src/services/personalization.ts` |
| 1.2 | Define TypeScript interfaces for 7 dimensions | `src/types/personalization.ts` |
| 1.3 | Create preference reader from auth DB | `src/services/preferenceReader.ts` |
| 1.4 | Set up caching layer (in-memory initially) | `src/cache/personalizationCache.ts` |
| 1.5 | Create API route `/api/personalization/profile` | `src/pages/api/personalization/profile.ts` |

### Phase 2: Profile Computation
**Goal**: Transform raw preferences into actionable personalization profile

| Task | Description | Files |
|------|-------------|-------|
| 2.1 | Build ProfileComputer with dimension mapping | `src/services/profileComputer.ts` |
| 2.2 | Implement complexity level calculator | (integrated in ProfileComputer) |
| 2.3 | Create default preferences for incomplete profiles | `src/config/defaultPreferences.ts` |
| 2.4 | Add profile validation and sanitization | `src/utils/profileValidator.ts` |

**Complexity Mapping Logic**:
```
Technical Background + Domain Knowledge → complexity_level
  Beginner + None/Some → "simple"
  Intermediate + Any → "balanced"
  Advanced + Experienced → "technical"
```

### Phase 3: Chatbot Response Adaptation (FR-055, FR-056, FR-057)
**Goal**: Modify chatbot responses based on user profile

| Task | Description | Files |
|------|-------------|-------|
| 3.1 | Create ResponseAdapter middleware | `src/services/responseAdapter.ts` |
| 3.2 | Build prompt templates per complexity level | `src/prompts/personalized/*.txt` |
| 3.3 | Integrate with 002-rag-chatbot endpoint | Modify `src/pages/api/chat.ts` |
| 3.4 | Add code example inclusion/exclusion logic | (integrated in ResponseAdapter) |

**Prompt Template Strategy**:
- `simple.txt`: "Explain in simple terms, avoid jargon, include analogies"
- `balanced.txt`: "Use standard technical language with brief explanations"
- `technical.txt`: "Use precise terminology, include edge cases and nuances"

### Phase 4: Content Recommendations (FR-058, FR-059)
**Goal**: Generate personalized content suggestions

| Task | Description | Files |
|------|-------------|-------|
| 4.1 | Create RecommendationEngine | `src/services/recommendationEngine.ts` |
| 4.2 | Build content metadata indexer | `src/services/contentIndexer.ts` |
| 4.3 | Implement relevance scoring algorithm | (integrated in RecommendationEngine) |
| 4.4 | Create Recommendations React component | `src/components/Recommendations.tsx` |
| 4.5 | Add API route `/api/personalization/recommendations` | `src/pages/api/personalization/recommendations.ts` |

**Relevance Scoring**:
```
score = (topic_match * 0.4) + (difficulty_match * 0.3) + (goal_alignment * 0.3)
  topic_match: user.focus_area matches section.topics
  difficulty_match: user.technical_background aligns with section.difficulty
  goal_alignment: content_type matches learning_goal (practical vs theoretical)
```

### Phase 5: Learning Path Generation (FR-060)
**Goal**: Create ordered content sequences based on time/goals

| Task | Description | Files |
|------|-------------|-------|
| 5.1 | Create LearningPathGenerator | `src/services/learningPathGenerator.ts` |
| 5.2 | Define path templates (condensed/comprehensive) | `src/config/pathTemplates.ts` |
| 5.3 | Build time estimation calculator | (integrated in LearningPathGenerator) |
| 5.4 | Create LearningPath React component | `src/components/LearningPath.tsx` |
| 5.5 | Add API route `/api/personalization/learning-path` | `src/pages/api/personalization/learning-path.ts` |

**Path Generation Logic**:
```
time_commitment → path_type
  <2 hours → condensed (key takeaways + essential sections only)
  2-5 hours → balanced (all theory + select code examples)
  5+ hours → comprehensive (everything + deep-dive sections)
```

### Phase 6: Chapter Personalization UI (FR-070, FR-071, FR-072, FR-073)
**Goal**: "Personalize" button that adapts chapter content in-place

| Task | Description | Files |
|------|-------------|-------|
| 6.1 | Create PersonalizeButton component | `src/components/PersonalizeButton.tsx` |
| 6.2 | Build ContentFilter for section visibility | `src/services/contentFilter.ts` |
| 6.3 | Create ConceptPrimer component | `src/components/ConceptPrimer.tsx` |
| 6.4 | Implement section collapse/expand logic | `src/hooks/usePersonalizedView.ts` |
| 6.5 | Add onboarding modal trigger for no-prefs users | (integrated in PersonalizeButton) |

**Content Filtering Rules**:
```
if user.technical_background === "Beginner":
  - collapse sections where difficulty === "advanced"
  - collapse sections where content_types includes "math"
  - prepend ConceptPrimer where requires_primer === true
```

### Phase 7: Language Integration (FR-061, FR-062)
**Goal**: Respect language preference, coordinate with 005-translation

| Task | Description | Files |
|------|-------------|-------|
| 7.1 | Add language to personalization profile | (extend ProfileComputer) |
| 7.2 | Create LanguageProvider context | `src/contexts/LanguageContext.tsx` |
| 7.3 | Add RTL CSS utilities | `src/styles/rtl.css` |
| 7.4 | Coordinate with translation feature API | (integration point) |

### Phase 8: Performance & Resilience (FR-065, FR-066, FR-069)
**Goal**: Meet latency targets and graceful degradation

| Task | Description | Files |
|------|-------------|-------|
| 8.1 | Implement response caching (1s target) | `src/cache/responseCache.ts` |
| 8.2 | Add cache invalidation on preference update | (integrated in preferenceReader) |
| 8.3 | Create fallback to unpersonalized content | `src/services/fallbackHandler.ts` |
| 8.4 | Add performance monitoring/logging | `src/utils/perfMonitor.ts` |

## Component Breakdown

### Core Services

| Component | Responsibility | Dependencies |
|-----------|----------------|--------------|
| `PersonalizationService` | Orchestrates all personalization | ProfileComputer, ResponseAdapter, RecommendEngine |
| `ProfileComputer` | Transforms preferences → profile | PreferenceReader, DefaultPreferences |
| `ResponseAdapter` | Modifies chatbot responses | ProfileComputer, PromptTemplates |
| `RecommendationEngine` | Generates content suggestions | ContentIndexer, ProfileComputer |
| `LearningPathGenerator` | Creates ordered content paths | RecommendationEngine, PathTemplates |
| `ContentFilter` | Filters chapter sections | ProfileComputer, ContentMetadata |

### React Components

| Component | Purpose | Props |
|-----------|---------|-------|
| `PersonalizeButton` | Triggers chapter personalization | `chapterId`, `onPersonalize` |
| `Recommendations` | Displays content suggestions | `userId`, `limit` |
| `LearningPath` | Shows ordered learning sequence | `userId`, `pathType` |
| `ConceptPrimer` | Beginner context block | `sectionId`, `content` |

### API Routes

| Route | Method | Purpose |
|-------|--------|---------|
| `/api/personalization/profile` | GET | Fetch computed profile |
| `/api/personalization/recommendations` | GET | Get content recommendations |
| `/api/personalization/learning-path` | GET | Get personalized path |
| `/api/personalization/apply` | POST | Apply chapter personalization |

## Dependencies & Sequencing

```
001-textbook-generation ─────┐
  (content + metadata)       │
                             ▼
002-rag-chatbot ────────► 004-personalization ◄──── 003-authentication
  (chatbot API)              │                       (user preferences)
                             │
                             ▼
                      005-translation
                      (language support)
```

### Implementation Order

1. **Phase 1** - Can start immediately (no blocking deps)
2. **Phase 2** - Requires 003-authentication preferences API
3. **Phase 3** - Requires 002-rag-chatbot API integration point
4. **Phase 4** - Requires 001-textbook-generation content metadata
5. **Phase 5** - Requires Phase 4 completion
6. **Phase 6** - Requires Phases 2, 4
7. **Phase 7** - Requires 005-translation (can stub initially)
8. **Phase 8** - Can run parallel to Phases 4-7

### Critical Path

```
Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 6
                              ↓
                          Phase 5
```

## Design Decisions

### D1: Server-Side Personalization
**Decision**: Run personalization logic server-side
**Rationale**: Protects user preference data, enables caching, consistent behavior
**Alternative Rejected**: Client-side (exposes preferences, harder to cache)

### D2: Profile Caching Strategy
**Decision**: Cache computed profiles for 5 minutes, responses for 1 hour
**Rationale**: Balance freshness with performance (4s/1s targets)
**Cache Keys**: `profile:{userId}`, `response:{userId}:{queryHash}`

### D3: Graceful Degradation
**Decision**: Fall back to unpersonalized content on any service failure
**Rationale**: Educational content remains accessible; personalization is enhancement
**Implementation**: Try-catch wrapper with fallback flag in response

### D4: Dimension Priority Order
**Decision**: focus_area > learning_goal > depth_preference > time_commitment
**Rationale**: Topic relevance most impactful; time is flexible
**Used For**: Resolving conflicts in recommendations

### D5: Content Metadata Source
**Decision**: Consume from `001-textbook-generation/contracts/content-schema.json`
**Rationale**: Single source of truth; personalization is consumer not owner
**Integration**: Read section metadata at build time, index for runtime queries

### D6: Prompt Engineering for Response Adaptation
**Decision**: Use separate prompt templates per complexity level (not dynamic generation)
**Rationale**: Predictable behavior, easier testing, consistent quality
**Templates**: `simple.txt`, `balanced.txt`, `technical.txt`

## Files to Create/Modify

### New Files
```
src/
├── services/
│   ├── personalization.ts
│   ├── profileComputer.ts
│   ├── responseAdapter.ts
│   ├── recommendationEngine.ts
│   ├── learningPathGenerator.ts
│   ├── contentFilter.ts
│   ├── contentIndexer.ts
│   ├── preferenceReader.ts
│   └── fallbackHandler.ts
├── types/
│   └── personalization.ts
├── config/
│   ├── defaultPreferences.ts
│   └── pathTemplates.ts
├── cache/
│   ├── personalizationCache.ts
│   └── responseCache.ts
├── components/
│   ├── PersonalizeButton.tsx
│   ├── Recommendations.tsx
│   ├── LearningPath.tsx
│   └── ConceptPrimer.tsx
├── contexts/
│   └── LanguageContext.tsx
├── hooks/
│   └── usePersonalizedView.ts
├── prompts/
│   └── personalized/
│       ├── simple.txt
│       ├── balanced.txt
│       └── technical.txt
├── pages/api/personalization/
│   ├── profile.ts
│   ├── recommendations.ts
│   ├── learning-path.ts
│   └── apply.ts
├── styles/
│   └── rtl.css
└── utils/
    ├── profileValidator.ts
    └── perfMonitor.ts
```

### Files to Modify
```
src/pages/api/chat.ts          # Integrate ResponseAdapter
src/components/Chapter.tsx     # Add PersonalizeButton
src/components/Layout.tsx      # Add Recommendations widget
```

## Success Criteria Mapping

| SC | Implementation |
|----|----------------|
| SC-035 | ResponseAdapter with complexity-based prompts |
| SC-036 | Word count limits in prompt templates |
| SC-037 | RecommendationEngine relevance scoring |
| SC-038 | LearningPathGenerator path templates |
| SC-039 | Performance monitoring + optimization |
| SC-040 | Response caching layer |
| SC-041 | ProfileComputer handles all 7 dimensions |
| SC-042 | LanguageContext + RTL CSS |
| SC-043 | FallbackHandler graceful degradation |
| SC-044 | ContentFilter + ConceptPrimer |
| SC-045 | PersonalizeButton modal trigger |
