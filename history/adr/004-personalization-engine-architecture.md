# ADR-004: Personalization Engine Architecture

**Status**: Accepted
**Date**: 2025-12-31
**Deciders**: Project Team

## Context

The Physical AI Educational Book requires personalization to adapt content based on user preferences collected during onboarding. The system must:
- Transform 10 preference dimensions into actionable profiles
- Adapt chatbot responses based on user expertise level
- Provide content recommendations aligned with user goals
- Generate learning paths based on time commitment
- Meet performance targets (<4s personalized, <1s cached responses)
- Degrade gracefully when personalization fails

## Decision

We adopt a **server-side personalization engine with tiered caching**:

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Processing Location** | Server-side (Vercel) | Protect preferences, enable caching |
| **Profile Computation** | ProfileComputer service | Transform preferences → actionable profile |
| **Response Adaptation** | Prompt templates (simple/balanced/technical) | Modify chatbot behavior per complexity |
| **Recommendations** | Relevance scoring algorithm | Match content to user goals |
| **Caching Strategy** | In-memory with tiered TTL | Profile: 5min, Response: 1hr |
| **Fallback** | Unpersonalized content | Graceful degradation on failure |

### Architecture

```
User Preferences (003-auth) → ProfileComputer → PersonalizationService
                                                      ↓
                              ┌────────────────────────┼────────────────────────┐
                              ↓                        ↓                        ↓
                       ResponseAdapter         RecommendEngine          LearningPathGen
                       (chatbot mod)           (content match)          (ordered paths)
                              ↓                        ↓                        ↓
                        Cache Layer (TTL-based invalidation)
```

### Complexity Mapping

```
Technical Background + Domain Knowledge → complexity_level
  Beginner + None/Some → "simple"
  Intermediate + Any → "balanced"
  Advanced + Experienced → "technical"
```

### Prompt Templates

- `simple.txt`: Avoid jargon, include analogies, beginner-friendly
- `balanced.txt`: Standard technical language with brief explanations
- `technical.txt`: Precise terminology, edge cases, advanced concepts

### Dimension Priority

```
focus_area > learning_goal > depth_preference > time_commitment
```

## Consequences

### Positive

- **Privacy Protection**: User preferences never exposed to client
- **Cacheable**: Server-side enables effective caching
- **Consistent Behavior**: Same inputs → same outputs
- **Testable**: Prompt templates can be unit tested
- **Resilient**: Fallback ensures content always available

### Negative

- **Server Dependency**: Personalization requires backend availability
- **Cold Start Impact**: First request after cold start may be slow
- **Cache Invalidation Complexity**: Preference updates must invalidate cache

### Risks

- **Stale Cache**: User changes preferences but sees old content (mitigated by cache invalidation)
- **Template Maintenance**: Three prompt templates to maintain

## Alternatives Considered

### Alternative 1: Client-Side Personalization
- **Pros**: No server round-trip, works offline
- **Cons**: Exposes preferences, harder to cache, inconsistent behavior across clients

### Alternative 2: Dynamic Prompt Generation
- **Pros**: More flexible, no templates to maintain
- **Cons**: Unpredictable behavior, harder to test, potential prompt injection

### Alternative 3: No Caching
- **Pros**: Always fresh responses
- **Cons**: Cannot meet <1s cached response target, higher LLM costs

### Alternative 4: Edge Personalization (Cloudflare Workers)
- **Pros**: Lower latency, global distribution
- **Cons**: Additional complexity, different runtime from Vercel

## References

- `specs/004-personalization/plan.md` - Full implementation plan
- `specs/003-authentication/plan.md` - Preference collection
- `specs/003-authentication/data-model.md` - Preference dimensions
