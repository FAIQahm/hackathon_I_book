# Feature Specification: Content Personalization Engine

**Feature Branch**: `004-personalization`
**Created**: 2025-12-30
**Status**: Draft
**Input**: User description: "Content personalization based on user preferences"

## Clarifications

### Session 2025-12-30

- Q: Should personalization define content metadata schema or consume from 001-textbook-generation? → A: Consume from 001-textbook-generation. Schema defined in `specs/001-textbook-generation/contracts/content-schema.json` with chapter-level (primary_topic, difficulty_range) and section-level (difficulty, content_types, topics, requires_primer) metadata.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Receive Personalized Chatbot Responses (Priority: P1)

A logged-in reader with stored preferences asks a question to the RAG chatbot. The chatbot adapts its response based on the reader's technical background, preferred depth, and domain knowledge - providing a beginner-friendly explanation to a newcomer or a technically dense answer to an experienced developer.

**Why this priority**: The chatbot is the primary interactive element; personalized responses directly demonstrate the value of capturing user preferences and represent the core personalization deliverable.

**Independent Test**: Can be fully tested by creating two user profiles (beginner vs advanced) asking the same question and verifying responses differ appropriately in complexity and terminology.

**Acceptance Scenarios**:

1. **Given** a user with "Beginner" programming experience asks about ROS 2 nodes, **When** the chatbot responds, **Then** the response uses simple language, avoids jargon, and includes foundational context
2. **Given** a user with "Advanced" programming experience asks the same question, **When** the chatbot responds, **Then** the response uses technical terminology, assumes prerequisite knowledge, and may reference advanced concepts
3. **Given** a user with "Deep-dive" depth preference, **When** the chatbot responds to any question, **Then** the response includes detailed explanations, edge cases, and technical nuances
4. **Given** a user with "Overview" depth preference, **When** the chatbot responds, **Then** the response provides concise summaries with links to deeper content
5. **Given** a user is on Chapter 2 and clicks "Personalize", **When** the engine processes their "Beginner" profile, **Then** advanced math blocks are collapsed/hidden and a "Concept Primer" is prepended to the chapter content

---

### User Story 2 - View Recommended Content (Priority: P2)

A logged-in reader lands on the book homepage or chapter page and sees content recommendations tailored to their interests. Based on their focus area preference and learning goals, the system highlights relevant chapters, sections, or topics they should explore next.

**Why this priority**: Content recommendations increase engagement and help readers navigate to relevant material, building on the base content and chatbot functionality.

**Independent Test**: Can be fully tested by creating users with different focus areas (e.g., "ROS 2" vs "Computer Vision") and verifying they see different recommended content on the same page.

**Acceptance Scenarios**:

1. **Given** a user with "ROS 2" as their focus area, **When** they view the homepage, **Then** ROS 2-related content is prominently featured in recommendations
2. **Given** a user with "Simulation" as their focus area, **When** they finish Chapter 1, **Then** they see recommendations for simulation-related sections in other chapters
3. **Given** a user with "Career change" learning goal, **When** they view recommendations, **Then** practical, job-relevant content is prioritized over theoretical material

---

### User Story 3 - Adaptive Learning Path (Priority: P3)

A logged-in reader with a defined time commitment and learning goal sees a suggested learning path through the book content. The system recommends an order for reading chapters and sections based on their available time and objectives.

**Why this priority**: Learning paths help users with limited time prioritize effectively, but require base content and recommendations to function.

**Independent Test**: Can be fully tested by creating users with different time commitments (<2 hours vs 5+ hours weekly) and verifying they receive different suggested paths.

**Acceptance Scenarios**:

1. **Given** a user with "<2 hours" weekly time commitment, **When** they view their learning path, **Then** they see a condensed path focusing on key concepts and takeaways
2. **Given** a user with "5+ hours" weekly time commitment, **When** they view their learning path, **Then** they see a comprehensive path including all chapters, code examples, and deep-dive sections
3. **Given** a user with "Hobby" learning goal, **When** they view their learning path, **Then** fun, project-oriented content is emphasized over career-focused material

---

### User Story 4 - Language-Adapted Content (Priority: P4)

A logged-in reader with Urdu as their language preference sees content in Urdu where available, with proper RTL rendering. The personalization engine ensures the reader's language choice is respected across all content delivery.

**Why this priority**: Language personalization is a constitution requirement but depends on translation feature being available.

**Independent Test**: Can be fully tested by creating a user with Urdu preference and verifying they see Urdu content with correct RTL formatting.

**Acceptance Scenarios**:

1. **Given** a user with "Urdu" language preference, **When** they view a translated chapter, **Then** the content displays in Urdu with RTL text direction
2. **Given** a user with "English" language preference, **When** they view the same chapter, **Then** the content displays in English with LTR text direction
3. **Given** a user with "Urdu" preference viewing untranslated content, **When** they view the page, **Then** they see a notice that translation is unavailable with fallback to English

---

### Edge Cases

- What happens when a guest (unauthenticated) user views the site? Display default content without personalization; show prompt to sign up for personalized experience.
- What happens when a user has incomplete preferences (abandoned onboarding)? Use available preferences; apply sensible defaults for missing dimensions; prompt to complete profile.
- How does the system handle conflicting preferences? Apply priority order: focus area > learning goal > depth preference > time commitment.
- What happens if personalization service is unavailable? Fall back to unpersonalized content with a subtle notice; log incident for monitoring.
- What happens when user preferences change mid-session? Apply new preferences to subsequent requests; do not reload current page content automatically.
- How does the system handle new content not yet categorized for recommendations? Exclude from personalized recommendations until metadata is complete; include in general navigation.
- What happens if a user clicks "Personalize" but hasn't set any preferences? Open the onboarding preferences modal to guide the user through setting their preferences before applying personalization.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-055**: System MUST adapt chatbot responses based on user's technical background preference (Beginner/Intermediate/Advanced)
- **FR-056**: System MUST adapt chatbot response depth based on user's depth preference (Overview/Balanced/Deep-dive)
- **FR-057**: System MUST include or exclude code examples in chatbot responses based on user's code examples importance preference
- **FR-058**: System MUST display personalized content recommendations on homepage and chapter pages for logged-in users
- **FR-059**: System MUST prioritize recommended content based on user's focus area preference
- **FR-060**: System MUST generate a suggested learning path based on user's time commitment and learning goal
- **FR-061**: System MUST respect user's language preference when displaying content
- **FR-062**: System MUST render Urdu content with RTL text direction
- **FR-063**: System MUST fall back to default (unpersonalized) content for unauthenticated users
- **FR-064**: System MUST apply sensible defaults for missing preference dimensions
- **FR-065**: System MUST deliver personalized content within 4 seconds
- **FR-066**: System MUST return cached personalized responses within 1 second
- **FR-067**: System MUST support minimum 5 personalization dimensions simultaneously
- **FR-068**: System MUST NOT require page reload when preferences are updated mid-session (next request applies new preferences)
- **FR-069**: System MUST gracefully degrade to unpersonalized content if personalization service fails
- **FR-070**: System MUST provide a "Personalize" action on chapter pages that triggers content adaptation based on user profile
- **FR-071**: System MUST collapse or hide advanced content sections (e.g., advanced math blocks) for users with "Beginner" profile when personalization is activated
- **FR-072**: System MUST prepend contextual "Concept Primer" sections for beginner users when personalization is activated
- **FR-073**: System MUST open the onboarding preferences modal when a user clicks "Personalize" without having set any preferences

### Personalization Dimensions (5+ Required per Constitution)

The system MUST personalize based on these dimensions collected during authentication signup:

1. **Technical Background** → Adjusts language complexity and assumed knowledge
2. **Domain Knowledge** → Adjusts amount of foundational context provided
3. **Preferred Depth** → Controls response length and detail level
4. **Code Examples Importance** → Includes/excludes code snippets in responses
5. **Focus Area** → Prioritizes topic-relevant content in recommendations
6. **Time Commitment** → Shapes learning path length and density
7. **Learning Goal** → Influences practical vs theoretical content emphasis

### Key Entities

- **Personalization Profile**: A computed profile derived from user preferences. Attributes include user reference, computed complexity level, content priorities, active dimensions.
- **Content Recommendation**: A suggested piece of content for a user. Attributes include content reference, relevance score, recommendation reason, user reference.
- **Learning Path**: An ordered sequence of content for a user. Attributes include user reference, path steps (ordered list), estimated completion time, path type (condensed/comprehensive).
- **Personalized Response**: A chatbot response adapted to user preferences. Attributes include base response, adaptation applied, user preference snapshot, response time.
- **Personalized Chapter View**: A transient state of a chapter page where content is visually modified or filtered based on the active Personalization Profile. Attributes include chapter reference, active profile, content modifications applied (collapsed sections, prepended primers), view timestamp.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-035**: Chatbot responses demonstrably differ for users with different technical backgrounds (verified by comparing 10 identical questions across Beginner vs Advanced profiles)
- **SC-036**: Chatbot responses differ in length/detail based on depth preference (Overview responses average <150 words, Deep-dive responses average >300 words, measured over 10 test questions)
- **SC-037**: Content recommendations differ based on focus area (users with different focus areas see <50% overlap in top 5 recommendations)
- **SC-038**: Learning paths differ based on time commitment (condensed path includes <60% of comprehensive path content)
- **SC-039**: Personalized content delivery meets 4-second threshold (p95 latency measured over 100 requests)
- **SC-040**: Cached personalized responses return within 1 second (p95 latency for repeated requests with same preferences)
- **SC-041**: System supports all 7 personalization dimensions simultaneously without degradation
- **SC-042**: Urdu content renders with correct RTL direction on all 3 chapter pages (verified visually and via CSS inspection)
- **SC-043**: Unpersonalized fallback works correctly when personalization service is disabled (verified by disabling service and confirming content loads)
- **SC-044**: Clicking "Personalize" on Chapter 2 with a "Beginner" profile results in advanced math blocks being collapsed and a "Concept Primer" being visible within 2 seconds
- **SC-045**: Users without preferences who click "Personalize" see the onboarding modal 100% of the time (verified over 10 test attempts)
- **SC-046**: All success criteria (SC-035 to SC-045) MUST be verified and passing by Nov 30, 2025 6:00 PM submission deadline

## Assumptions

- User preferences are collected and stored via the authentication feature (003-authentication)
- Preference data is accessible to the personalization engine via secure internal API or shared database
- Book content (001-textbook-generation) is tagged with metadata per `specs/001-textbook-generation/contracts/content-schema.json`: primary_topic, difficulty_range at chapter level; difficulty, content_types, topics, requires_primer at section level
- RAG chatbot (002-rag-chatbot) exposes extension points for response adaptation
- Urdu translations are available from a separate translation feature
- Personalization logic runs server-side to protect user preference data
- Cache invalidation occurs when user updates their preferences
- Default preferences exist for all dimensions to handle incomplete profiles
