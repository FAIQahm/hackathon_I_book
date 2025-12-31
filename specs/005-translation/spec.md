# Feature Specification: Urdu Translation & RTL Support

**Feature Branch**: `005-translation`
**Created**: 2025-12-30
**Status**: Draft
**Input**: User description: "Urdu translation with RTL rendering for Physical AI book"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Read Chapter in Urdu (Priority: P1)

An Urdu-speaking reader visits the Physical AI book website and selects Urdu as their preferred language. They can read all chapter content fully translated into Urdu with proper Right-to-Left (RTL) text rendering, making the educational content accessible in their native language.

**Why this priority**: Full chapter translation is the core deliverable - without translated content, other features (language switching, RTL) have no purpose.

**Independent Test**: Can be fully tested by viewing each chapter in Urdu mode and verifying all text is translated, displayed RTL, and reads naturally.

**Acceptance Scenarios**:

1. **Given** a reader selects Urdu language, **When** they view Chapter 1, **Then** all headings, body text, learning objectives, and key takeaways display in Urdu
2. **Given** a reader views any translated chapter, **When** they read the content, **Then** the text flows Right-to-Left with proper Urdu typography
3. **Given** a reader views a fully translated chapter in Urdu, **When** they check the content quality, **Then** the translation maintains 85% or higher accuracy compared to the English source

---

### User Story 2 - Switch Between Languages (Priority: P2)

A bilingual reader wants to compare content in both English and Urdu. They can easily switch between languages using a language toggle without losing their place in the content.

**Why this priority**: Language switching enables readers to compare translations and choose their preferred reading experience, building on the base translation.

**Independent Test**: Can be fully tested by toggling between English and Urdu on any page and verifying the content updates correctly while maintaining scroll position.

**Acceptance Scenarios**:

1. **Given** a reader is viewing a chapter in English, **When** they click the language toggle, **Then** the page displays in Urdu within 2 seconds
2. **Given** a reader switches from Urdu to English, **When** the page updates, **Then** they remain at the same content section (scroll position preserved)
3. **Given** a reader has set Urdu as their preference, **When** they navigate to a new chapter, **Then** that chapter loads in Urdu by default

---

### User Story 3 - View Code Examples with Translated Context (Priority: P3)

A reader viewing translated content encounters code examples. The code itself remains in its original programming language (Python, etc.), but all surrounding explanatory text, comments descriptions, and annotations are translated into Urdu.

**Why this priority**: Code examples are critical for technical learning but cannot be translated; the surrounding context must be to maintain educational value.

**Independent Test**: Can be fully tested by viewing code blocks in Urdu mode and verifying code is unchanged while explanations are translated.

**Acceptance Scenarios**:

1. **Given** a reader views a code example in Urdu mode, **When** they examine the code block, **Then** the code syntax remains in English/original language
2. **Given** a code example has explanatory text around it, **When** viewed in Urdu mode, **Then** all explanations, captions, and context are translated to Urdu
3. **Given** a code block has inline comments, **When** viewed in Urdu mode, **Then** code comments remain in English (as they are part of the code)

---

### User Story 4 - Navigate with Translated Interface (Priority: P4)

A reader using the Urdu interface can navigate the entire book using translated navigation elements. Menu items, chapter titles in navigation, buttons, and UI labels all appear in Urdu.

**Why this priority**: Complete UI translation ensures a fully immersive Urdu reading experience but requires base content translation first.

**Independent Test**: Can be fully tested by navigating all pages in Urdu mode and verifying all UI elements are translated.

**Acceptance Scenarios**:

1. **Given** a reader is in Urdu mode, **When** they view the navigation menu, **Then** all menu items, chapter titles, and navigation links display in Urdu
2. **Given** a reader is in Urdu mode, **When** they view any button or UI control, **Then** the label displays in Urdu (e.g., "اگلا باب" instead of "Next Chapter")
3. **Given** a reader is in Urdu mode, **When** they view the table of contents, **Then** all chapter titles and section headings are translated

---

### Edge Cases

- What happens when a specific term has no Urdu equivalent? Preserve the English term in parentheses with Urdu transliteration, e.g., "روبوٹکس (Robotics)".
- What happens if translation is incomplete for a section? Display the English content with a subtle notice indicating "Translation in progress" in Urdu.
- How does the system handle mixed LTR/RTL content? Technical terms and code remain LTR within the RTL text flow using proper Unicode bidirectional controls.
- What happens to diagrams and images with English text? Images remain unchanged; alt text and captions are translated to Urdu.
- How does the chatbot respond in Urdu mode? Chatbot continues to respond in English; Urdu chatbot responses are out of scope for initial release.
- What happens if a reader's browser doesn't support RTL? Content still displays (potentially with alignment issues); recommend modern browser in notice.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-074**: System MUST provide complete Urdu translation for all 3 chapter contents
- **FR-075**: System MUST render Urdu text with Right-to-Left (RTL) direction
- **FR-076**: System MUST provide a language toggle accessible from any page
- **FR-077**: Language toggle MUST switch content within 2 seconds
- **FR-078**: System MUST preserve reader's scroll position when switching languages
- **FR-079**: System MUST remember user's language preference across sessions
- **FR-080**: System MUST translate all navigation elements (menus, buttons, links) to Urdu
- **FR-081**: System MUST translate all chapter titles, headings, and section labels to Urdu
- **FR-082**: System MUST preserve code examples in their original programming language (no translation)
- **FR-083**: System MUST translate explanatory text surrounding code examples
- **FR-084**: System MUST use proper Urdu typography including appropriate fonts for readability
- **FR-085**: System MUST handle mixed LTR/RTL content correctly (English terms within Urdu text)
- **FR-086**: System MUST translate image alt text and captions to Urdu
- **FR-087**: System MUST display translation quality notice for incomplete sections
- **FR-088**: System MUST maintain 85% or higher translation accuracy across all content

### Translation Scope

The following content MUST be translated:

1. **Chapter Content** → All 3 chapters (learning objectives, body text, key takeaways)
2. **Navigation UI** → Menus, buttons, links, table of contents
3. **Metadata** → Page titles, headings, section labels
4. **Supporting Text** → Image captions, alt text, code explanations
5. **Error Messages** → User-facing error and status messages

The following MUST NOT be translated:

1. **Code Examples** → Python, JavaScript, and other programming code
2. **Technical Commands** → Terminal commands, file paths, API endpoints
3. **Diagrams with Text** → Images containing English text (would require regeneration)
4. **Chatbot Responses** → RAG chatbot continues in English

### Key Entities

- **Translation Unit**: A segment of content with source (English) and target (Urdu) versions. Attributes include content reference, source text, translated text, translation status (complete/in-progress/pending), quality score.
- **Language Preference**: User's selected display language. Attributes include user reference (or session), selected language code, last updated timestamp.
- **Bilingual Content Block**: A content section that renders differently based on language selection. Attributes include English version, Urdu version, content type (text/heading/caption), RTL flag.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-047**: All 3 chapters have complete Urdu translations with 0 untranslated sections
- **SC-048**: Translation quality achieves 85% or higher accuracy score, measured by native Urdu speaker rating 20 randomly selected paragraphs on a rubric (fluency, accuracy, naturalness each scored 1-5, requiring average ≥4.25/5.0)
- **SC-049**: All Urdu text renders with correct RTL direction on all pages (verified visually on Chrome, Firefox, Safari, Edge)
- **SC-050**: Language toggle switches content within 2 seconds (p95 latency measured over 50 toggle actions)
- **SC-051**: Scroll position is preserved within 100 pixels when switching languages (verified over 10 test switches at different scroll positions)
- **SC-052**: 100% of navigation UI elements are translated (verified by exhaustive checklist of all menus, buttons, and links)
- **SC-053**: Code examples remain in original programming language in Urdu mode (verified by inspecting all code blocks in translated chapters)
- **SC-054**: Mixed LTR/RTL content displays correctly with English terms properly embedded in Urdu text (verified on 10 test paragraphs with technical terms)
- **SC-055**: Language preference persists across browser sessions (verified by selecting Urdu, closing browser, and reopening)
- **SC-056**: All success criteria (SC-047 to SC-055) MUST be verified and passing by Nov 30, 2025 6:00 PM submission deadline

## Assumptions

- English content (001-textbook-generation) is complete and stable before translation begins
- Native Urdu speaker is available for translation quality review
- Modern web browsers (Chrome, Firefox, Safari, Edge latest versions) support RTL rendering
- Urdu fonts are web-safe or can be loaded via standard web font methods
- Translation is manual or AI-assisted with human review (not fully automated)
- User's language preference can be stored in browser local storage or user account if authenticated
- 85% accuracy is measured by fluency, accuracy, and naturalness criteria by native speaker
- Technical terms in Physical AI/Robotics domain may not have established Urdu equivalents
- Chatbot translation is explicitly out of scope (per edge case documentation)
