# Data Model: RAG Chatbot for Physical AI Book

**Feature Branch**: `002-rag-chatbot`
**Created**: 2025-12-30
**Status**: Approved

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────┐       ┌─────────────────┐
│   Conversation  │───────│     Message     │───────│    Citation     │
│                 │ 1   N │                 │ 1   N │                 │
└─────────────────┘       └─────────────────┘       └─────────────────┘
        │                         │
        │                         │
        ▼                         ▼
┌─────────────────┐       ┌─────────────────┐
│     Session     │       │     Chunk       │
│  (localStorage) │       │   (Qdrant)      │
└─────────────────┘       └─────────────────┘
```

## Entities

### 1. Conversation

A session of exchanges between a reader and the chatbot.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| session_id | String | Not Null, Indexed | Browser session identifier |
| created_at | Timestamp | Not Null | Conversation start time |
| updated_at | Timestamp | Not Null | Last activity time |
| message_count | Integer | Not Null, Default 0 | Number of messages |
| status | Enum | Not Null | active, expired, archived |

**Constraints**:
- Session expires after 30 minutes of inactivity (SC-019)
- Maximum 100 messages per conversation

**Indexes**:
- `idx_conversation_session` on (session_id)
- `idx_conversation_updated` on (updated_at)

---

### 2. Message

A single query or response in a conversation.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| conversation_id | UUID | Foreign Key, Not Null | Parent conversation |
| role | Enum | Not Null | user, assistant |
| content | Text | Not Null, Max 10000 chars | Message text |
| selected_text | Text | Nullable, Max 1000 chars | Text selection context |
| confidence_score | Float | Nullable, 0.0-1.0 | Response confidence |
| response_time_ms | Integer | Nullable | Time to generate (ms) |
| cached | Boolean | Default false | Whether from cache |
| degraded_mode | Boolean | Default false | Whether in fallback mode |
| created_at | Timestamp | Not Null | Message timestamp |

**Constraints**:
- User messages limited to 500 characters (per spec)
- Selected text limited to 1000 characters
- Response time tracked for performance monitoring

**Indexes**:
- `idx_message_conversation` on (conversation_id)
- `idx_message_created` on (created_at)

---

### 3. Citation

A reference to source material in a response.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| message_id | UUID | Foreign Key, Not Null | Parent message |
| chapter_id | String | Not Null | Chapter identifier |
| section_heading | String | Not Null | Section title |
| chunk_id | String | Not Null | Qdrant chunk reference |
| relevance_score | Float | Not Null, 0.0-1.0 | Retrieval similarity |
| excerpt | Text | Max 500 chars | Source text excerpt |

**Constraints**:
- Citations only for assistant messages
- Minimum 1 citation for substantive answers (>50 words) per SC-015

**Indexes**:
- `idx_citation_message` on (message_id)
- `idx_citation_chapter` on (chapter_id)

---

### 4. Chunk (Vector Store - Qdrant)

Indexed content segment for retrieval.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key | Qdrant point ID |
| content | Text | Not Null | Chunk text content |
| embedding | Vector[1536] | Not Null | ada-002 embedding |
| chapter_id | String | Payload | Source chapter |
| section_heading | String | Payload | Section title |
| paragraph_index | Integer | Payload | Position in section |
| token_count | Integer | Payload | Chunk size in tokens |
| created_at | Timestamp | Payload | Indexing timestamp |

**Qdrant Configuration**:
- Collection: `physical_ai_book`
- Vector size: 1536 (ada-002)
- Distance metric: Cosine
- HNSW index: m=16, ef_construct=100

---

### 5. ResponseCache (Neon Postgres)

Cached responses for performance optimization.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | Primary Key, Not Null | Unique identifier |
| query_hash | String | Not Null, Unique | Hash of normalized query |
| query_embedding | Vector[1536] | Not Null | Query embedding for semantic match |
| response_content | Text | Not Null | Cached response |
| citations_json | JSONB | Not Null | Serialized citations |
| hit_count | Integer | Default 0 | Cache hit counter |
| created_at | Timestamp | Not Null | Cache entry time |
| expires_at | Timestamp | Not Null | TTL expiration |

**Constraints**:
- TTL: 24 hours for exact match cache
- Semantic similarity threshold: 0.95 for semantic cache hits

**Indexes**:
- `idx_cache_query_hash` on (query_hash)
- `idx_cache_expires` on (expires_at)
- `idx_cache_embedding` using ivfflat for vector similarity

---

### 6. ServiceHealth (Runtime Monitoring)

Health status of external services.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| service_name | String | Primary Key | Service identifier |
| status | Enum | Not Null | healthy, degraded, down |
| last_check | Timestamp | Not Null | Last health check |
| failure_count | Integer | Default 0 | Consecutive failures |
| circuit_state | Enum | Not Null | closed, open, half_open |

**Services Monitored**:
- `openai`: LLM service
- `qdrant`: Vector database
- `neon`: Postgres cache

---

## Enumerations

### MessageRole
```python
class MessageRole(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"
```

### ConversationStatus
```python
class ConversationStatus(str, Enum):
    ACTIVE = "active"
    EXPIRED = "expired"
    ARCHIVED = "archived"
```

### ServiceStatus
```python
class ServiceStatus(str, Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    DOWN = "down"
```

### CircuitState
```python
class CircuitState(str, Enum):
    CLOSED = "closed"      # Normal operation
    OPEN = "open"          # Failing, rejecting requests
    HALF_OPEN = "half_open" # Testing recovery
```

---

## Relationships

| Parent | Child | Cardinality | Description |
|--------|-------|-------------|-------------|
| Conversation | Message | 1:N | Conversation contains messages |
| Message | Citation | 1:N | Response includes citations |
| Chunk | Citation | 1:N | Chunk referenced by citations |

---

## Database Schema (Neon Postgres)

```sql
-- Enable vector extension
CREATE EXTENSION IF NOT EXISTS vector;

-- Conversations table
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    message_count INTEGER DEFAULT 0,
    status VARCHAR(20) DEFAULT 'active'
);

CREATE INDEX idx_conversation_session ON conversations(session_id);
CREATE INDEX idx_conversation_updated ON conversations(updated_at);

-- Messages table
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL,
    content TEXT NOT NULL,
    selected_text TEXT,
    confidence_score FLOAT,
    response_time_ms INTEGER,
    cached BOOLEAN DEFAULT FALSE,
    degraded_mode BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_message_conversation ON messages(conversation_id);
CREATE INDEX idx_message_created ON messages(created_at);

-- Citations table
CREATE TABLE citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    chapter_id VARCHAR(50) NOT NULL,
    section_heading VARCHAR(255) NOT NULL,
    chunk_id VARCHAR(100) NOT NULL,
    relevance_score FLOAT NOT NULL,
    excerpt TEXT
);

CREATE INDEX idx_citation_message ON citations(message_id);
CREATE INDEX idx_citation_chapter ON citations(chapter_id);

-- Response cache table
CREATE TABLE response_cache (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query_hash VARCHAR(64) NOT NULL UNIQUE,
    query_embedding vector(1536) NOT NULL,
    response_content TEXT NOT NULL,
    citations_json JSONB NOT NULL,
    hit_count INTEGER DEFAULT 0,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL
);

CREATE INDEX idx_cache_query_hash ON response_cache(query_hash);
CREATE INDEX idx_cache_expires ON response_cache(expires_at);
CREATE INDEX idx_cache_embedding ON response_cache
    USING ivfflat (query_embedding vector_cosine_ops) WITH (lists = 100);

-- Service health table
CREATE TABLE service_health (
    service_name VARCHAR(50) PRIMARY KEY,
    status VARCHAR(20) NOT NULL DEFAULT 'healthy',
    last_check TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    failure_count INTEGER DEFAULT 0,
    circuit_state VARCHAR(20) DEFAULT 'closed'
);

-- Insert initial service records
INSERT INTO service_health (service_name, status, circuit_state) VALUES
    ('openai', 'healthy', 'closed'),
    ('qdrant', 'healthy', 'closed'),
    ('neon', 'healthy', 'closed');
```

---

## Pydantic Models (Python)

```python
from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from uuid import UUID
from enum import Enum

class MessageRole(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"

class Citation(BaseModel):
    chapter_id: str
    section_heading: str
    chunk_id: str
    relevance_score: float = Field(ge=0.0, le=1.0)
    excerpt: Optional[str] = Field(max_length=500)

class Message(BaseModel):
    id: UUID
    conversation_id: UUID
    role: MessageRole
    content: str = Field(max_length=10000)
    selected_text: Optional[str] = Field(max_length=1000)
    confidence_score: Optional[float] = Field(ge=0.0, le=1.0)
    response_time_ms: Optional[int]
    cached: bool = False
    degraded_mode: bool = False
    citations: List[Citation] = []
    created_at: datetime

class Conversation(BaseModel):
    id: UUID
    session_id: str
    created_at: datetime
    updated_at: datetime
    message_count: int = 0
    messages: List[Message] = []

class QueryRequest(BaseModel):
    query: str = Field(max_length=500)
    session_id: str
    selected_text: Optional[str] = Field(max_length=1000)
    conversation_id: Optional[UUID]

class QueryResponse(BaseModel):
    message: Message
    conversation_id: UUID
    cached: bool
    degraded_mode: bool
    service_status: dict
```

---

## TypeScript Interfaces (Frontend)

```typescript
interface Citation {
  chapterId: string;
  sectionHeading: string;
  relevanceScore: number;
  excerpt?: string;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  selectedText?: string;
  citations: Citation[];
  cached: boolean;
  degradedMode: boolean;
  timestamp: Date;
}

interface Conversation {
  id: string;
  sessionId: string;
  messages: Message[];
  createdAt: Date;
  updatedAt: Date;
}

interface ChatState {
  conversation: Conversation | null;
  isLoading: boolean;
  error: string | null;
  serviceStatus: {
    openai: 'healthy' | 'degraded' | 'down';
    qdrant: 'healthy' | 'degraded' | 'down';
  };
}
```
