# Research: RAG Chatbot for Physical AI Book

**Feature**: 002-rag-chatbot
**Date**: 2025-12-30

## Technology Decisions

### 1. LLM Provider: OpenAI GPT-4o-mini

**Decision**: Use OpenAI GPT-4o-mini for response generation
**Rationale**:
- Required by constitution (Tech Stack Mandatory)
- Cost-effective for educational chatbot use case
- Good balance of quality and speed for <2s response requirement
- Well-documented API with robust SDK

**Alternatives Considered**:
- GPT-4o: Better quality but higher latency and cost
- Claude: Different provider, not in constitution
- Local models: Infrastructure complexity, harder to meet latency targets

### 2. Vector Database: Qdrant Cloud

**Decision**: Use Qdrant Cloud for vector storage and similarity search
**Rationale**:
- Required by constitution (Tech Stack Mandatory)
- Managed service reduces operational overhead
- Good performance for semantic search
- Native Python client with async support

**Alternatives Considered**:
- Pinecone: Similar capabilities but not in constitution
- ChromaDB: Local-first, harder to scale
- pgvector: Could use with Neon, but Qdrant is specified

### 3. Embedding Model: text-embedding-ada-002

**Decision**: Use OpenAI's ada-002 for text embeddings
**Rationale**:
- Same provider as LLM (simplified auth)
- 1536 dimensions, good quality/size balance
- Well-tested for RAG applications
- Fast inference (~100ms)

**Configuration**:
- Dimension: 1536
- Batch size: 100 texts per request
- Max tokens: 8191 per text

### 4. Chunking Strategy

**Decision**: 512 tokens with 50 token overlap, section-aware
**Rationale**:
- 512 tokens fits well in context window
- Overlap prevents context loss at boundaries
- Section awareness preserves semantic coherence

**Implementation Details**:
```python
CHUNK_SIZE = 512  # tokens
CHUNK_OVERLAP = 50  # tokens
SEPARATORS = ["\n## ", "\n### ", "\n\n", "\n", " "]
```

### 5. Retrieval Configuration

**Decision**: Top-5 chunks with 0.7 similarity threshold
**Rationale**:
- 5 chunks provide enough context without noise
- 0.7 threshold filters out low-relevance results
- Cross-encoder reranking improves precision

**Configuration**:
```python
TOP_K = 5
SIMILARITY_THRESHOLD = 0.7
RERANK_MODEL = "cross-encoder/ms-marco-MiniLM-L-6-v2"
```

### 6. Caching Architecture

**Decision**: Two-tier cache (exact + semantic)
**Rationale**:
- Exact cache: Fast lookup for repeated queries
- Semantic cache: Handles paraphrased questions
- Meets <1s cached response requirement

**Cache Configuration**:
- Exact match TTL: 24 hours
- Semantic similarity threshold: 0.95
- Storage: Neon Postgres (per constitution)

### 7. Session Management

**Decision**: localStorage + Neon Postgres hybrid
**Rationale**:
- localStorage for session ID persistence
- Neon for conversation history storage
- Works without authentication (FR-024)
- 30-minute inactive TTL (SC-019)

### 8. Graceful Degradation

**Decision**: Tiered fallback system
**Rationale**:
- Per FR-027 and clarification answer
- Maintains some functionality during outages
- Clear status indication to users

**Fallback Tiers**:
1. Full RAG (Qdrant + OpenAI)
2. Cache-only (Qdrant fails → semantic cache lookup)
3. Keyword search (Qdrant fails → BM25 on indexed content)
4. Error message (both fail → friendly error)

## Best Practices Applied

### RAG Best Practices

1. **Chunk with overlap**: Prevents losing context at boundaries
2. **Include metadata**: Chapter/section info enables citations
3. **Confidence scoring**: Prevents hallucinations on low-confidence queries
4. **Explicit "I don't know"**: Maintains educational integrity

### API Best Practices

1. **Streaming responses**: Better UX for longer answers
2. **Request validation**: Query length limits, rate limiting
3. **Health endpoints**: Service monitoring and status
4. **Structured errors**: Consistent error response format

### Frontend Best Practices

1. **Keyboard accessibility**: Tab/Enter/Escape navigation
2. **Touch targets**: Min 44x44px for mobile
3. **Responsive design**: Works on mobile/tablet/desktop
4. **Loading states**: Clear indicators during processing

## Performance Optimization

### Meeting <2s Latency Target

1. **Embedding caching**: Cache query embeddings for repeated questions
2. **Connection pooling**: Reuse connections to external services
3. **Async processing**: Non-blocking I/O throughout
4. **Response streaming**: Start sending before full generation

### Meeting <1s Cached Latency

1. **In-memory cache layer**: Redis or local cache for hot queries
2. **Pre-computed embeddings**: Store embeddings with cached responses
3. **Index optimization**: Qdrant HNSW parameters tuned for speed

## Security Considerations

1. **No PII in logs**: Sanitize user queries before logging
2. **Rate limiting**: Prevent abuse and control costs
3. **Input validation**: Prevent injection attacks
4. **HTTPS only**: All API communication encrypted
