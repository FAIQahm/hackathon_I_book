# Quickstart: RAG Chatbot Development

**Feature Branch**: `002-rag-chatbot`
**Created**: 2025-12-30

## Prerequisites

- Python 3.11+
- Node.js 18+ and npm
- Docker (optional, for local Qdrant)
- Git

## Environment Setup

### 1. Clone and Branch

```bash
git clone <repository-url>
cd hackathon-1
git checkout 002-rag-chatbot
```

### 2. Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or: venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt
```

### 3. Environment Variables

Create `.env` file in `backend/`:

```env
# OpenAI Configuration
OPENAI_API_KEY=your-openai-api-key

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION=physical_ai_book

# Neon Postgres Configuration
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# Application Settings
DEBUG=true
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000
```

### 4. Database Setup

```bash
# Run migrations
python -m alembic upgrade head

# Or manually create tables
python scripts/init_db.py
```

### 5. Frontend Setup

```bash
cd frontend

# Install dependencies
npm install

# Create .env.local
echo "NEXT_PUBLIC_API_URL=http://localhost:8000/api" > .env.local
```

## Running the Application

### Start Backend (Development)

```bash
cd backend
source venv/bin/activate
uvicorn src.main:app --reload --port 8000
```

Backend will be available at `http://localhost:8000`
- API docs: `http://localhost:8000/docs`
- Health check: `http://localhost:8000/api/health`

### Start Frontend (Development)

```bash
cd frontend
npm run dev
```

Frontend will be available at `http://localhost:3000`

## Content Indexing

Before the chatbot can answer questions, book content must be indexed:

```bash
cd backend
python scripts/index_content.py --content-dir ../content/chapters

# Options:
#   --content-dir   Path to chapter markdown files
#   --chunk-size    Token count per chunk (default: 512)
#   --overlap       Token overlap between chunks (default: 50)
#   --dry-run       Preview without indexing
```

### Verify Indexing

```bash
# Check collection stats
python scripts/check_index.py

# Expected output:
# Collection: physical_ai_book
# Points: ~150-200 (depends on content)
# Dimensions: 1536
```

## Testing

### Run Backend Tests

```bash
cd backend

# All tests
pytest

# Unit tests only
pytest tests/unit/

# Integration tests (requires running services)
pytest tests/integration/

# With coverage
pytest --cov=src --cov-report=html
```

### Run Frontend Tests

```bash
cd frontend

# All tests
npm test

# Watch mode
npm test -- --watch

# Coverage
npm test -- --coverage
```

### Test Questions Validation

```bash
cd backend

# Run accuracy test against 20 standard questions
python scripts/test_accuracy.py --questions tests/chatbot_test_questions.json

# Expected output:
# Accuracy: 18/20 (90%)
# Average response time: 1.2s
# Cache hit rate: 0% (first run)
```

## Local Development with Docker

For local Qdrant without cloud:

```bash
# Start Qdrant
docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant

# Update .env
QDRANT_URL=http://localhost:6333
# Remove QDRANT_API_KEY for local
```

## API Quick Reference

### Submit Query

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is Physical AI?",
    "session_id": "test-session-123"
  }'
```

### Get History

```bash
curl http://localhost:8000/api/history/test-session-123
```

### Health Check

```bash
curl http://localhost:8000/api/health
```

## Debugging

### Enable Debug Logging

```env
DEBUG=true
LOG_LEVEL=DEBUG
```

### Common Issues

| Issue | Solution |
|-------|----------|
| OpenAI rate limit | Check usage dashboard; implement backoff |
| Qdrant connection failed | Verify URL and API key; check firewall |
| Low accuracy | Re-index content; check chunk size |
| Slow responses | Enable caching; check network latency |

### Performance Profiling

```bash
# Profile a query
python scripts/profile_query.py "What is ROS 2?"

# Output:
# Embedding: 95ms
# Retrieval: 45ms
# Generation: 850ms
# Total: 990ms
```

## Project Structure

```
backend/
├── src/
│   ├── main.py              # FastAPI app entry
│   ├── api/                 # Route handlers
│   ├── services/            # Business logic
│   ├── cache/               # Caching layer
│   ├── models/              # Pydantic models
│   └── utils/               # Helpers
├── scripts/                 # CLI tools
├── tests/                   # Test suites
└── requirements.txt

frontend/
├── src/
│   ├── components/          # React components
│   ├── hooks/               # Custom hooks
│   ├── services/            # API client
│   └── theme/               # Docusaurus integration
└── package.json
```

## Next Steps

1. Complete Phase 1 (Content Indexing Pipeline)
2. Implement Phase 2 (RAG Query Engine)
3. Build Phase 3 (API Layer)
4. Develop Phase 4 (Frontend Components)

See `plan.md` for detailed implementation phases.
