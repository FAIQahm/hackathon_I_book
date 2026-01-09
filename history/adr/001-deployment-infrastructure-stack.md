# ADR-001: Deployment Infrastructure Stack

**Status**: Accepted
**Date**: 2025-12-31
**Deciders**: Project Team

## Context

The Physical AI Educational Book requires a deployment architecture that supports:
- Static documentation content (Docusaurus)
- Python-based FastAPI backend for RAG chatbot and authentication
- Persistent data storage for users, preferences, and sessions
- Vector storage for RAG embeddings
- Cost-effective hosting for an educational project

The architecture must be serverless-friendly, support both static and dynamic content, and work within free-tier constraints where possible.

## Decision

We adopt a **four-tier serverless deployment stack**:

| Component | Technology | Purpose |
|-----------|------------|---------|
| **Frontend Hosting** | GitHub Pages | Static Docusaurus site |
| **Backend Hosting** | Vercel Serverless Functions | FastAPI endpoints (auth, RAG, personalization) |
| **Relational Database** | Neon Serverless Postgres | Users, preferences, sessions, conversation history |
| **Vector Database** | Qdrant Cloud | RAG embeddings for content retrieval |

### Configuration Details

**GitHub Pages**:
- Triggered by GitHub Actions on push to `main`
- Static files only (no SSR)
- URL: `https://<org>.github.io/<project>/`

**Vercel**:
- Python 3.11 runtime
- Serverless functions with cold start optimization
- API routes under `/api/*`
- Environment variables managed via Vercel dashboard

**Neon Postgres**:
- Serverless Postgres with auto-scaling
- Connection pooling via `sslmode=require`
- Branching for development environments

**Qdrant Cloud**:
- Free tier (1GB storage, 1M vectors)
- REST API for vector operations
- Managed embeddings storage

## Consequences

### Positive

- **Cost Effective**: All services have generous free tiers suitable for educational project
- **No Infrastructure Management**: Fully managed serverless stack
- **Scalability**: Auto-scaling on all tiers handles traffic spikes
- **Developer Experience**: Simple deployment via git push and Vercel CLI
- **Separation of Concerns**: Static content separate from dynamic API

### Negative

- **Cold Starts**: Vercel serverless functions have ~500ms cold start
- **CORS Complexity**: Cross-origin requests between GitHub Pages and Vercel require careful configuration
- **Vendor Lock-in**: Tightly coupled to specific providers
- **Limited Customization**: Constrained by platform capabilities

### Risks

- **Performance**: Cold starts may affect <4s response time target (mitigated by keep-alive endpoints)
- **Rate Limits**: Free tier limits may be exceeded (mitigated by caching)

## Alternatives Considered

### Alternative 1: Single Platform (Vercel for Everything)
- **Pros**: Simpler CORS, unified deployment
- **Cons**: Higher cost for static hosting, less optimal for Docusaurus

### Alternative 2: Self-Hosted (Docker + VPS)
- **Pros**: Full control, no vendor lock-in
- **Cons**: Infrastructure management overhead, higher cost, requires DevOps expertise

### Alternative 3: AWS Stack (S3 + Lambda + RDS)
- **Pros**: Enterprise-grade, scalable
- **Cons**: Complex setup, higher learning curve, cost uncertainty

## References

- `CLAUDE.md` - Deployment & Infrastructure section
- `specs/001-textbook-generation/plan.md` - GitHub Pages deployment
- `specs/002-rag-chatbot/plan.md` - Vercel configuration
- `specs/003-authentication/plan.md` - Database and auth deployment
