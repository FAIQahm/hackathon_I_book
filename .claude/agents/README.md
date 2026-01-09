# Agent Team

This directory contains the agent definitions for the Physical AI Educational Book project.

## Team Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        AGENT TEAM                                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────┐      │
│  │ BookArchitect│    │ BackendIntegrator│    │ SecurityLead │      │
│  │  Content &   │    │    DevOps Lead   │    │    Auth      │      │
│  │ Infra Lead   │    │                  │    │  Specialist  │      │
│  └──────┬───────┘    └────────┬─────────┘    └──────┬───────┘      │
│         │                     │                      │              │
│         ▼                     ▼                      ▼              │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────┐      │
│  │ docusaurus-  │    │ vercel-fastapi-  │    │ auth-connect │      │
│  │ scaffold     │    │ link             │    │  (pending)   │      │
│  │ github-pages │    │                  │    │              │      │
│  │ -deploy      │    │                  │    │              │      │
│  └──────────────┘    └──────────────────┘    └──────────────┘      │
│                                                                      │
│  ┌──────────────┐    ┌──────────────────┐                          │
│  │  AIEngineer  │    │     Linguist     │                          │
│  │     RAG      │    │   Translation    │                          │
│  │  Specialist  │    │     Expert       │                          │
│  └──────┬───────┘    └────────┬─────────┘                          │
│         │                     │                                      │
│         ▼                     ▼                                      │
│  ┌──────────────┐    ┌──────────────────┐                          │
│  │qdrant-manager│    │ translation-sync │                          │
│  │rag-personali-│    │                  │                          │
│  │zer           │    │                  │                          │
│  └──────────────┘    └──────────────────┘                          │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

## Agents

| Agent | Role | Skills | Status |
|-------|------|--------|--------|
| [BookArchitect](./BookArchitect.md) | Content & Infrastructure Lead | docusaurus-scaffold, github-pages-deploy | Active |
| [BackendIntegrator](./BackendIntegrator.md) | DevOps Lead | vercel-fastapi-link | Active |
| [SecurityLead](./SecurityLead.md) | Authentication Specialist | auth-connect | Active |
| [AIEngineer](./AIEngineer.md) | RAG Specialist | qdrant-manager, rag-personalizer | Active |
| [Linguist](./Linguist.md) | Translation Expert | translation-sync | Active |

## Skill Inventory

| Skill | Version | Agent | Description |
|-------|---------|-------|-------------|
| docusaurus-scaffold | 1.2.0 | BookArchitect | Scaffold textbook chapters |
| github-pages-deploy | 1.0.0 | BookArchitect | Deploy to GitHub Pages |
| vercel-fastapi-link | 1.0.0 | BackendIntegrator | FastAPI on Vercel |
| auth-connect | 1.0.0 | SecurityLead | JWT authentication & RBAC |
| qdrant-manager | 1.1.0 | AIEngineer | Vector database management |
| rag-personalizer | 1.0.0 | AIEngineer | Personalized RAG responses |
| translation-sync | 1.1.0 | Linguist | EN/UR translation sync |

## Collaboration Matrix

```
                    BookArchitect  BackendIntegrator  SecurityLead  AIEngineer  Linguist
BookArchitect            -              API docs        Auth docs    Content     Deploy
BackendIntegrator    API docs              -            Auth impl    RAG API     -
SecurityLead        Auth docs         Auth impl            -         Secure      -
AIEngineer          Content           RAG API          Secure          -        Index
Linguist            Deploy               -                -          Index        -
```

## Usage

### Invoke an Agent
```
@BookArchitect scaffold 2 new chapters
@AIEngineer index the new content
@Linguist translate to Urdu
@BackendIntegrator deploy the API
```

### Check Agent Status
Each agent's status can be checked via their associated skills:
```bash
# BookArchitect - check deployment
.claude/skills/github-pages-deploy/scripts/setup.sh --status

# AIEngineer - check vector stats
.claude/skills/qdrant-manager/scripts/setup.sh --stats textbook_vectors

# Linguist - check translation status
.claude/skills/translation-sync/scripts/setup.sh --status
```

## Workflow Example

```
1. @BookArchitect scaffold new chapter
       ↓
2. [Manual] Write chapter content
       ↓
3. @Linguist translate to Urdu
       ↓
4. @AIEngineer index for RAG
       ↓
5. @BookArchitect deploy to GitHub Pages
       ↓
6. @BackendIntegrator verify API endpoints
```

## Adding New Agents

1. Create a new markdown file in this directory
2. Follow the template structure:
   - Metadata table
   - Skills table
   - Responsibilities
   - Capabilities with code examples
   - Decision authority matrix
   - Collaboration notes
   - Error handling
   - Metrics

## Future Enhancements

- [ ] Add agent orchestration system
- [ ] Implement agent-to-agent communication
- [ ] Add agent performance metrics dashboard
