#!/bin/bash
# =============================================================================
# Qdrant Manager Setup Script
# Manage Qdrant Cloud collections for RAG retrieval
# Version: 1.0.0
# Agent: AIEngineer
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Defaults
COLLECTION_NAME="textbook_chapters"
EMBEDDING_MODEL="text-embedding-3-small"
VECTOR_SIZE=1536
BATCH_SIZE=10
TOP_K=5

# Actions
ACTION=""
VECTORIZE_PATH=""
QUERY_TEXT=""

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

log_success() {
    echo -e "${GREEN}✓${NC} $1"
}

log_error() {
    echo -e "${RED}✗${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

show_help() {
    cat << EOF
Qdrant Manager - Manage Qdrant Cloud collections for RAG retrieval

Usage: setup.sh [ACTION] [OPTIONS]

Actions:
  --init                 Initialize/create the collection
  --vectorize PATH       Vectorize markdown files in PATH
  --query TEXT           Query the collection
  --delete               Delete the collection
  --info                 Show collection info
  --list                 List all collections

Options:
  --collection NAME      Collection name (default: $COLLECTION_NAME)
  --model MODEL          Embedding model (default: $EMBEDDING_MODEL)
  --batch-size N         Batch size for vectorization (default: $BATCH_SIZE)
  --top-k N              Number of results for query (default: $TOP_K)
  -h, --help             Show this help message

Environment Variables (required):
  QDRANT_URL             Qdrant Cloud cluster URL
  QDRANT_API_KEY         Qdrant API key
  OPENAI_API_KEY         OpenAI API key (for embeddings)

Examples:
  setup.sh --init
  setup.sh --vectorize docs/
  setup.sh --query "What is ROS 2?"
  setup.sh --info --collection my_collection
EOF
}

check_env() {
    local missing=0

    if [[ -z "$QDRANT_URL" ]]; then
        log_error "QDRANT_URL environment variable not set"
        missing=1
    fi

    if [[ -z "$QDRANT_API_KEY" ]]; then
        log_error "QDRANT_API_KEY environment variable not set"
        missing=1
    fi

    if [[ -z "$OPENAI_API_KEY" ]]; then
        log_error "OPENAI_API_KEY environment variable not set"
        missing=1
    fi

    if [[ $missing -eq 1 ]]; then
        echo ""
        log_info "Set environment variables in .env file:"
        echo "  QDRANT_URL=https://your-cluster.qdrant.io"
        echo "  QDRANT_API_KEY=your-api-key"
        echo "  OPENAI_API_KEY=sk-..."
        exit 1
    fi
}

check_python_deps() {
    local missing=()

    python3 -c "import qdrant_client" 2>/dev/null || missing+=("qdrant-client")
    python3 -c "import openai" 2>/dev/null || missing+=("openai")
    python3 -c "import tiktoken" 2>/dev/null || missing+=("tiktoken")
    python3 -c "import rich" 2>/dev/null || missing+=("rich")

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_warn "Missing Python packages: ${missing[*]}"
        log_info "Installing dependencies..."
        pip install "${missing[@]}" --quiet
        log_success "Dependencies installed"
    fi
}

# =============================================================================
# Parse Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --init)
            ACTION="init"
            shift
            ;;
        --vectorize)
            ACTION="vectorize"
            VECTORIZE_PATH="$2"
            shift 2
            ;;
        --query)
            ACTION="query"
            QUERY_TEXT="$2"
            shift 2
            ;;
        --delete)
            ACTION="delete"
            shift
            ;;
        --info)
            ACTION="info"
            shift
            ;;
        --list)
            ACTION="list"
            shift
            ;;
        --collection)
            COLLECTION_NAME="$2"
            shift 2
            ;;
        --model)
            EMBEDDING_MODEL="$2"
            shift 2
            ;;
        --batch-size)
            BATCH_SIZE="$2"
            shift 2
            ;;
        --top-k)
            TOP_K="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
done

# =============================================================================
# Validate
# =============================================================================

if [[ -z "$ACTION" ]]; then
    log_error "No action specified"
    echo ""
    show_help
    exit 1
fi

# Load .env if exists
if [[ -f "$PROJECT_ROOT/.env" ]]; then
    set -a
    source "$PROJECT_ROOT/.env"
    set +a
fi

check_env
check_python_deps

# =============================================================================
# Banner
# =============================================================================

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🔷 Qdrant Manager v1.0.0"
echo "   Agent: AIEngineer"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# =============================================================================
# Execute Action
# =============================================================================

case $ACTION in
    init)
        log_info "Initializing collection: $COLLECTION_NAME"

        python3 << EOF
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance
import os

client = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"]
)

try:
    client.get_collection("$COLLECTION_NAME")
    print("Collection '$COLLECTION_NAME' already exists")
except:
    client.create_collection(
        collection_name="$COLLECTION_NAME",
        vectors_config=VectorParams(size=$VECTOR_SIZE, distance=Distance.COSINE)
    )
    print("Created collection '$COLLECTION_NAME'")

info = client.get_collection("$COLLECTION_NAME")
print(f"Vectors: {info.vectors_count}")
print(f"Status: {info.status}")
EOF

        log_success "Collection initialized"
        ;;

    vectorize)
        if [[ -z "$VECTORIZE_PATH" ]]; then
            log_error "No path specified for vectorization"
            exit 1
        fi

        if [[ ! -d "$VECTORIZE_PATH" ]]; then
            log_error "Directory not found: $VECTORIZE_PATH"
            exit 1
        fi

        log_info "Vectorizing markdown files in: $VECTORIZE_PATH"
        log_info "Collection: $COLLECTION_NAME"
        log_info "Model: $EMBEDDING_MODEL"
        log_info "Batch size: $BATCH_SIZE"

        python3 << EOF
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
from openai import OpenAI
import tiktoken
import uuid
import os
import re
from pathlib import Path
from rich.progress import Progress, SpinnerColumn, TextColumn
from datetime import datetime

# Initialize clients
qdrant = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"]
)
openai_client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
encoder = tiktoken.encoding_for_model("$EMBEDDING_MODEL")

def extract_metadata(filepath, content):
    """Extract metadata from markdown file."""
    metadata = {
        "source": str(filepath),
        "language": "ur" if "/ur/" in str(filepath) or "/i18n/ur/" in str(filepath) else "en",
        "created_at": datetime.utcnow().isoformat() + "Z"
    }

    # Extract chapter number
    match = re.search(r'chapter-(\d+)', str(filepath))
    if match:
        metadata["chapter"] = int(match.group(1))

    # Extract title from first H1
    title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    if title_match:
        metadata["title"] = title_match.group(1).strip()

    return metadata

def chunk_markdown(content, max_tokens=500):
    """Split markdown by headers for semantic coherence."""
    chunks = []
    current_chunk = ""
    current_section = "Introduction"

    for line in content.split("\n"):
        if line.startswith("## "):
            if current_chunk.strip():
                chunks.append({"content": current_chunk.strip(), "section": current_section})
            current_section = line[3:].strip()
            current_chunk = line + "\n"
        elif line.startswith("### "):
            if len(encoder.encode(current_chunk)) > max_tokens // 2:
                chunks.append({"content": current_chunk.strip(), "section": current_section})
                current_chunk = ""
            current_section = line[4:].strip()
            current_chunk += line + "\n"
        else:
            current_chunk += line + "\n"
            if len(encoder.encode(current_chunk)) > max_tokens:
                chunks.append({"content": current_chunk.strip(), "section": current_section})
                current_chunk = ""

    if current_chunk.strip():
        chunks.append({"content": current_chunk.strip(), "section": current_section})

    return chunks

def embed(text):
    """Generate embedding for text."""
    response = openai_client.embeddings.create(
        input=text[:8000],  # Truncate to avoid token limit
        model="$EMBEDDING_MODEL"
    )
    return response.data[0].embedding

# Find all markdown files
path = Path("$VECTORIZE_PATH")
md_files = list(path.rglob("*.md"))

print(f"Found {len(md_files)} markdown files")

total_vectors = 0
batch = []

with Progress(
    SpinnerColumn(),
    TextColumn("[progress.description]{task.description}"),
) as progress:
    task = progress.add_task("Vectorizing...", total=len(md_files))

    for filepath in md_files:
        try:
            content = filepath.read_text(encoding='utf-8')
            metadata = extract_metadata(filepath, content)
            chunks = chunk_markdown(content)

            for i, chunk in enumerate(chunks):
                vector = embed(chunk["content"])
                point = PointStruct(
                    id=str(uuid.uuid4()),
                    vector=vector,
                    payload={
                        **metadata,
                        "content": chunk["content"],
                        "section": chunk["section"],
                        "chunk_index": i,
                        "total_chunks": len(chunks),
                        "word_count": len(chunk["content"].split())
                    }
                )
                batch.append(point)

                if len(batch) >= $BATCH_SIZE:
                    qdrant.upsert(collection_name="$COLLECTION_NAME", points=batch)
                    total_vectors += len(batch)
                    batch = []

            progress.advance(task)

        except Exception as e:
            print(f"Error processing {filepath}: {e}")

    # Upload remaining batch
    if batch:
        qdrant.upsert(collection_name="$COLLECTION_NAME", points=batch)
        total_vectors += len(batch)

print(f"\n✓ Vectorized {total_vectors} chunks from {len(md_files)} files")
EOF

        log_success "Vectorization complete"
        ;;

    query)
        if [[ -z "$QUERY_TEXT" ]]; then
            log_error "No query text specified"
            exit 1
        fi

        log_info "Querying: $QUERY_TEXT"
        log_info "Collection: $COLLECTION_NAME"
        log_info "Top-k: $TOP_K"
        echo ""

        python3 << EOF
from qdrant_client import QdrantClient
from openai import OpenAI
import os

# Initialize clients
qdrant = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"]
)
openai_client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])

# Generate query embedding
response = openai_client.embeddings.create(
    input="$QUERY_TEXT",
    model="$EMBEDDING_MODEL"
)
query_vector = response.data[0].embedding

# Search
results = qdrant.search(
    collection_name="$COLLECTION_NAME",
    query_vector=query_vector,
    limit=$TOP_K
)

print("━" * 50)
print("📚 Search Results")
print("━" * 50)

for i, hit in enumerate(results, 1):
    print(f"\n[{i}] Score: {hit.score:.4f}")
    print(f"    Source: {hit.payload.get('source', 'N/A')}")
    print(f"    Section: {hit.payload.get('section', 'N/A')}")
    print(f"    Language: {hit.payload.get('language', 'N/A')}")
    content = hit.payload.get('content', '')[:200]
    print(f"    Content: {content}...")

print("\n" + "━" * 50)
EOF
        ;;

    delete)
        log_warn "Deleting collection: $COLLECTION_NAME"
        read -p "Are you sure? (y/N) " confirm

        if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
            python3 << EOF
from qdrant_client import QdrantClient
import os

client = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"]
)

client.delete_collection("$COLLECTION_NAME")
print("Collection deleted")
EOF
            log_success "Collection deleted"
        else
            log_info "Cancelled"
        fi
        ;;

    info)
        log_info "Collection info: $COLLECTION_NAME"
        echo ""

        python3 << EOF
from qdrant_client import QdrantClient
import os

client = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"]
)

try:
    info = client.get_collection("$COLLECTION_NAME")
    print("━" * 50)
    print("📊 Collection Info")
    print("━" * 50)
    print(f"Name: $COLLECTION_NAME")
    print(f"Status: {info.status}")
    print(f"Vectors: {info.vectors_count}")
    print(f"Points: {info.points_count}")
    print(f"Indexed: {info.indexed_vectors_count}")
    print("━" * 50)
except Exception as e:
    print(f"Collection not found: {e}")
EOF
        ;;

    list)
        log_info "Listing all collections"
        echo ""

        python3 << EOF
from qdrant_client import QdrantClient
import os

client = QdrantClient(
    url=os.environ["QDRANT_URL"],
    api_key=os.environ["QDRANT_API_KEY"]
)

collections = client.get_collections()

print("━" * 50)
print("📚 Collections")
print("━" * 50)

for col in collections.collections:
    info = client.get_collection(col.name)
    print(f"  • {col.name} ({info.vectors_count} vectors)")

print("━" * 50)
EOF
        ;;
esac

echo ""
