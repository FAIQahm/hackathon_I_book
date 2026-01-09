#!/usr/bin/env python3
"""
Qdrant Manager - Vectorization Module
Standalone Python module for textbook vectorization and RAG queries.

Usage:
    from vectorize import QdrantManager

    manager = QdrantManager()
    manager.init_collection("textbook_chapters")
    manager.vectorize_directory("docs/")
    results = manager.query("What is ROS 2?", language="en")
"""

import os
import re
import json
import uuid
import hashlib
from pathlib import Path
from datetime import datetime, timezone
from typing import Optional

try:
    from qdrant_client import QdrantClient
    from qdrant_client.models import (
        PointStruct,
        VectorParams,
        Distance,
        Filter,
        FieldCondition,
        MatchValue
    )
    from openai import OpenAI
    import tiktoken
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Run: pip install qdrant-client openai tiktoken")
    raise


class QdrantManager:
    """Manage Qdrant collections for RAG retrieval."""

    DEFAULT_COLLECTION = "textbook_chapters"
    DEFAULT_MODEL = "text-embedding-3-small"
    DEFAULT_VECTOR_SIZE = 1536
    DEFAULT_CHUNK_SIZE = 500
    DEFAULT_CHUNK_OVERLAP = 100
    STATE_FILE = ".vectorize-state.json"

    def __init__(
        self,
        qdrant_url: Optional[str] = None,
        qdrant_api_key: Optional[str] = None,
        openai_api_key: Optional[str] = None
    ):
        """Initialize clients from parameters or environment variables."""
        self.qdrant_url = qdrant_url or os.environ.get("QDRANT_URL")
        self.qdrant_api_key = qdrant_api_key or os.environ.get("QDRANT_API_KEY")
        self.openai_api_key = openai_api_key or os.environ.get("OPENAI_API_KEY")

        if not all([self.qdrant_url, self.qdrant_api_key, self.openai_api_key]):
            raise ValueError(
                "Missing credentials. Set QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY"
            )

        self.qdrant = QdrantClient(url=self.qdrant_url, api_key=self.qdrant_api_key)
        self.openai = OpenAI(api_key=self.openai_api_key)
        self.encoder = tiktoken.encoding_for_model(self.DEFAULT_MODEL)
        self.model = self.DEFAULT_MODEL

    def set_model(self, model: str, vector_size: int):
        """Set embedding model and vector size."""
        self.model = model
        self.vector_size = vector_size
        self.encoder = tiktoken.encoding_for_model(model)

    # =========================================================================
    # Collection Management
    # =========================================================================

    def init_collection(
        self,
        name: str = DEFAULT_COLLECTION,
        vector_size: int = DEFAULT_VECTOR_SIZE
    ) -> dict:
        """Create or verify collection exists."""
        try:
            info = self.qdrant.get_collection(name)
            return {
                "status": "exists",
                "name": name,
                "vectors": info.vectors_count
            }
        except Exception:
            self.qdrant.create_collection(
                collection_name=name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
            )
            return {"status": "created", "name": name, "vectors": 0}

    def delete_collection(self, name: str = DEFAULT_COLLECTION) -> dict:
        """Delete a collection."""
        self.qdrant.delete_collection(name)
        return {"status": "deleted", "name": name}

    def get_collection_info(self, name: str = DEFAULT_COLLECTION) -> dict:
        """Get collection statistics."""
        try:
            info = self.qdrant.get_collection(name)
            return {
                "name": name,
                "status": str(info.status),
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "indexed_vectors_count": info.indexed_vectors_count
            }
        except Exception as e:
            return {"error": str(e)}

    def list_collections(self) -> list:
        """List all collections with stats."""
        collections = self.qdrant.get_collections()
        result = []
        for col in collections.collections:
            info = self.qdrant.get_collection(col.name)
            result.append({
                "name": col.name,
                "vectors": info.vectors_count
            })
        return result

    # =========================================================================
    # Embedding & Chunking
    # =========================================================================

    def embed(self, text: str) -> list:
        """Generate embedding for text."""
        # Truncate to avoid token limits
        truncated = text[:8000]
        response = self.openai.embeddings.create(
            input=truncated,
            model=self.model
        )
        return response.data[0].embedding

    def chunk_markdown(
        self,
        content: str,
        max_tokens: int = DEFAULT_CHUNK_SIZE
    ) -> list:
        """Split markdown by headers for semantic coherence."""
        chunks = []
        current_chunk = ""
        current_section = "Introduction"

        for line in content.split("\n"):
            if line.startswith("## "):
                if current_chunk.strip():
                    chunks.append({
                        "content": current_chunk.strip(),
                        "section": current_section
                    })
                current_section = line[3:].strip()
                current_chunk = line + "\n"
            elif line.startswith("### "):
                if len(self.encoder.encode(current_chunk)) > max_tokens // 2:
                    chunks.append({
                        "content": current_chunk.strip(),
                        "section": current_section
                    })
                    current_chunk = ""
                current_section = line[4:].strip()
                current_chunk += line + "\n"
            else:
                current_chunk += line + "\n"
                if len(self.encoder.encode(current_chunk)) > max_tokens:
                    chunks.append({
                        "content": current_chunk.strip(),
                        "section": current_section
                    })
                    current_chunk = ""

        if current_chunk.strip():
            chunks.append({
                "content": current_chunk.strip(),
                "section": current_section
            })

        return chunks

    def extract_metadata(self, filepath: Path, content: str) -> dict:
        """Extract metadata from markdown file."""
        filepath_str = str(filepath)

        metadata = {
            "source": filepath_str,
            "language": "ur" if "/ur/" in filepath_str or "/i18n/ur/" in filepath_str else "en",
            "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        }

        # Extract chapter number
        match = re.search(r'chapter-(\d+)', filepath_str)
        if match:
            metadata["chapter"] = int(match.group(1))

        # Extract title from first H1
        title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if title_match:
            metadata["title"] = title_match.group(1).strip()

        # Extract keywords from front matter or content
        keywords = []
        if "ros2" in content.lower() or "ros 2" in content.lower():
            keywords.append("ros2")
        if "gazebo" in content.lower():
            keywords.append("gazebo")
        if "robot" in content.lower():
            keywords.append("robotics")
        metadata["keywords"] = keywords

        return metadata

    # =========================================================================
    # Vectorization
    # =========================================================================

    def _get_file_hash(self, filepath: Path) -> str:
        """Calculate MD5 hash of file content."""
        content = filepath.read_bytes()
        return hashlib.md5(content).hexdigest()

    def _load_state(self, state_path: Path) -> dict:
        """Load vectorization state from file."""
        if state_path.exists():
            return json.loads(state_path.read_text())
        return {"files": {}, "last_run": None}

    def _save_state(self, state_path: Path, state: dict):
        """Save vectorization state to file."""
        state["last_run"] = datetime.now(timezone.utc).isoformat()
        state_path.write_text(json.dumps(state, indent=2))

    def vectorize_file(
        self,
        filepath: Path,
        collection: str = DEFAULT_COLLECTION
    ) -> int:
        """Vectorize a single markdown file."""
        content = filepath.read_text(encoding='utf-8')
        metadata = self.extract_metadata(filepath, content)
        chunks = self.chunk_markdown(content)

        points = []
        for i, chunk in enumerate(chunks):
            vector = self.embed(chunk["content"])
            points.append(PointStruct(
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
            ))

        if points:
            self.qdrant.upsert(collection_name=collection, points=points)

        return len(points)

    def vectorize_directory(
        self,
        directory: str,
        collection: str = DEFAULT_COLLECTION,
        batch_size: int = 10,
        update_only: bool = False,
        callback=None
    ) -> dict:
        """
        Vectorize all markdown files in directory.

        Args:
            directory: Path to directory containing markdown files
            collection: Qdrant collection name
            batch_size: Number of points to upsert at once
            update_only: If True, only process files changed since last run
            callback: Optional callback(filepath, status) for progress

        Returns:
            dict with stats: total_files, processed, skipped, vectors
        """
        path = Path(directory)
        md_files = list(path.rglob("*.md"))

        # State tracking for incremental updates
        state_path = path / self.STATE_FILE
        state = self._load_state(state_path) if update_only else {"files": {}}

        stats = {
            "total_files": len(md_files),
            "processed": 0,
            "skipped": 0,
            "vectors": 0,
            "errors": []
        }

        batch = []

        for filepath in md_files:
            try:
                filepath_str = str(filepath)
                file_hash = self._get_file_hash(filepath)

                # Skip unchanged files in update mode
                if update_only and state["files"].get(filepath_str) == file_hash:
                    stats["skipped"] += 1
                    if callback:
                        callback(filepath, "skipped")
                    continue

                # Process file
                content = filepath.read_text(encoding='utf-8')
                metadata = self.extract_metadata(filepath, content)
                chunks = self.chunk_markdown(content)

                for i, chunk in enumerate(chunks):
                    vector = self.embed(chunk["content"])
                    batch.append(PointStruct(
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
                    ))

                    # Upsert batch
                    if len(batch) >= batch_size:
                        self.qdrant.upsert(collection_name=collection, points=batch)
                        stats["vectors"] += len(batch)
                        batch = []

                # Update state
                state["files"][filepath_str] = file_hash
                stats["processed"] += 1

                if callback:
                    callback(filepath, "processed")

            except Exception as e:
                stats["errors"].append({"file": str(filepath), "error": str(e)})
                if callback:
                    callback(filepath, f"error: {e}")

        # Upload remaining batch
        if batch:
            self.qdrant.upsert(collection_name=collection, points=batch)
            stats["vectors"] += len(batch)

        # Save state for incremental updates
        if update_only or stats["processed"] > 0:
            self._save_state(state_path, state)

        return stats

    # =========================================================================
    # Query
    # =========================================================================

    def query(
        self,
        text: str,
        collection: str = DEFAULT_COLLECTION,
        top_k: int = 5,
        language: Optional[str] = None,
        chapter: Optional[int] = None
    ) -> list:
        """
        Query the collection with optional filters.

        Args:
            text: Query text
            collection: Collection name
            top_k: Number of results
            language: Filter by language ("en" or "ur")
            chapter: Filter by chapter number

        Returns:
            List of results with score, source, section, content
        """
        query_vector = self.embed(text)

        # Build filter conditions
        filter_conditions = []
        if language:
            filter_conditions.append(
                FieldCondition(key="language", match=MatchValue(value=language))
            )
        if chapter is not None:
            filter_conditions.append(
                FieldCondition(key="chapter", match=MatchValue(value=chapter))
            )

        query_filter = None
        if filter_conditions:
            query_filter = Filter(must=filter_conditions)

        results = self.qdrant.search(
            collection_name=collection,
            query_vector=query_vector,
            query_filter=query_filter,
            limit=top_k
        )

        return [
            {
                "score": hit.score,
                "source": hit.payload.get("source", "N/A"),
                "section": hit.payload.get("section", "N/A"),
                "language": hit.payload.get("language", "N/A"),
                "chapter": hit.payload.get("chapter"),
                "content": hit.payload.get("content", "")[:500]
            }
            for hit in results
        ]


# =============================================================================
# CLI Interface
# =============================================================================

def main():
    """Command-line interface for QdrantManager."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Qdrant Manager - Vectorization and RAG queries"
    )

    subparsers = parser.add_subparsers(dest="command", help="Commands")

    # init
    init_parser = subparsers.add_parser("init", help="Initialize collection")
    init_parser.add_argument("--collection", default="textbook_chapters")
    init_parser.add_argument("--vector-size", type=int, default=1536)

    # vectorize
    vec_parser = subparsers.add_parser("vectorize", help="Vectorize markdown files")
    vec_parser.add_argument("path", help="Directory path")
    vec_parser.add_argument("--collection", default="textbook_chapters")
    vec_parser.add_argument("--batch-size", type=int, default=10)
    vec_parser.add_argument("--update", action="store_true",
                           help="Only process changed files")

    # query
    query_parser = subparsers.add_parser("query", help="Query collection")
    query_parser.add_argument("text", help="Query text")
    query_parser.add_argument("--collection", default="textbook_chapters")
    query_parser.add_argument("--top-k", type=int, default=5)
    query_parser.add_argument("--language", choices=["en", "ur"],
                             help="Filter by language")
    query_parser.add_argument("--chapter", type=int, help="Filter by chapter")

    # info
    info_parser = subparsers.add_parser("info", help="Show collection info")
    info_parser.add_argument("--collection", default="textbook_chapters")

    # list
    subparsers.add_parser("list", help="List all collections")

    # delete
    del_parser = subparsers.add_parser("delete", help="Delete collection")
    del_parser.add_argument("--collection", default="textbook_chapters")
    del_parser.add_argument("--force", action="store_true")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    try:
        manager = QdrantManager()

        if args.command == "init":
            result = manager.init_collection(args.collection, args.vector_size)
            print(f"Collection '{result['name']}': {result['status']}")

        elif args.command == "vectorize":
            def progress(fp, status):
                print(f"  {status}: {fp}")

            print(f"Vectorizing {args.path}...")
            stats = manager.vectorize_directory(
                args.path,
                collection=args.collection,
                batch_size=args.batch_size,
                update_only=args.update,
                callback=progress
            )
            print(f"\nProcessed: {stats['processed']}/{stats['total_files']}")
            print(f"Skipped: {stats['skipped']}")
            print(f"Vectors: {stats['vectors']}")

        elif args.command == "query":
            results = manager.query(
                args.text,
                collection=args.collection,
                top_k=args.top_k,
                language=args.language,
                chapter=args.chapter
            )
            print(f"\nResults for: '{args.text}'")
            print("=" * 50)
            for i, r in enumerate(results, 1):
                print(f"\n[{i}] Score: {r['score']:.4f}")
                print(f"    Source: {r['source']}")
                print(f"    Section: {r['section']}")
                print(f"    Language: {r['language']}")
                print(f"    Content: {r['content'][:200]}...")

        elif args.command == "info":
            info = manager.get_collection_info(args.collection)
            if "error" in info:
                print(f"Error: {info['error']}")
            else:
                print(f"Collection: {info['name']}")
                print(f"Status: {info['status']}")
                print(f"Vectors: {info['vectors_count']}")

        elif args.command == "list":
            collections = manager.list_collections()
            print("Collections:")
            for col in collections:
                print(f"  - {col['name']} ({col['vectors']} vectors)")

        elif args.command == "delete":
            if not args.force:
                confirm = input(f"Delete '{args.collection}'? (y/N): ")
                if confirm.lower() != 'y':
                    print("Cancelled")
                    return
            manager.delete_collection(args.collection)
            print(f"Deleted: {args.collection}")

    except Exception as e:
        print(f"Error: {e}")
        raise


if __name__ == "__main__":
    main()
