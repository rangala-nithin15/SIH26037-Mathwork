"""Typed content model for a scraped MathWorks page.

The model mirrors the page's own structure so that every extracted element can
be traced back to the section it came from::

    Document
      |-- metadata (title, url, release, products, description, hashes)
      |-- sections: list[Section]        # flat list, ordered as on the page
      |     |-- heading: Heading         # text, level, anchor id
      |     |-- path: tuple[str, ...]    # ("Setup", "Motion Planner")
      |     |-- blocks: list[Block]      # text / code / image, document order
      |-- resources: list[Resource]      # links, downloads, related examples

Every block also carries ``source_section`` so that a block can stand alone
once serialised (this is what makes the ``# From: <section>`` attribution in
the exported files possible).
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


def _utcnow() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass
class Heading:
    text: str
    level: int
    anchor: Optional[str] = None

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Block:
    """Base class for anything appearing in the flow of a section."""

    kind: str = "block"
    source_section: str = ""
    section_path: Tuple[str, ...] = ()
    order: int = 0

    def to_dict(self) -> dict:
        d = asdict(self)
        d["section_path"] = list(self.section_path)
        return d


@dataclass
class TextBlock(Block):
    """A paragraph, list item group, note or equation rendered as text."""

    text: str = ""
    #: "paragraph" | "list" | "note" | "equation" | "caption"
    text_type: str = "paragraph"
    #: Hyperlinks inside this block: {"text", "url", "kind"}
    links: List[Dict[str, str]] = field(default_factory=list)
    #: Raw MathML for equations. Flattening MathML to text is lossy, so the
    #: original markup is kept for downstream rendering (KaTeX/MathJax).
    mathml: Optional[str] = None

    def __post_init__(self) -> None:
        self.kind = "text"


@dataclass
class CodeBlock(Block):
    """A code listing.

    ``language`` is the source language as published (always ``matlab`` on
    MathWorks example pages). ``python_equivalent`` is populated only when a
    faithful, non-speculative translation exists; otherwise it stays ``None``
    and ``translation_note`` explains why.
    """

    code: str = ""
    language: str = "matlab"
    #: "example" | "output" | "function" | "inline"
    block_type: str = "example"
    line_count: int = 0
    #: MATLAB identifiers referenced; useful for cross-linking to API docs.
    referenced_functions: List[str] = field(default_factory=list)
    python_equivalent: Optional[str] = None
    translation_note: Optional[str] = None

    def __post_init__(self) -> None:
        self.kind = "code"
        if not self.line_count:
            self.line_count = len(self.code.splitlines()) if self.code else 0

    @property
    def digest(self) -> str:
        return hashlib.sha256(self.code.encode("utf-8")).hexdigest()[:12]


@dataclass
class ImageAsset(Block):
    """A figure, screenshot or animation embedded in the page."""

    url: str = ""
    relative_src: str = ""
    alt: str = ""
    #: "figure" | "animation" | "diagram"
    media_type: str = "figure"
    caption: str = ""

    def __post_init__(self) -> None:
        self.kind = "image"


@dataclass
class Resource:
    """A link out of the page: download, related example, API reference."""

    url: str
    text: str
    #: "download" | "openExample" | "related_example" | "api_reference"
    #: | "product" | "external" | "internal"
    kind: str = "internal"
    filename: Optional[str] = None
    description: str = ""
    source_section: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Section:
    heading: Heading
    path: Tuple[str, ...]
    blocks: List[Block] = field(default_factory=list)

    @property
    def title(self) -> str:
        return self.heading.text

    def code_blocks(self) -> List[CodeBlock]:
        return [b for b in self.blocks if isinstance(b, CodeBlock)]

    def text_blocks(self) -> List[TextBlock]:
        return [b for b in self.blocks if isinstance(b, TextBlock)]

    def images(self) -> List[ImageAsset]:
        return [b for b in self.blocks if isinstance(b, ImageAsset)]

    def plain_text(self) -> str:
        return "\n\n".join(b.text for b in self.text_blocks() if b.text)

    def to_dict(self) -> dict:
        return {
            "heading": self.heading.to_dict(),
            "path": list(self.path),
            "blocks": [b.to_dict() for b in self.blocks],
        }


@dataclass
class Document:
    """The complete extraction result for one page."""

    url: str
    title: str = ""
    description: str = ""
    release: str = ""
    products: List[str] = field(default_factory=list)
    sections: List[Section] = field(default_factory=list)
    resources: List[Resource] = field(default_factory=list)
    scraped_at: str = field(default_factory=_utcnow)
    html_sha256: str = ""
    scraper_version: str = "1.0.0"
    warnings: List[str] = field(default_factory=list)

    # ----------------------------------------------------------------- views

    def iter_blocks(self) -> Iterable[Block]:
        for section in self.sections:
            yield from section.blocks

    def all_code(self, include_output: bool = False) -> List[CodeBlock]:
        return [
            b
            for b in self.iter_blocks()
            if isinstance(b, CodeBlock)
            and (include_output or b.block_type != "output")
        ]

    def all_images(self) -> List[ImageAsset]:
        return [b for b in self.iter_blocks() if isinstance(b, ImageAsset)]

    def section_by_title(self, title: str) -> Optional[Section]:
        wanted = title.strip().lower()
        for s in self.sections:
            if s.title.strip().lower() == wanted:
                return s
        return None

    def downloads(self) -> List[Resource]:
        return [r for r in self.resources if r.kind in {"download", "openExample"}]

    def summary(self) -> Dict[str, Any]:
        return {
            "url": self.url,
            "title": self.title,
            "sections": len(self.sections),
            "code_blocks": len(self.all_code()),
            "code_lines": sum(c.line_count for c in self.all_code()),
            "text_blocks": sum(
                1 for b in self.iter_blocks() if isinstance(b, TextBlock)
            ),
            "images": len(self.all_images()),
            "resources": len(self.resources),
            "warnings": len(self.warnings),
        }

    # --------------------------------------------------------------- exports

    def to_dict(self) -> dict:
        return {
            "metadata": {
                "url": self.url,
                "title": self.title,
                "description": self.description,
                "release": self.release,
                "products": self.products,
                "scraped_at": self.scraped_at,
                "html_sha256": self.html_sha256,
                "scraper_version": self.scraper_version,
            },
            "summary": self.summary(),
            "sections": [s.to_dict() for s in self.sections],
            "resources": [r.to_dict() for r in self.resources],
            "warnings": self.warnings,
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent, ensure_ascii=False)

    def to_json_file(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(self.to_json(), encoding="utf-8")
        return p

    def to_markdown(self) -> str:
        from .exporters import document_to_markdown

        return document_to_markdown(self)

    def to_markdown_file(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(self.to_markdown(), encoding="utf-8")
        return p

    def write_code_files(self, directory: str | Path) -> List[Path]:
        """Dump each MATLAB listing to its own ``.m`` file with attribution."""
        from .exporters import write_code_files

        return write_code_files(self, directory)
