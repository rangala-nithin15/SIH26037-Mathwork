"""Rendering a :class:`~mwscraper.models.Document` into shareable artefacts.

Three exports are provided:

* :func:`document_to_markdown` - one readable file preserving section order,
  prose, fenced MATLAB code, equations, figures and a resource index.
* :func:`write_code_files` - each MATLAB listing as its own ``.m`` file, with
  a provenance header (``% From: <section>``).
* :func:`document_to_sections_dict` - a flat ``{section: {...}}`` mapping for
  programmatic consumers that do not want the nested block model.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, List

from .models import CodeBlock, Document, ImageAsset, TextBlock


def _slugify(value: str, fallback: str = "section") -> str:
    slug = re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")
    return slug or fallback


def document_to_markdown(doc: Document) -> str:
    """Render the whole document as Markdown with source attribution."""
    out: List[str] = []
    add = out.append

    add(f"# {doc.title or 'Untitled'}")
    add("")
    if doc.description:
        add(f"> {doc.description}")
        add("")
    add("| Field | Value |")
    add("| --- | --- |")
    add(f"| Source URL | <{doc.url}> |")
    if doc.release:
        add(f"| Introduced | {doc.release} |")
    if doc.products:
        add(f"| Products | {', '.join(doc.products)} |")
    add(f"| Scraped at | {doc.scraped_at} |")
    add(f"| Content SHA-256 | `{doc.html_sha256[:16]}...` |")
    summary = doc.summary()
    add(
        f"| Extracted | {summary['sections']} sections, "
        f"{summary['code_blocks']} code blocks "
        f"({summary['code_lines']} lines), "
        f"{summary['text_blocks']} text blocks, "
        f"{summary['images']} images |"
    )
    add("")

    if doc.warnings:
        add("**Extraction warnings**")
        add("")
        for warning in doc.warnings:
            add(f"- {warning}")
        add("")

    add("## Table of contents")
    add("")
    for section in doc.sections:
        if not section.blocks:
            continue
        indent = "  " * max(0, section.heading.level - 1)
        add(f"{indent}- [{section.title}](#{_slugify(section.title)})")
    add("")
    add("---")
    add("")

    for section in doc.sections:
        if not section.blocks:
            continue
        hashes = "#" * min(6, max(2, section.heading.level))
        add(f"{hashes} {section.title}")
        add("")
        if len(section.path) > 1:
            add(f"*Section path: {' > '.join(section.path)}*")
            add("")

        for block in section.blocks:
            if isinstance(block, TextBlock):
                if block.text_type == "equation":
                    add("**Equation (rendered from MathML):**")
                    add("")
                    add("```text")
                    add(block.text)
                    add("```")
                elif block.text_type == "list":
                    add(block.text)
                elif block.text_type == "note":
                    add(f"> **Note:** {block.text}")
                else:
                    add(block.text)
                if block.links:
                    refs = ", ".join(
                        f"[{link['text']}]({link['url']})" for link in block.links
                    )
                    add("")
                    add(f"*Links in this paragraph: {refs}*")
                add("")

            elif isinstance(block, CodeBlock):
                add(f"```matlab")
                add(f"% From: {block.source_section}")
                add(block.code)
                add("```")
                add("")
                if block.referenced_functions:
                    add(
                        "*Functions referenced: "
                        + ", ".join(f"`{fn}`" for fn in block.referenced_functions)
                        + "*"
                    )
                    add("")
                if block.python_equivalent:
                    add("<details><summary>Python equivalent</summary>")
                    add("")
                    add("```python")
                    add(f"# From: {block.source_section}")
                    add(block.python_equivalent)
                    add("```")
                    add("")
                    add("</details>")
                    add("")
                elif block.translation_note:
                    add(f"*Translation: {block.translation_note}*")
                    add("")

            elif isinstance(block, ImageAsset):
                label = block.alt or block.relative_src.rsplit("/", 1)[-1]
                add(f"![{label}]({block.url})")
                add("")
                add(f"*Figure ({block.media_type}) from: {block.source_section}*")
                add("")

        add("---")
        add("")

    add("## Resources")
    add("")
    if not doc.resources:
        add("*No resources extracted.*")
    else:
        by_kind: Dict[str, List] = {}
        for resource in doc.resources:
            by_kind.setdefault(resource.kind, []).append(resource)
        for kind in sorted(by_kind):
            add(f"### {kind.replace('_', ' ').title()}")
            add("")
            add("| Name | File | URL | Description | From section |")
            add("| --- | --- | --- | --- | --- |")
            for resource in by_kind[kind]:
                url = resource.url.replace("|", "%7C")
                add(
                    f"| {resource.text} | {resource.filename or '-'} | "
                    f"{url} | {resource.description} | "
                    f"{resource.source_section or '-'} |"
                )
            add("")
    add("")

    return "\n".join(out)


def write_code_files(doc: Document, directory: str | Path) -> List[Path]:
    """Write every MATLAB listing to ``directory`` as a separate ``.m`` file."""
    out_dir = Path(directory)
    out_dir.mkdir(parents=True, exist_ok=True)
    written: List[Path] = []
    counters: Dict[str, int] = {}

    for block in doc.all_code():
        base = _slugify(block.source_section, "snippet")
        counters[base] = counters.get(base, 0) + 1
        name = f"{len(written) + 1:02d}_{base}_{counters[base]}.m"
        path = out_dir / name
        header = [
            f"% From: {block.source_section}",
            f"% Section path: {' > '.join(block.section_path)}",
            f"% Source: {doc.url}",
            f"% Block type: {block.block_type} | digest: {block.digest}",
            "",
        ]
        path.write_text("\n".join(header) + block.code + "\n", encoding="utf-8")
        written.append(path)

    return written


def document_to_sections_dict(doc: Document) -> Dict[str, Any]:
    """Flat ``{section_title: {...}}`` view for programmatic consumers."""
    result: Dict[str, Any] = {}
    for section in doc.sections:
        if not section.blocks:
            continue
        result[section.title] = {
            "path": list(section.path),
            "level": section.heading.level,
            "anchor": section.heading.anchor,
            "text": section.plain_text(),
            "code": [
                {
                    "language": c.language,
                    "type": c.block_type,
                    "code": c.code,
                    "functions": c.referenced_functions,
                    "python_equivalent": c.python_equivalent,
                }
                for c in section.code_blocks()
            ],
            "images": [
                {"url": i.url, "alt": i.alt, "type": i.media_type}
                for i in section.images()
            ],
        }
    return result
