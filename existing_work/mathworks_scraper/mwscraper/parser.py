"""HTML -> Document extraction for MathWorks documentation pages.

Page structure this parser targets (verified against the live Frenet
reference-path example page in August 2026)::

    <section id="doc_center_content">
      <h1 class="r2026a">Object Tracking and Motion Planning ...</h1>
      <div class="doc_topic_desc"><em>Since R2021b</em></div>
      <div class="examples_short_list" data-products="ML DR TF NV"> ... </div>
      <p>intro prose with <a class="olink"> cross references</a></p>
      <div class="procedure">
        <h3 class="title" id="...-1">Introduction</h3>
        <p>...</p>
        <h4 class="title">Object State Transition and Measurement Modeling</h4>
        <div class="code_responsive"><math>...</math></div>     <- equation
        <div class="code_responsive -has_code_copy">
          <div class="btn-group code_actions">Get / Copy</div>   <- chrome
          <div class="programlisting"><div class="codeinput"><pre>MATLAB</pre>
        </div>
      </div>

Key consequences for the implementation:

* The body is **flat**, not nested per section, so sectioning is done by
  walking the DOM in document order and switching the "current section"
  whenever a heading is met.
* ``div.code_responsive`` is used for *both* equations and code. The
  ``-has_code_copy`` modifier class (and the presence of ``pre``)
  distinguishes real code from a MathML equation.
* ``div.btn-group.code_actions`` holds the "Get"/"Copy Code" UI and must be
  stripped, otherwise the word "Get" leaks into every code section.
* Equations are MathML, not images, so they are captured as text plus the
  raw MathML for downstream rendering.
"""

from __future__ import annotations

import hashlib
import logging
import re
from typing import Dict, Iterable, List, Optional, Tuple
from urllib.parse import urljoin, urlparse

from bs4 import BeautifulSoup, NavigableString, Tag

from .models import (
    CodeBlock,
    Document,
    Heading,
    ImageAsset,
    Resource,
    Section,
    TextBlock,
)
from .translate import translate_matlab_snippet

logger = logging.getLogger(__name__)


class ParseError(RuntimeError):
    """Raised when the page does not look like a MathWorks doc page at all."""


#: Candidate selectors for the main content region, most specific first. Kept
#: as a list so a MathWorks template change degrades instead of breaking.
CONTENT_SELECTORS: Tuple[str, ...] = (
    "section#doc_center_content",
    "div#doc_center_content",
    "div.content_container",
    "main",
)

#: Page chrome that must never contribute content.
SKIP_CLASSES = frozenset(
    {
        "code_actions",
        "dropdown-menu",
        "copy_code",
        "modal",
        "modal-dialog",
        "modal-content",
        "modal-header",
        "modal-body",
        "modal-footer",
        "examples_short_list",
        "doc_topic_desc",
        "rating_widget",
        "feedback",
        "footer",
        "site_footer",
        "breadcrumbs",
        "sticky_header_container",
        "offcanvas",
        "search_crux",
        "add_margin_4",
    }
)

SKIP_TAGS = frozenset(
    {"script", "style", "noscript", "nav", "header", "footer", "svg", "button", "form"}
)

HEADING_TAGS = ("h1", "h2", "h3", "h4", "h5", "h6")

#: File extensions we treat as downloadable resources.
DOWNLOAD_EXT = re.compile(
    r"\.(zip|mlx|mlapp|m|slx|mdl|mat|p|pdf|csv|xlsx|json|png|gif|mp4)(?:\?|#|$)", re.I
)

#: MATLAB language keywords that must not be mistaken for function calls.
MATLAB_KEYWORDS = frozenset(
    {
        "if", "else", "elseif", "end", "for", "while", "switch", "case",
        "otherwise", "function", "return", "break", "continue", "try",
        "catch", "global", "persistent", "parfor", "spmd", "classdef",
        "properties", "methods", "events", "enumeration", "arguments",
    }
)

#: MathWorks product codes used in `data-products` attributes.
PRODUCT_CODES = {
    "ML": "MATLAB",
    "DR": "Automated Driving Toolbox",
    "TF": "Sensor Fusion and Tracking Toolbox",
    "NV": "Navigation Toolbox",
    "SL": "Simulink",
}


def _classes(tag: Tag) -> List[str]:
    value = tag.get("class")
    if not value:
        return []
    if isinstance(value, str):
        return value.split()
    return list(value)


def _should_skip(tag: Tag) -> bool:
    if tag.name in SKIP_TAGS:
        return True
    return any(c in SKIP_CLASSES for c in _classes(tag))


def _clean_text(value: str) -> str:
    """Collapse whitespace but keep sentence spacing readable."""
    return re.sub(r"\s+", " ", value or "").strip()


class MathWorksParser:
    """Parse a MathWorks documentation page into a :class:`Document`."""

    def __init__(self, backend: str = "lxml") -> None:
        self.backend = backend

    # ------------------------------------------------------------- utilities

    def _make_soup(self, html: str) -> BeautifulSoup:
        try:
            return BeautifulSoup(html, self.backend)
        except Exception:  # pragma: no cover - only when lxml is absent
            logger.warning("Parser backend %r unavailable; using html.parser", self.backend)
            return BeautifulSoup(html, "html.parser")

    def _find_content_root(self, soup: BeautifulSoup) -> Tag:
        for selector in CONTENT_SELECTORS:
            node = soup.select_one(selector)
            if node is not None:
                logger.debug("Content root matched %s", selector)
                return node
        body = soup.body
        if body is None:
            raise ParseError("Document has no <body> and no recognised content root")
        logger.warning("No known content root matched; falling back to <body>")
        return body

    # -------------------------------------------------------------- metadata

    def _extract_metadata(self, soup: BeautifulSoup, root: Tag, doc: Document) -> None:
        h1 = root.find("h1")
        if h1 is not None:
            doc.title = _clean_text(h1.get_text())
        if not doc.title:
            og = soup.find("meta", property="og:title")
            if og and og.get("content"):
                doc.title = _clean_text(og["content"]).replace(
                    " - MATLAB & Simulink", ""
                )
            elif soup.title:
                doc.title = _clean_text(soup.title.get_text())
        if not doc.title:
            doc.warnings.append("No page title found")

        desc = soup.find("meta", attrs={"name": "description"})
        if desc and desc.get("content"):
            doc.description = _clean_text(desc["content"])

        release = root.find(string=re.compile(r"Since\s+R20\d\d[ab]"))
        if release:
            match = re.search(r"Since\s+(R20\d\d[ab])", str(release))
            if match:
                doc.release = match.group(1)

        products: List[str] = []
        for anchor in soup.select("a.coming_from_product"):
            name = _clean_text(anchor.get_text())
            if name and name not in products:
                products.append(name)
        if not products:
            holder = soup.select_one("[data-products]")
            if holder:
                for code in str(holder.get("data-products", "")).split():
                    products.append(PRODUCT_CODES.get(code, code))
        doc.products = products

    # -------------------------------------------------------------- sections

    def _new_section(
        self, tag: Tag, stack: Dict[int, str]
    ) -> Tuple[Section, Dict[int, str]]:
        level = int(tag.name[1])
        text = _clean_text(tag.get_text())
        # Drop deeper levels, then record this one, so `path` is the true
        # ancestor chain rather than every heading seen so far.
        stack = {lv: t for lv, t in stack.items() if lv < level}
        stack[level] = text
        path = tuple(stack[lv] for lv in sorted(stack))
        heading = Heading(text=text, level=level, anchor=tag.get("id"))
        return Section(heading=heading, path=path), stack

    # ---------------------------------------------------------------- blocks

    def _extract_code(self, container: Tag) -> Optional[str]:
        """Return code text from a code container, stripping the Get/Copy UI."""
        clone = BeautifulSoup(str(container), "html.parser")
        for junk in clone.select(".code_actions, .dropdown-menu, .copy_code, button"):
            junk.decompose()
        pre = clone.find("pre")
        target = pre if pre is not None else clone
        # get_text() on <pre> preserves the newlines that MATLAB code needs;
        # the syntax-highlight <span>s disappear cleanly.
        code = target.get_text()
        code = code.replace("\r\n", "\n").replace("\xa0", " ")
        code = "\n".join(line.rstrip() for line in code.split("\n"))
        return code.strip("\n") or None

    @staticmethod
    def _referenced_functions(code: str) -> List[str]:
        names = re.findall(r"\b([A-Za-z]\w*)\s*\(", code)
        seen: List[str] = []
        for name in names:
            if name in MATLAB_KEYWORDS or name in seen:
                continue
            seen.append(name)
        return seen

    @staticmethod
    def _classify_code(code: str) -> str:
        stripped = code.lstrip()
        if re.match(r"^function\b", stripped):
            return "function"
        return "example"

    def _links_in(self, tag: Tag, base_url: str) -> List[Dict[str, str]]:
        links = []
        for anchor in tag.find_all("a", href=True):
            href = anchor["href"].strip()
            text = _clean_text(anchor.get_text())
            if not text:
                continue
            url = self._absolute(href, base_url)
            links.append({"text": text, "url": url, "kind": self._link_kind(href, url)})
        return links

    @staticmethod
    def _absolute(href: str, base_url: str) -> str:
        if href.startswith(("matlab:", "mailto:", "javascript:")):
            return href
        return urljoin(base_url, href)

    @staticmethod
    def _link_kind(href: str, url: str = "") -> str:
        """Classify a link.

        Classification runs on the *resolved* URL: MathWorks pages link
        internally with relative paths such as ``../../nav/ref/foo.html``,
        which contain no ``/help/`` segment even though they resolve to
        reference documentation.
        """
        if href.startswith("matlab:openExample"):
            return "openExample"
        if href.startswith("matlab:"):
            return "product"
        target = url or href
        if DOWNLOAD_EXT.search(target) and not target.lower().endswith(".html"):
            return "download"
        path = urlparse(target).path
        if "/ug/" in path or "/examples/" in path:
            return "related_example"
        if "/ref/" in path or "/help/" in path:
            return "api_reference"
        if target.startswith("http"):
            return "external"
        return "internal"

    # ------------------------------------------------------------------ walk

    def _walk(
        self,
        node: Tag,
        doc: Document,
        base_url: str,
        state: dict,
    ) -> None:
        """Depth-first, document-order traversal emitting blocks."""
        for child in node.children:
            if isinstance(child, NavigableString):
                continue
            if not isinstance(child, Tag):
                continue
            if _should_skip(child):
                continue

            name = child.name
            classes = _classes(child)

            # --- headings start a new section -----------------------------
            if name in HEADING_TAGS:
                text = _clean_text(child.get_text())
                # The page <h1> repeats the title we already used for the
                # preamble section; adopt its anchor instead of duplicating.
                if name == "h1" and not state["seen_h1"]:
                    state["seen_h1"] = True
                    preamble = state["section"]
                    preamble.heading.anchor = child.get("id")
                    if text:
                        preamble.heading.text = text
                        preamble.path = (text,)
                        state["stack"] = {1: text}
                    continue
                section, state["stack"] = self._new_section(child, state["stack"])
                doc.sections.append(section)
                state["section"] = section
                continue

            section = state["section"]

            # --- code -----------------------------------------------------
            is_code_container = (
                "codeinput" in classes
                or "programlisting" in classes
                or ("code_responsive" in classes and child.find("pre") is not None)
            )
            if is_code_container:
                code = self._extract_code(child)
                if code:
                    self._emit_code(code, section, doc, state)
                continue

            if name == "pre":
                code = self._extract_code(child)
                if code:
                    self._emit_code(code, section, doc, state)
                continue

            # --- displayed MathML equation --------------------------------
            # Only *dedicated* equation containers count. A <p> of prose may
            # contain inline <math> (e.g. "at time step k"); treating that as
            # an equation would swallow the surrounding paragraph.
            is_equation_container = (
                child.find("pre") is None
                and child.find("math") is not None
                and (
                    ("code_responsive" in classes and name == "div")
                    or "programlistingindent" in classes
                    or "equation" in classes
                )
            )
            if is_equation_container:
                math = child.find("math")
                text = _clean_text(child.get_text())
                if text:
                    section.blocks.append(
                        TextBlock(
                            text=text,
                            text_type="equation",
                            mathml=str(math),
                            source_section=section.title,
                            section_path=section.path,
                            order=state["order"],
                        )
                    )
                    state["order"] += 1
                continue

            # --- images ---------------------------------------------------
            if name == "img":
                self._emit_image(child, section, doc, base_url, state)
                continue

            # --- paragraphs / notes --------------------------------------
            if name == "p":
                text = _clean_text(child.get_text())
                if text:
                    section.blocks.append(
                        TextBlock(
                            text=text,
                            text_type="note" if "note" in classes else "paragraph",
                            links=self._links_in(child, base_url),
                            source_section=section.title,
                            section_path=section.path,
                            order=state["order"],
                        )
                    )
                    state["order"] += 1
                # A <p> may still wrap an <img>; capture those before moving on.
                for img in child.find_all("img"):
                    self._emit_image(img, section, doc, base_url, state)
                continue

            # --- lists ----------------------------------------------------
            if name in {"ul", "ol"}:
                items = [
                    _clean_text(li.get_text())
                    for li in child.find_all("li", recursive=False)
                ]
                items = [i for i in items if i]
                if items:
                    section.blocks.append(
                        TextBlock(
                            text="\n".join(f"- {i}" for i in items),
                            text_type="list",
                            links=self._links_in(child, base_url),
                            source_section=section.title,
                            section_path=section.path,
                            order=state["order"],
                        )
                    )
                    state["order"] += 1
                continue

            # --- anything else: descend -----------------------------------
            self._walk(child, doc, base_url, state)

    def _emit_code(self, code: str, section: Section, doc: Document, state: dict) -> None:
        if code in state["seen_code"]:
            return
        state["seen_code"].add(code)
        python_code, note = translate_matlab_snippet(code)
        section.blocks.append(
            CodeBlock(
                code=code,
                language="matlab",
                block_type=self._classify_code(code),
                referenced_functions=self._referenced_functions(code),
                python_equivalent=python_code,
                translation_note=note,
                source_section=section.title,
                section_path=section.path,
                order=state["order"],
            )
        )
        state["order"] += 1

    def _emit_image(
        self, tag: Tag, section: Section, doc: Document, base_url: str, state: dict
    ) -> None:
        src = (tag.get("src") or tag.get("data-src") or "").strip()
        if not src:
            return
        url = self._absolute(src, base_url)
        if url in state["seen_images"]:
            return
        state["seen_images"].add(url)
        # Skip site chrome (logos, icons) which live on the CDN path.
        if "/etc.clientlibs/" in url or "logo" in url.lower():
            return
        alt = _clean_text(tag.get("alt", ""))
        media = "animation" if url.lower().endswith(".gif") else "figure"
        section.blocks.append(
            ImageAsset(
                url=url,
                relative_src=src,
                alt=alt,
                media_type=media,
                source_section=section.title,
                section_path=section.path,
                order=state["order"],
            )
        )
        state["order"] += 1
        doc.resources.append(
            Resource(
                url=url,
                text=alt or src.rsplit("/", 1)[-1],
                kind="download",
                filename=src.rsplit("/", 1)[-1],
                description=f"{media.title()} embedded in '{section.title}'",
                source_section=section.title,
            )
        )

    # ------------------------------------------------------------- resources

    def _extract_resources(self, root: Tag, doc: Document, base_url: str) -> None:
        seen = {r.url for r in doc.resources}
        for anchor in root.find_all("a", href=True):
            href = anchor["href"].strip()
            if not href or href.startswith("#"):
                continue
            text = _clean_text(anchor.get_text())
            if not text:
                continue
            url = self._absolute(href, base_url)
            if url in seen:
                continue
            kind = self._link_kind(href, url)
            # Drop site navigation chrome, which is not page content.
            if re.search(r"s_tid=(CRUX_topnav|gn_|srchtitle)", url):
                continue
            if kind == "internal":
                continue
            seen.add(url)
            filename = None
            if kind == "download":
                filename = urlparse(url).path.rsplit("/", 1)[-1]
            elif kind == "openExample":
                # BeautifulSoup unescapes &#39; to ', so accept both forms.
                match = re.search(
                    r"openExample\(\s*(?:&#39;|['\"])?([^'\"&)]+)", href
                )
                if match:
                    filename = match.group(1)
            doc.resources.append(
                Resource(
                    url=url,
                    text=text,
                    kind=kind,
                    filename=filename,
                    description=self._describe_resource(kind, text),
                    source_section=self._nearest_section(anchor),
                )
            )

    @staticmethod
    def _describe_resource(kind: str, text: str) -> str:
        return {
            "openExample": "Opens the complete, runnable example in MATLAB "
            "(includes all attached helper files).",
            "related_example": f"Related MathWorks example: {text}",
            "api_reference": f"Reference documentation for {text}",
            "product": f"Required product/add-on: {text}",
            "download": f"Downloadable asset: {text}",
            "external": f"External link: {text}",
        }.get(kind, text)

    @staticmethod
    def _nearest_section(tag: Tag) -> str:
        for prev in tag.find_all_previous(HEADING_TAGS):
            return _clean_text(prev.get_text())
        return ""

    # ------------------------------------------------------------------ main

    def parse(self, html: str, url: str) -> Document:
        """Parse ``html`` (fetched from ``url``) into a :class:`Document`."""
        if not html or not html.strip():
            raise ParseError("Empty HTML supplied to parser")

        soup = self._make_soup(html)
        root = self._find_content_root(soup)

        doc = Document(
            url=url,
            html_sha256=hashlib.sha256(html.encode("utf-8", "replace")).hexdigest(),
        )
        self._extract_metadata(soup, root, doc)

        # Every page starts with an implicit "preamble" section so that intro
        # prose appearing before the first heading is never dropped.
        preamble = Section(
            heading=Heading(text=doc.title or "Overview", level=1, anchor=None),
            path=(doc.title or "Overview",),
        )
        doc.sections.append(preamble)

        state = {
            "section": preamble,
            "stack": {1: doc.title or "Overview"},
            "order": 0,
            "seen_code": set(),
            "seen_images": set(),
            "seen_h1": False,
        }
        self._walk(root, doc, url, state)

        # Drop sections that ended up completely empty (usually stray
        # headings from page chrome). A heading with no blocks of its own is
        # still kept when it is an ancestor of a section that does have
        # content (e.g. "Setup", whose prose lives in its subsections).
        ancestors = {name for s in doc.sections if s.blocks for name in s.path}
        doc.sections = [
            s
            for i, s in enumerate(doc.sections)
            if s.blocks or i == 0 or s.title in ancestors
        ]

        self._extract_resources(root, doc, url)

        if not doc.all_code():
            doc.warnings.append("No code blocks were extracted - selectors may be stale")
        if len(doc.sections) <= 1:
            doc.warnings.append("Only one section found - heading selectors may be stale")

        logger.info("Parsed %s -> %s", url, doc.summary())
        return doc
