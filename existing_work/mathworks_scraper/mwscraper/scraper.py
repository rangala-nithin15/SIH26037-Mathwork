"""Orchestration: fetch + parse + export, with logging set up once."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Iterable, List, Optional

from .config import ScraperConfig
from .fetcher import Fetcher, FetchError
from .models import Document
from .parser import MathWorksParser, ParseError

logger = logging.getLogger(__name__)

DEFAULT_URL = (
    "https://nl.mathworks.com/help/fusion/ug/"
    "object-tracking-and-motion-planning-using-frenet-reference-path.html"
)


def configure_logging(level: str = "INFO") -> None:
    """Idempotent root logging setup for CLI use."""
    logging.basicConfig(
        level=getattr(logging, str(level).upper(), logging.INFO),
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


class MathWorksScraper:
    """High-level facade tying the fetcher and parser together."""

    def __init__(
        self,
        config: Optional[ScraperConfig] = None,
        fetcher: Optional[Fetcher] = None,
        parser: Optional[MathWorksParser] = None,
    ) -> None:
        self.config = config or ScraperConfig()
        self.fetcher = fetcher or Fetcher(self.config)
        self.parser = parser or MathWorksParser(self.config.parser_backend)

    def scrape(self, url: str = DEFAULT_URL) -> Document:
        """Fetch and parse a single page.

        Raises :class:`FetchError` or :class:`ParseError` on failure - callers
        decide whether a single bad page should abort a batch.
        """
        logger.info("Scraping %s", url)
        html = self.fetcher.fetch(url)
        doc = self.parser.parse(html, url)
        logger.info("Done: %s", doc.summary())
        return doc

    def scrape_from_file(self, path: str | Path, url: str = DEFAULT_URL) -> Document:
        """Parse a locally saved HTML file (offline re-processing, tests)."""
        html = Path(path).read_text(encoding="utf-8", errors="replace")
        return self.parser.parse(html, url)

    def scrape_many(
        self, urls: Iterable[str], continue_on_error: bool = True
    ) -> List[Document]:
        """Scrape several pages, respecting the shared rate limiter."""
        docs: List[Document] = []
        for url in urls:
            try:
                docs.append(self.scrape(url))
            except (FetchError, ParseError) as exc:
                logger.error("Failed to scrape %s: %s", url, exc)
                if not continue_on_error:
                    raise
        return docs

    def export_all(self, doc: Document, out_dir: str | Path) -> dict:
        """Write JSON, Markdown and per-snippet ``.m`` files."""
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        json_path = doc.to_json_file(out / "extracted_content.json")
        md_path = doc.to_markdown_file(out / "extracted_content.md")
        code_paths = doc.write_code_files(out / "code")
        logger.info(
            "Exported: %s, %s, %d code files", json_path.name, md_path.name, len(code_paths)
        )
        return {
            "json": json_path,
            "markdown": md_path,
            "code_files": code_paths,
        }

    def close(self) -> None:
        self.fetcher.close()

    def __enter__(self) -> "MathWorksScraper":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
