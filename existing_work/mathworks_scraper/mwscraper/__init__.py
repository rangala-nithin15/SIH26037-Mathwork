"""mwscraper - production scraper for MathWorks documentation/example pages.

Public API::

    from mwscraper import ScraperConfig, MathWorksScraper

    scraper = MathWorksScraper(ScraperConfig())
    doc = scraper.scrape(URL)
    doc.to_json_file("out.json")
    doc.to_markdown_file("out.md")
"""

from .config import ScraperConfig
from .models import (
    CodeBlock,
    Document,
    Heading,
    ImageAsset,
    Resource,
    Section,
    TextBlock,
)
from .fetcher import Fetcher, FetchError, PermanentFetchError, TransientFetchError
from .parser import MathWorksParser, ParseError
from .scraper import MathWorksScraper

__version__ = "1.0.0"

__all__ = [
    "ScraperConfig",
    "MathWorksScraper",
    "MathWorksParser",
    "Fetcher",
    "Document",
    "Section",
    "Heading",
    "TextBlock",
    "CodeBlock",
    "ImageAsset",
    "Resource",
    "FetchError",
    "TransientFetchError",
    "PermanentFetchError",
    "ParseError",
]
