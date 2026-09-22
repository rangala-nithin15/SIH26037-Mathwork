"""Command line entry point.

Examples
--------
Scrape the default Frenet reference-path page into ``./output``::

    python -m mwscraper

Re-parse a saved page without touching the network::

    python -m mwscraper --from-file tests/fixtures/frenet_page.html

Be extra polite and cache the raw HTML::

    python -m mwscraper --rate-interval 3 --cache-dir .cache --use-cache
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .config import ScraperConfig
from .fetcher import FetchError
from .parser import ParseError
from .scraper import DEFAULT_URL, MathWorksScraper, configure_logging


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="mwscraper",
        description="Extract code, prose, figures and resources from a "
        "MathWorks documentation page.",
    )
    p.add_argument("url", nargs="?", default=DEFAULT_URL, help="Page URL to scrape.")
    p.add_argument(
        "--from-file",
        metavar="PATH",
        help="Parse this local HTML file instead of making a request.",
    )
    p.add_argument(
        "-o", "--out-dir", default="output", help="Output directory (default: output)."
    )
    p.add_argument(
        "--rate-interval",
        type=float,
        default=1.0,
        help="Minimum seconds between requests (default: 1.0).",
    )
    p.add_argument(
        "--max-retries", type=int, default=4, help="Retry attempts after first failure."
    )
    p.add_argument("--timeout", type=float, default=30.0, help="Read timeout in seconds.")
    p.add_argument("--cache-dir", default=None, help="Directory for raw HTML cache.")
    p.add_argument(
        "--use-cache", action="store_true", help="Read from --cache-dir when present."
    )
    p.add_argument(
        "--ignore-robots",
        action="store_true",
        help="Skip the robots.txt check (use only where you have permission).",
    )
    p.add_argument("--contact-email", default=None, help="Sent in the From: header.")
    p.add_argument(
        "--summary-only",
        action="store_true",
        help="Print the extraction summary as JSON and write nothing.",
    )
    p.add_argument("--log-level", default="INFO", help="DEBUG, INFO, WARNING, ERROR.")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_level)

    config = ScraperConfig(
        request_timeout=args.timeout,
        max_retries=args.max_retries,
        min_request_interval=args.rate_interval,
        respect_robots=not args.ignore_robots,
        cache_dir=args.cache_dir,
        use_cache=args.use_cache,
        contact_email=args.contact_email,
        log_level=args.log_level,
    )

    try:
        with MathWorksScraper(config) as scraper:
            if args.from_file:
                doc = scraper.scrape_from_file(args.from_file, args.url)
            else:
                doc = scraper.scrape(args.url)

            if args.summary_only:
                print(json.dumps(doc.summary(), indent=2))
                return 0

            written = scraper.export_all(doc, args.out_dir)

    except FileNotFoundError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except ParseError as exc:
        print(f"parse error: {exc}", file=sys.stderr)
        return 3
    except FetchError as exc:
        print(f"fetch error: {exc}", file=sys.stderr)
        return 4
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130

    summary = doc.summary()
    print()
    print(f"Title      : {summary['title']}")
    print(f"Sections   : {summary['sections']}")
    print(f"Code blocks: {summary['code_blocks']} ({summary['code_lines']} lines)")
    print(f"Text blocks: {summary['text_blocks']}")
    print(f"Images     : {summary['images']}")
    print(f"Resources  : {summary['resources']}")
    if doc.warnings:
        print(f"Warnings   : {len(doc.warnings)}")
        for warning in doc.warnings:
            print(f"  - {warning}")
    print()
    print(f"JSON       : {written['json']}")
    print(f"Markdown   : {written['markdown']}")
    print(f"Code files : {len(written['code_files'])} in "
          f"{Path(written['markdown']).parent / 'code'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
