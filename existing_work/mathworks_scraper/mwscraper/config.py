"""Configuration for the MathWorks scraper.

Every knob that affects network behaviour lives here so that deployment
environments can tune politeness/robustness without touching scraping logic.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, asdict
from typing import Dict, Sequence

# MathWorks fronts its docs with a WAF that returns HTTP 403 to clients that do
# not send a full browser-like header set (verified: bare `curl` -> 403,
# browser headers -> 200). These headers are therefore functional, not
# cosmetic. We still identify honestly in the `From` header when configured.
DEFAULT_HEADERS: Dict[str, str] = {
    "User-Agent": (
        "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
        "(KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36"
    ),
    "Accept": (
        "text/html,application/xhtml+xml,application/xml;q=0.9,"
        "image/avif,image/webp,*/*;q=0.8"
    ),
    "Accept-Language": "en-US,en;q=0.9",
    "Sec-Fetch-Dest": "document",
    "Sec-Fetch-Mode": "navigate",
    "Sec-Fetch-Site": "none",
    "Sec-Fetch-User": "?1",
    "Upgrade-Insecure-Requests": "1",
}

#: HTTP statuses worth retrying. 403 is included because the MathWorks WAF
#: sometimes soft-blocks a first request and lets a retry through.
DEFAULT_RETRY_STATUSES: Sequence[int] = (403, 408, 425, 429, 500, 502, 503, 504)


@dataclass
class ScraperConfig:
    """Tunable scraper settings.

    Attributes
    ----------
    request_timeout:
        Per-request (connect, read) timeout in seconds.
    max_retries:
        Number of *additional* attempts after the first failure.
    backoff_factor:
        Base for exponential backoff: sleep = backoff_factor * 2**attempt,
        capped at ``backoff_max`` and jittered to avoid thundering herds.
    min_request_interval:
        Rate limit. Minimum wall-clock seconds between two requests issued by
        the same scraper instance. 1.0 s is deliberately conservative for a
        documentation site we do not own.
    respect_robots:
        When True, ``robots.txt`` is fetched once per host and consulted
        before every request. A disallowed URL raises PermanentFetchError.
    cache_dir:
        If set, raw HTML is written/read here, so re-parsing during
        development costs zero requests.
    """

    request_timeout: float = 30.0
    connect_timeout: float = 10.0
    max_retries: int = 4
    backoff_factor: float = 0.75
    backoff_max: float = 30.0
    jitter: float = 0.25
    min_request_interval: float = 1.0
    respect_robots: bool = True
    #: Fall back to curl / curl_cffi when the primary transport is blocked by
    #: TLS fingerprinting (MathWorks returns 403 to Python's TLS stack).
    transport_fallback: bool = True
    retry_statuses: Sequence[int] = field(default=tuple(DEFAULT_RETRY_STATUSES))
    headers: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_HEADERS))
    contact_email: str | None = None
    cache_dir: str | None = None
    use_cache: bool = False
    parser_backend: str = "lxml"  # falls back to html.parser if lxml missing
    log_level: str = "INFO"

    def __post_init__(self) -> None:
        if self.max_retries < 0:
            raise ValueError("max_retries must be >= 0")
        if self.min_request_interval < 0:
            raise ValueError("min_request_interval must be >= 0")
        if self.contact_email:
            # Polite, standards-based way to identify the operator.
            self.headers.setdefault("From", self.contact_email)

    @classmethod
    def from_env(cls, prefix: str = "MWSCRAPER_") -> "ScraperConfig":
        """Build a config from environment variables (12-factor friendly)."""
        def _get(name, cast, default):
            raw = os.environ.get(prefix + name)
            if raw is None:
                return default
            if cast is bool:
                return raw.strip().lower() in {"1", "true", "yes", "on"}
            return cast(raw)

        return cls(
            request_timeout=_get("TIMEOUT", float, 30.0),
            max_retries=_get("MAX_RETRIES", int, 4),
            backoff_factor=_get("BACKOFF", float, 0.75),
            min_request_interval=_get("RATE_INTERVAL", float, 1.0),
            respect_robots=_get("RESPECT_ROBOTS", bool, True),
            cache_dir=_get("CACHE_DIR", str, None),
            use_cache=_get("USE_CACHE", bool, False),
            contact_email=_get("CONTACT_EMAIL", str, None),
            log_level=_get("LOG_LEVEL", str, "INFO"),
        )

    def to_dict(self) -> dict:
        d = asdict(self)
        d["retry_statuses"] = list(self.retry_statuses)
        # Never serialise a contact address into shared output.
        d.pop("contact_email", None)
        return d
