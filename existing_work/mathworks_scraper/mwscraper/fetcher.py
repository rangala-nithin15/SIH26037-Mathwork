"""HTTP layer: polite, retrying, cache-aware fetching.

Responsibilities
----------------
* **Rate limiting** - a process-local token gate guarantees at least
  ``config.min_request_interval`` seconds between requests from one Fetcher.
* **Retries** - transient failures (connection resets, timeouts, 5xx, 429,
  and the occasional WAF 403) are retried with *exponential backoff plus
  jitter*, honouring ``Retry-After`` when the server supplies it.
* **robots.txt** - fetched once per host and cached; disallowed URLs raise
  ``PermanentFetchError`` rather than being requested.
* **Caching** - optional on-disk raw HTML cache so that re-parsing during
  development or test runs costs no network traffic.

Design note: we deliberately do *not* use ``urllib3.Retry`` because we need
per-attempt logging, ``Retry-After`` handling and jitter that the built-in
adapter policy does not express cleanly.
"""

from __future__ import annotations

import hashlib
import logging
import random
import threading
import time
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse
from urllib.robotparser import RobotFileParser

import requests

from .config import ScraperConfig
from .transports import (
    InvalidRequestError,
    RequestsTransport,
    Transport,
    TransportError,
    TransportResponse,
    default_transports,
)

logger = logging.getLogger(__name__)


class FetchError(RuntimeError):
    """Base class for all fetch failures."""


class TransientFetchError(FetchError):
    """Failure that is worth retrying (timeout, 5xx, connection reset)."""


class PermanentFetchError(FetchError):
    """Failure that will not improve on retry (404, robots.txt disallow)."""


class RateLimiter:
    """Thread-safe minimum-interval gate.

    Simpler and more predictable than a token bucket for a single-host
    crawl: it enforces a hard floor on the gap between consecutive requests.
    """

    def __init__(self, min_interval: float) -> None:
        self.min_interval = max(0.0, float(min_interval))
        self._lock = threading.Lock()
        self._last_call: float = 0.0

    def acquire(self) -> float:
        """Block until the next request is allowed. Returns seconds slept."""
        if self.min_interval <= 0:
            return 0.0
        with self._lock:
            now = time.monotonic()
            wait = self._last_call + self.min_interval - now
            if wait > 0:
                logger.debug("Rate limiter sleeping %.2fs", wait)
                time.sleep(wait)
                slept = wait
            else:
                slept = 0.0
            self._last_call = time.monotonic()
            return slept


class Fetcher:
    """Fetches HTML with retries, rate limiting and optional caching."""

    def __init__(
        self,
        config: Optional[ScraperConfig] = None,
        session: Optional[requests.Session] = None,
        transports: Optional[list] = None,
    ) -> None:
        self.config = config or ScraperConfig()
        self.session = session or requests.Session()
        self.session.headers.update(self.config.headers)
        # When a session is injected (tests, custom auth) we honour it as the
        # only transport; otherwise we build the full fallback chain.
        if transports is not None:
            self.transports = list(transports)
        elif session is not None or not self.config.transport_fallback:
            self.transports = [RequestsTransport(self.session)]
        else:
            self.transports = default_transports(self.session)
        logger.debug(
            "Transport chain: %s", ", ".join(t.name for t in self.transports)
        )
        self.limiter = RateLimiter(self.config.min_request_interval)
        self._robots: dict[str, Optional[RobotFileParser]] = {}
        self._robots_lock = threading.Lock()
        #: Index of the transport that last succeeded; tried first next time.
        self._preferred = 0

    # ------------------------------------------------------------ robots.txt

    def _robots_for(self, url: str) -> Optional[RobotFileParser]:
        parts = urlparse(url)
        origin = f"{parts.scheme}://{parts.netloc}"
        with self._robots_lock:
            if origin in self._robots:
                return self._robots[origin]
        parser: Optional[RobotFileParser] = None
        try:
            resp = self.session.get(
                f"{origin}/robots.txt",
                timeout=(self.config.connect_timeout, self.config.request_timeout),
            )
            if resp.status_code == 200:
                parser = RobotFileParser()
                parser.parse(resp.text.splitlines())
            else:
                logger.info(
                    "robots.txt for %s returned %s; treating as permissive",
                    origin,
                    resp.status_code,
                )
        except requests.RequestException as exc:
            # Fail open: an unreachable robots.txt should not stop the job,
            # but we log loudly so operators can notice.
            logger.warning("Could not fetch robots.txt for %s: %s", origin, exc)
        with self._robots_lock:
            self._robots[origin] = parser
        return parser

    def is_allowed(self, url: str) -> bool:
        if not self.config.respect_robots:
            return True
        parser = self._robots_for(url)
        if parser is None:
            return True
        ua = self.config.headers.get("User-Agent", "*")
        return parser.can_fetch(ua, url)

    # ---------------------------------------------------------------- cache

    def _cache_path(self, url: str) -> Optional[Path]:
        if not self.config.cache_dir:
            return None
        digest = hashlib.sha256(url.encode("utf-8")).hexdigest()[:20]
        return Path(self.config.cache_dir) / f"{digest}.html"

    def _read_cache(self, url: str) -> Optional[str]:
        path = self._cache_path(url)
        if path and path.is_file():
            logger.info("Cache hit for %s (%s)", url, path.name)
            return path.read_text(encoding="utf-8", errors="replace")
        return None

    def _write_cache(self, url: str, html: str) -> None:
        path = self._cache_path(url)
        if not path:
            return
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(html, encoding="utf-8")
        except OSError as exc:
            logger.warning("Could not write cache for %s: %s", url, exc)

    # ---------------------------------------------------------------- backoff

    def _sleep_for_attempt(self, attempt: int, retry_after: Optional[float]) -> None:
        """Exponential backoff with full-ish jitter, capped."""
        if retry_after is not None:
            delay = min(retry_after, self.config.backoff_max)
        else:
            delay = min(
                self.config.backoff_factor * (2**attempt), self.config.backoff_max
            )
        delay += random.uniform(0, self.config.jitter)
        logger.info("Backing off %.2fs before retry %d", delay, attempt + 1)
        time.sleep(delay)

    @staticmethod
    def _retry_after_seconds(resp: requests.Response) -> Optional[float]:
        raw = resp.headers.get("Retry-After")
        if not raw:
            return None
        try:
            return float(raw)
        except ValueError:
            # HTTP-date form; we do not parse it, backoff handles the wait.
            return None

    # ----------------------------------------------------------------- fetch

    def _get_via_transports(self, url: str, timeout) -> TransportResponse:
        """Try each transport until one returns a non-blocked response.

        A 403 from the first transport is treated as a *fingerprinting block*
        rather than a real authorisation failure, so the next transport in the
        chain gets a turn. The transport that succeeds is remembered and tried
        first on subsequent requests, so the fallback cost is paid once.
        """
        order = list(range(len(self.transports)))
        order = order[self._preferred:] + order[: self._preferred]
        last_response: Optional[TransportResponse] = None
        last_error: Optional[str] = None

        for index in order:
            transport = self.transports[index]
            if not transport.available():
                continue
            try:
                response = transport.get(url, dict(self.config.headers), timeout[1])
            except InvalidRequestError:
                raise
            except TransportError as exc:
                last_error = f"{transport.name}: {exc}"
                logger.warning("Transport %s failed: %s", transport.name, exc)
                continue

            # 403 here means "blocked", which another transport may survive.
            if response.status_code == 403 and len(self.transports) > 1:
                logger.info(
                    "Transport %s got 403 (likely TLS fingerprinting); "
                    "trying next transport",
                    transport.name,
                )
                last_response = response
                continue

            if index != self._preferred:
                logger.info("Switching preferred transport to %s", transport.name)
                self._preferred = index
            return response

        if last_response is not None:
            return last_response
        raise requests.ConnectionError(last_error or "all transports failed")

    def fetch(self, url: str) -> str:
        """Return the HTML for ``url``.

        Raises
        ------
        PermanentFetchError
            robots.txt disallow, 4xx that is not retryable, or empty body.
        TransientFetchError
            All retries exhausted on a retryable condition.
        """
        if self.config.use_cache:
            cached = self._read_cache(url)
            if cached is not None:
                return cached

        if not self.is_allowed(url):
            raise PermanentFetchError(f"robots.txt disallows fetching {url}")

        timeout = (self.config.connect_timeout, self.config.request_timeout)
        last_error: Optional[str] = None

        for attempt in range(self.config.max_retries + 1):
            self.limiter.acquire()
            try:
                logger.info(
                    "GET %s (attempt %d/%d)", url, attempt + 1, self.config.max_retries + 1
                )
                resp = self._get_via_transports(url, timeout)
            except (requests.Timeout, requests.ConnectionError) as exc:
                last_error = f"{type(exc).__name__}: {exc}"
                logger.warning("Network error for %s: %s", url, last_error)
                if attempt < self.config.max_retries:
                    self._sleep_for_attempt(attempt, None)
                    continue
                raise TransientFetchError(
                    f"Network failure for {url} after "
                    f"{self.config.max_retries + 1} attempts: {last_error}"
                ) from exc
            except InvalidRequestError as exc:
                raise PermanentFetchError(f"Invalid request for {url}: {exc}") from exc
            except requests.RequestException as exc:
                raise PermanentFetchError(f"Request failed for {url}: {exc}") from exc

            status = resp.status_code
            if status == 200:
                html = resp.text or ""
                if not html.strip():
                    raise PermanentFetchError(f"Empty response body for {url}")
                self._write_cache(url, html)
                logger.info("Fetched %s (%d bytes)", url, len(html))
                return html

            if status in self.config.retry_statuses and attempt < self.config.max_retries:
                last_error = f"HTTP {status}"
                logger.warning("Retryable HTTP %s for %s", status, url)
                self._sleep_for_attempt(attempt, self._retry_after_seconds(resp))
                continue

            if 400 <= status < 500 and status not in self.config.retry_statuses:
                raise PermanentFetchError(f"HTTP {status} for {url}")

            last_error = f"HTTP {status}"

        raise TransientFetchError(
            f"Exhausted {self.config.max_retries + 1} attempts for {url} "
            f"(last error: {last_error})"
        )

    def close(self) -> None:
        self.session.close()

    def __enter__(self) -> "Fetcher":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
