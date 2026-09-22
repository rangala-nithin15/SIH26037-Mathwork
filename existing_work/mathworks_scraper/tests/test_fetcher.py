"""Network-layer tests: rate limiting, retries, backoff, robots, caching.

No real HTTP happens here. ``requests.Session`` is replaced with a fake whose
scripted responses let us assert on *behaviour* (how many attempts, how long
we slept, which exception type) rather than on network luck. ``time.sleep`` is
patched so the suite stays fast while still recording the delays requested.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest
import requests

from mwscraper.config import ScraperConfig
from mwscraper.fetcher import (
    Fetcher,
    PermanentFetchError,
    RateLimiter,
    TransientFetchError,
)

URL = "https://nl.mathworks.com/help/fusion/ug/example.html"


class FakeResponse:
    def __init__(self, status_code=200, text="<html><body>ok</body></html>", headers=None):
        self.status_code = status_code
        self.text = text
        self.headers = headers or {}


class FakeSession:
    """Replays a scripted sequence of responses/exceptions."""

    def __init__(self, script, robots_text="User-agent: *\nAllow: /\n"):
        self.script = list(script)
        self.robots_text = robots_text
        self.calls = []
        self.headers = {}

    def get(self, url, timeout=None, **kwargs):
        if url.endswith("/robots.txt"):
            return FakeResponse(200, self.robots_text)
        self.calls.append(url)
        if not self.script:
            return FakeResponse()
        item = self.script.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

    def close(self):
        pass


@pytest.fixture
def no_sleep(monkeypatch):
    """Patch sleep everywhere the fetcher can call it; record the delays."""
    slept = []

    def fake_sleep(seconds):
        slept.append(seconds)

    monkeypatch.setattr("mwscraper.fetcher.time.sleep", fake_sleep)
    return slept


def make_fetcher(script, **overrides):
    config = ScraperConfig(
        min_request_interval=0,
        max_retries=overrides.pop("max_retries", 3),
        backoff_factor=overrides.pop("backoff_factor", 1.0),
        jitter=0.0,
        respect_robots=overrides.pop("respect_robots", False),
        **overrides,
    )
    return Fetcher(config, session=FakeSession(script))


# ------------------------------------------------------------- rate limiting


class TestRateLimiter:
    def test_first_call_does_not_block(self):
        limiter = RateLimiter(0.5)
        assert limiter.acquire() == 0.0

    def test_second_call_waits_for_the_interval(self):
        limiter = RateLimiter(0.15)
        limiter.acquire()
        start = time.monotonic()
        limiter.acquire()
        assert time.monotonic() - start >= 0.12

    def test_zero_interval_disables_limiting(self):
        limiter = RateLimiter(0)
        limiter.acquire()
        assert limiter.acquire() == 0.0

    def test_negative_interval_is_clamped(self):
        assert RateLimiter(-5).min_interval == 0.0

    def test_fetcher_applies_the_limiter_between_requests(self, monkeypatch):
        config = ScraperConfig(min_request_interval=0.1, respect_robots=False)
        fetcher = Fetcher(config, session=FakeSession([FakeResponse(), FakeResponse()]))
        start = time.monotonic()
        fetcher.fetch(URL)
        fetcher.fetch(URL)
        assert time.monotonic() - start >= 0.09


# -------------------------------------------------------------------- success


class TestSuccess:
    def test_returns_html(self):
        fetcher = make_fetcher([FakeResponse(200, "<html>hello</html>")])
        assert fetcher.fetch(URL) == "<html>hello</html>"

    def test_single_attempt_when_first_succeeds(self):
        fetcher = make_fetcher([FakeResponse()])
        fetcher.fetch(URL)
        assert len(fetcher.session.calls) == 1

    def test_empty_body_is_permanent_error(self):
        fetcher = make_fetcher([FakeResponse(200, "   ")])
        with pytest.raises(PermanentFetchError, match="Empty response"):
            fetcher.fetch(URL)


# --------------------------------------------------------------------- retry


class TestRetries:
    def test_retries_then_succeeds(self, no_sleep):
        fetcher = make_fetcher(
            [FakeResponse(503), FakeResponse(503), FakeResponse(200, "<html>ok</html>")]
        )
        assert "ok" in fetcher.fetch(URL)
        assert len(fetcher.session.calls) == 3

    def test_backoff_is_exponential(self, no_sleep):
        fetcher = make_fetcher(
            [FakeResponse(500), FakeResponse(500), FakeResponse(200)],
            backoff_factor=1.0,
        )
        fetcher.fetch(URL)
        assert no_sleep == [1.0, 2.0]  # 1*2^0, 1*2^1

    def test_backoff_is_capped(self, no_sleep):
        fetcher = make_fetcher(
            [FakeResponse(500)] * 6 + [FakeResponse(200)],
            max_retries=6,
            backoff_factor=10.0,
        )
        fetcher.config.backoff_max = 15.0
        fetcher.fetch(URL)
        assert max(no_sleep) <= 15.0

    def test_retry_after_header_is_honoured(self, no_sleep):
        fetcher = make_fetcher(
            [FakeResponse(429, headers={"Retry-After": "7"}), FakeResponse(200)]
        )
        fetcher.fetch(URL)
        assert no_sleep == [7.0]

    def test_retry_after_http_date_falls_back_to_backoff(self, no_sleep):
        fetcher = make_fetcher(
            [
                FakeResponse(429, headers={"Retry-After": "Wed, 21 Oct 2026 07:28:00 GMT"}),
                FakeResponse(200),
            ],
            backoff_factor=1.0,
        )
        fetcher.fetch(URL)
        assert no_sleep == [1.0]

    def test_waf_403_is_retried(self, no_sleep):
        """MathWorks' WAF soft-blocks intermittently; 403 is retryable."""
        fetcher = make_fetcher([FakeResponse(403), FakeResponse(200, "<html>x</html>")])
        assert fetcher.fetch(URL)
        assert len(fetcher.session.calls) == 2

    def test_exhausted_retries_raise_transient(self, no_sleep):
        fetcher = make_fetcher([FakeResponse(503)] * 4, max_retries=3)
        with pytest.raises(TransientFetchError, match="Exhausted"):
            fetcher.fetch(URL)
        assert len(fetcher.session.calls) == 4

    def test_timeout_is_retried_then_raises(self, no_sleep):
        fetcher = make_fetcher([requests.Timeout("slow")] * 3, max_retries=2)
        with pytest.raises(TransientFetchError, match="Network failure"):
            fetcher.fetch(URL)
        assert len(fetcher.session.calls) == 3

    def test_connection_error_recovers(self, no_sleep):
        fetcher = make_fetcher(
            [requests.ConnectionError("reset"), FakeResponse(200, "<html>back</html>")]
        )
        assert "back" in fetcher.fetch(URL)

    def test_zero_retries_means_one_attempt(self, no_sleep):
        fetcher = make_fetcher([FakeResponse(503)], max_retries=0)
        with pytest.raises(TransientFetchError):
            fetcher.fetch(URL)
        assert len(fetcher.session.calls) == 1


# ----------------------------------------------------------------- permanent


class TestPermanentFailures:
    def test_404_is_not_retried(self, no_sleep):
        fetcher = make_fetcher([FakeResponse(404)])
        with pytest.raises(PermanentFetchError, match="404"):
            fetcher.fetch(URL)
        assert len(fetcher.session.calls) == 1

    def test_410_is_not_retried(self, no_sleep):
        fetcher = make_fetcher([FakeResponse(410)])
        with pytest.raises(PermanentFetchError):
            fetcher.fetch(URL)

    def test_invalid_url_raises_permanent(self, no_sleep):
        fetcher = make_fetcher([requests.exceptions.MissingSchema("bad url")])
        with pytest.raises(PermanentFetchError):
            fetcher.fetch(URL)


# -------------------------------------------------------------------- robots


class TestRobots:
    def test_disallowed_url_is_refused(self):
        session = FakeSession([], robots_text="User-agent: *\nDisallow: /help/\n")
        fetcher = Fetcher(
            ScraperConfig(min_request_interval=0, respect_robots=True), session=session
        )
        with pytest.raises(PermanentFetchError, match="robots.txt"):
            fetcher.fetch(URL)
        assert session.calls == []

    def test_allowed_url_proceeds(self):
        session = FakeSession(
            [FakeResponse(200, "<html>fine</html>")],
            robots_text="User-agent: *\nDisallow: /private/\n",
        )
        fetcher = Fetcher(
            ScraperConfig(min_request_interval=0, respect_robots=True), session=session
        )
        assert "fine" in fetcher.fetch(URL)

    def test_robots_is_fetched_once_per_host(self):
        session = FakeSession([FakeResponse(), FakeResponse()])
        fetcher = Fetcher(
            ScraperConfig(min_request_interval=0, respect_robots=True), session=session
        )
        fetcher.fetch(URL)
        fetcher.fetch(URL)
        assert len(fetcher._robots) == 1

    def test_unreachable_robots_fails_open(self, monkeypatch):
        class BrokenSession(FakeSession):
            def get(self, url, timeout=None, **kwargs):
                if url.endswith("/robots.txt"):
                    raise requests.ConnectionError("no robots")
                return super().get(url, timeout=timeout, **kwargs)

        session = BrokenSession([FakeResponse(200, "<html>ok</html>")])
        fetcher = Fetcher(
            ScraperConfig(min_request_interval=0, respect_robots=True), session=session
        )
        assert "ok" in fetcher.fetch(URL)

    def test_respect_robots_false_skips_the_check(self):
        session = FakeSession(
            [FakeResponse(200, "<html>ok</html>")],
            robots_text="User-agent: *\nDisallow: /\n",
        )
        fetcher = Fetcher(
            ScraperConfig(min_request_interval=0, respect_robots=False), session=session
        )
        assert "ok" in fetcher.fetch(URL)


# --------------------------------------------------------------------- cache


class TestCache:
    def test_write_then_read_avoids_network(self, tmp_path):
        config = ScraperConfig(
            min_request_interval=0,
            respect_robots=False,
            cache_dir=str(tmp_path),
            use_cache=True,
        )
        session = FakeSession([FakeResponse(200, "<html>cached</html>")])
        fetcher = Fetcher(config, session=session)

        assert "cached" in fetcher.fetch(URL)
        assert len(session.calls) == 1

        second = Fetcher(config, session=FakeSession([]))
        assert "cached" in second.fetch(URL)
        assert second.session.calls == []

    def test_cache_disabled_by_default(self, tmp_path):
        config = ScraperConfig(
            min_request_interval=0, respect_robots=False, cache_dir=str(tmp_path)
        )
        session = FakeSession([FakeResponse(), FakeResponse()])
        fetcher = Fetcher(config, session=session)
        fetcher.fetch(URL)
        fetcher.fetch(URL)
        assert len(session.calls) == 2

    def test_unwritable_cache_dir_does_not_break_fetch(self, monkeypatch, tmp_path):
        config = ScraperConfig(
            min_request_interval=0, respect_robots=False, cache_dir=str(tmp_path)
        )
        fetcher = Fetcher(config, session=FakeSession([FakeResponse(200, "<html>x</html>")]))
        monkeypatch.setattr(
            "pathlib.Path.write_text",
            lambda *a, **k: (_ for _ in ()).throw(OSError("read-only")),
        )
        assert "x" in fetcher.fetch(URL)


# -------------------------------------------------------------------- config


class TestConfig:
    def test_rejects_negative_retries(self):
        with pytest.raises(ValueError):
            ScraperConfig(max_retries=-1)

    def test_rejects_negative_rate_interval(self):
        with pytest.raises(ValueError):
            ScraperConfig(min_request_interval=-1)

    def test_contact_email_becomes_from_header(self):
        config = ScraperConfig(contact_email="ops@example.com")
        assert config.headers["From"] == "ops@example.com"

    def test_browser_headers_present(self):
        """The WAF returns 403 without these, so their absence is a bug."""
        config = ScraperConfig()
        assert "User-Agent" in config.headers
        assert "Accept-Language" in config.headers

    def test_to_dict_omits_contact_email(self):
        config = ScraperConfig(contact_email="ops@example.com")
        assert "contact_email" not in config.to_dict()

    def test_from_env(self, monkeypatch):
        monkeypatch.setenv("MWSCRAPER_MAX_RETRIES", "9")
        monkeypatch.setenv("MWSCRAPER_RESPECT_ROBOTS", "false")
        config = ScraperConfig.from_env()
        assert config.max_retries == 9
        assert config.respect_robots is False

    def test_context_manager_closes_session(self):
        fetcher = make_fetcher([FakeResponse()])
        with fetcher as f:
            assert f is fetcher
