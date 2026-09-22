"""Tests for the transport abstraction and the WAF fallback chain.

``curl`` is never actually invoked here: ``subprocess.run`` is stubbed so the
tests are hermetic and run on machines without curl installed.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
import requests

from mwscraper.config import ScraperConfig
from mwscraper.fetcher import Fetcher, PermanentFetchError, TransientFetchError
from mwscraper.transports import (
    CurlTransport,
    InvalidRequestError,
    RequestsTransport,
    TransportError,
    TransportResponse,
    default_transports,
)

URL = "https://nl.mathworks.com/help/fusion/ug/example.html"


class StubTransport:
    """Scriptable transport used to drive the fallback logic."""

    def __init__(self, name, script, is_available=True):
        self.name = name
        self.script = list(script)
        self.is_available = is_available
        self.calls = 0

    def available(self):
        return self.is_available

    def get(self, url, headers, timeout):
        self.calls += 1
        item = self.script.pop(0) if self.script else TransportResponse(200, "<html>ok</html>")
        if isinstance(item, Exception):
            raise item
        return item


def make_fetcher(transports, **overrides):
    config = ScraperConfig(
        min_request_interval=0,
        respect_robots=False,
        jitter=0.0,
        max_retries=overrides.pop("max_retries", 2),
        **overrides,
    )
    return Fetcher(config, transports=transports)


@pytest.fixture
def no_sleep(monkeypatch):
    monkeypatch.setattr("mwscraper.fetcher.time.sleep", lambda s: None)


# ----------------------------------------------------------- fallback chain


class TestFallbackChain:
    def test_403_falls_through_to_next_transport(self, no_sleep):
        blocked = StubTransport("requests", [TransportResponse(403, "denied")])
        working = StubTransport("curl", [TransportResponse(200, "<html>real</html>")])
        fetcher = make_fetcher([blocked, working])

        assert "real" in fetcher.fetch(URL)
        assert blocked.calls == 1
        assert working.calls == 1

    def test_successful_transport_becomes_preferred(self, no_sleep):
        blocked = StubTransport(
            "requests", [TransportResponse(403, "d"), TransportResponse(403, "d")]
        )
        working = StubTransport(
            "curl",
            [TransportResponse(200, "<html>a</html>"), TransportResponse(200, "<html>b</html>")],
        )
        fetcher = make_fetcher([blocked, working])

        fetcher.fetch(URL)
        fetcher.fetch(URL)
        # The blocked transport is skipped on the second call.
        assert blocked.calls == 1
        assert working.calls == 2

    def test_transport_error_tries_the_next_one(self, no_sleep):
        broken = StubTransport("requests", [TransportError("tls handshake failed")])
        working = StubTransport("curl", [TransportResponse(200, "<html>ok</html>")])
        fetcher = make_fetcher([broken, working])
        assert "ok" in fetcher.fetch(URL)

    def test_unavailable_transport_is_skipped(self, no_sleep):
        missing = StubTransport("curl", [], is_available=False)
        working = StubTransport("requests", [TransportResponse(200, "<html>ok</html>")])
        fetcher = make_fetcher([missing, working])
        assert "ok" in fetcher.fetch(URL)
        assert missing.calls == 0

    def test_403_from_every_transport_is_reported(self, no_sleep):
        a = StubTransport("requests", [TransportResponse(403, "d")] * 3)
        b = StubTransport("curl", [TransportResponse(403, "d")] * 3)
        fetcher = make_fetcher([a, b], max_retries=2)
        with pytest.raises(TransientFetchError):
            fetcher.fetch(URL)

    def test_single_transport_403_is_not_swallowed(self, no_sleep):
        """With one transport there is nothing to fall back to."""
        only = StubTransport("requests", [TransportResponse(403, "d")] * 3)
        fetcher = make_fetcher([only], max_retries=2)
        with pytest.raises(TransientFetchError):
            fetcher.fetch(URL)
        assert only.calls == 3

    def test_malformed_url_fails_fast_without_fallback(self, no_sleep):
        bad = StubTransport("requests", [InvalidRequestError("No schema supplied")])
        other = StubTransport("curl", [TransportResponse(200, "<html>ok</html>")])
        fetcher = make_fetcher([bad, other])
        with pytest.raises(PermanentFetchError, match="Invalid request"):
            fetcher.fetch("not-a-url")
        assert other.calls == 0

    def test_404_is_still_permanent_through_the_chain(self, no_sleep):
        only = StubTransport("requests", [TransportResponse(404, "missing")])
        fetcher = make_fetcher([only])
        with pytest.raises(PermanentFetchError, match="404"):
            fetcher.fetch(URL)

    def test_retry_after_survives_the_transport_layer(self, monkeypatch):
        slept = []
        monkeypatch.setattr("mwscraper.fetcher.time.sleep", lambda s: slept.append(s))
        only = StubTransport(
            "requests",
            [
                TransportResponse(429, "slow down", {"Retry-After": "4"}),
                TransportResponse(200, "<html>ok</html>"),
            ],
        )
        fetcher = make_fetcher([only])
        fetcher.fetch(URL)
        assert slept == [4.0]


# ------------------------------------------------------------ curl transport


class FakeProc:
    def __init__(self, stdout="", returncode=0, stderr=""):
        self.stdout = stdout
        self.returncode = returncode
        self.stderr = stderr


class TestCurlTransport:
    def test_parses_body_and_status(self, monkeypatch, tmp_path):
        def fake_run(command, **kwargs):
            # Write the header file curl was told to produce.
            header_path = Path(command[command.index("-D") + 1])
            header_path.write_text("HTTP/2 200\ncontent-type: text/html\n")
            return FakeProc(stdout="<html>page</html>200")

        monkeypatch.setattr(subprocess, "run", fake_run)
        resp = CurlTransport(executable="curl").get(URL, {"User-Agent": "x"}, 10)
        assert resp.status_code == 200
        assert resp.text == "<html>page</html>"
        assert resp.headers["content-type"] == "text/html"

    def test_captures_retry_after_header(self, monkeypatch):
        def fake_run(command, **kwargs):
            Path(command[command.index("-D") + 1]).write_text(
                "HTTP/2 429\nRetry-After: 12\n"
            )
            return FakeProc(stdout="throttled429")

        monkeypatch.setattr(subprocess, "run", fake_run)
        resp = CurlTransport(executable="curl").get(URL, {}, 10)
        assert resp.status_code == 429
        assert resp.headers["Retry-After"] == "12"

    def test_passes_headers_through(self, monkeypatch):
        captured = {}

        def fake_run(command, **kwargs):
            captured["cmd"] = command
            Path(command[command.index("-D") + 1]).write_text("HTTP/2 200\n")
            return FakeProc(stdout="ok200")

        monkeypatch.setattr(subprocess, "run", fake_run)
        CurlTransport(executable="curl").get(URL, {"User-Agent": "Mozilla/5.0"}, 10)
        assert "User-Agent: Mozilla/5.0" in captured["cmd"]

    def test_nonzero_exit_raises(self, monkeypatch):
        monkeypatch.setattr(
            subprocess, "run", lambda *a, **k: FakeProc(returncode=6, stderr="no host")
        )
        with pytest.raises(TransportError, match="curl exited 6"):
            CurlTransport(executable="curl").get(URL, {}, 10)

    def test_timeout_raises_transport_error(self, monkeypatch):
        def boom(*a, **k):
            raise subprocess.TimeoutExpired("curl", 10)

        monkeypatch.setattr(subprocess, "run", boom)
        with pytest.raises(TransportError, match="timed out"):
            CurlTransport(executable="curl").get(URL, {}, 10)

    def test_unparseable_output_raises(self, monkeypatch):
        monkeypatch.setattr(subprocess, "run", lambda *a, **k: FakeProc(stdout="no code"))
        with pytest.raises(TransportError, match="status code"):
            CurlTransport(executable="curl").get(URL, {}, 10)

    def test_unavailable_when_curl_missing(self, monkeypatch):
        monkeypatch.setattr("mwscraper.transports.shutil.which", lambda name: None)
        assert CurlTransport().available() is False

    def test_temp_header_file_is_cleaned_up(self, monkeypatch):
        """Regression: an open mkstemp handle broke unlink on Windows."""
        seen = {}

        def fake_run(command, **kwargs):
            path = Path(command[command.index("-D") + 1])
            path.write_text("HTTP/2 200\n")
            seen["path"] = path
            return FakeProc(stdout="body200")

        monkeypatch.setattr(subprocess, "run", fake_run)
        CurlTransport(executable="curl").get(URL, {}, 10)
        assert not seen["path"].exists()


class TestRequestsTransport:
    def test_wraps_network_errors(self):
        class BoomSession:
            headers = {}

            def get(self, *a, **k):
                raise requests.ConnectionError("reset")

        with pytest.raises(TransportError):
            RequestsTransport(BoomSession()).get(URL, {}, 10)

    def test_malformed_url_raises_invalid_request(self):
        class BadSession:
            headers = {}

            def get(self, *a, **k):
                raise requests.exceptions.MissingSchema("no schema")

        with pytest.raises(InvalidRequestError):
            RequestsTransport(BadSession()).get(URL, {}, 10)

    def test_returns_transport_response(self):
        class OkSession:
            headers = {}

            def get(self, *a, **k):
                class R:
                    status_code = 200
                    text = "<html>hi</html>"
                    headers = {"X": "1"}

                return R()

        resp = RequestsTransport(OkSession()).get(URL, {}, 10)
        assert isinstance(resp, TransportResponse)
        assert resp.status_code == 200


class TestDefaultChain:
    def test_requests_is_always_first(self):
        chain = default_transports()
        assert chain[0].name == "requests"

    def test_chain_is_not_empty(self):
        assert len(default_transports()) >= 1

    def test_fallback_can_be_disabled(self):
        fetcher = Fetcher(ScraperConfig(transport_fallback=False))
        assert [t.name for t in fetcher.transports] == ["requests"]
