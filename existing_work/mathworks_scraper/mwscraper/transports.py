"""Pluggable HTTP transports.

Why this module exists
----------------------
MathWorks sits behind an Akamai-style WAF that fingerprints the **TLS
handshake**, not just the headers. Measured on 2026-08-28 against
``nl.mathworks.com`` with a byte-identical header set:

===============================  ===========
client                           status
===============================  ===========
``curl``                         200
``requests`` / ``urllib3``       403
===============================  ===========

No amount of header spoofing fixes this, because Python's TLS ClientHello
(cipher order, extensions, ALPN) differs from a browser's. The practical
answers are (a) use a client that reproduces a browser handshake, or
(b) drive a real browser.

So the transport is an interface with several implementations, tried in
order of cost:

1. :class:`RequestsTransport` - fastest, works on any site without TLS
   fingerprinting. Always tried first.
2. :class:`CurlTransport` - shells out to the system ``curl``, whose
   handshake the WAF accepts. No extra Python dependency; ``curl`` ships with
   Windows 10+, macOS and virtually every Linux image.
3. :class:`CurlCffiTransport` - used only if the optional ``curl_cffi``
   package is installed; a true browser-impersonating TLS stack, and the best
   choice for a long-running production deployment.

If every transport fails, drive a real browser (Playwright/Selenium); see the
README's "Known limitations".
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Protocol

logger = logging.getLogger(__name__)


@dataclass
class TransportResponse:
    """Minimal response shape shared by all transports."""

    status_code: int
    text: str
    headers: Dict[str, str] = field(default_factory=dict)


class TransportError(RuntimeError):
    """The transport could not complete the request at all (retryable)."""


class InvalidRequestError(TransportError):
    """The request itself is malformed, so no transport can help.

    Kept distinct from :class:`TransportError` because a bad URL must fail
    fast rather than burn the whole retry budget.
    """


class Transport(Protocol):
    name: str

    def available(self) -> bool: ...

    def get(
        self, url: str, headers: Dict[str, str], timeout: float
    ) -> TransportResponse: ...


class RequestsTransport:
    """Standard ``requests`` transport. Fast, but TLS-fingerprintable."""

    name = "requests"

    def __init__(self, session=None) -> None:
        import requests

        self._requests = requests
        self.session = session or requests.Session()

    def available(self) -> bool:
        return True

    def get(self, url, headers, timeout) -> TransportResponse:
        exceptions = self._requests.exceptions
        try:
            resp = self.session.get(url, headers=headers, timeout=timeout)
        except (
            exceptions.MissingSchema,
            exceptions.InvalidSchema,
            exceptions.InvalidURL,
            exceptions.URLRequired,
        ) as exc:
            raise InvalidRequestError(str(exc)) from exc
        except exceptions.RequestException as exc:
            raise TransportError(str(exc)) from exc
        return TransportResponse(
            status_code=resp.status_code,
            text=resp.text or "",
            headers=dict(resp.headers),
        )

    def close(self) -> None:
        self.session.close()


class CurlTransport:
    """Transport that shells out to the system ``curl`` binary.

    Chosen because its TLS handshake is accepted by the MathWorks WAF while
    Python's is not, and because it needs no extra Python dependency.
    Response headers are captured via ``-D`` into a temp file so that
    ``Retry-After`` still reaches the retry logic.
    """

    name = "curl"

    def __init__(self, executable: Optional[str] = None) -> None:
        self.executable = executable or shutil.which("curl")

    def available(self) -> bool:
        return bool(self.executable)

    def get(self, url, headers, timeout) -> TransportResponse:
        if not self.available():
            raise TransportError("curl executable not found on PATH")

        # mkstemp hands back an *open* descriptor; on Windows leaving it open
        # makes the later unlink fail with "used by another process".
        handle, header_path = tempfile.mkstemp(prefix="mwscraper_hdr_")
        os.close(handle)
        header_file = Path(header_path)
        try:
            command: List[str] = [
                self.executable,
                "-sS",
                "--compressed",
                "-L",
                "--max-time",
                str(int(timeout)),
                "-D",
                str(header_file),
                "-w",
                "%{http_code}",
            ]
            for key, value in headers.items():
                command += ["-H", f"{key}: {value}"]
            command.append(url)

            try:
                proc = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=timeout + 15,
                )
            except subprocess.TimeoutExpired as exc:
                raise TransportError(f"curl timed out after {timeout}s") from exc
            except OSError as exc:
                raise TransportError(f"could not run curl: {exc}") from exc

            if proc.returncode != 0:
                raise TransportError(
                    f"curl exited {proc.returncode}: {(proc.stderr or '').strip()[:200]}"
                )

            body = proc.stdout or ""
            # -w appends the status code to the very end of stdout.
            status_code, body = self._split_status(body)
            return TransportResponse(
                status_code=status_code,
                text=body,
                headers=self._parse_headers(header_file),
            )
        finally:
            try:
                header_file.unlink(missing_ok=True)
            except OSError as exc:  # pragma: no cover - platform dependent
                logger.debug("Could not remove temp header file: %s", exc)

    @staticmethod
    def _split_status(payload: str) -> tuple[int, str]:
        trailing = payload[-3:]
        if trailing.isdigit():
            return int(trailing), payload[:-3]
        raise TransportError("could not read status code from curl output")

    @staticmethod
    def _parse_headers(path: Path) -> Dict[str, str]:
        headers: Dict[str, str] = {}
        try:
            raw = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return headers
        # With -L there may be several header blocks; the last one wins.
        for line in raw.splitlines():
            if ":" in line:
                key, _, value = line.partition(":")
                headers[key.strip()] = value.strip()
        return headers


class CurlCffiTransport:
    """Optional browser-impersonating transport (``pip install curl_cffi``)."""

    name = "curl_cffi"

    def __init__(self, impersonate: str = "chrome") -> None:
        self.impersonate = impersonate
        try:
            from curl_cffi import requests as curl_requests

            self._client = curl_requests
        except ImportError:
            self._client = None

    def available(self) -> bool:
        return self._client is not None

    def get(self, url, headers, timeout) -> TransportResponse:
        if not self.available():
            raise TransportError("curl_cffi is not installed")
        try:
            resp = self._client.get(
                url, headers=headers, timeout=timeout, impersonate=self.impersonate
            )
        except Exception as exc:  # curl_cffi raises its own error hierarchy
            raise TransportError(str(exc)) from exc
        return TransportResponse(
            status_code=resp.status_code,
            text=resp.text or "",
            headers=dict(resp.headers),
        )


def default_transports(session=None) -> List[Transport]:
    """Build the standard fallback chain, cheapest first."""
    chain: List[Transport] = [RequestsTransport(session)]
    curl_cffi = CurlCffiTransport()
    if curl_cffi.available():
        chain.append(curl_cffi)
    curl = CurlTransport()
    if curl.available():
        chain.append(curl)
    return chain
