"""Shared fixtures.

``frenet_page.html`` is a byte-for-byte copy of the live MathWorks page,
captured 2026-08-28. Pinning it means the test suite validates extraction
logic deterministically and never touches the network.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mwscraper.parser import MathWorksParser  # noqa: E402

FIXTURE_DIR = Path(__file__).parent / "fixtures"
PAGE_URL = (
    "https://nl.mathworks.com/help/fusion/ug/"
    "object-tracking-and-motion-planning-using-frenet-reference-path.html"
)


@pytest.fixture(scope="session")
def page_html() -> str:
    path = FIXTURE_DIR / "frenet_page.html"
    if not path.is_file():
        pytest.skip(f"fixture missing: {path}")
    return path.read_text(encoding="utf-8", errors="replace")


@pytest.fixture(scope="session")
def document(page_html):
    """The parsed real page, shared across tests (parsing is pure)."""
    return MathWorksParser().parse(page_html, PAGE_URL)


@pytest.fixture
def parser() -> MathWorksParser:
    return MathWorksParser()


MINIMAL_PAGE = """
<html><head><title>Tiny Example - MATLAB &amp; Simulink</title>
<meta name="description" content="A tiny page."></head>
<body><section id="doc_center_content">
  <h1>Tiny Example</h1>
  <div class="doc_topic_desc"><em>Since R2023a</em></div>
  <p>Intro paragraph with a <a href="../ref/foo.html">foo</a> link.</p>
  <h3 class="title" id="s1">First Section</h3>
  <p>Explanation of the first section.</p>
  <ul><li>Point one</li><li>Point two</li></ul>
  <div class="code_responsive -has_code_copy">
    <div class="btn-group code_actions"><button>Copy Code</button>Get</div>
    <div class="programlisting"><div class="codeinput"><pre>x = 5;
y = trackerJPDA(x);</pre></div></div>
  </div>
  <img src="../../examples/img/figure_01.png" alt="A figure">
</section></body></html>
"""
