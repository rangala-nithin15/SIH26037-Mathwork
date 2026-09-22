# MathWorks Documentation Scraper

Extracts every piece of content from the MathWorks example page
[**Object Tracking and Motion Planning Using Frenet Reference Path**](https://nl.mathworks.com/help/fusion/ug/object-tracking-and-motion-planning-using-frenet-reference-path.html)
— MATLAB code listings, prose, MathML equations, figures, and outbound
resources — and maps all of it back to the section it came from.

Written against the live page, not a guess at its markup. The selectors below
were verified by querying the rendered DOM.

---

## Quick start

```bash
pip install -r requirements.txt
```

Scrape the default page into `./output`:

```bash
python -m mwscraper
```

Re-parse a saved copy with no network access at all:

```bash
python -m mwscraper --from-file tests/fixtures/frenet_page.html
```

Run the test suite (174 tests, fully offline):

```bash
python -m pytest
```

Run the executable Python port of the documented motion model:

```bash
python python_port/frenet_motion_model.py
```

---

## What you get

Running the scraper writes three artefacts into `--out-dir`:

| Artefact | Contents |
| --- | --- |
| `extracted_content.json` | Full structured model: metadata, sections, blocks, resources, warnings |
| `extracted_content.md` | Human-readable rendering with fenced MATLAB, equations, figures, resource tables |
| `code/*.m` | Each MATLAB listing as its own file with a `% From: <section>` provenance header |

Verified extraction from the live page:

```
Sections   : 14      (matches all 14 headings on the page)
Code blocks: 10      (matches all 10 div.codeinput listings)
Code lines : 167
Text blocks: 25      (paragraphs, bullet lists, 2 MathML equations)
Images     : 6       (5 figures + 1 animated GIF)
Resources  : 23      (11 API refs, 6 downloads, 3 products, 2 related examples, 1 openExample)
Warnings   : 0
```

---

## Architecture

```
mwscraper/
  config.py       ScraperConfig - timeouts, retries, rate limit, robots, cache
  transports.py   Transport chain: requests -> curl_cffi -> curl  (see below)
  fetcher.py      Rate limiting, exponential backoff, robots.txt, disk cache
  parser.py       HTML -> Document. All page-structure knowledge lives here.
  translate.py    Conservative MATLAB -> Python translation (refuses to guess)
  models.py       Typed content model (Document/Section/CodeBlock/TextBlock/...)
  exporters.py    Markdown, per-snippet .m files, flat section dict
  scraper.py      Facade: fetch + parse + export
  cli.py          Command line entry point
python_port/
  frenet_motion_model.py   Runnable Python implementation of the motion model
tests/                     174 offline tests + a pinned copy of the real page
```

---

## Static HTML, not JavaScript — and why that mattered less than TLS

**The page is fully server-rendered.** All prose, all ten code listings, both
MathML equations and every figure are present in the initial HTML response. I
confirmed this by diffing the rendered DOM against the raw response body.
**Selenium and Playwright are therefore unnecessary**, and the scraper uses
`requests` + BeautifulSoup, which is roughly two orders of magnitude cheaper.

The real obstacle was different. MathWorks sits behind a WAF that fingerprints
the **TLS handshake**, not the headers. Measured 2026-08-28 with a
byte-identical header set:

| Client | Result |
| --- | --- |
| `curl` | **200 OK** |
| Python `requests` / `urllib3` | **403 Access Denied** |

Header spoofing cannot fix this — Python's TLS ClientHello (cipher ordering,
extensions, ALPN) simply differs from a browser's. So `transports.py` defines
a transport interface with a fallback chain, tried cheapest first:

1. **`RequestsTransport`** — fast; works on any site without TLS fingerprinting.
2. **`CurlCffiTransport`** — used only if the optional `curl_cffi` package is
   installed. A genuine browser-impersonating TLS stack; **the recommended
   choice for production.**
3. **`CurlTransport`** — shells out to the system `curl`, whose handshake the
   WAF accepts. Needs no extra Python dependency (`curl` ships with Windows
   10+, macOS and nearly every Linux image).

A `403` from one transport is treated as *"blocked"* rather than *"forbidden"*,
so the next transport gets a turn. Whichever succeeds is remembered and tried
first afterwards, so the fallback is paid for once, not per request. Real log
from a live run:

```
GET .../object-tracking-...-frenet-reference-path.html (attempt 1/5)
Transport requests got 403 (likely TLS fingerprinting); trying next transport
Switching preferred transport to curl
Fetched ... (95360 bytes)
```

Disable the chain with `ScraperConfig(transport_fallback=False)`.

---

## Rate limiting and retry strategy

**Rate limiting.** A thread-safe `RateLimiter` enforces a hard floor of
`min_request_interval` seconds (default **1.0 s**, CLI `--rate-interval`)
between consecutive requests from one `Fetcher`. A minimum-interval gate is
preferred over a token bucket because a bucket permits bursts, and a burst is
exactly what a documentation host should not receive.

**Retries.** Up to `max_retries` (default 4) additional attempts on retryable
conditions — connection resets, timeouts, `5xx`, `429`, and `403` (which the
WAF returns for soft blocks). Delay is `backoff_factor * 2**attempt`, capped at
`backoff_max` (30 s) and jittered to avoid synchronised retry storms:

| Attempt | Delay (default `backoff_factor=0.75`) |
| --- | --- |
| 1 | ~0.75 s |
| 2 | ~1.5 s |
| 3 | ~3.0 s |
| 4 | ~6.0 s |

`Retry-After` always wins when the server sends it — including through the
curl transport, which captures response headers via `curl -D`.

**Non-retryable** statuses (`404`, `410`, malformed URLs) raise
`PermanentFetchError` immediately rather than burning the retry budget.

**robots.txt** is fetched once per host and consulted before every request; a
disallowed URL raises `PermanentFetchError` without being requested. An
unreachable `robots.txt` fails *open* but logs a warning. `nl.mathworks.com`
serves `403` for `/robots.txt` itself, which is treated as permissive.

**Caching.** `--cache-dir` plus `--use-cache` stores raw HTML on disk so
re-parsing during development costs zero requests.

---

## Extraction approach

The page body is **flat**, not nested per section: headings and content are
siblings inside `section#doc_center_content`. The parser therefore walks the
DOM in document order and switches the "current section" whenever it meets a
heading. This is what preserves the relationship between a code block and the
prose that explains it.

Verified selectors and the traps they avoid:

| Target | Selector | Note |
| --- | --- | --- |
| Content root | `section#doc_center_content` | Falls back through `.content_container`, `main`, `body` |
| Sections | `h1`–`h4` in document order | Heading `id`s captured as anchors |
| MATLAB code | `div.codeinput > pre` | `get_text()` on `<pre>` keeps newlines and drops highlight `<span>`s |
| Copy/Get UI | `div.btn-group.code_actions` | **Stripped** — otherwise "Get"/"Copy Code" leaks into every snippet |
| Equations | `div.code_responsive` with `<math>` and no `<pre>` | `code_responsive` is used for *both* code and equations |
| Figures | `img` inside the content root | Site logos under `/etc.clientlibs/` excluded |
| Products | `a.coming_from_product`, `[data-products]` | |

Two subtleties worth calling out, both covered by regression tests:

* **Inline vs. displayed math.** Prose paragraphs contain inline `<math>`
  (e.g. "at time step *k*"). Treating any element containing `<math>` as an
  equation swallowed whole paragraphs, so only dedicated equation containers
  qualify.
* **Relative links.** Cross-references are relative (`../../nav/ref/foo.html`)
  and contain no `/help/` segment. Classification runs on the *resolved*
  absolute URL, otherwise every API reference is misfiled as "internal".

Equations are stored as flattened text **plus the raw MathML**, since
flattening is lossy and downstream consumers may want to render properly.

---

## MATLAB → Python translation

Automatic MATLAB-to-Python translation is not generally sound: MATLAB is
1-indexed with inclusive ranges, `a(i)` is ambiguous between indexing and a
function call, `end` is context-dependent, and `trackerJPDA` /
`referencePathFrenet` / `dynamicCapsuleList` have no Python counterpart.

`translate.py` therefore **refuses to guess.** It emits Python only when every
line falls in a provably safe subset (comments, scalar assignments, ranges —
with the inclusive-endpoint correction `np.arange(a, b + step/2, step)`).
Anything else yields `None` plus a `translation_note` explaining why, so a
reader can tell a deliberate refusal from a silent miss.

For real Python, `python_port/frenet_motion_model.py` is a hand-written,
tested implementation of the model the page documents:

* `ReferencePath` — arc-length polyline with `frenet2global` / `global2frenet`
* `FrenetMotionModel` — the published transition matrix, noise gain `G`, and
  covariance propagation `P' = FPF' + Q`
* `predict_trajectory` — the 5 s / 0.5 s prediction loop from *Run Simulation*

It reproduces the page's central claim. Over a 5-second horizon on a curving
highway, a constant-velocity model ends up **38 m away** from the
road-integrated prediction and **35 m outside the lane** — precisely the false
collision the example describes:

```
Road-integrated end point  : (  117.02,   37.81)
Constant-velocity end point: (  125.00,    0.16)
divergence after 5 s       : 38.48 m
```

It is deliberately **not** a port of `trackerJPDA`. JPDA data association and
track management are proprietary toolbox functionality; the page's MATLAB for
those is preserved verbatim rather than approximated.

---

## Testing

174 tests, no network access, ~1 second:

| File | Tests | Covers |
| --- | --- | --- |
| `test_parser.py` | 52 | Extraction accuracy against a pinned copy of the real page; malformed HTML, missing elements, empty input, parser-backend independence |
| `test_fetcher.py` | 36 | Rate limiting, backoff arithmetic, `Retry-After`, robots.txt, caching, permanent vs transient errors |
| `test_transports.py` | 23 | Fallback chain, 403-means-blocked, curl invocation and header parsing (subprocess stubbed) |
| `test_exporters_and_translate.py` | 32 | Markdown/JSON/`.m` output, attribution, translator refusals, CLI exit codes |
| `test_python_port.py` | 31 | Transition matrix vs. the published equations, lane-change convergence, round-trip Frenet conversion |

Expected values are anchored to the **rendered page** (10 `div.codeinput`, 14
headings, 6 content images), so a failure means the parser drifted from the
page rather than that assertions were fitted to the parser's own output.

Tests are run against `tests/fixtures/frenet_page.html`, a byte-for-byte copy
captured 2026-08-28. Refresh it with:

```bash
python -m mwscraper --cache-dir tests/fixtures --out-dir /tmp/check
```

---

## Assumptions

1. **The page is server-rendered.** Verified. If MathWorks moves to
   client-side rendering, the parser will emit a "No code blocks were
   extracted" warning rather than silently returning nothing.
2. **`div.codeinput` marks executable MATLAB.** True across MathWorks example
   pages. Listings are labelled `matlab`; nothing is auto-detected by content.
3. **The page has no separate downloadable files.** Its supporting helpers
   (`helperTrackingAndPlanningScenario`, `HelperTrackingAndPlanningDisplay`,
   …) ship *inside MATLAB* via `openExample('driving_fusion_nav/...')`, not as
   HTTP downloads. That link is captured as an `openExample` resource; the
   figures and the animated GIF are captured as `download` resources. If you
   need the helper sources, run the `openExample` command in MATLAB — they are
   not served over the web.
4. **`robots.txt` returning 403 means permissive.** MathWorks blocks the file
   itself; there is no directive to honour.
5. **Scraping is for personal/research use.** MathWorks documentation is
   copyrighted. Respect their terms before redistributing extracted content.

## Known limitations

* **TLS fingerprinting** is the fragile point. If MathWorks starts blocking
  `curl` too, install `curl_cffi` (`pip install curl_cffi`, then uncomment it
  in `requirements.txt`) — it is picked up automatically. If that also fails,
  drive a real browser; the parser is completely independent of the fetcher,
  so only the transport needs replacing.
* **Equation text is lossy.** Flattened MathML renders as `xk=[sksk˙dkdk˙]`.
  The raw MathML is preserved in the `mathml` field for proper rendering.
* **Images are referenced, not downloaded.** URLs are absolute and ready to
  fetch; add a download step if you need the bytes.
* **Single-page focused.** `scrape_many()` handles batches and shares the rate
  limiter, but there is no link-following crawler by design.
* **`--ignore-robots`** exists for sites where you have explicit permission.
  Leave it off for MathWorks.
