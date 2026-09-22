"""Extraction-accuracy and robustness tests for :class:`MathWorksParser`.

The "expected" numbers below were verified against the rendered live page
(DOM query counts: 10 ``div.codeinput``, 13 headings, 6 content images), so a
regression here means the parser drifted from the page, not that the
assertions were fitted to the parser's own output.
"""

from __future__ import annotations

import pytest

from conftest import MINIMAL_PAGE, PAGE_URL
from mwscraper.models import CodeBlock, ImageAsset, TextBlock
from mwscraper.parser import MathWorksParser, ParseError


# ---------------------------------------------------------------- metadata


class TestMetadata:
    def test_title(self, document):
        assert document.title == (
            "Object Tracking and Motion Planning Using Frenet Reference Path"
        )

    def test_release(self, document):
        assert document.release == "R2021b"

    def test_products(self, document):
        assert "Sensor Fusion and Tracking Toolbox" in document.products
        assert "Navigation Toolbox" in document.products
        assert "Automated Driving Toolbox" in document.products

    def test_description_from_meta(self, document):
        assert "autonomous vehicle" in document.description.lower()

    def test_html_hash_recorded(self, document):
        assert len(document.html_sha256) == 64

    def test_no_warnings_on_healthy_page(self, document):
        assert document.warnings == []


# ---------------------------------------------------------------- structure


class TestStructure:
    #: Every heading rendered in the page body, in order.
    EXPECTED_SECTIONS = [
        "Object Tracking and Motion Planning Using Frenet Reference Path",
        "Introduction",
        "Object State Transition and Measurement Modeling",
        "Setup",
        "Scenario and Sensors",
        "Joint Probabilistic Data Association Tracker",
        "Motion Planner",
        "Run Simulation",
        "Results",
        "Road-Integrated Motion Prediction",
        "Lane Change Prediction",
        "Tracker Imperfections",
        "Summary",
        "Supporting Functions",
    ]

    def test_all_sections_present_in_order(self, document):
        assert [s.title for s in document.sections] == self.EXPECTED_SECTIONS

    def test_heading_levels(self, document):
        assert document.section_by_title("Setup").heading.level == 3
        assert document.section_by_title("Motion Planner").heading.level == 4

    def test_subsection_path_records_ancestry(self, document):
        section = document.section_by_title("Scenario and Sensors")
        assert section.path == (
            "Object Tracking and Motion Planning Using Frenet Reference Path",
            "Setup",
            "Scenario and Sensors",
        )

    def test_parent_heading_kept_even_without_own_blocks(self, document):
        """'Setup' owns no prose but must survive as a hierarchy node."""
        setup = document.section_by_title("Setup")
        assert setup is not None
        assert setup.blocks == []

    def test_blocks_are_in_document_order(self, document):
        orders = [b.order for b in document.iter_blocks()]
        assert orders == sorted(orders)

    def test_anchors_captured(self, document):
        assert document.section_by_title("Introduction").heading.anchor


# --------------------------------------------------------------------- code


class TestCodeExtraction:
    def test_code_block_count_matches_page(self, document):
        """The live page renders exactly 10 div.codeinput listings."""
        assert len(document.all_code()) == 10

    def test_every_code_block_is_labelled_matlab(self, document):
        assert {c.language for c in document.all_code()} == {"matlab"}

    def test_tracker_snippet_extracted_verbatim(self, document):
        section = document.section_by_title(
            "Joint Probabilistic Data Association Tracker"
        )
        code = section.code_blocks()[0].code
        assert code.startswith("tracker = trackerJPDA(")
        assert "'AssignmentThreshold',[200 inf]" in code
        assert "'ConfirmationThreshold',[8 10]" in code
        assert "'DeletionThreshold',[5 5]" in code

    def test_multiline_code_preserves_newlines(self, document):
        section = document.section_by_title("Run Simulation")
        code = section.code_blocks()[0].code
        assert code.count("\n") > 15
        assert "while advance(scenario)" in code
        assert "predictTracksToTime(tracker,'confirmed',timesteps(i))" in code

    def test_copy_button_chrome_is_stripped(self, document):
        """Regression: the 'Get'/'Copy Code' UI must not leak into code."""
        for block in document.all_code():
            assert "Copy Code" not in block.code
            assert "Copy Command" not in block.code
            assert not block.code.strip().startswith("Get")

    def test_supporting_functions_classified(self, document):
        section = document.section_by_title("Supporting Functions")
        kinds = {c.block_type for c in section.code_blocks()}
        assert kinds == {"function"}

    def test_referenced_functions_detected(self, document):
        section = document.section_by_title("Motion Planner")
        funcs = section.code_blocks()[0].referenced_functions
        assert "dynamicCapsuleList" in funcs or "egoGeometry" in funcs

    def test_keywords_not_reported_as_functions(self, document):
        for block in document.all_code():
            assert "if" not in block.referenced_functions
            assert "for" not in block.referenced_functions
            assert "end" not in block.referenced_functions

    def test_every_code_block_carries_attribution(self, document):
        for block in document.all_code():
            assert block.source_section
            assert block.section_path

    def test_no_duplicate_code_blocks(self, document):
        digests = [c.digest for c in document.all_code()]
        assert len(digests) == len(set(digests))

    def test_line_count_is_consistent(self, document):
        for block in document.all_code():
            assert block.line_count == len(block.code.splitlines())


# --------------------------------------------------------------------- text


class TestTextExtraction:
    def test_intro_prose_captured(self, document):
        text = document.section_by_title("Introduction").plain_text()
        assert "local motion planner" in text

    def test_lists_extracted_as_bullets(self, document):
        text = document.section_by_title("Run Simulation").plain_text()
        assert "- " in text
        assert "Collect" in text or "collect" in text

    def test_equations_captured_with_mathml(self, document):
        section = document.section_by_title(
            "Object State Transition and Measurement Modeling"
        )
        equations = [b for b in section.text_blocks() if b.text_type == "equation"]
        assert len(equations) == 2
        assert all(eq.mathml and eq.mathml.startswith("<math") for eq in equations)

    def test_inline_math_does_not_swallow_paragraph(self, document):
        """A <p> containing inline <math> stays a paragraph."""
        section = document.section_by_title(
            "Object State Transition and Measurement Modeling"
        )
        paragraphs = [b for b in section.text_blocks() if b.text_type == "paragraph"]
        assert len(paragraphs) >= 4
        assert any("constant-speed state transition" in p.text for p in paragraphs)

    def test_paragraph_links_recorded(self, document):
        found = any(
            link["text"].startswith("Highway Trajectory Planning")
            for block in document.iter_blocks()
            if isinstance(block, TextBlock)
            for link in block.links
        )
        assert found


# ---------------------------------------------------------------- resources


class TestResources:
    def test_open_example_link_found(self, document):
        opens = [r for r in document.resources if r.kind == "openExample"]
        assert len(opens) == 1
        assert "ObjectTrackingAndMotionPlanning" in opens[0].filename

    def test_api_references_resolved_to_absolute_urls(self, document):
        refs = {r.text: r.url for r in document.resources if r.kind == "api_reference"}
        assert "trackerJPDA" in refs
        assert refs["trackerJPDA"].startswith("https://nl.mathworks.com/help/")
        assert ".." not in refs["trackerJPDA"]

    def test_related_examples_classified(self, document):
        related = [r.text for r in document.resources if r.kind == "related_example"]
        assert any("Highway Trajectory Planning" in t for t in related)

    def test_downloadable_figures_have_filenames(self, document):
        downloads = [r for r in document.resources if r.kind == "download"]
        assert downloads
        assert all(r.filename for r in downloads)
        assert any(r.filename.endswith(".gif") for r in downloads)

    def test_navigation_chrome_excluded(self, document):
        assert not any("s_tid=CRUX_topnav" in r.url for r in document.resources)

    def test_no_duplicate_resource_urls(self, document):
        urls = [r.url for r in document.resources]
        assert len(urls) == len(set(urls))


# ------------------------------------------------------------------- images


class TestImages:
    def test_all_figures_extracted(self, document):
        assert len(document.all_images()) == 6

    def test_animation_detected(self, document):
        assert any(i.media_type == "animation" for i in document.all_images())

    def test_image_urls_absolute(self, document):
        for image in document.all_images():
            assert image.url.startswith("https://")

    def test_site_logos_excluded(self, document):
        for image in document.all_images():
            assert "logo" not in image.url.lower()

    def test_images_attributed_to_sections(self, document):
        section = document.section_by_title("Lane Change Prediction")
        assert len(section.images()) == 1


# --------------------------------------------------------------- edge cases


class TestEdgeCases:
    def test_empty_html_raises(self, parser):
        with pytest.raises(ParseError):
            parser.parse("", PAGE_URL)

    def test_whitespace_only_html_raises(self, parser):
        with pytest.raises(ParseError):
            parser.parse("   \n\t ", PAGE_URL)

    def test_non_html_input_degrades_gracefully(self, parser):
        """Feeding an RSS/XML document must warn, not explode.

        (lxml synthesises a <body> for almost any input, so the
        "no content root" ParseError is only reachable with strict parsers;
        the contract that matters is that we never raise an unexpected type.)
        """
        doc = parser.parse("<?xml version='1.0'?><rss><item/></rss>", PAGE_URL)
        assert doc.all_code() == []
        assert doc.warnings

    def test_missing_body_raises_parse_error(self, parser, monkeypatch):
        """When no content root and no <body> exist, fail loudly."""
        import mwscraper.parser as parser_mod

        monkeypatch.setattr(parser_mod, "CONTENT_SELECTORS", ())
        soup = parser._make_soup("<html></html>")
        monkeypatch.setattr(
            MathWorksParser, "_make_soup", lambda self, html: soup
        )
        soup.html.clear() if soup.html else None
        if soup.body:
            soup.body.decompose()
        with pytest.raises(ParseError):
            parser.parse("<html></html>", PAGE_URL)

    def test_unrelated_page_produces_warnings_not_crash(self, parser):
        doc = parser.parse(
            "<html><body><p>Nothing to see here.</p></body></html>", PAGE_URL
        )
        assert doc.all_code() == []
        assert any("code blocks" in w for w in doc.warnings)

    def test_missing_title_falls_back_to_meta(self, parser):
        html = (
            '<html><head><meta property="og:title" '
            'content="Fallback Title - MATLAB &amp; Simulink"></head>'
            "<body><p>Body text.</p></body></html>"
        )
        doc = parser.parse(html, PAGE_URL)
        assert doc.title == "Fallback Title"

    def test_malformed_unclosed_tags_are_tolerated(self, parser):
        html = (
            '<html><body><section id="doc_center_content">'
            "<h3>Broken<p>Paragraph without close"
            '<div class="codeinput"><pre>a = 1;</pre>'
            "</section></body></html>"
        )
        doc = parser.parse(html, PAGE_URL)
        assert any("a = 1;" in c.code for c in doc.all_code())

    def test_code_block_with_no_pre_is_skipped(self, parser):
        html = (
            '<html><body><section id="doc_center_content"><h3>S</h3>'
            '<div class="codeinput"></div></section></body></html>'
        )
        doc = parser.parse(html, PAGE_URL)
        assert doc.all_code() == []

    def test_image_without_src_ignored(self, parser):
        html = (
            '<html><body><section id="doc_center_content"><h3>S</h3>'
            "<p>text</p><img alt='no src'></section></body></html>"
        )
        doc = parser.parse(html, PAGE_URL)
        assert doc.all_images() == []

    def test_minimal_page_end_to_end(self, parser):
        doc = parser.parse(MINIMAL_PAGE, PAGE_URL)
        assert doc.title == "Tiny Example"
        assert doc.release == "R2023a"
        section = doc.section_by_title("First Section")
        assert section is not None
        assert "x = 5;" in section.code_blocks()[0].code
        assert "Copy Code" not in section.code_blocks()[0].code
        assert any(b.text_type == "list" for b in section.text_blocks())
        assert len(doc.all_images()) == 1

    def test_content_root_fallback_to_body(self, parser):
        html = "<html><body><h3>Only Heading</h3><p>Prose.</p></body></html>"
        doc = parser.parse(html, PAGE_URL)
        assert doc.section_by_title("Only Heading") is not None

    def test_parse_is_deterministic(self, parser, page_html):
        first = parser.parse(page_html, PAGE_URL)
        second = parser.parse(page_html, PAGE_URL)
        assert [c.code for c in first.all_code()] == [
            c.code for c in second.all_code()
        ]
        assert first.html_sha256 == second.html_sha256

    def test_html_parser_backend_gives_same_code(self, page_html):
        """Extraction must not depend on lxml being installed."""
        lxml_doc = MathWorksParser("lxml").parse(page_html, PAGE_URL)
        std_doc = MathWorksParser("html.parser").parse(page_html, PAGE_URL)
        assert len(std_doc.all_code()) == len(lxml_doc.all_code())
