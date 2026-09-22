"""Tests for the export layer, the MATLAB translator and the CLI wiring."""

from __future__ import annotations

import json

import pytest

from conftest import PAGE_URL
from mwscraper.exporters import (
    document_to_markdown,
    document_to_sections_dict,
    write_code_files,
)
from mwscraper.models import CodeBlock, Document, Heading, Section, TextBlock
from mwscraper.translate import translate_matlab_snippet


# ----------------------------------------------------------------- translate


class TestTranslator:
    def test_comment_only_snippet(self):
        code, note = translate_matlab_snippet("% Collision check time stamps")
        assert code == "# Collision check time stamps"
        assert "Auto-translated" in note

    def test_scalar_assignments(self):
        source = "tHorizon = 5; % seconds\ndeltaT = 0.5; % seconds"
        code, _ = translate_matlab_snippet(source)
        assert "tHorizon = 5  # seconds" in code
        assert "deltaT = 0.5  # seconds" in code

    def test_range_uses_inclusive_endpoint_correction(self):
        code, _ = translate_matlab_snippet("tSteps = deltaT:deltaT:tHorizon;")
        assert "np.arange(deltaT, tHorizon + deltaT / 2, deltaT)" in code
        assert code.startswith("import numpy as np")

    def test_translated_range_is_executable_and_matches_matlab(self):
        """0.5:0.5:5 in MATLAB yields 10 values ending at 5.0."""
        code, _ = translate_matlab_snippet("tSteps = deltaT:deltaT:tHorizon;")
        namespace = {"deltaT": 0.5, "tHorizon": 5}
        exec(code, namespace)  # noqa: S102 - exercising generated code on purpose
        steps = namespace["tSteps"]
        assert len(steps) == 10
        assert steps[0] == pytest.approx(0.5)
        assert steps[-1] == pytest.approx(5.0)

    def test_refuses_toolbox_calls(self):
        code, note = translate_matlab_snippet(
            "tracker = trackerJPDA('AssignmentThreshold',[200 inf]);"
        )
        assert code is None
        assert "no safe mechanical Python equivalent" in note

    def test_refuses_control_flow(self):
        code, note = translate_matlab_snippet("while advance(scenario)\n  x = 1;\nend")
        assert code is None
        assert "trackerJPDA" in note  # names the toolbox caveat

    def test_empty_snippet(self):
        code, note = translate_matlab_snippet("")
        assert code is None
        assert "Empty" in note

    def test_blank_lines_alone_are_not_translated(self):
        code, note = translate_matlab_snippet("\n\n   \n")
        assert code is None

    def test_translation_never_silently_drops_lines(self):
        """If it translates at all, output must cover every input line."""
        source = "% one\n% two\nx = 3;"
        code, _ = translate_matlab_snippet(source)
        assert len(code.splitlines()) == 3


# ----------------------------------------------------------------- exporters


_SAMPLE_CODE = "tHorizon = 5; % seconds"
_SAMPLE_PY, _SAMPLE_NOTE = translate_matlab_snippet(_SAMPLE_CODE)


@pytest.fixture
def small_doc() -> Document:
    section = Section(
        heading=Heading(text="Motion Planner", level=4, anchor="mp"),
        path=("Example", "Setup", "Motion Planner"),
    )
    section.blocks = [
        TextBlock(
            text="The planner uses a 5 second horizon.",
            source_section="Motion Planner",
            section_path=section.path,
            order=0,
        ),
        CodeBlock(
            code=_SAMPLE_CODE,
            source_section="Motion Planner",
            section_path=section.path,
            order=1,
            referenced_functions=["dynamicCapsuleList"],
            python_equivalent=_SAMPLE_PY,
            translation_note=_SAMPLE_NOTE,
        ),
    ]
    return Document(
        url=PAGE_URL,
        title="Example Page",
        release="R2021b",
        products=["Navigation Toolbox"],
        sections=[section],
    )


class TestMarkdown:
    def test_includes_title_and_source(self, small_doc):
        md = document_to_markdown(small_doc)
        assert "# Example Page" in md
        assert PAGE_URL in md

    def test_code_is_fenced_as_matlab_with_attribution(self, small_doc):
        """Attribution uses the *target language's* comment character."""
        md = document_to_markdown(small_doc)
        assert "```matlab" in md
        assert "% From: Motion Planner" in md  # % is MATLAB's comment char

    def test_python_equivalent_uses_hash_attribution(self, small_doc):
        md = document_to_markdown(small_doc)
        # `tHorizon = 5; % seconds` is in the safely translatable subset.
        assert "```python" in md
        assert "# From: Motion Planner" in md

    def test_section_path_rendered(self, small_doc):
        md = document_to_markdown(small_doc)
        assert "Example > Setup > Motion Planner" in md

    def test_real_page_markdown_has_every_section(self, document):
        md = document_to_markdown(document)
        for section in document.sections:
            assert section.title in md

    def test_real_page_markdown_has_every_code_block(self, document):
        md = document_to_markdown(document)
        for block in document.all_code():
            assert block.code.splitlines()[0] in md

    def test_resource_table_present(self, document):
        md = document_to_markdown(document)
        assert "## Resources" in md
        assert "trackerJPDA" in md

    def test_markdown_is_not_empty_for_empty_doc(self):
        md = document_to_markdown(Document(url="u", title="Empty"))
        assert "# Empty" in md
        assert "*No resources extracted.*" in md


class TestCodeFiles:
    def test_writes_one_file_per_block(self, document, tmp_path):
        paths = write_code_files(document, tmp_path)
        assert len(paths) == len(document.all_code())
        assert all(p.suffix == ".m" for p in paths)

    def test_each_file_has_provenance_header(self, document, tmp_path):
        for path in write_code_files(document, tmp_path):
            text = path.read_text(encoding="utf-8")
            assert text.startswith("% From: ")
            assert "% Source: https://" in text

    def test_filenames_are_unique(self, document, tmp_path):
        paths = write_code_files(document, tmp_path)
        assert len({p.name for p in paths}) == len(paths)

    def test_creates_missing_directory(self, small_doc, tmp_path):
        target = tmp_path / "nested" / "code"
        paths = write_code_files(small_doc, target)
        assert paths and target.is_dir()


class TestJson:
    def test_round_trips(self, document, tmp_path):
        path = document.to_json_file(tmp_path / "out.json")
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data["metadata"]["title"] == document.title
        assert len(data["sections"]) == len(document.sections)

    def test_summary_counts_match_model(self, document):
        data = document.to_dict()
        assert data["summary"]["code_blocks"] == len(document.all_code())

    def test_json_is_utf8_safe(self, document, tmp_path):
        """Equations contain non-ASCII (Δ, ˙); serialisation must not mangle."""
        path = document.to_json_file(tmp_path / "out.json")
        text = path.read_text(encoding="utf-8")
        assert "\\u0394" not in text  # ensure_ascii=False keeps real characters


class TestSectionsDict:
    def test_flat_mapping_keyed_by_section(self, document):
        mapping = document_to_sections_dict(document)
        assert "Motion Planner" in mapping
        assert mapping["Motion Planner"]["code"]

    def test_empty_sections_excluded(self, document):
        mapping = document_to_sections_dict(document)
        assert "Setup" not in mapping  # heading node with no blocks


# ------------------------------------------------------------------ CLI/glue


class TestCli:
    def test_summary_only_mode(self, capsys, tmp_path):
        from mwscraper.cli import main

        code = main(
            [
                "--from-file",
                "tests/fixtures/frenet_page.html",
                "--summary-only",
                "--log-level",
                "ERROR",
            ]
        )
        assert code == 0
        payload = json.loads(capsys.readouterr().out)
        assert payload["code_blocks"] == 10

    def test_full_export_writes_all_artefacts(self, tmp_path):
        from mwscraper.cli import main

        out = tmp_path / "out"
        code = main(
            [
                "--from-file",
                "tests/fixtures/frenet_page.html",
                "--out-dir",
                str(out),
                "--log-level",
                "ERROR",
            ]
        )
        assert code == 0
        assert (out / "extracted_content.json").is_file()
        assert (out / "extracted_content.md").is_file()
        assert len(list((out / "code").glob("*.m"))) == 10

    def test_missing_file_returns_error_code(self, tmp_path):
        from mwscraper.cli import main

        assert main(["--from-file", str(tmp_path / "nope.html"), "--log-level", "ERROR"]) == 2


class TestScraperFacade:
    def test_scrape_from_file(self):
        from mwscraper.scraper import MathWorksScraper

        doc = MathWorksScraper().scrape_from_file(
            "tests/fixtures/frenet_page.html", PAGE_URL
        )
        assert len(doc.all_code()) == 10

    def test_scrape_many_continues_past_failures(self, monkeypatch):
        from mwscraper.fetcher import PermanentFetchError
        from mwscraper.scraper import MathWorksScraper

        scraper = MathWorksScraper()
        calls = {"n": 0}

        def flaky(url):
            calls["n"] += 1
            if calls["n"] == 1:
                raise PermanentFetchError("404")
            return "<html><body><section id='doc_center_content'>"\
                   "<h3>S</h3><p>ok</p></section></body></html>"

        monkeypatch.setattr(scraper.fetcher, "fetch", flaky)
        docs = scraper.scrape_many(["a", "b"])
        assert len(docs) == 1

    def test_scrape_many_can_fail_fast(self, monkeypatch):
        from mwscraper.fetcher import PermanentFetchError
        from mwscraper.scraper import MathWorksScraper

        scraper = MathWorksScraper()
        monkeypatch.setattr(
            scraper.fetcher,
            "fetch",
            lambda url: (_ for _ in ()).throw(PermanentFetchError("404")),
        )
        with pytest.raises(PermanentFetchError):
            scraper.scrape_many(["a"], continue_on_error=False)
