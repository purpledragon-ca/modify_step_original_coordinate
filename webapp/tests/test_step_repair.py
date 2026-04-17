"""Tests for step_repair.py — the two Shapr3D STEP export defect fixes."""

import pytest
from step_repair import fix_direction_literals, repair, RepairReport


# ---------------------------------------------------------------------------
# Fix 1: REAL literal normalisation
# ---------------------------------------------------------------------------

class TestFixDirectionLiterals:
    def test_negative_zero_becomes_zero_real(self):
        line = "#13=DIRECTION('',(-0,0,-1));\n"
        fixed = fix_direction_literals(line)
        assert "-0" not in fixed
        assert "0.,0.,-1." in fixed

    def test_bare_integers_get_decimal(self):
        line = "#127=DIRECTION('',(0,0,1));\n"
        fixed = fix_direction_literals(line)
        assert "(0.,0.,1.)" in fixed

    def test_already_valid_reals_unchanged(self):
        line = "#46=DIRECTION('',(0.,0.,1.));\n"
        assert fix_direction_literals(line) == line

    def test_exponent_notation_unchanged(self):
        line = "#47=DIRECTION('',(1,-1.22464679915E-16,0));\n"
        fixed = fix_direction_literals(line)
        # 1 → 1.,  0 → 0.,  exponent part left alone
        assert "1.,-1.22464679915E-16,0." in fixed

    def test_non_direction_line_untouched(self):
        line = "#12=CARTESIAN_POINT('',(-69.849007031,1889.5,1795.4));\n"
        assert fix_direction_literals(line) == line

    def test_negative_one_gets_decimal(self):
        line = "#x=DIRECTION('',(-1,0,0));\n"
        fixed = fix_direction_literals(line)
        assert "(-1.,0.,0.)" in fixed


# ---------------------------------------------------------------------------
# Fix 2: Z-axis flip
# ---------------------------------------------------------------------------

def _make_step(extra_entities: str = "") -> str:
    """Minimal STEP file with one ITEM_DEFINED_TRANSFORMATION."""
    return (
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
        # Product CSYS (the 'to' reference)
        "#12=CARTESIAN_POINT('',(0.,0.,0.));\n"
        "#13=DIRECTION('',(-0,0,-1));\n"   # Z-down — must be flipped
        "#14=DIRECTION('',(1.,0.,0.));\n"
        "#15=AXIS2_PLACEMENT_3D('TS3D_PRODUCT_CSYS',#12,#13,#14);\n"
        # Component placement ('from')
        "#45=CARTESIAN_POINT('',(10.,20.,30.));\n"
        "#46=DIRECTION('',(-0,0,-1));\n"   # Z-down — must be flipped
        "#47=DIRECTION('',(1.,0.,0.));\n"
        "#48=AXIS2_PLACEMENT_3D('',#45,#46,#47);\n"
        "#49=ITEM_DEFINED_TRANSFORMATION('','',#48,#15);\n"
        # A geometry normal that is also Z-down but NOT in an assembly transform
        "#99=DIRECTION('',(-0,0,-1));\n"   # geometry — must NOT be flipped
        + extra_entities
        + "ENDSEC;\nEND-ISO-10303-21;\n"
    )


class TestFlipAssemblyZ:
    def test_csys_direction_flipped(self):
        text, report = repair(_make_step(), fix_syntax=False, fix_z=True)
        # #13 is the Z-direction of #15 (TS3D_PRODUCT_CSYS, the 'to')
        assert "#13=DIRECTION('',(0.,0.,1.));" in text
        assert report.z_directions_flipped >= 1

    def test_from_placement_direction_flipped(self):
        text, _ = repair(_make_step(), fix_syntax=False, fix_z=True)
        # #46 is the Z-direction of #48 (the 'from' placement)
        assert "#46=DIRECTION('',(0.,0.,1.));" in text

    def test_geometry_normal_not_flipped(self):
        text, _ = repair(_make_step(), fix_syntax=False, fix_z=True)
        # #99 is a standalone geometry direction not referenced by any AXIS2 in IDT
        assert "#99=DIRECTION('',(-0,0,-1));" in text or "#99=DIRECTION('',(0.,0.,-1.));" in text

    def test_already_z_up_no_changes(self):
        step = (
            "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n"
            "#12=CARTESIAN_POINT('',(0.,0.,0.));\n"
            "#13=DIRECTION('',(0.,0.,1.));\n"
            "#14=DIRECTION('',(1.,0.,0.));\n"
            "#15=AXIS2_PLACEMENT_3D('TS3D_PRODUCT_CSYS',#12,#13,#14);\n"
            "#45=CARTESIAN_POINT('',(10.,0.,0.));\n"
            "#46=DIRECTION('',(0.,0.,1.));\n"
            "#47=DIRECTION('',(1.,0.,0.));\n"
            "#48=AXIS2_PLACEMENT_3D('',#45,#46,#47);\n"
            "#49=ITEM_DEFINED_TRANSFORMATION('','',#48,#15);\n"
            "ENDSEC;\nEND-ISO-10303-21;\n"
        )
        _, report = repair(step, fix_syntax=False, fix_z=True)
        assert report.z_directions_flipped == 0

    def test_line_count_preserved(self):
        original = _make_step()
        repaired, _ = repair(original, fix_syntax=True, fix_z=True)
        assert repaired.count("\n") == original.count("\n")


# ---------------------------------------------------------------------------
# Combined repair()
# ---------------------------------------------------------------------------

class TestRepair:
    def test_report_counts_both_fixes(self):
        text = _make_step()
        _, report = repair(text, fix_syntax=True, fix_z=True)
        assert report.syntax_lines_fixed > 0
        assert report.z_directions_flipped > 0

    def test_any_changes_true_when_fixed(self):
        _, report = repair(_make_step())
        assert report.any_changes

    def test_summary_non_empty(self):
        _, report = repair(_make_step())
        assert report.summary() != "No changes needed"

    def test_no_negative_zero_after_repair(self):
        text = _make_step()
        repaired, _ = repair(text)
        assert "(-0," not in repaired

    def test_fix_syntax_false_skips_literal_fix(self):
        text = "#x=DIRECTION('',(-0,0,-1));\n"
        repaired, report = repair(text, fix_syntax=False, fix_z=False)
        assert repaired == text
        assert report.syntax_lines_fixed == 0

    def test_real_file_no_negative_zero(self):
        """Smoke test: the actual shelf file should have zero -0 after repair."""
        import pathlib
        shelf = pathlib.Path("/home/purpledragon/Downloads/left_shelf_centered_centered.step")
        if not shelf.exists():
            pytest.skip("shelf STEP file not present")
        text = shelf.read_text(encoding="utf-8")
        repaired, report = repair(text)
        assert "(-0," not in repaired
        assert report.syntax_lines_fixed > 0
        assert report.z_directions_flipped > 0
