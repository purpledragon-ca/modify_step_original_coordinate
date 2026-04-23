"""Test fixtures. OCC call sites are monkeypatched in `mock_occ` so tests don't
exercise real OpenCASCADE parsing — but importing `find_bottom_center` itself
still requires OCP to be importable (its top-level imports are not guarded)."""

from pathlib import Path
import sys
import types
import pytest

SCRIPT_DIR = Path(__file__).resolve().parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

RESOURCES = Path(__file__).resolve().parents[3] / "2_cluster_names" / "resources"
SAMPLE_STEP = RESOURCES / "cc_waste_bin.step"


@pytest.fixture
def sample_step_path() -> str:
    assert SAMPLE_STEP.exists(), f"test fixture missing: {SAMPLE_STEP}"
    return str(SAMPLE_STEP)


@pytest.fixture
def mock_occ(monkeypatch):
    """Replace all OCC-dependent functions in find_bottom_center with fakes.

    Returns a dict recording call counts so tests can assert behavior without
    needing OpenCASCADE actually installed.
    """
    import find_bottom_center as fbc

    calls = {"load_step": 0, "analyze_faces": 0,
             "triangulate_faces": 0, "find_bottom_center_rules": 0,
             "find_joint_features": 0, "detect_step_unit": 0}

    def fake_load_step(path):
        calls["load_step"] += 1
        return types.SimpleNamespace(_tag="fake-shape", _path=path)

    def fake_detect_unit(path):
        calls["detect_step_unit"] += 1
        return "mm"

    def fake_analyze_faces(shape):
        calls["analyze_faces"] += 1
        return []

    def fake_triangulate(shape):
        calls["triangulate_faces"] += 1
        return []

    def fake_bottom(faces):
        calls["find_bottom_center_rules"] += 1
        return []

    def fake_joints(faces):
        calls["find_joint_features"] += 1
        return []

    class _FakeBB:
        def Get(self): return (0.0, 0.0, 0.0, 1.0, 1.0, 1.0)

    class _FakeBndBox:
        def __init__(self): self._bb = _FakeBB()
        def Get(self): return self._bb.Get()

    class _FakeLib:
        @staticmethod
        def Add_s(shape, bb): pass

    monkeypatch.setattr(fbc, "load_step", fake_load_step)
    monkeypatch.setattr(fbc, "detect_step_unit", fake_detect_unit)
    monkeypatch.setattr(fbc, "analyze_faces", fake_analyze_faces)
    monkeypatch.setattr(fbc, "triangulate_faces", fake_triangulate)
    monkeypatch.setattr(fbc, "find_bottom_center_rules", fake_bottom)
    monkeypatch.setattr(fbc, "find_joint_features", fake_joints)
    monkeypatch.setattr(fbc, "Bnd_Box", _FakeBndBox)
    monkeypatch.setattr(fbc, "BRepBndLib", _FakeLib)
    monkeypatch.setattr(fbc, "print_results", lambda *a, **k: None)

    # Reset _Handler class attrs between tests.
    fbc._Handler._shape = None
    fbc._Handler._step_path = None
    fbc._Handler._unit = "mm"
    fbc._Handler._decimals = 8
    fbc._Handler._surfacecurve_mode = 0
    fbc._Handler._write_props = False
    fbc._Handler._model_data = None
    fbc._Handler._busy = False

    return calls
