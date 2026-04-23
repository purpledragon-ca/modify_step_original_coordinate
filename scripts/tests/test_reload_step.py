import pytest

import find_bottom_center as fbc


def test_reload_step_sets_all_handler_attributes(sample_step_path, mock_occ):
    fbc.reload_step(sample_step_path)

    h = fbc._Handler
    assert h._shape is not None
    assert h._step_path == sample_step_path
    assert h._unit == "mm"
    assert h._decimals == 8
    assert h._surfacecurve_mode == 0
    assert h._write_props is False
    assert h._model_data is not None
    assert h._busy is False
    assert h._model_data["filename"].endswith(".step")
    assert "faces" in h._model_data
    assert "meshes" in h._model_data
    assert "proposals" in h._model_data
    assert "features" in h._model_data
    assert "bbox" in h._model_data


def test_reload_step_replaces_prior_state(sample_step_path, mock_occ):
    fbc._Handler._step_path = "/old/path.step"
    fbc._Handler._model_data = {"filename": "old.step", "stale": True}

    fbc.reload_step(sample_step_path)

    assert fbc._Handler._step_path == sample_step_path
    assert "stale" not in fbc._Handler._model_data


def test_reload_step_sets_busy_only_during_call(sample_step_path, mock_occ):
    assert fbc._Handler._busy is False
    fbc.reload_step(sample_step_path)
    # After return, busy is cleared.
    assert fbc._Handler._busy is False


def test_reload_step_missing_file_raises(mock_occ):
    with pytest.raises(FileNotFoundError):
        fbc.reload_step("/does/not/exist.step")
