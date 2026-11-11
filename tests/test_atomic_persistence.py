#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Testes de Persistência Atômica e Tolerância a Falhas (tests/test_atomic_persistence.py)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import json
import tempfile
from unittest.mock import patch
import pytest

from core.persistence import (
    atomic_write_json,
    load_json_safe,
    save_vision_profile,
    load_vision_profile
)
from core.hardware import save_calibration, load_calibration


def test_atomic_write_creates_valid_file(tmp_path):
    target = str(tmp_path / "test_data.json")
    data = {"system": "ball_and_plate", "microsteps": 8, "offsets": [12, -4, 0]}

    ok = atomic_write_json(target, data)
    assert ok is True
    assert os.path.isfile(target)

    # Lê e compara conteúdo
    with open(target, "r", encoding="utf-8") as f:
        loaded = json.load(f)
    assert loaded == data


def test_atomic_write_creates_nested_directories(tmp_path):
    target = str(tmp_path / "subdir" / "nested" / "deep_data.json")
    data = {"status": "ok", "nested": True}

    ok = atomic_write_json(target, data)
    assert ok is True
    assert os.path.isfile(target)


def test_atomic_write_cleans_up_temp_on_failure(tmp_path):
    target = str(tmp_path / "fail_data.json")
    data = {"test": "fail"}

    # Força exceção durante json.dump
    with patch("json.dump", side_effect=IOError("Simulated Disk Error")):
        ok = atomic_write_json(target, data)
        assert ok is False

    assert not os.path.isfile(target)
    # Garante que nenhum arquivo temporário órfão permaneceu
    temp_files = [f for f in os.listdir(tmp_path) if f.startswith("tmp")]
    assert len(temp_files) == 0


def test_atomic_write_protects_existing_file_on_write_error(tmp_path):
    target = str(tmp_path / "protected.json")
    original_data = {"version": 1, "valid": True}

    # Gravação inicial bem sucedida
    assert atomic_write_json(target, original_data) is True

    # Tentativa subsequente com falha simulada antes do replace
    with patch("os.fsync", side_effect=OSError("Simulated Sync Crash")):
        ok = atomic_write_json(target, {"version": 2, "valid": False})
        assert ok is False

    # O arquivo original NÃO pode ter sido corrompido nem truncado
    with open(target, "r", encoding="utf-8") as f:
        survived = json.load(f)
    assert survived == original_data


def test_load_json_safe_fallbacks(tmp_path):
    # Arquivo não existente
    non_existent = str(tmp_path / "ghost.json")
    default_val = {"fallback": True}
    assert load_json_safe(non_existent, default_val) == default_val

    # Arquivo corrompido / truncated
    corrupt_file = str(tmp_path / "corrupt.json")
    with open(corrupt_file, "w", encoding="utf-8") as f:
        f.write("{ invalid json truncate: [1, 2,")

    res = load_json_safe(corrupt_file, {"fallback": "corrupt"})
    assert res == {"fallback": "corrupt"}


def test_hardware_save_and_load_calibration_atomic(tmp_path):
    calib_file = str(tmp_path / "calib_offsets.json")
    offsets = [15, -20, 8]

    saved = save_calibration(
        step_offsets=offsets,
        steps_per_rev=1600,
        nominal_neutral_deg=126.73,
        is_calibrated=True,
        filepath=calib_file
    )
    assert saved is True
    assert os.path.isfile(calib_file)

    loaded = load_calibration(calib_file)
    assert loaded["is_calibrated"] is True
    assert loaded["step_offsets"] == offsets
    assert loaded["steps_per_rev"] == 1600
    assert loaded["nominal_neutral_deg"] == 126.73
    assert len(loaded["calibrated_neutral_angles_deg"]) == 3
    assert loaded["timestamp"] is not None


def test_vision_profile_save_and_load_atomic(tmp_path):
    vis_file = str(tmp_path / "vision_profile.json")
    profile = {
        "cam_height_mm": 450.0,
        "cam_rotation": 90,
        "invert_x": True,
        "invert_y": False,
        "crop": {"x": 10, "y": 20, "w": 300, "h": 200},
        "hsv_lower": [10, 100, 100],
        "hsv_upper": [25, 255, 255]
    }

    assert save_vision_profile(vis_file, profile) is True
    assert os.path.isfile(vis_file)

    loaded = load_vision_profile(vis_file)
    assert loaded["cam_rotation"] == 90
    assert loaded["invert_x"] is True
    assert loaded["crop"]["w"] == 300
    assert "timestamp" in loaded
