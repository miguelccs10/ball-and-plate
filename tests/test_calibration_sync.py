#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Testes de Sincronização de Calibração e Telemetria (tests/test_calibration_sync.py)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import pytest
from core.hardware import MotorController, compute_nominal_neutral_deg, save_calibration, load_calibration


def test_nominal_neutral_deg_calculation():
    # Para 1600 passos/rev (8 micropassos)
    neutral_1600 = compute_nominal_neutral_deg(1600)
    assert 120.0 <= neutral_1600 <= 135.0

    # Para 3200 passos/rev (16 micropassos)
    neutral_3200 = compute_nominal_neutral_deg(3200)
    assert abs(neutral_1600 - neutral_3200) < 0.1


def test_motor_controller_save_current_as_neutral_sync(isolate_config_files, mock_pigpio):
    calib_file = isolate_config_files["calib_file"]
    motor_ctrl = MotorController(steps_per_rev=1600, apply_offsets_on_init=False)
    motor_ctrl.current_steps = [30, -20, 10]

    ok = motor_ctrl.save_current_as_neutral(filepath=calib_file)
    assert ok is True
    assert motor_ctrl.calib_offsets == [30, -20, 10]
    assert motor_ctrl.current_steps == [0, 0, 0]
    assert motor_ctrl.is_calibrated is True
    assert motor_ctrl.calib_timestamp is not None

    # Telemetry snapshot
    snap = motor_ctrl.get_telemetry_snapshot()
    assert snap["calib_offsets"] == [30, -20, 10]
    assert snap["current_steps"] == [0, 0, 0]
    assert snap["is_calibrated"] is True
    assert len(snap["calibrated_neutral_angles_deg"]) == 3
    assert len(snap["motor_angles_deg"]) == 3


def test_motor_controller_reset_calibration_sync(isolate_config_files, mock_pigpio):
    calib_file = isolate_config_files["calib_file"]
    motor_ctrl = MotorController(steps_per_rev=1600, apply_offsets_on_init=False)
    motor_ctrl.calib_offsets = [50, 50, 50]
    motor_ctrl.is_calibrated = True

    ok = motor_ctrl.reset_calibration(filepath=calib_file)
    assert ok is True
    assert motor_ctrl.calib_offsets == [0, 0, 0]
    assert motor_ctrl.is_calibrated is False
    assert motor_ctrl.calib_timestamp is None

    snap = motor_ctrl.get_telemetry_snapshot()
    assert snap["calib_offsets"] == [0, 0, 0]
    assert snap["is_calibrated"] is False
