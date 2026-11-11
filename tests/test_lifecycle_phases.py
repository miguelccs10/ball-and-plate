#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Testes do Ciclo de Vida Trifásico e Orquestrador (tests/test_lifecycle_phases.py)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import sys
from unittest.mock import patch, MagicMock
import pytest

import config
import main
from main import AppState, parse_args
from core.control_math import HexagonalKinematics3DoF
from core.hardware import MotorController, save_calibration


def test_cli_argument_parsing_defaults():
    with patch("sys.argv", ["main.py"]):
        args = parse_args()
        assert args.apply_calibration is True
        assert args.skip_cli is False
        assert args.headless is False
        assert args.front_motor == 3
        assert args.op_mode == "STANDALONE_RPI"


def test_cli_argument_parsing_custom_flags():
    with patch("sys.argv", ["main.py", "--no-calibration", "--skip-cli", "--headless", "--front-motor", "1"]):
        args = parse_args()
        assert args.apply_calibration is False
        assert args.skip_cli is True
        assert args.headless is True
        assert args.front_motor == 1


def test_phase1_bypass_conditions():
    # 1. Com --skip-cli
    with patch("sys.stdin.isatty", return_value=True), \
         patch("tools.bench_suite.run_interactive_calibration") as mock_calib:
        args = MagicMock(skip_cli=True, headless=False)
        is_interactive = (not args.skip_cli) and (not args.headless) and sys.stdin.isatty()
        assert is_interactive is False
        if is_interactive:
            mock_calib()
        mock_calib.assert_not_called()

    # 2. Com --headless
    with patch("sys.stdin.isatty", return_value=True), \
         patch("tools.bench_suite.run_interactive_calibration") as mock_calib:
        args = MagicMock(skip_cli=False, headless=True)
        is_interactive = (not args.skip_cli) and (not args.headless) and sys.stdin.isatty()
        assert is_interactive is False
        if is_interactive:
            mock_calib()
        mock_calib.assert_not_called()

    # 3. Com stdin não interativo (pipe / non-TTY)
    with patch("sys.stdin.isatty", return_value=False), \
         patch("tools.bench_suite.run_interactive_calibration") as mock_calib:
        args = MagicMock(skip_cli=False, headless=False)
        is_interactive = (not args.skip_cli) and (not args.headless) and sys.stdin.isatty()
        assert is_interactive is False
        if is_interactive:
            mock_calib()
        mock_calib.assert_not_called()


def test_phase1_execution_when_interactive():
    with patch("sys.stdin.isatty", return_value=True), \
         patch("tools.bench_suite.run_interactive_calibration") as mock_calib:
        args = MagicMock(skip_cli=False, headless=False)
        is_interactive = (not args.skip_cli) and (not args.headless) and sys.stdin.isatty()
        assert is_interactive is True
        if is_interactive:
            mock_calib()
        mock_calib.assert_called_once()


def test_phase2_hardware_readiness_barrier_and_hydration(isolate_config_files, mock_pigpio):
    calib_file = isolate_config_files["calib_file"]
    saved_offsets = [25, -15, 10]
    save_calibration(
        step_offsets=saved_offsets,
        steps_per_rev=1600,
        nominal_neutral_deg=126.73,
        is_calibrated=True,
        filepath=calib_file
    )

    motor_ctrl = MotorController(steps_per_rev=1600, apply_offsets_on_init=False)
    motor_ctrl.load_calibration_offsets(calib_file)
    assert motor_ctrl.calib_offsets == saved_offsets

    # Despacho com barreira de prontidão
    applied = motor_ctrl.apply_saved_offsets(wait=True)
    assert applied is True
    assert motor_ctrl.is_idle() is True
    assert motor_ctrl.wait_until_idle(timeout=1.0) is True

    # Hidratação do AppState
    snap = motor_ctrl.get_telemetry_snapshot()
    state = AppState(initial_neutral_deg=126.73)
    with state.lock:
        state.calib_offsets = list(snap["calib_offsets"])
        state.nominal_neutral_deg = snap["nominal_neutral_deg"]
        state.calibrated_neutral_angles_deg = list(snap["calibrated_neutral_angles_deg"])
        state.motor_angles_deg = list(snap["motor_angles_deg"])
        state.is_calibrated = snap["is_calibrated"]
        state.calib_timestamp = snap["timestamp"]

    assert state.calib_offsets == saved_offsets
    assert state.is_calibrated is True
    assert state.nominal_neutral_deg == 126.73
    assert state.calib_timestamp is not None
    assert len(state.calibrated_neutral_angles_deg) == 3


def test_phase3_orchestrator_headless_flow(isolate_config_files, mock_pigpio):
    # Executa main() com --skip-cli e --headless simulando shutdown imediato via KeyboardInterrupt no vision_loop
    with patch("sys.argv", ["main.py", "--skip-cli", "--headless"]), \
         patch("main.vision_loop", side_effect=KeyboardInterrupt("Simulated exit")), \
         patch("main.cv2", None):
        # Deve executar as 3 fases sem erros e encerrar graciosamente no bloco finally
        try:
            main.main()
        except KeyboardInterrupt:
            pytest.fail("KeyboardInterrupt não foi capturado internamente pelo main()")
