#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Testes Unitários da Bench Suite Portada (tests/test_bench_suite_port.py)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import time
import pytest
from core.control_math import HexagonalKinematics3DoF
from core.hardware import MotorController
from web.bench_runner import BenchRunner


@pytest.fixture
def bench_runner_env(isolate_config_files, mock_pigpio):
    steps_per_rev = 1600
    kinematics = HexagonalKinematics3DoF(steps_per_rev=steps_per_rev, front_motor_id=3)
    motor_ctrl = MotorController(steps_per_rev=steps_per_rev, apply_offsets_on_init=False)
    runner = BenchRunner(motor_ctrl, kinematics)
    return {
        "runner": runner,
        "motor_ctrl": motor_ctrl,
        "kinematics": kinematics
    }


def test_bench_runner_cardinal_execution(bench_runner_env):
    runner = bench_runner_env["runner"]
    assert runner.is_running() is False

    started = runner.start_test("cardinal", {"max_deg": 5.0})
    assert started is True
    assert runner.is_running() is True

    # Aguarda término ou cancelamento rápido
    timeout = time.monotonic() + 5.0
    while runner.is_running() and time.monotonic() < timeout:
        status = runner.get_status()
        assert status["test"] == "cardinal"
        time.sleep(0.05)

    runner.stop()
    status = runner.get_status()
    assert runner.is_running() is False


def test_bench_runner_trajectories_execution(bench_runner_env):
    runner = bench_runner_env["runner"]

    for traj in ["circle", "triangle", "square", "figure8"]:
        started = runner.start_test("trajectory", {"trajectory": traj, "radius_deg": 3.0})
        assert started is True
        time.sleep(0.1)
        st = runner.get_status()
        assert st["test"] == "trajectory"
        assert st["running"] is True
        # Cancela para o próximo
        runner.stop()
        time.sleep(0.05)
        assert runner.is_running() is False


def test_bench_runner_diagnostics_execution(bench_runner_env):
    runner = bench_runner_env["runner"]
    started = runner.start_test("diagnostics", {"motor": 1})
    assert started is True
    time.sleep(0.1)
    runner.stop()
    assert runner.is_running() is False


def test_bench_runner_repeatability_execution(bench_runner_env):
    runner = bench_runner_env["runner"]
    started = runner.start_test("repeatability")
    assert started is True
    time.sleep(0.1)
    runner.stop()
    assert runner.is_running() is False


def test_bench_runner_rejects_concurrent_execution(bench_runner_env):
    runner = bench_runner_env["runner"]
    assert runner.start_test("cardinal") is True
    # Segunda tentativa imediata deve ser rejeitada
    assert runner.start_test("trajectory") is False
    runner.stop()
