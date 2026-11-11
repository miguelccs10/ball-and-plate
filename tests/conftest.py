#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Fixtures Compartilhadas para a Suíte de Testes (tests/conftest.py)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import sys
import socket
import tempfile
import threading
import time
from unittest.mock import MagicMock
import pytest

# Garante raiz no sys.path
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

import config
from main import AppState
from core.control_math import HexagonalKinematics3DoF, DiscretePID, DiscreteKalman2D
from core.hardware import MotorController, SerialBridge
from core.vision import AprilTagCalibrator
from web.bench_runner import BenchRunner
from web.server import DashboardServerContext, run_dashboard_server


@pytest.fixture(autouse=True)
def isolate_config_files(tmp_path, monkeypatch):
    """
    Isola os caminhos dos arquivos JSON de persistência para um diretório temporário,
    evitando que testes modifiquem os arquivos reais de calibração do repositório.
    """
    calib_file = str(tmp_path / "calibration_offsets.json")
    vision_file = str(tmp_path / "vision_profile.json")
    ctrl_file = str(tmp_path / "control_profile.json")

    monkeypatch.setattr(config, "CALIBRATION_FILE", calib_file)
    monkeypatch.setattr(config, "VISION_PROFILE_FILE", vision_file)
    monkeypatch.setattr(config, "CONTROL_PROFILE_FILE", ctrl_file)

    yield {
        "calib_file": calib_file,
        "vision_file": vision_file,
        "ctrl_file": ctrl_file,
        "tmp_dir": str(tmp_path)
    }


@pytest.fixture
def mock_pigpio(monkeypatch):
    """
    Mock robusto do daemon pigpio para simulação de acionamento de hardware em ambiente x86.
    """
    mock_pi_instance = MagicMock()
    mock_pi_instance.connected = True
    mock_pi_instance.set_mode.return_value = None
    mock_pi_instance.write.return_value = None
    mock_pi_instance.set_bank_1.return_value = None
    mock_pi_instance.clear_bank_1.return_value = None

    mock_pigpio_module = MagicMock()
    mock_pigpio_module.pi.return_value = mock_pi_instance
    mock_pigpio_module.OUTPUT = 1

    monkeypatch.setattr("core.hardware.pigpio", mock_pigpio_module)
    return mock_pi_instance


@pytest.fixture
def mock_camera():
    """
    Mock do stream de captura da câmera OpenCV.
    """
    cam = MagicMock()
    cam.is_running = True
    cam.start.return_value = None
    cam.stop.return_value = None
    cam.wait_for_frame.return_value = (None, time.time(), 0)
    return cam


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def live_server(isolate_config_files, mock_pigpio):
    """
    Fixture de servidor web ativo em porta efêmera para testes de integração HTTP e browser.
    """
    port = find_free_port()
    steps_per_rev = 1600
    kinematics = HexagonalKinematics3DoF(steps_per_rev=steps_per_rev, front_motor_id=3)
    motor_ctrl = MotorController(steps_per_rev=steps_per_rev, apply_offsets_on_init=False)
    state = AppState(initial_neutral_deg=kinematics.neutral_angle_deg)
    state.op_mode = "STANDALONE_RPI"

    pid_x = DiscretePID(1.35, 0.05, 0.85, n_filter=20.0, ts=0.033)
    pid_y = DiscretePID(1.35, 0.05, 0.85, n_filter=20.0, ts=0.033)
    kalman = DiscreteKalman2D(ts=0.033)
    calibrator = AprilTagCalibrator()
    bench_runner = BenchRunner(motor_ctrl, kinematics)

    ctx = DashboardServerContext(
        state=state,
        motor_ctrl=motor_ctrl,
        kinematics=kinematics,
        pid_x=pid_x,
        pid_y=pid_y,
        kalman=kalman,
        serial_bridge=None,
        on_detector_change=None,
        calibrator=calibrator,
        bench_runner=bench_runner
    )

    server = run_dashboard_server("127.0.0.1", port, ctx)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    # Aguarda o socket responder
    base_url = f"http://127.0.0.1:{port}"
    timeout = time.monotonic() + 3.0
    while time.monotonic() < timeout:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                break
        except OSError:
            time.sleep(0.05)

    yield {
        "url": base_url,
        "port": port,
        "context": ctx,
        "state": state,
        "motor_ctrl": motor_ctrl,
        "bench_runner": bench_runner
    }

    try:
        server.shutdown()
    except Exception:
        pass
    motor_ctrl.stop()
