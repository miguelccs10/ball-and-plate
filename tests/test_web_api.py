#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Testes da API REST e Endpoints de Telemetria e Bancada (tests/test_web_api.py)
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import time
import urllib.request
import urllib.error
import json
import pytest


def make_request(url: str, method: str = "GET", data: dict = None) -> tuple:
    req = urllib.request.Request(url, method=method)
    if data is not None:
        body = json.dumps(data).encode("utf-8")
        req.add_header("Content-Type", "application/json")
        req.data = body

    try:
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            code = resp.status
            content = resp.read().decode("utf-8")
            return code, json.loads(content) if content else {}
    except urllib.error.HTTPError as e:
        return e.code, {}


def test_telemetry_endpoint_contains_all_r2_calibration_fields(live_server):
    url = f"{live_server['url']}/api/telemetry"
    status_code, data = make_request(url)

    assert status_code == 200
    # Verificação dos 6 campos obrigatórios de calibração (R2)
    assert "calib_offsets" in data
    assert isinstance(data["calib_offsets"], list)
    assert len(data["calib_offsets"]) == 3

    assert "nominal_neutral_deg" in data
    assert isinstance(data["nominal_neutral_deg"], (int, float))

    assert "calibrated_neutral_angles_deg" in data
    assert isinstance(data["calibrated_neutral_angles_deg"], list)
    assert len(data["calibrated_neutral_angles_deg"]) == 3

    assert "motor_angles_deg" in data
    assert isinstance(data["motor_angles_deg"], list)
    assert len(data["motor_angles_deg"]) == 3

    assert "is_calibrated" in data
    assert isinstance(data["is_calibrated"], bool)

    assert "timestamp" in data

    # Verificação de bench_status e gpio_pins (R3)
    assert "bench_status" in data
    assert isinstance(data["bench_status"], dict)
    assert "running" in data["bench_status"]
    assert "progress" in data["bench_status"]
    assert "message" in data["bench_status"]

    assert "gpio_pins" in data
    assert "1" in data["gpio_pins"]
    assert "2" in data["gpio_pins"]
    assert "3" in data["gpio_pins"]


def test_action_jog_individual_and_all_motors(live_server):
    url = f"{live_server['url']}/api/action"
    motor_ctrl = live_server["motor_ctrl"]

    # 1. Jog individual em passos
    code, res = make_request(url, method="POST", data={"action": "jog", "motor": 1, "steps": 10})
    assert code == 200
    assert res.get("status") == "ok"
    assert motor_ctrl.target_steps[0] == 10

    # 2. Jog em graus
    code, res = make_request(url, method="POST", data={"action": "jog", "motor": 2, "deg": 2.25})
    assert code == 200
    assert res.get("status") == "ok"
    # 2.25 deg @ 1600 spr = 10 steps
    assert motor_ctrl.target_steps[1] == 10

    # 3. Jog simultâneo em todos os motores (motor: 0)
    code, res = make_request(url, method="POST", data={"action": "jog", "motor": 0, "steps": -5})
    assert code == 200
    assert res.get("status") == "ok"
    assert motor_ctrl.target_steps[0] == 5
    assert motor_ctrl.target_steps[1] == 5
    assert motor_ctrl.target_steps[2] == -5


def test_action_save_neutral_and_reset_calibration(live_server):
    url_action = f"{live_server['url']}/api/action"
    url_telem = f"{live_server['url']}/api/telemetry"
    motor_ctrl = live_server["motor_ctrl"]

    # Posiciona os motores com passos
    motor_ctrl.current_steps = [20, -15, 8]

    # Salva posição atual como neutro permanente
    code, res = make_request(url_action, method="POST", data={"action": "save_neutral"})
    assert code == 200
    assert res.get("status") == "ok"
    assert motor_ctrl.calib_offsets == [20, -15, 8]
    assert motor_ctrl.current_steps == [0, 0, 0]

    # Verifica telemetria sincronizada
    _, telem = make_request(url_telem)
    assert telem["calib_offsets"] == [20, -15, 8]
    assert telem["is_calibrated"] is True
    assert telem["timestamp"] is not None

    # Reseta calibração de fábrica
    code, res = make_request(url_action, method="POST", data={"action": "reset_calibration"})
    assert code == 200
    assert res.get("status") == "ok"
    assert motor_ctrl.calib_offsets == [0, 0, 0]

    _, telem_reset = make_request(url_telem)
    assert telem_reset["calib_offsets"] == [0, 0, 0]
    assert telem_reset["is_calibrated"] is False


def test_action_async_bench_suite_non_blocking(live_server):
    url_action = f"{live_server['url']}/api/action"
    url_telem = f"{live_server['url']}/api/telemetry"

    # Inicia teste de bancada assíncrono
    t_start = time.monotonic()
    code, res = make_request(url_action, method="POST", data={"action": "bench_start", "test": "cardinal"})
    t_duration = time.monotonic() - t_start

    assert code == 200
    # O endpoint DEVE responder imediatamente em menos de 200 ms sem travar na execução do teste
    assert t_duration < 0.200

    # Consulta telemetria imediatamente: o bench_runner deve indicar em execução
    time.sleep(0.05)
    _, telem = make_request(url_telem)
    assert telem["bench_status"]["test"] == "cardinal"

    # Cancela o teste de bancada
    code, res = make_request(url_action, method="POST", data={"action": "bench_stop"})
    assert code == 200


def test_vision_and_control_profile_persistence_endpoints(live_server):
    url_action = f"{live_server['url']}/api/action"
    url_config = f"{live_server['url']}/api/config"
    state = live_server["state"]

    # 1. Modifica parâmetros visuais
    make_request(url_config, method="POST", data={"cam_rotation": 180, "hsv_lower": [12, 110, 95]})
    assert state.cam_rotation == 180
    assert state.hsv_lower == [12, 110, 95]

    # Salva perfil de visão
    code, res = make_request(url_action, method="POST", data={"action": "save_vision_profile"})
    assert code == 200

    # Altera em memória e recarrega
    with state.lock:
        state.cam_rotation = 0
    code, res = make_request(url_action, method="POST", data={"action": "load_vision_profile"})
    assert code == 200
    assert state.cam_rotation == 180

    # 2. Modifica parâmetros PID e salva perfil de controle
    make_request(url_config, method="POST", data={"kp": 2.15, "kd": 1.25})
    assert state.kp == 2.15
    code, res = make_request(url_action, method="POST", data={"action": "save_control_profile"})
    assert code == 200

    with state.lock:
        state.kp = 1.0
    code, res = make_request(url_action, method="POST", data={"action": "load_control_profile"})
    assert code == 200
    assert state.kp == 2.15


def test_static_confinement_prevents_path_traversal(live_server):
    url_traversal = f"{live_server['url']}/static/../../../../etc/passwd"
    req = urllib.request.Request(url_traversal)
    try:
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            pytest.fail("Path traversal não foi bloqueado e retornou HTTP 200")
    except urllib.error.HTTPError as e:
        # Confinamento seguro: 403 Forbidden ou 404 Not Found
        assert e.code in (403, 404)
