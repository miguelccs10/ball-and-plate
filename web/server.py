#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Servidor Web Integrado e API REST (web/server.py)
Endpoints REST Consolidados (config / action / telemetry), Streaming MJPEG
e Confinamento Estrito de Recursos Estáticos
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import json
import math
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
from typing import Dict, Any

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(WEB_DIR, "static")
REAL_STATIC_DIR = os.path.realpath(STATIC_DIR)
INDEX_HTML_PATH = os.path.join(WEB_DIR, "index.html")


def _safe_float(val, default: float, min_val: float = None, max_val: float = None) -> float:
    try:
        if val is None or isinstance(val, bool):
            return default
        out = float(val)
        if not math.isfinite(out):
            return default
        if min_val is not None:
            out = max(min_val, out)
        if max_val is not None:
            out = min(max_val, out)
        return out
    except (ValueError, TypeError):
        return default


def _safe_int(val, default: int, min_val: int = None, max_val: int = None) -> int:
    try:
        if val is None or isinstance(val, bool):
            return default
        out = int(float(val))
        if min_val is not None:
            out = max(min_val, out)
        if max_val is not None:
            out = min(max_val, out)
        return out
    except (ValueError, TypeError):
        return default


def _safe_hsv_triplet(val, default_triplet):
    if not isinstance(val, (list, tuple)) or len(val) != 3:
        return default_triplet
    try:
        return [
            max(0, min(179, int(val[0]))),
            max(0, min(255, int(val[1]))),
            max(0, min(255, int(val[2])))
        ]
    except Exception:
        return default_triplet


class DashboardServerContext:
    def __init__(self, state, motor_ctrl, kinematics, pid_x, pid_y, kalman, serial_bridge=None, on_detector_change=None):
        self.state = state
        self.motor_ctrl = motor_ctrl
        self.kinematics = kinematics
        self.pid_x = pid_x
        self.pid_y = pid_y
        self.kalman = kalman
        self.serial_bridge = serial_bridge
        self.on_detector_change = on_detector_change


class ThreadedServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def create_handler(context: DashboardServerContext):
    class DashboardHandler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def _serve_file(self, file_path: str, content_type: str):
            if not os.path.isfile(file_path):
                self.send_error(404, "Arquivo nao encontrado")
                return

            try:
                with open(file_path, "rb") as f:
                    content = f.read()

                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except OSError:
                self.send_error(500, "Erro interno ao ler recurso")

        def do_GET(self):
            parsed = urlparse(self.path)
            state = context.state
            motor_ctrl = context.motor_ctrl
            ser_b = context.serial_bridge

            # 1. Página Principal
            if parsed.path in ["/", "/index.html"]:
                self._serve_file(INDEX_HTML_PATH, "text/html; charset=utf-8")

            # 2. Recursos Estáticos (CSS, JS, Imagens) com Confinamento
            elif parsed.path.startswith("/static/"):
                rel_path = parsed.path[len("/static/"):].lstrip("/")
                abs_path = os.path.realpath(os.path.join(REAL_STATIC_DIR, rel_path))

                try:
                    is_within_static = (os.path.commonpath([REAL_STATIC_DIR, abs_path]) == REAL_STATIC_DIR)
                except ValueError:
                    is_within_static = False

                if not is_within_static:
                    self.send_error(403, "Acesso negado")
                    return

                content_type = "application/octet-stream"
                if abs_path.endswith(".css"):
                    content_type = "text/css; charset=utf-8"
                elif abs_path.endswith(".js"):
                    content_type = "application/javascript; charset=utf-8"
                elif abs_path.endswith(".png"):
                    content_type = "image/png"
                elif abs_path.endswith(".jpg") or abs_path.endswith(".jpeg"):
                    content_type = "image/jpeg"

                self._serve_file(abs_path, content_type)

            # 3. Streaming MJPEG Sincronizado por Condição
            elif parsed.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.end_headers()

                last_seq = -1
                try:
                    while True:
                        with state.frame_cond:
                            state.frame_cond.wait_for(lambda: state.jpeg_seq != last_seq)
                            jpeg_bytes = state.latest_jpeg
                            last_seq = state.jpeg_seq

                        if jpeg_bytes is None:
                            continue

                        self.wfile.write(b"--frame\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpeg_bytes)))
                        self.end_headers()
                        self.wfile.write(jpeg_bytes)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            # 4. Telemetria Unificada em Tempo Real (10 Hz)
            elif parsed.path == "/api/telemetry":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()

                motor_snap = motor_ctrl.get_telemetry_snapshot()
                with state.lock:
                    hist_sample = list(state.history)[-40:] if hasattr(state, "history") else []
                    tel = {
                        "fps": state.fps,
                        "detected": state.ball_detected,
                        "x_mm": state.ball_x_mm,
                        "y_mm": state.ball_y_mm,
                        "vel_x_mms": getattr(state, "vel_x_mms", 0.0),
                        "vel_y_mms": getattr(state, "vel_y_mms", 0.0),
                        "comp_x_mm": getattr(state, "comp_x_mm", 0.0),
                        "comp_y_mm": getattr(state, "comp_y_mm", 0.0),
                        "conf": state.confidence,
                        "pitch_deg": state.plate_pitch_deg,
                        "roll_deg": state.plate_roll_deg,
                        "motor_steps": motor_snap["current_steps"],
                        "target_steps": motor_snap["target_steps"],
                        "pigpio": motor_snap["connected"] if state.op_mode == "STANDALONE_RPI" else True,
                        "motors_enabled": motor_snap["is_enabled"],
                        "mode": state.mode,
                        "op_mode": state.op_mode,
                        "invert": motor_snap["invert_dir"],
                        "step_delay_us": motor_snap["step_delay_us"],
                        "detector": getattr(state, "detector_type", "hsv"),
                        # Orientação geométrica
                        "cam_rotation": getattr(state, "cam_rotation", 0),
                        "invert_x": getattr(state, "invert_x", False),
                        "invert_y": getattr(state, "invert_y", False),
                        "swap_xy": getattr(state, "swap_xy", False),
                        "front_motor": getattr(state, "front_motor", 1),
                        # Sintonia e referências
                        "kp": state.kp,
                        "ki": state.ki,
                        "kd": state.kd,
                        "n": state.n_filter,
                        "sp_x": state.setpoint_x_mm,
                        "sp_y": state.setpoint_y_mm,
                        "q_accel": getattr(state, "kalman_q_accel", 0.05),
                        "r_meas": getattr(state, "kalman_r_meas", 2.25e-6),
                        "latency_ms": getattr(state, "kalman_latency_ms", 30.0),
                        "use_kalman": getattr(state, "use_kalman", True),
                        "blur_size": getattr(state, "blur_size", 5),
                        "min_radius": getattr(state, "min_radius", 4),
                        "max_radius": getattr(state, "max_radius", 40),
                        "hsv_lower": list(state.hsv_lower),
                        "hsv_upper": list(state.hsv_upper),
                        "show_mask": getattr(state, "show_mask", False),
                        "crop_x": getattr(state, "crop_x", 0),
                        "crop_y": getattr(state, "crop_y", 0),
                        "crop_w": getattr(state, "crop_w", 320),
                        "crop_h": getattr(state, "crop_h", 240),
                        "uart_connected": (ser_b is not None and not ser_b.mock_mode and ser_b.ser is not None and ser_b.ser.is_open),
                        "uart_packets_sent": getattr(ser_b, "packets_sent", 0) if ser_b else 0,
                        "uart_packets_failed": getattr(ser_b, "packets_failed", 0) if ser_b else 0,
                        "history": hist_sample
                    }
                self.wfile.write(json.dumps(tel).encode("utf-8"))

            else:
                self.send_error(404, "Não encontrado")

        def do_POST(self):
            content_length = int(self.headers.get("Content-Length", 0))
            if content_length > 4096:
                self.send_error(413, "Payload Too Large")
                return

            body = self.rfile.read(content_length) if content_length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
                if not isinstance(data, dict):
                    data = {}
            except Exception:
                data = {}

            parsed = urlparse(self.path)

            # Rota Unificada 1: POST /api/config
            if parsed.path == "/api/config" or parsed.path in [
                "/api/pid", "/api/kalman", "/api/orientation", "/api/op_mode",
                "/api/mode", "/api/motor_speed", "/api/invert_motor", "/api/crop",
                "/api/hsv", "/api/toggle_mask", "/api/detector"
            ]:
                self._handle_config_update(data, parsed.path)
                self._send_json_ok()

            # Rota Unificada 2: POST /api/action
            elif parsed.path == "/api/action" or parsed.path in [
                "/api/level", "/api/zero", "/api/reset_calibration",
                "/api/jog_motor", "/api/manual_tilt", "/api/torque", "/api/preset"
            ]:
                self._handle_action(data, parsed.path)
                self._send_json_ok()

            else:
                self.send_error(404, "Rota desconhecida")

        def _handle_config_update(self, data: Dict[str, Any], path: str):
            state = context.state
            motor_ctrl = context.motor_ctrl
            kinematics = context.kinematics
            pid_x = context.pid_x
            pid_y = context.pid_y
            kalman = context.kalman
            ser_b = context.serial_bridge

            with state.lock:
                # PID & Saturação
                if "kp" in data: state.kp = _safe_float(data["kp"], state.kp, 0.0, 100.0)
                if "ki" in data: state.ki = _safe_float(data["ki"], state.ki, 0.0, 100.0)
                if "kd" in data: state.kd = _safe_float(data["kd"], state.kd, 0.0, 100.0)
                if "n" in data: state.n_filter = _safe_float(data["n"], state.n_filter, 0.1, 100.0)
                if "setpoint_x" in data: state.setpoint_x_mm = _safe_float(data["setpoint_x"], state.setpoint_x_mm, -200.0, 200.0)
                if "setpoint_y" in data: state.setpoint_y_mm = _safe_float(data["setpoint_y"], state.setpoint_y_mm, -200.0, 200.0)
                if "max_tilt_deg" in data:
                    state.max_tilt_deg = _safe_float(data["max_tilt_deg"], state.max_tilt_deg, 1.0, 25.0)
                    max_rad = math.radians(state.max_tilt_deg)
                    pid_x.set_limits(-max_rad, max_rad)
                    pid_y.set_limits(-max_rad, max_rad)
                pid_x.set_gains(kp=state.kp, ki=state.ki, kd=state.kd, n_filter=state.n_filter)
                pid_y.set_gains(kp=state.kp, ki=state.ki, kd=state.kd, n_filter=state.n_filter)

                # Kalman
                if "q_accel" in data: state.kalman_q_accel = _safe_float(data["q_accel"], state.kalman_q_accel, 1e-6, 10.0)
                if "r_meas" in data: state.kalman_r_meas = _safe_float(data["r_meas"], state.kalman_r_meas, 1e-9, 1.0)
                if "latency_ms" in data: state.kalman_latency_ms = _safe_float(data["latency_ms"], state.kalman_latency_ms, 0.0, 200.0)
                if "use_kalman" in data: state.use_kalman = bool(data["use_kalman"])
                if kalman:
                    kalman.set_parameters(
                        q_accel=state.kalman_q_accel,
                        r_meas=state.kalman_r_meas,
                        latency_sec=state.kalman_latency_ms / 1000.0
                    )

                # Orientação Geométrica
                if "cam_rotation" in data: state.cam_rotation = _safe_int(data["cam_rotation"], state.cam_rotation, 0, 270)
                if "invert_x" in data: state.invert_x = bool(data["invert_x"])
                if "invert_y" in data: state.invert_y = bool(data["invert_y"])
                if "swap_xy" in data: state.swap_xy = bool(data["swap_xy"])
                if "front_motor" in data:
                    state.front_motor = _safe_int(data["front_motor"], state.front_motor, 1, 3)
                    if kinematics:
                        kinematics.set_front_motor(state.front_motor)
                    if state.op_mode == "HYBRID_STM32" and ser_b:
                        ser_b.send_system_command(4, state.front_motor)

                # Modo de Controle
                if "mode" in data and str(data["mode"]) in ["AUTO", "MANUAL_TILT", "MANUAL_MOTOR"]:
                    state.mode = str(data["mode"])
                    if state.mode == "AUTO":
                        pid_x.reset()
                        pid_y.reset()

                # Modo Operacional (STANDALONE_RPI / HYBRID_STM32)
                if "op_mode" in data and str(data["op_mode"]) in ["STANDALONE_RPI", "HYBRID_STM32"]:
                    state.op_mode = str(data["op_mode"])
                    motor_ctrl.set_op_mode(state.op_mode)

                # ROI Crop
                if "crop_x" in data: state.crop_x = _safe_int(data["crop_x"], state.crop_x, 0, 1920)
                if "crop_y" in data: state.crop_y = _safe_int(data["crop_y"], state.crop_y, 0, 1080)
                if "crop_w" in data: state.crop_w = _safe_int(data["crop_w"], state.crop_w, 10, 1920)
                if "crop_h" in data: state.crop_h = _safe_int(data["crop_h"], state.crop_h, 10, 1080)

                # Filtros HSV
                if "hsv_lower" in data or "lower" in data:
                    low_val = data.get("hsv_lower", data.get("lower"))
                    state.hsv_lower = _safe_hsv_triplet(low_val, state.hsv_lower)
                if "hsv_upper" in data or "upper" in data:
                    up_val = data.get("hsv_upper", data.get("upper"))
                    state.hsv_upper = _safe_hsv_triplet(up_val, state.hsv_upper)
                if "blur_size" in data: state.blur_size = _safe_int(data["blur_size"], state.blur_size, 1, 31)
                if "min_radius" in data: state.min_radius = _safe_int(data["min_radius"], state.min_radius, 1, 100)
                if "max_radius" in data: state.max_radius = _safe_int(data["max_radius"], state.max_radius, 1, 200)

                # Máscara
                if "show_mask" in data: state.show_mask = bool(data["show_mask"])

                # Detector
                if "detector" in data:
                    det_name = str(data["detector"]).lower().strip()
                    state.detector_type = det_name
                    if callable(context.on_detector_change):
                        context.on_detector_change(det_name)

            # Sincronização UART se setpoint mudou em HYBRID_STM32
            if state.op_mode == "HYBRID_STM32" and ser_b and ("setpoint_x" in data or "setpoint_y" in data):
                ser_b.send_setpoint(state.setpoint_x_mm, state.setpoint_y_mm)

            # Velocidade de passos
            if "step_delay_us" in data or "delay_us" in data:
                del_us = _safe_int(data.get("step_delay_us", data.get("delay_us")), 1400, 300, 10000)
                motor_ctrl.set_step_delay_us(del_us)

            # Inversão de direção
            if "invert_motor" in data:
                inv_info = data["invert_motor"]
                if isinstance(inv_info, dict) and "motor" in inv_info and "invert" in inv_info:
                    motor_ctrl.set_invert_direction(_safe_int(inv_info["motor"], 1, 1, 3), bool(inv_info["invert"]))
            elif "motor" in data and "invert" in data and path == "/api/invert_motor":
                motor_ctrl.set_invert_direction(_safe_int(data["motor"], 1, 1, 3), bool(data["invert"]))

        def _handle_action(self, data: Dict[str, Any], path: str):
            state = context.state
            motor_ctrl = context.motor_ctrl
            kinematics = context.kinematics
            pid_x = context.pid_x
            pid_y = context.pid_y
            kalman = context.kalman
            ser_b = context.serial_bridge

            action = data.get("action", "")
            if not action:
                if path == "/api/level": action = "level"
                elif path == "/api/zero": action = "zero"
                elif path == "/api/reset_calibration": action = "reset_calibration"
                elif path == "/api/jog_motor": action = "jog"
                elif path == "/api/manual_tilt": action = "manual_tilt"
                elif path == "/api/torque": action = "torque"
                elif path == "/api/preset": action = "preset"

            if action == "level":
                with state.lock:
                    state.plate_pitch_deg = 0.0
                    state.plate_roll_deg = 0.0
                    state.motor_angles_deg = [round(kinematics.neutral_angle_deg, 2)] * 3
                    pid_x.reset()
                    pid_y.reset()
                    if kalman: kalman.reset()
                if state.op_mode == "STANDALONE_RPI":
                    motor_ctrl.level()
                elif state.op_mode == "HYBRID_STM32" and ser_b:
                    ser_b.send_system_command(3)

            elif action == "zero":
                save_disk = bool(data.get("save_to_disk", True))
                if state.op_mode == "STANDALONE_RPI":
                    motor_ctrl.zero_calibration(save_to_disk=save_disk)
                elif state.op_mode == "HYBRID_STM32" and ser_b:
                    ser_b.send_system_command(3)
                with state.lock:
                    state.plate_pitch_deg = 0.0
                    state.plate_roll_deg = 0.0
                    state.motor_angles_deg = [round(kinematics.neutral_angle_deg, 2)] * 3
                    pid_x.reset()
                    pid_y.reset()
                    if kalman: kalman.reset()

            elif action == "reset_calibration":
                if state.op_mode == "STANDALONE_RPI":
                    motor_ctrl.reset_calibration()
                with state.lock:
                    state.plate_pitch_deg = 0.0
                    state.plate_roll_deg = 0.0
                    state.motor_angles_deg = [round(kinematics.neutral_angle_deg, 2)] * 3
                    pid_x.reset()
                    pid_y.reset()
                    if kalman: kalman.reset()
                    
            elif action == "jog":
                motor_id = _safe_int(data.get("motor", 1), 1, 1, 3)
                delta_steps = _safe_int(data.get("steps", 0), 0, -2000, 2000)
                with state.lock:
                    state.mode = "MANUAL_MOTOR"
                if state.op_mode == "STANDALONE_RPI":
                    motor_ctrl.jog(motor_id, delta_steps)

            elif action == "manual_tilt":
                with state.lock:
                    state.mode = "MANUAL_TILT"
                    if "pitch_deg" in data:
                        state.plate_pitch_deg = _safe_float(data["pitch_deg"], state.plate_pitch_deg, -state.max_tilt_deg, state.max_tilt_deg)
                    if "roll_deg" in data:
                        state.plate_roll_deg = _safe_float(data["roll_deg"], state.plate_roll_deg, -state.max_tilt_deg, state.max_tilt_deg)
                    p_rad = math.radians(state.plate_pitch_deg)
                    r_rad = math.radians(state.plate_roll_deg)
                    ok, res = kinematics.compute_inverse(p_rad, r_rad)
                    if ok:
                        state.motor_angles_deg = res["angles_deg"]
                        step_deltas = res["step_deltas"]
                    else:
                        step_deltas = None
                if ok and step_deltas is not None and state.op_mode == "STANDALONE_RPI":
                    motor_ctrl.set_targets(step_deltas)

            elif action == "torque":
                hold = bool(data.get("hold", True))
                if hold:
                    motor_ctrl.enable_all()
                    if state.op_mode == "HYBRID_STM32" and ser_b:
                        ser_b.send_system_command(1)
                else:
                    motor_ctrl.disable_all()
                    if state.op_mode == "HYBRID_STM32" and ser_b:
                        ser_b.send_system_command(2)

            elif action == "preset":
                preset_name = str(data.get("name", ""))
                with state.lock:
                    if preset_name == "conservative":
                        state.kp, state.ki, state.kd, state.n_filter = 1.05, 0.02, 0.65, 18.0
                    elif preset_name == "balanced":
                        state.kp, state.ki, state.kd, state.n_filter = 1.35, 0.05, 0.85, 20.0
                    elif preset_name == "sport":
                        state.kp, state.ki, state.kd, state.n_filter = 1.80, 0.08, 1.15, 25.0
                    elif preset_name == "orange_ball":
                        state.hsv_lower = [5, 120, 100]
                        state.hsv_upper = [25, 255, 255]
                        state.min_radius = 4
                        state.max_radius = 42
                    elif preset_name == "white_ball":
                        state.hsv_lower = [0, 0, 180]
                        state.hsv_upper = [179, 60, 255]
                        state.min_radius = 4
                        state.max_radius = 42
                    pid_x.set_gains(kp=state.kp, ki=state.ki, kd=state.kd, n_filter=state.n_filter)
                    pid_y.set_gains(kp=state.kp, ki=state.ki, kd=state.kd, n_filter=state.n_filter)

        def _send_json_ok(self):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')

    return DashboardHandler


def run_dashboard_server(host, port, context):
    handler_class = create_handler(context)
    httpd = ThreadedServer((host, port), handler_class)
    return httpd
