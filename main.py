#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Ponto de Entrada Único do Sistema Ball and Plate 3-DoF (main.py)
Unifica Visão Computacional, Controle Fechado (RPi/STM32) e Dashboard Web
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import sys
import time
import math
import argparse
import threading
from collections import deque
from typing import Optional

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

import config
from core.vision import (
    CameraStream,
    AprilTagCalibrator,
    roi_scale_mm_per_px,
    pixel_to_plate_mm,
    plate_mm_to_pixel,
    create_detector
)
from core.control_math import DiscreteKalman2D, DiscretePID, HexagonalKinematics3DoF
from core.hardware import MotorController, SerialBridge
from tools.bench_suite import run_interactive_calibration
from web.server import DashboardServerContext, run_dashboard_server


# ==============================================================================
# ESTADO GLOBAL COMPARTILHADO (THREAD-SAFE COM RLock)
# ==============================================================================
class AppState:
    def __init__(self, initial_neutral_deg: float = 0.0):
        self.lock = threading.RLock()
        self.frame_cond = threading.Condition(self.lock)

        self.latest_frame = None
        self.latest_jpeg = None
        self.jpeg_seq = 0

        self.fps = 0.0
        self.ball_detected = False
        self.ball_x_mm = 0.0
        self.ball_y_mm = 0.0
        self.vel_x_mms = 0.0
        self.vel_y_mms = 0.0
        self.comp_x_mm = 0.0
        self.comp_y_mm = 0.0
        self.confidence = 0.0
        self.missing_frames = 0

        self.plate_pitch_deg = 0.0
        self.plate_roll_deg = 0.0
        self.motor_angles_deg = [round(initial_neutral_deg, 2)] * 3

        # Campos de calibração física e sincronização (R1 / R2 / Telemetria)
        self.calib_offsets = [0, 0, 0]
        self.nominal_neutral_deg = float(initial_neutral_deg)
        self.calibrated_neutral_angles_deg = [round(initial_neutral_deg, 3)] * 3
        self.is_calibrated = False
        self.calib_timestamp = None

        self.kp = config.PID_DEFAULT_KP
        self.ki = config.PID_DEFAULT_KI
        self.kd = config.PID_DEFAULT_KD
        self.n_filter = config.PID_DEFAULT_FILTER_N
        self.max_tilt_deg = config.MAX_PLATE_TILT_DEG

        self.setpoint_x_mm = 0.0
        self.setpoint_y_mm = 0.0

        self.kalman_q_accel = config.KALMAN_DEFAULT_Q_ACCEL
        self.kalman_r_meas = config.KALMAN_DEFAULT_R_MEAS
        self.kalman_latency_ms = config.KALMAN_DEFAULT_LATENCY_SEC * 1000.0
        self.use_kalman = True

        self.crop_x = 0
        self.crop_y = 0
        self.crop_w = config.FRAME_WIDTH
        self.crop_h = config.FRAME_HEIGHT
        self.show_mask = False

        self.cam_rotation = config.CAMERA_ROTATION_DEG
        self.invert_x = config.CAMERA_INVERT_X
        self.invert_y = config.CAMERA_INVERT_Y
        self.swap_xy = config.CAMERA_SWAP_XY
        self.front_motor = config.MOTOR_FRONT_ID

        self.mode = "MANUAL_TILT"  # 'AUTO', 'MANUAL_TILT', 'MANUAL_MOTOR'
        self.op_mode = config.DEFAULT_OP_MODE
        self.detector_type = "hsv"

        self.cam_height_mm = getattr(config, "CAMERA_HEIGHT_MM", 450.0)
        self.request_tag_calib = True
        self.tag_calibrated = False
        self.tags_detected = 0
        self.cam_tilt_deg = 0.0

        self.hsv_lower = list(config.HSV_LOWER)
        self.hsv_upper = list(config.HSV_UPPER)
        self.blur_size = config.BALL_BLUR_SIZE
        self.min_radius = config.BALL_MIN_RADIUS_PX
        self.max_radius = config.BALL_MAX_RADIUS_PX

        self.history = deque(maxlen=200)
        self.t_start = time.time()


# ==============================================================================
# LOOP CENTRAL DE VISÃO, ESTIMAÇÃO E CONTROLE
# ==============================================================================
def vision_loop(camera: Optional[CameraStream], detector_ref, serial_b: Optional[SerialBridge],
                state: AppState, motor_ctrl: MotorController, kinematics: HexagonalKinematics3DoF,
                pid_x: DiscretePID, pid_y: DiscretePID, kalman: DiscreteKalman2D,
                calibrator: Optional[AprilTagCalibrator] = None, preview: bool = False):
    if calibrator is None:
        calibrator = AprilTagCalibrator()
    last_frame_id = -1
    t_prev = time.time()
    fps_counter = 0
    t_fps = time.time()
    current_fps = 0.0

    while True:
        if camera:
            frame, cap_timestamp, frame_id = camera.wait_for_frame(last_frame_id, timeout=0.1)
            if frame is None:
                continue
            last_frame_id = frame_id
        else:
            time.sleep(0.033)
            cap_timestamp = time.time()
            if cv2 is not None and np is not None:
                frame = 25 * np.ones((config.FRAME_HEIGHT, config.FRAME_WIDTH, 3), dtype=np.uint8)
                cv2.circle(frame, (160, 120), 10, (0, 165, 255), -1)
            else:
                frame = None

        now = time.time()
        dt_loop = max(0.001, min(0.1, now - t_prev))
        t_prev = now

        with state.lock:
            cx = state.crop_x
            cy = state.crop_y
            cw = state.crop_w
            ch = state.crop_h
            hsv_low = tuple(state.hsv_lower)
            hsv_up = tuple(state.hsv_upper)
            blur_sz = state.blur_size
            min_r = state.min_radius
            max_r = state.max_radius
            use_kalman = state.use_kalman
            sp_x = state.setpoint_x_mm
            sp_y = state.setpoint_y_mm
            mode = state.mode
            op_mode = state.op_mode
            cur_pitch_deg = state.plate_pitch_deg
            cur_roll_deg = state.plate_roll_deg
            cam_rot = state.cam_rotation
            inv_x = state.invert_x
            inv_y = state.invert_y
            sw_xy = state.swap_xy
            show_mask_flag = state.show_mask
            req_calib = state.request_tag_calib
            cam_h_mm = state.cam_height_mm

        if req_calib and frame is not None:
            calibrator.calibrate_from_frame(frame)
            with state.lock:
                state.request_tag_calib = False
                state.tag_calibrated = calibrator.is_calibrated
                state.tags_detected = calibrator.tags_detected
                state.cam_tilt_deg = calibrator.estimated_cam_tilt_deg
        elif not calibrator.is_calibrated:
            with state.lock:
                state.tag_calibrated = False
                state.tags_detected = calibrator.tags_detected
                state.cam_tilt_deg = 0.0

        cur_pitch_rad = math.radians(cur_pitch_deg)
        cur_roll_rad = math.radians(cur_roll_deg)

        detector = detector_ref["instance"]
        if detector.get_name() == "HSV" and hasattr(detector, "set_parameters"):
            detector.set_parameters(lower=hsv_low, upper=hsv_up, blur=blur_sz, min_radius=min_r, max_radius=max_r)

        if frame is not None:
            h_orig, w_orig = frame.shape[:2]
            cx = max(0, min(w_orig - 10, cx))
            cy = max(0, min(h_orig - 10, cy))
            cw = max(10, min(w_orig - cx, cw))
            ch = max(10, min(h_orig - cy, ch))
            roi_frame = frame[cy:cy+ch, cx:cx+cw]

            detected, coords, conf, bbox = detector.detect(roi_frame)
            scale = roi_scale_mm_per_px(cw, ch, config.PLATE_DIAMETER_MM)

            if detected:
                ball_px_x, ball_px_y, radius = coords
                global_ball_x = int(ball_px_x + cx)
                global_ball_y = int(ball_px_y + cy)

                if calibrator.is_calibrated:
                    pos_x_mm, pos_y_mm = calibrator.pixel_to_mm(
                        global_ball_x, global_ball_y,
                        cam_height_mm=cam_h_mm,
                        z_ball=getattr(config, "BALL_RADIUS_MM", 20.0)
                    )
                else:
                    pos_x_mm, pos_y_mm = pixel_to_plate_mm(
                        ball_px_x, ball_px_y, cw, ch, scale,
                        rotation_deg=cam_rot, invert_x=inv_x, invert_y=inv_y, swap_xy=sw_xy
                    )
            else:
                pos_x_mm, pos_y_mm = 0.0, 0.0
                global_ball_x, global_ball_y = 0, 0
        else:
            detected = False
            conf = 0.0
            pos_x_mm, pos_y_mm = 0.0, 0.0
            global_ball_x, global_ball_y = 0, 0

        # Coasting de 5 quadros no Kalman com reset suave na reaquisição
        MAX_COAST_FRAMES = 5
        with state.lock:
            prev_missing = state.missing_frames
            if detected:
                state.missing_frames = 0
                if prev_missing > MAX_COAST_FRAMES:
                    kalman.reset(init_x=pos_x_mm * 1e-3, init_y=pos_y_mm * 1e-3, init_vx=0.0, init_vy=0.0)
                    pid_x.reset()
                    pid_y.reset()
                else:
                    kalman.predict(cur_pitch_rad, cur_roll_rad, dt=dt_loop)
                    kalman.update(pos_x_mm * 1e-3, pos_y_mm * 1e-3)
            else:
                state.missing_frames = prev_missing + 1
                if state.missing_frames <= MAX_COAST_FRAMES:
                    kalman.predict(cur_pitch_rad, cur_roll_rad, dt=dt_loop)
                else:
                    kalman.reset()
                    pid_x.reset()
                    pid_y.reset()

            fx_m, fvx_ms, fy_m, fvy_ms = kalman.get_filtered_state()
            cx_m, cvx_ms, cy_m, cvy_ms = kalman.get_compensated_state(cur_pitch_rad, cur_roll_rad)

            vel_x_mms = fvx_ms * 1000.0
            vel_y_mms = fvy_ms * 1000.0
            comp_x_mm = cx_m * 1000.0
            comp_y_mm = cy_m * 1000.0

            tracking_active = detected or (use_kalman and state.missing_frames <= MAX_COAST_FRAMES)
            ctrl_x_m = cx_m if (use_kalman and tracking_active) else (pos_x_mm * 1e-3)
            ctrl_y_m = cy_m if (use_kalman and tracking_active) else (pos_y_mm * 1e-3)

            step_deltas = None
            if mode == "AUTO":
                if tracking_active:
                    cmd_pitch = pid_x.update(ctrl_x_m, sp_x * 1e-3, dt=dt_loop)
                    cmd_roll = pid_y.update(ctrl_y_m, sp_y * 1e-3, dt=dt_loop)
                else:
                    cmd_pitch, cmd_roll = 0.0, 0.0

                ok, res = kinematics.compute_inverse(cmd_pitch, cmd_roll)
                if ok:
                    state.plate_pitch_deg = math.degrees(cmd_pitch)
                    state.plate_roll_deg = math.degrees(cmd_roll)
                    state.motor_angles_deg = res["angles_deg"]
                    step_deltas = res["step_deltas"]

        if mode == "AUTO" and step_deltas is not None and op_mode == "STANDALONE_RPI":
            motor_ctrl.set_targets(step_deltas)

        if serial_b:
            ts_ms = int(cap_timestamp * 1000) & 0xFFFFFFFF
            serial_b.send_coordinates(
                timestamp_ms=ts_ms,
                pos_x_mm=pos_x_mm,
                pos_y_mm=pos_y_mm,
                detected=detected,
                confidence=conf
            )

        if cv2 is not None and frame is not None:
            if show_mask_flag and detector.get_name() == "HSV" and getattr(detector, "latest_mask", None) is not None:
                vis_frame = cv2.cvtColor(detector.latest_mask, cv2.COLOR_GRAY2BGR)
            else:
                vis_frame = frame.copy()

                # Polígonos dos cantos das AprilTags detectadas
                if calibrator and getattr(calibrator, "last_corners", None):
                    for tag_c in calibrator.last_corners:
                        pts = np.int32(tag_c).reshape((-1, 1, 2))
                        cv2.polylines(vis_frame, [pts], isClosed=True, color=(0, 255, 255), thickness=2)

                # Bounding box da ROI manual
                cv2.rectangle(vis_frame, (cx, cy), (cx + cw, cy + ch), (0, 240, 255), 1)

                # Círculo da esfera detectada
                if detected:
                    cv2.circle(vis_frame, (global_ball_x, global_ball_y), int(radius), (0, 255, 0), 2)
                    cv2.circle(vis_frame, (global_ball_x, global_ball_y), 3, (0, 0, 255), -1)

                # Crosshair de Setpoint (calibrador AprilTag ou fallback ROI)
                if calibrator and calibrator.is_calibrated:
                    sp_px_x, sp_px_y = calibrator.mm_to_pixel(sp_x, sp_y)
                    sp_pt = (int(round(sp_px_x)), int(round(sp_px_y)))
                else:
                    sp_px_x, sp_px_y = plate_mm_to_pixel(
                        sp_x, sp_y, cw, ch, scale,
                        rotation_deg=cam_rot, invert_x=inv_x, invert_y=inv_y, swap_xy=sw_xy
                    )
                    sp_pt = (int(round(sp_px_x + cx)), int(round(sp_px_y + cy)))

                cv2.drawMarker(vis_frame, sp_pt, (0, 165, 255),
                               markerType=cv2.MARKER_CROSS, markerSize=14, thickness=2)

            ok_enc, buf = cv2.imencode(".jpg", vis_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            if ok_enc:
                with state.lock:
                    state.latest_jpeg = buf.tobytes()
                    state.jpeg_seq += 1
                    state.ball_detected = detected
                    state.ball_x_mm = pos_x_mm
                    state.ball_y_mm = pos_y_mm
                    state.vel_x_mms = vel_x_mms
                    state.vel_y_mms = vel_y_mms
                    state.comp_x_mm = comp_x_mm
                    state.comp_y_mm = comp_y_mm
                    state.confidence = conf
                    state.fps = current_fps

                    t_rel = now - state.t_start
                    state.history.append({
                        "t": round(t_rel, 3),
                        "x": round(pos_x_mm, 1),
                        "y": round(pos_y_mm, 1),
                        "cx": round(comp_x_mm, 1),
                        "cy": round(comp_y_mm, 1),
                        "sp_x": round(sp_x, 1),
                        "sp_y": round(sp_y, 1),
                        "pitch": round(state.plate_pitch_deg, 2),
                        "roll": round(state.plate_roll_deg, 2)
                    })
                    state.frame_cond.notify_all()

            if preview:
                cv2.imshow("Ball & Plate Preview", vis_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        fps_counter += 1
        if now - t_fps >= 1.0:
            current_fps = fps_counter / (now - t_fps)
            fps_counter = 0
            t_fps = now


# ==============================================================================
# MAIN ENTRYPOINT & CICLO DE VIDA TRIFÁSICO (R1)
# ==============================================================================
def parse_args():
    parser = argparse.ArgumentParser(description="Sistema Ball and Plate 3-DoF (RPi 3 / STM32)")
    parser.add_argument("--port", type=int, default=8080, help="Porta HTTP do servidor web (padrão: 8080)")
    parser.add_argument("--camera", type=int, default=config.CAMERA_INDEX, help="Índice da câmera V4L2")
    parser.add_argument("--serial-port", default=config.SERIAL_PORT, help="Porta UART para o STM32")
    parser.add_argument("--baud", type=int, default=config.SERIAL_BAUDRATE, help="Baudrate da serial")
    parser.add_argument("--detector", choices=["hsv"], default="hsv", help="Detector inicial (padrão: hsv)")
    parser.add_argument("--op-mode", choices=["STANDALONE_RPI", "HYBRID_STM32"], default=config.DEFAULT_OP_MODE,
                        help="Modo de operação: STANDALONE_RPI ou HYBRID_STM32")
    parser.add_argument("--microsteps", type=int, default=config.DEFAULT_MICROSTEPS, help="Micropassos TMC2209")
    parser.add_argument("--front-motor", type=int, choices=[1, 2, 3], default=config.MOTOR_FRONT_ID, help="ID motor frontal")
    parser.add_argument("--apply-calibration", dest="apply_calibration", action="store_true", default=True,
                        help="Aplica offsets de calibração fisicamente ao iniciar (padrão: True)")
    parser.add_argument("--no-calibration", dest="apply_calibration", action="store_false",
                        help="Não aplica offsets de calibração ao iniciar")
    parser.add_argument("--skip-cli", action="store_true", default=False,
                        help="Pula rotina CLI interativa pré-servidor e inicia diretamente")
    parser.add_argument("--headless", action="store_true", default=False,
                        help="Executa sem interface web HTTP e sem CLI interativa")
    parser.add_argument("--preview", action="store_true", help="Exibe janela OpenCV local de visualização")
    return parser.parse_args()


def main():
    args = parse_args()

    steps_per_rev = 200 * args.microsteps
    kinematics = HexagonalKinematics3DoF(steps_per_rev=steps_per_rev, front_motor_id=args.front_motor)

    print("=" * 72)
    print("INICIANDO SISTEMA BALL AND PLATE 3-DoF — ORQUESTRADOR CENTRAL")
    print(f"Modo Operacional:    {args.op_mode}")
    print(f"Motor Frontal:       Motor {args.front_motor}")
    print(f"Ângulo Neutro:       {kinematics.neutral_angle_deg:.2f}° (calculado da geometria)")
    print(f"Resolução:           {steps_per_rev} passos/rev ({args.microsteps} micropassos)")
    print(f"Detector Inicial:    {args.detector.upper()}")
    print(f"Calibração Inicial:  {'Aplicar Offsets' if args.apply_calibration else 'Offsets Ignorados (--no-calibration)'}")
    print(f"Painel Web:          {'Desativado (--headless)' if args.headless else f'Ativo em http://0.0.0.0:{args.port}'}")
    print("=" * 72)

    state = AppState(initial_neutral_deg=kinematics.neutral_angle_deg)
    state.op_mode = args.op_mode
    state.front_motor = args.front_motor
    state.detector_type = args.detector

    # Instanciação do controlador de motores (despacho explícito gerenciado pela Fase 2)
    motor_ctrl = MotorController(steps_per_rev=steps_per_rev, apply_offsets_on_init=False)
    motor_ctrl.set_op_mode(args.op_mode)

    # =========================================================================
    # FASE 1: CLI PRÉ-SERVIDOR (Calibração Interativa / Verificação via Terminal)
    # =========================================================================
    is_interactive = (not args.skip_cli) and (not args.headless) and sys.stdin.isatty()
    if is_interactive:
        print("\n[FASE 1] Iniciando rotina interativa de calibração via terminal...")
        run_interactive_calibration(motor_ctrl)
    else:
        print("\n[FASE 1] CLI interativa ignorada (--skip-cli, --headless ou ambiente não interativo).")

    # =========================================================================
    # FASE 2: PERSISTÊNCIA & HARDWARE (Barreira de Prontidão, AprilTag e AppState)
    # =========================================================================
    print("\n[FASE 2] Sincronizando persistência, aplicando calibração e verificando estabilidade...")
    calib_data = motor_ctrl.load_calibration_offsets()

    if args.apply_calibration:
        print(f"[FASE 2] Aplicando offsets de calibração salvos: {motor_ctrl.calib_offsets}...")
        motor_ctrl.apply_saved_offsets(wait=True)
    else:
        print("[FASE 2] Nivelamento sem aplicação de offsets (--no-calibration)...")
        motor_ctrl.level()

    # Barreira de prontidão (Readiness Barrier)
    hardware_idle = motor_ctrl.wait_until_idle(timeout=5.0)
    if not hardware_idle:
        print("[FASE 2] ⚠️ Aviso: Timeout aguardando estabilização dos atuadores.")
    else:
        print("[FASE 2] ✓ Barreira de prontidão concluída: Hardware em repouso e nivelado.")

    # Inicialização da Câmera
    camera = None
    if cv2 is not None:
        try:
            camera = CameraStream(src=args.camera, width=config.FRAME_WIDTH, height=config.FRAME_HEIGHT, fps=config.TARGET_FPS)
            camera.start()
            print("[CAMERA] Stream de captura ativo.")
        except Exception as e:
            print(f"[CAMERA] Aviso: Câmera física não disponível ({e}).")
            camera = None

    # Calibração AprilTag com placa fisicamente nivelada
    calibrator = AprilTagCalibrator()
    if camera:
        print("[FASE 2] Executando calibração AprilTag com placa nivelada...")
        frame, _, _ = camera.wait_for_frame(-1, timeout=0.5)
        if frame is not None:
            calibrator.calibrate_from_frame(frame)
            if calibrator.is_calibrated:
                print(f"[FASE 2] ✓ AprilTag calibrado com sucesso ({calibrator.tags_detected}/4 tags detectadas).")

    # Hidratação Completa do AppState a partir do MotorController e Calibrador
    snap = motor_ctrl.get_telemetry_snapshot()
    with state.lock:
        state.calib_offsets = list(snap["calib_offsets"])
        state.nominal_neutral_deg = snap["nominal_neutral_deg"]
        state.calibrated_neutral_angles_deg = list(snap["calibrated_neutral_angles_deg"])
        state.motor_angles_deg = list(snap["motor_angles_deg"])
        state.is_calibrated = snap["is_calibrated"]
        state.calib_timestamp = snap["timestamp"]
        state.tag_calibrated = calibrator.is_calibrated
        state.tags_detected = calibrator.tags_detected
        state.cam_tilt_deg = calibrator.estimated_cam_tilt_deg
        state.request_tag_calib = not calibrator.is_calibrated

    # Configuração dos controladores PID e Filtro de Kalman
    max_angle_rad = math.radians(state.max_tilt_deg)
    pid_x = DiscretePID(state.kp, state.ki, state.kd, n_filter=state.n_filter,
                        out_min=-max_angle_rad, out_max=max_angle_rad, ts=0.033, reverse_acting=True)
    pid_y = DiscretePID(state.kp, state.ki, state.kd, n_filter=state.n_filter,
                        out_min=-max_angle_rad, out_max=max_angle_rad, ts=0.033, reverse_acting=True)

    kalman = DiscreteKalman2D(
        ts=0.033,
        q_accel=state.kalman_q_accel,
        r_meas=state.kalman_r_meas,
        latency_sec=state.kalman_latency_ms / 1000.0
    )

    detector_hsv = create_detector("hsv", hsv_lower=config.HSV_LOWER, hsv_upper=config.HSV_UPPER)
    active_detector = {"instance": detector_hsv}

    # =========================================================================
    # FASE 3: DAEMON & SERVIÇOS (UART STM32, Servidor Web e Loop de Visão)
    # =========================================================================
    print("\n[FASE 3] Iniciando serviços de comunicação e controle...")

    # Comunicação Serial UART com STM32
    serial_bridge = SerialBridge(port=args.serial_port, baudrate=args.baud, mock_mode=False)
    if serial_bridge.connect():
        serial_bridge.send_system_command(4, args.front_motor)
        serial_bridge.send_setpoint(state.setpoint_x_mm, state.setpoint_y_mm)

    web_server = None
    if not args.headless:
        server_ctx = DashboardServerContext(
            state=state,
            motor_ctrl=motor_ctrl,
            kinematics=kinematics,
            pid_x=pid_x,
            pid_y=pid_y,
            kalman=kalman,
            serial_bridge=serial_bridge,
            on_detector_change=None,
            calibrator=calibrator
        )
        web_server = run_dashboard_server("0.0.0.0", args.port, server_ctx)
        server_thread = threading.Thread(target=web_server.serve_forever, daemon=True)
        server_thread.start()
        print(f"[WEB] Servidor ativo em http://0.0.0.0:{args.port}/")

    # Inicia Loop de Visão e Controle
    try:
        vision_loop(
            camera=camera,
            detector_ref=active_detector,
            serial_b=serial_bridge,
            state=state,
            motor_ctrl=motor_ctrl,
            kinematics=kinematics,
            pid_x=pid_x,
            pid_y=pid_y,
            kalman=kalman,
            calibrator=calibrator,
            preview=args.preview
        )
    except KeyboardInterrupt:
        print("\n[SISTEMA] Encerrando com segurança...")
    finally:
        if camera:
            camera.stop()
        if web_server:
            try:
                web_server.shutdown()
            except Exception:
                pass
        motor_ctrl.stop()
        serial_bridge.close()
        if cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()