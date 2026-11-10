#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Camada de Hardware e Atuação (core/hardware.py)
Acionamento de Passos via pigpio, Calibração Persistente e Ponte Serial UART
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os
import time
import json
import struct
import binascii
import threading
from datetime import datetime
from typing import List

try:
    import pigpio
except ImportError:
    pigpio = None

try:
    import serial
except ImportError:
    serial = None

import config
from core.control_math import HexagonalKinematics3DoF


# ==============================================================================
# 1. UTILITÁRIOS DE CALIBRAÇÃO JSON (SEM ÂNGULOS HARDCODED)
# ==============================================================================
def compute_nominal_neutral_deg(steps_per_rev: int = 1600) -> float:
    """Calcula analiticamente o ângulo neutro nominal a partir da geometria física."""
    kin = HexagonalKinematics3DoF(steps_per_rev=steps_per_rev)
    return round(kin.neutral_angle_deg, 2)


def get_default_calibration_path() -> str:
    return getattr(
        config,
        "CALIBRATION_FILE",
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calibration_offsets.json")
    )


def load_calibration(filepath: str = None, nominal_neutral_deg: float = None) -> dict:
    path = filepath or get_default_calibration_path()
    spr_default = 200 * getattr(config, "DEFAULT_MICROSTEPS", 8)
    nom_deg = nominal_neutral_deg if nominal_neutral_deg is not None else compute_nominal_neutral_deg(spr_default)

    default_data = {
        "timestamp": None,
        "steps_per_rev": spr_default,
        "nominal_neutral_deg": nom_deg,
        "step_offsets": [0, 0, 0],
        "angle_offsets_deg": [0.0, 0.0, 0.0],
        "calibrated_neutral_angles_deg": [nom_deg] * 3,
        "is_calibrated": False
    }

    if not os.path.isfile(path):
        return default_data

    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        offsets = data.get("step_offsets", data.get("offsets_steps", [0, 0, 0]))
        if len(offsets) == 3:
            default_data["step_offsets"] = [int(x) for x in offsets]
            default_data["is_calibrated"] = bool(data.get("is_calibrated", any(default_data["step_offsets"])))

        spr = int(data.get("steps_per_rev", spr_default))
        default_data["steps_per_rev"] = spr

        deg_per_step = 360.0 / float(spr)
        angle_offsets = [s * deg_per_step for s in default_data["step_offsets"]]
        default_data["angle_offsets_deg"] = [round(a, 3) for a in angle_offsets]
        default_data["calibrated_neutral_angles_deg"] = [round(nom_deg + a, 3) for a in angle_offsets]
        default_data["timestamp"] = data.get("timestamp")
        return default_data
    except Exception as e:
        print(f"[CALIBRAÇÃO] Erro ao ler {path}: {e}")
        return default_data


def save_calibration(step_offsets: List[int],
                     steps_per_rev: int = 1600,
                     nominal_neutral_deg: float = None,
                     is_calibrated: bool = True,
                     filepath: str = None) -> bool:
    path = filepath or get_default_calibration_path()
    if nominal_neutral_deg is None:
        nominal_neutral_deg = compute_nominal_neutral_deg(steps_per_rev)
    try:
        step_offsets = [int(s) for s in step_offsets]
        deg_per_step = 360.0 / float(steps_per_rev)
        angle_offsets = [round(s * deg_per_step, 3) for s in step_offsets]
        calibrated_angles = [round(nominal_neutral_deg + a, 3) for a in angle_offsets]

        payload = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S") if is_calibrated else None,
            "steps_per_rev": int(steps_per_rev),
            "nominal_neutral_deg": float(nominal_neutral_deg),
            "step_offsets": step_offsets,
            "angle_offsets_deg": angle_offsets,
            "calibrated_neutral_angles_deg": calibrated_angles,
            "is_calibrated": bool(is_calibrated),
            "description": "Offsets de nivelamento físico dos braços 3-DoF"
        }

        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        return True
    except Exception as e:
        print(f"[CALIBRAÇÃO] Erro ao gravar {path}: {e}")
        return False


# ==============================================================================
# 2. CONTROLADOR DE MOTORES DE PASSO (pigpio + DDA)
# ==============================================================================
class MotorController:
    """Controlador multieixo para 3 motores de passo NEMA 17 com drivers TMC2209."""

    def __init__(self, steps_per_rev=1600, step_delay_us=None,
                 max_excursion_deg=None, jog_excursion_deg=None,
                 invert_dir=None, apply_offsets_on_init: bool = False):
        self.lock = threading.RLock()
        self.pi = None
        self.steps_per_rev = int(steps_per_rev)
        self.current_steps = [0, 0, 0]
        self.target_steps = [0, 0, 0]
        self.invert_dir = list(invert_dir) if invert_dir is not None else list(config.DEFAULT_INVERT_DIR)
        self.step_delay_us = int(step_delay_us if step_delay_us is not None else config.DEFAULT_STEP_DELAY_US)
        self.is_enabled = False
        self.running = True
        self.op_mode = config.DEFAULT_OP_MODE

        self.max_excursion_deg = float(max_excursion_deg if max_excursion_deg is not None else config.MAX_MOTOR_EXCURSION_DEG)
        self.jog_excursion_deg = float(jog_excursion_deg if jog_excursion_deg is not None else config.MAX_JOG_EXCURSION_DEG)
        self._update_limits()

        # Ângulo neutro obtido analiticamente da cinemática (sem valor bruto)
        self.nominal_neutral_deg = compute_nominal_neutral_deg(self.steps_per_rev)
        self.calib_offsets = [0, 0, 0]
        self.load_calibration_offsets()

        self._last_connect_try = 0.0
        self._last_dir = [None, None, None]
        self._dda_acc = [0.0, 0.0, 0.0]

        self._apply_offsets_pending = bool(apply_offsets_on_init)
        self.worker = threading.Thread(target=self._motion_worker, name="StepperWorker", daemon=True)
        self.worker.start()

        if self._apply_offsets_pending and self.op_mode == "STANDALONE_RPI":
            if self.connect(force=True):
                self._apply_offsets_pending = False
                self.apply_saved_offsets(wait=True)

    def _update_limits(self):
        spd = self.steps_per_rev / 360.0
        self.max_step_limit = int(round(self.max_excursion_deg * spd))
        self.jog_step_limit = max(self.max_step_limit, int(round(self.jog_excursion_deg * spd)))

    def set_op_mode(self, mode: str):
        with self.lock:
            self.op_mode = mode
            if mode == "HYBRID_STM32":
                self.disable_all()
            else:
                if self.connect(force=True):
                    if getattr(self, "_apply_offsets_pending", False):
                        self._apply_offsets_pending = False
                        self.apply_saved_offsets(wait=True)

    def is_connected(self) -> bool:
        return self.pi is not None and self.pi.connected

    def connect(self, force: bool = False):
        if self.is_connected():
            return True
        now = time.monotonic()
        if not force and (now - self._last_connect_try) < 2.0:
            return False
        self._last_connect_try = now

        if pigpio is None:
            return False

        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.pi = None
                return False

            for m_id, cfg in config.PIGPIO_MOTORS.items():
                self.pi.set_mode(cfg['en'], pigpio.OUTPUT)
                self.pi.set_mode(cfg['dir'], pigpio.OUTPUT)
                self.pi.set_mode(cfg['step'], pigpio.OUTPUT)
                self.pi.write(cfg['en'], 1)  # TMC2209 EN ativo baixo -> 1 desativa inicialmente
                self.pi.write(cfg['step'], 0)

            self.is_enabled = False
            self._last_dir = [None, None, None]
            return True
        except Exception:
            self.pi = None
            return False

    def enable_all(self):
        with self.lock:
            if not self.is_connected():
                if not self.connect(force=True):
                    return
            for cfg in config.PIGPIO_MOTORS.values():
                self.pi.write(cfg['en'], 0)  # Ativo baixo
            self.is_enabled = True

    def disable_all(self):
        with self.lock:
            if self.is_connected():
                for cfg in config.PIGPIO_MOTORS.values():
                    self.pi.write(cfg['en'], 1)
            self.is_enabled = False

    def set_targets(self, step_targets):
        with self.lock:
            lim = self.max_step_limit
            clamped = [max(-lim, min(lim, int(s))) for s in step_targets]
            self._set_targets_locked(clamped)

    def set_targets_unclamped(self, step_targets):
        with self.lock:
            lim = self.jog_step_limit
            clamped = [max(-lim, min(lim, int(s))) for s in step_targets]
            self._set_targets_locked(clamped)

    def _set_targets_locked(self, targets):
        if not self.is_connected():
            self.target_steps = list(targets)
            self.current_steps = list(targets)
            return

        if not self.is_enabled:
            self.enable_all()

        self.target_steps = list(targets)
        for i, m_id in enumerate((1, 2, 3)):
            diff = self.target_steps[i] - self.current_steps[i]
            if diff == 0:
                continue
            desired_dir = 1 if diff > 0 else 0
            if self.invert_dir[i]:
                desired_dir = 1 - desired_dir
            if desired_dir != self._last_dir[i]:
                self.pi.write(config.PIGPIO_MOTORS[m_id]['dir'], desired_dir)
                self._last_dir[i] = desired_dir

    def jog(self, motor_index: int, steps: int):
        with self.lock:
            idx = motor_index - 1
            if 0 <= idx < 3:
                lim = self.jog_step_limit
                new_val = max(-lim, min(lim, self.target_steps[idx] + int(steps)))
                targets = list(self.target_steps)
                targets[idx] = new_val
                self._set_targets_locked(targets)

    def level(self):
        """Retorna todos os motores suavemente para a posição neutra calibrada (0, 0, 0)."""
        with self.lock:
            self._set_targets_locked([0, 0, 0])

    def set_invert_direction(self, motor_index: int, invert: bool):
        with self.lock:
            idx = motor_index - 1
            if 0 <= idx < 3:
                self.invert_dir[idx] = bool(invert)
                self._last_dir[idx] = None

    def set_step_delay_us(self, delay_us: int):
        with self.lock:
            self.step_delay_us = max(400, min(10000, int(delay_us)))

    def load_calibration_offsets(self, filepath: str = None) -> dict:
        data = load_calibration(filepath, nominal_neutral_deg=self.nominal_neutral_deg)
        with self.lock:
            self.calib_offsets = list(data.get("step_offsets", [0, 0, 0]))
        return data

    def save_current_as_neutral(self, filepath: str = None) -> bool:
        """Soma os passos atuais aos offsets anteriores, salva no JSON e zera os contadores."""
        with self.lock:
            new_offsets = [int(self.calib_offsets[i] + self.current_steps[i]) for i in range(3)]
            ok = save_calibration(
                step_offsets=new_offsets,
                steps_per_rev=self.steps_per_rev,
                nominal_neutral_deg=self.nominal_neutral_deg,
                is_calibrated=True,
                filepath=filepath
            )
            if ok:
                self.calib_offsets = new_offsets
                self.current_steps = [0, 0, 0]
                self.target_steps = [0, 0, 0]
                self._dda_acc = [0.0, 0.0, 0.0]
            return ok

    # Alias para compatibilidade com scripts de calibração anteriores
    save_current_calibration = save_current_as_neutral

    def zero_calibration(self, save_to_disk: bool = False, filepath: str = None) -> bool:
        """Define a posição física atual como zero lógico (Tare), salvando opcionalmente em disco."""
        if save_to_disk:
            return self.save_current_as_neutral(filepath=filepath)
        with self.lock:
            self.current_steps = [0, 0, 0]
            self.target_steps = [0, 0, 0]
            self._dda_acc = [0.0, 0.0, 0.0]
        return True

    def reset_calibration(self, filepath: str = None) -> bool:
        """Restaura a calibração para o padrão de fábrica ([0, 0, 0]) e limpa o JSON."""
        with self.lock:
            ok = save_calibration(
                step_offsets=[0, 0, 0],
                steps_per_rev=self.steps_per_rev,
                nominal_neutral_deg=self.nominal_neutral_deg,
                is_calibrated=False,
                filepath=filepath
            )
            self.calib_offsets = [0, 0, 0]
            self.current_steps = [0, 0, 0]
            self.target_steps = [0, 0, 0]
            self._dda_acc = [0.0, 0.0, 0.0]
            return ok

    def apply_saved_offsets(self, wait: bool = False) -> bool:
        with self.lock:
            if not any(self.calib_offsets):
                return True
            if self.op_mode == "STANDALONE_RPI" and not self.is_connected():
                if not self.connect(force=True):
                    return False
            lim = self.jog_step_limit
            clamped = [max(-lim, min(lim, int(s))) for s in self.calib_offsets]
            self._set_targets_locked(clamped)
        if wait:
            self.wait_until_idle(timeout=5.0)
            with self.lock:
                self.current_steps = [0, 0, 0]
                self.target_steps = [0, 0, 0]
                self._dda_acc = [0.0, 0.0, 0.0]
        return True

    def get_telemetry_snapshot(self) -> dict:
        with self.lock:
            return {
                "current_steps": list(self.current_steps),
                "target_steps": list(self.target_steps),
                "calib_offsets": list(self.calib_offsets),
                "invert_dir": list(self.invert_dir),
                "step_delay_us": int(self.step_delay_us),
                "is_enabled": bool(self.is_enabled),
                "connected": self.is_connected(),
            }

    def get_motor_angles_deg(self) -> List[float]:
        with self.lock:
            deg_per_step = 360.0 / float(self.steps_per_rev)
            return [
                round(self.nominal_neutral_deg + ((self.calib_offsets[i] + self.current_steps[i]) * deg_per_step), 2)
                for i in range(3)
            ]

    def is_idle(self) -> bool:
        with self.lock:
            return self.current_steps == self.target_steps

    def wait_until_idle(self, timeout: float = 5.0) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if self.is_idle():
                return True
            time.sleep(0.01)
        return False

    def _motion_worker(self):
        """Worker DDA para pulso simultâneo de múltiplos motores."""
        step_pin_map = [config.PIGPIO_MOTORS[m]['step'] for m in (1, 2, 3)]
        while self.running:
            if not self.is_enabled or not self.is_connected() or self.op_mode != "STANDALONE_RPI":
                time.sleep(0.01)
                continue

            with self.lock:
                diffs = [self.target_steps[i] - self.current_steps[i] for i in range(3)]
                max_diff = max(abs(d) for d in diffs)
                if max_diff == 0:
                    self._dda_acc = [0.0, 0.0, 0.0]
                    idle = True
                else:
                    idle = False
                    mask_set = 0
                    signs = [1 if d > 0 else (-1 if d < 0 else 0) for d in diffs]
                    for i in range(3):
                        if diffs[i] == 0:
                            continue
                        self._dda_acc[i] += abs(diffs[i])
                        if self._dda_acc[i] >= max_diff:
                            self._dda_acc[i] -= max_diff
                            self.current_steps[i] += signs[i]
                            mask_set |= (1 << step_pin_map[i])
                    half_period = self.step_delay_us * 1e-6

            if idle:
                time.sleep(0.002)
                continue

            if mask_set:
                self.pi.set_bank_1(mask_set)
                time.sleep(half_period)
                self.pi.clear_bank_1(mask_set)
                time.sleep(half_period)
            else:
                time.sleep(0.001)

    def stop(self):
        self.running = False
        self.disable_all()
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
        if self.pi:
            self.pi.stop()


# ==============================================================================
# 3. PONTE SERIAL UART RPi 3 <-> STM32F446RE (CRC16-CCITT)
# ==============================================================================
def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT (polinômio 0x1021, valor inicial 0xFFFF, sem reflexão)."""
    return binascii.crc_hqx(data, 0xFFFF)


class SerialBridge:
    """Comunicação serial binária determinística com proteção por mutex."""

    PREAMBLE = bytes([0xAA, 0x55])
    MSG_VISION_COORDS = 0x01
    MSG_SETPOINT_CMD = 0x02
    MSG_CONTROL_STATE = 0x03
    MSG_SYSTEM_COMMAND = 0x04

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, mock_mode=False, timeout_s=0.01):
        self._lock = threading.Lock()
        self.port = port
        self.baudrate = baudrate
        self.mock_mode = mock_mode
        self.timeout_s = timeout_s
        self.ser = None
        self.packets_sent = 0
        self.packets_failed = 0

    def connect(self):
        with self._lock:
            if self.mock_mode:
                return True
            if serial is None:
                self.mock_mode = True
                return False
            try:
                self.ser = serial.Serial(
                    port=self.port, baudrate=self.baudrate,
                    timeout=self.timeout_s, write_timeout=self.timeout_s
                )
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                return True
            except Exception:
                self.mock_mode = True
                return False

    def send_coordinates(self, timestamp_ms: int, pos_x_mm: float, pos_y_mm: float,
                         detected: bool, is_yolo: bool = False, confidence: float = 0.0):
        x_tenths = max(-32768, min(32767, int(round(pos_x_mm * 10.0))))
        y_tenths = max(-32768, min(32767, int(round(pos_y_mm * 10.0))))
        flags = (1 if detected else 0) | ((1 if is_yolo else 0) << 1)
        conf_byte = max(0, min(100, int(round(confidence * 100.0))))

        payload = struct.pack("<IhhBB", timestamp_ms & 0xFFFFFFFF, x_tenths, y_tenths, flags, conf_byte)
        header_and_payload = bytes([self.MSG_VISION_COORDS, len(payload)]) + payload
        crc_val = crc16_ccitt(header_and_payload)
        packet = self.PREAMBLE + header_and_payload + struct.pack(">H", crc_val)
        return self._write_packet(packet)

    def send_setpoint(self, sp_x_mm: float, sp_y_mm: float):
        x_tenths = max(-32768, min(32767, int(round(sp_x_mm * 10.0))))
        y_tenths = max(-32768, min(32767, int(round(sp_y_mm * 10.0))))
        payload = struct.pack("<hh", x_tenths, y_tenths)
        header_and_payload = bytes([self.MSG_SETPOINT_CMD, len(payload)]) + payload
        crc_val = crc16_ccitt(header_and_payload)
        packet = self.PREAMBLE + header_and_payload + struct.pack(">H", crc_val)
        return self._write_packet(packet)

    def send_system_command(self, cmd_type: int, param: int = 0):
        payload = struct.pack("<BB", cmd_type & 0xFF, param & 0xFF)
        header_and_payload = bytes([self.MSG_SYSTEM_COMMAND, len(payload)]) + payload
        crc_val = crc16_ccitt(header_and_payload)
        packet = self.PREAMBLE + header_and_payload + struct.pack(">H", crc_val)
        return self._write_packet(packet)

    def _write_packet(self, packet: bytes):
        with self._lock:
            if self.mock_mode:
                self.packets_sent += 1
                return True, packet
            try:
                if self.ser is None or not self.ser.is_open:
                    self.packets_failed += 1
                    return False, packet
                self.ser.write(packet)
                self.packets_sent += 1
                return True, packet
            except Exception:
                self.packets_failed += 1
                return False, packet

    def close(self):
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.close()