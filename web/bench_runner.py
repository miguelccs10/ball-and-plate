#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Serviço Assíncrono de Testes de Bancada (web/bench_runner.py)
Executa rotinas de amplitude cardeal, trajetórias programadas e diagnósticos
elétricos em thread desacoplada, com monitoramento de progresso e cancelamento.
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import math
import time
import threading
from typing import Dict, Any, Optional

import config
from core.control_math import HexagonalKinematics3DoF
from core.hardware import MotorController


class BenchRunner:
    """
    Executor desacoplado de rotinas de teste de bancada e diagnóstico.
    Executa testes em segundo plano sem bloquear o servidor HTTP.
    """

    def __init__(self, motor_ctrl: MotorController, kinematics: HexagonalKinematics3DoF):
        self.motor_ctrl = motor_ctrl
        self.kinematics = kinematics
        self.lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._status = {
            "running": False,
            "test": "",
            "progress": 0,
            "message": "Pronto para execução"
        }

    def get_status(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self._status)

    def is_running(self) -> bool:
        with self.lock:
            return self._status["running"]

    def _set_status(self, running: bool, test: str, progress: int, message: str):
        with self.lock:
            self._status["running"] = running
            self._status["test"] = test
            self._status["progress"] = max(0, min(100, int(progress)))
            self._status["message"] = message

    def stop(self, wait: bool = True):
        """Sinaliza cancelamento cooperativo do teste em execução."""
        self._cancel_event.set()
        with self.lock:
            self._status["message"] = "Cancelando teste..."
        if wait and self._thread and self._thread.is_alive() and threading.current_thread() != self._thread:
            self._thread.join(timeout=2.0)

    def _sleep_or_cancel(self, seconds: float):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            if self._cancel_event.is_set():
                break
            time.sleep(0.02)

    def start_test(self, test_name: str, params: Optional[Dict[str, Any]] = None) -> bool:
        """Inicia teste assíncrono em thread secundária."""
        with self.lock:
            if self._status["running"]:
                return False
            self._cancel_event.clear()
            self._status = {
                "running": True,
                "test": test_name,
                "progress": 0,
                "message": f"Iniciando {test_name}..."
            }

        params = params or {}
        self._thread = threading.Thread(
            target=self._run_worker,
            args=(test_name, params),
            daemon=True,
            name="BenchWorker"
        )
        self._thread.start()
        return True

    def _run_worker(self, test_name: str, params: Dict[str, Any]):
        try:
            if test_name == "cardinal":
                self._run_cardinal_test(params)
            elif test_name == "trajectory":
                self._run_trajectory_test(params)
            elif test_name == "diagnostics":
                self._run_diagnostics_test(params)
            elif test_name == "repeatability":
                self._run_repeatability_test(params)
            else:
                self._set_status(False, test_name, 0, f"Teste desconhecido: {test_name}")
                return

            if self._cancel_event.is_set():
                self._set_status(False, test_name, 0, "Teste cancelado pelo usuário.")
            else:
                self._set_status(False, test_name, 100, "Teste concluído com sucesso.")
        except Exception as e:
            self._set_status(False, test_name, 0, f"Erro na execução: {e}")
        finally:
            # Garante que os atuadores retornam ao neutro com segurança
            self.motor_ctrl.level()
            self.motor_ctrl.wait_until_idle(timeout=2.0)

    def _wait_idle_or_cancel(self, timeout: float = 3.0) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if self._cancel_event.is_set():
                return False
            if self.motor_ctrl.is_idle():
                return True
            time.sleep(0.02)
        return False

    def _run_cardinal_test(self, params: Dict[str, Any]):
        max_deg = float(params.get("max_deg", 12.0))
        cardinal_points = [
            ("Neutro (0°, 0°)", 0.0, 0.0),
            (f"Pitch Positivo (+{max_deg}°)", math.radians(max_deg), 0.0),
            ("Neutro (0°, 0°)", 0.0, 0.0),
            (f"Pitch Negativo (-{max_deg}°)", math.radians(-max_deg), 0.0),
            ("Neutro (0°, 0°)", 0.0, 0.0),
            (f"Roll Positivo (+{max_deg}°)", 0.0, math.radians(max_deg)),
            ("Neutro (0°, 0°)", 0.0, 0.0),
            (f"Roll Negativo (-{max_deg}°)", 0.0, math.radians(-max_deg)),
            (f"Diagonal Nordeste (+{max_deg*0.7:.1f}°, +{max_deg*0.7:.1f}°)",
             math.radians(max_deg * 0.7), math.radians(max_deg * 0.7)),
            (f"Diagonal Sudoeste (-{max_deg*0.7:.1f}°, -{max_deg*0.7:.1f}°)",
             math.radians(-max_deg * 0.7), math.radians(-max_deg * 0.7)),
            ("Neutro Final (0°, 0°)", 0.0, 0.0)
        ]

        total = len(cardinal_points)
        for idx, (desc, p_rad, r_rad) in enumerate(cardinal_points):
            if self._cancel_event.is_set():
                return
            progress = int((idx / total) * 100)
            self._set_status(True, "cardinal", progress, f"[{idx+1}/{total}] {desc}")

            ok, res = self.kinematics.compute_inverse(p_rad, r_rad)
            if ok:
                self.motor_ctrl.set_targets(res["step_deltas"])
            self._wait_idle_or_cancel(timeout=2.5)
            self._sleep_or_cancel(0.2)

    def _run_trajectory_test(self, params: Dict[str, Any]):
        traj_type = str(params.get("trajectory", "1")).lower()
        radius_deg = float(params.get("radius_deg", 6.0))
        r_rad = math.radians(radius_deg)

        if traj_type in ("1", "circle", "circulo"):
            steps = 60
            cycles = 2
            total_steps = cycles * steps
            step_count = 0
            for cycle in range(cycles):
                for i in range(steps):
                    if self._cancel_event.is_set():
                        return
                    step_count += 1
                    prog = int((step_count / total_steps) * 100)
                    angle = (2.0 * math.pi * i) / steps
                    p = r_rad * math.cos(angle)
                    r = r_rad * math.sin(angle)
                    ok, res = self.kinematics.compute_inverse(p, r)
                    if ok:
                        self.motor_ctrl.set_targets(res["step_deltas"])
                    self._set_status(True, "trajectory", prog, f"Círculo: ciclo {cycle+1}/{cycles} ({prog}%)")
                    time.sleep(0.04)

        elif traj_type in ("2", "triangle", "triangulo"):
            vertices = [
                (r_rad, 0.0),
                (-0.5 * r_rad, r_rad * math.sqrt(3) / 2.0),
                (-0.5 * r_rad, -r_rad * math.sqrt(3) / 2.0),
                (r_rad, 0.0)
            ]
            cycles = 2
            steps_per_edge = 15
            total_steps = cycles * 3 * steps_per_edge
            step_count = 0
            for cycle in range(cycles):
                for v_idx in range(3):
                    p0, r0 = vertices[v_idx]
                    p1, r1 = vertices[v_idx + 1]
                    for s in range(steps_per_edge):
                        if self._cancel_event.is_set():
                            return
                        step_count += 1
                        prog = int((step_count / total_steps) * 100)
                        t = s / float(steps_per_edge)
                        ok, res = self.kinematics.compute_inverse(p0 + t * (p1 - p0), r0 + t * (r1 - r0))
                        if ok:
                            self.motor_ctrl.set_targets(res["step_deltas"])
                        self._set_status(True, "trajectory", prog, f"Triângulo: ciclo {cycle+1}/{cycles} ({prog}%)")
                        time.sleep(0.04)

        elif traj_type in ("3", "square", "quadrado"):
            vertices = [
                (r_rad, r_rad), (-r_rad, r_rad),
                (-r_rad, -r_rad), (r_rad, -r_rad), (r_rad, r_rad)
            ]
            cycles = 2
            steps_per_edge = 12
            total_steps = cycles * 4 * steps_per_edge
            step_count = 0
            for cycle in range(cycles):
                for v_idx in range(4):
                    p0, r0 = vertices[v_idx]
                    p1, r1 = vertices[v_idx + 1]
                    for s in range(steps_per_edge):
                        if self._cancel_event.is_set():
                            return
                        step_count += 1
                        prog = int((step_count / total_steps) * 100)
                        t = s / float(steps_per_edge)
                        ok, res = self.kinematics.compute_inverse(p0 + t * (p1 - p0), r0 + t * (r1 - r0))
                        if ok:
                            self.motor_ctrl.set_targets(res["step_deltas"])
                        self._set_status(True, "trajectory", prog, f"Quadrado: ciclo {cycle+1}/{cycles} ({prog}%)")
                        time.sleep(0.04)

        elif traj_type in ("4", "figure8", "oito", "lissajous"):
            steps = 80
            cycles = 2
            total_steps = cycles * steps
            step_count = 0
            for cycle in range(cycles):
                for i in range(steps):
                    if self._cancel_event.is_set():
                        return
                    step_count += 1
                    prog = int((step_count / total_steps) * 100)
                    t = (2.0 * math.pi * i) / steps
                    p = r_rad * math.sin(t)
                    r = r_rad * math.sin(2.0 * t) * 0.5
                    ok, res = self.kinematics.compute_inverse(p, r)
                    if ok:
                        self.motor_ctrl.set_targets(res["step_deltas"])
                    self._set_status(True, "trajectory", prog, f"Figura-8: ciclo {cycle+1}/{cycles} ({prog}%)")
                    time.sleep(0.04)

    def _run_diagnostics_test(self, params: Dict[str, Any]):
        motor_choice = params.get("motor", 0)
        motors = [int(motor_choice)] if int(motor_choice) in (1, 2, 3) else [1, 2, 3]

        total = len(motors)
        for idx, m in enumerate(motors):
            if self._cancel_event.is_set():
                return
            prog = int((idx / total) * 100)
            self._set_status(True, "diagnostics", prog, f"Testando Motor {m} (+80 passos)...")
            self.motor_ctrl.jog(m, 80)
            self._wait_idle_or_cancel(timeout=2.0)
            self._sleep_or_cancel(0.2)

            if self._cancel_event.is_set():
                return
            self._set_status(True, "diagnostics", prog + int(50 / total), f"Testando Motor {m} (-80 passos)...")
            self.motor_ctrl.jog(m, -80)
            self._wait_idle_or_cancel(timeout=2.0)
            self._sleep_or_cancel(0.2)

    def _run_repeatability_test(self, params: Dict[str, Any]):
        self._set_status(True, "repeatability", 20, "Afastando +60 passos temporariamente...")
        for m in (1, 2, 3):
            self.motor_ctrl.jog(m, 60)
        self._wait_idle_or_cancel(timeout=2.5)
        self._sleep_or_cancel(0.4)

        if self._cancel_event.is_set():
            return

        self._set_status(True, "repeatability", 60, "Retornando ao ponto neutro calibrado (level)...")
        self.motor_ctrl.level()
        self._wait_idle_or_cancel(timeout=2.5)
        self._sleep_or_cancel(0.2)
