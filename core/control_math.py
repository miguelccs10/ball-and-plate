#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Matemática de Controle e Cinemática Inversa 3-DoF (core/control_math.py)
Filtro de Kalman 2D, Dual PID com Anti-Windup e Cinemática Hexagonal de Weierstrass
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import math
from typing import Tuple, Optional, Dict, Any


# ==============================================================================
# 1. FILTRO DE KALMAN DISCRETO 1D E 2D COM COMPENSAÇÃO DE LATÊNCIA
# ==============================================================================
class DiscreteKalman1D:
    """Filtro de Kalman linear para um eixo de aceleração controlada."""

    def __init__(self, ts: float = 0.033, q_accel: float = 0.05, r_meas: float = 2.25e-6):
        self.ts = float(ts)
        self.q_accel = float(q_accel)
        self.r_meas = float(r_meas)

        self.x = 0.0
        self.v = 0.0
        self.p00 = 0.01
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 0.1
        self._update_q_matrix()

    def _update_q_matrix(self):
        dt = self.ts
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        self.q00 = self.q_accel * (0.25 * dt4)
        self.q01 = self.q_accel * (0.50 * dt3)
        self.q10 = self.q01
        self.q11 = self.q_accel * dt2

    def set_parameters(self, ts: float = None, q_accel: float = None, r_meas: float = None):
        if ts is not None and ts > 1e-4:
            self.ts = float(ts)
        if q_accel is not None and q_accel >= 0.0:
            self.q_accel = float(q_accel)
        if r_meas is not None and r_meas > 1e-9:
            self.r_meas = float(r_meas)
        self._update_q_matrix()

    def reset(self, init_pos: float = 0.0, init_vel: float = 0.0):
        self.x = float(init_pos)
        self.v = float(init_vel)
        self.p00 = 0.005
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 0.05

    def predict(self, control_accel: float = 0.0, dt: float = None):
        dt_val = self.ts if dt is None else float(dt)
        dt2_half = 0.5 * dt_val * dt_val

        self.x += (self.v * dt_val) + (control_accel * dt2_half)
        self.v += control_accel * dt_val

        p00_prev, p01_prev = self.p00, self.p01
        p10_prev, p11_prev = self.p10, self.p11

        self.p00 = p00_prev + dt_val * (p10_prev + p01_prev) + (dt_val * dt_val * p11_prev) + self.q00
        self.p01 = p01_prev + (dt_val * p11_prev) + self.q01
        self.p10 = p10_prev + (dt_val * p11_prev) + self.q10
        self.p11 = p11_prev + self.q11

    def update(self, measurement_pos: float):
        y = measurement_pos - self.x
        s = max(1e-7, self.p00 + self.r_meas)
        s_inv = 1.0 / s

        k0 = self.p00 * s_inv
        k1 = self.p10 * s_inv

        self.x += k0 * y
        self.v += k1 * y

        p00_temp = (1.0 - k0) * self.p00
        p01_temp = (1.0 - k0) * self.p01
        p10_temp = self.p10 - (k1 * self.p00)
        p11_temp = self.p11 - (k1 * self.p01)

        self.p00, self.p01 = p00_temp, p01_temp
        self.p10, self.p11 = p10_temp, p11_temp

    def predict_forward(self, delay_sec: float, control_accel: float = 0.0) -> Tuple[float, float]:
        if delay_sec <= 0.0:
            return self.x, self.v
        dt2_half = 0.5 * delay_sec * delay_sec
        x_comp = self.x + (self.v * delay_sec) + (control_accel * dt2_half)
        v_comp = self.v + (control_accel * delay_sec)
        return x_comp, v_comp


class DiscreteKalman2D:
    """Filtro de Kalman 2D para a esfera com compensação de atraso visual de 30 ms."""

    CG_ACCEL = (5.0 / 7.0) * 9.80665  # ~7.00475 m/s²

    def __init__(self, ts: float = 0.033, q_accel: float = 0.05, r_meas: float = 2.25e-6, latency_sec: float = 0.030):
        self.kf_x = DiscreteKalman1D(ts, q_accel, r_meas)
        self.kf_y = DiscreteKalman1D(ts, q_accel, r_meas)
        self.latency_sec = float(latency_sec)

    def set_parameters(self, q_accel: float = None, r_meas: float = None, latency_sec: float = None, ts: float = None):
        if latency_sec is not None:
            self.latency_sec = max(0.0, float(latency_sec))
        self.kf_x.set_parameters(ts=ts, q_accel=q_accel, r_meas=r_meas)
        self.kf_y.set_parameters(ts=ts, q_accel=q_accel, r_meas=r_meas)

    def reset(self):
        self.kf_x.reset()
        self.kf_y.reset()

    def predict(self, pitch_rad: float, roll_rad: float, dt: float = None):
        u_accel_x = -self.CG_ACCEL * pitch_rad
        u_accel_y = -self.CG_ACCEL * roll_rad
        self.kf_x.predict(u_accel_x, dt=dt)
        self.kf_y.predict(u_accel_y, dt=dt)

    def update(self, meas_x_m: float, meas_y_m: float):
        self.kf_x.update(meas_x_m)
        self.kf_y.update(meas_y_m)

    def get_filtered_state(self) -> Tuple[float, float, float, float]:
        return self.kf_x.x, self.kf_x.v, self.kf_y.x, self.kf_y.v

    def get_compensated_state(self, pitch_rad: float, roll_rad: float) -> Tuple[float, float, float, float]:
        u_accel_x = -self.CG_ACCEL * pitch_rad
        u_accel_y = -self.CG_ACCEL * roll_rad
        x_c, vx_c = self.kf_x.predict_forward(self.latency_sec, u_accel_x)
        y_c, vy_c = self.kf_y.predict_forward(self.latency_sec, u_accel_y)
        return x_c, vx_c, y_c, vy_c


# ==============================================================================
# 2. CONTROLADOR PID DISCRETO COM ANTI-WINDUP CONDICIONAL
# ==============================================================================
class DiscretePID:
    """Controlador PID Discreto com filtragem passa-baixa e eliminação de derivative kick."""

    def __init__(self, kp: float, ki: float, kd: float, n_filter: float = 20.0,
                 out_min: float = -0.2618, out_max: float = 0.2618, ts: float = 0.033,
                 reverse_acting: bool = True, derivative_on_measurement: bool = True):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.n_filter = float(n_filter)
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.ts = float(ts)
        self.reverse_acting = bool(reverse_acting)
        self.derivative_on_measurement = bool(derivative_on_measurement)

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_derivative = 0.0
        self.last_output = 0.0
        self.is_saturated = False
        self._has_prev_measurement = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_derivative = 0.0
        self.last_output = 0.0
        self.is_saturated = False
        self._has_prev_measurement = False

    def set_gains(self, kp: float = None, ki: float = None, kd: float = None, n_filter: float = None):
        if kp is not None: self.kp = float(kp)
        if ki is not None: self.ki = float(ki)
        if kd is not None: self.kd = float(kd)
        if n_filter is not None and n_filter > 0.1: self.n_filter = float(n_filter)

    def set_limits(self, out_min: float, out_max: float):
        self.out_min = float(out_min)
        self.out_max = float(out_max)

    def update(self, measurement: float, setpoint: float = 0.0, dt: float = None) -> float:
        dt_val = self.ts if dt is None else max(1e-4, float(dt))

        if self.reverse_acting:
            error = measurement - setpoint
        else:
            error = setpoint - measurement

        if not self._has_prev_measurement:
            self.prev_measurement = measurement
            self.prev_error = error
            self._has_prev_measurement = True

        p_term = self.kp * error

        if self.derivative_on_measurement:
            delta_meas = measurement - self.prev_measurement
            d_input_delta = delta_meas if self.reverse_acting else -delta_meas
        else:
            d_input_delta = error - self.prev_error

        d_num = (self.kd * self.n_filter * d_input_delta) + self.prev_derivative
        d_den = 1.0 + (self.n_filter * dt_val)
        d_term = d_num / d_den
        self.prev_derivative = d_term
        self.prev_error = error
        self.prev_measurement = measurement

        would_worsen_pos = (self.last_output >= self.out_max) and (error > 0.0)
        would_worsen_neg = (self.last_output <= self.out_min) and (error < 0.0)

        if not would_worsen_pos and not would_worsen_neg:
            self.integral += error * dt_val

        i_term = self.ki * self.integral

        u_raw = p_term + i_term + d_term
        u_clamped = max(self.out_min, min(self.out_max, u_raw))
        self.is_saturated = (u_clamped != u_raw)
        self.last_output = u_clamped

        return u_clamped


# ==============================================================================
# 3. CINEMÁTICA INVERSA 3-DoF ANALÍTICA (SUBSTITUIÇÃO DE WEIERSTRASS)
# ==============================================================================
class HexagonalKinematics3DoF:
    """Cinemática inversa analítica com 3 atuadores simétricos a 120°."""

    def __init__(self,
                 base_radius=0.090,
                 plate_radius=0.140,
                 crank_arm=0.060,
                 rod_length=0.090,
                 nominal_z0=0.075,
                 steps_per_rev=1600,
                 front_motor_id=1):
        self.Rb = base_radius
        self.Rp = plate_radius
        self.r = crank_arm
        self.L = rod_length
        self.z0 = nominal_z0
        self.steps_per_rev = steps_per_rev
        self.steps_per_rad = steps_per_rev / (2.0 * math.pi)
        self.front_motor_id = int(front_motor_id)

        self.set_front_motor(self.front_motor_id)
        self.neutral_angle_rad = self._solve_leg_angle(self.Rp, self.z0)
        if self.neutral_angle_rad is None:
            self.neutral_angle_rad = math.radians(126.73)

    def set_front_motor(self, front_motor_id: int):
        self.front_motor_id = int(front_motor_id)
        if self.front_motor_id == 1:
            self.leg_angles = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]
        elif self.front_motor_id == 2:
            self.leg_angles = [4.0 * math.pi / 3.0, 0.0, 2.0 * math.pi / 3.0]
        elif self.front_motor_id == 3:
            self.leg_angles = [2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0, 0.0]
        else:
            self.leg_angles = [0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0]

    def set_steps_per_rev(self, spr: int):
        self.steps_per_rev = int(spr)
        self.steps_per_rad = self.steps_per_rev / (2.0 * math.pi)

    def _solve_leg_angle(self, rho_p: float, z_p: float) -> Optional[float]:
        delta_rho = rho_p - self.Rb
        E = -2.0 * self.r * delta_rho
        F = -2.0 * self.r * z_p
        G = (delta_rho ** 2) + (z_p ** 2) + (self.r ** 2) - (self.L ** 2)

        discr = (E ** 2) + (F ** 2) - (G ** 2)
        if discr < 0.0:
            return None

        sqrt_discr = math.sqrt(discr)
        denom = G - E
        if abs(denom) < 1e-7:
            return None

        t = (-F + sqrt_discr) / denom
        return 2.0 * math.atan(t)

    def compute_inverse(self, pitch_rad: float, roll_rad: float, z0: float = None) -> Tuple[bool, Optional[Dict[str, Any]]]:
        if z0 is None:
            z0 = self.z0

        max_angle = math.radians(15.0)
        pitch = max(-max_angle, min(max_angle, pitch_rad))
        roll = max(-max_angle, min(max_angle, roll_rad))

        cos_a = math.cos(pitch)
        sin_a = math.sin(pitch)
        cos_b = math.cos(roll)
        sin_b = math.sin(roll)

        motor_angles_rad = []
        motor_angles_deg = []
        motor_step_deltas = []

        for phi in self.leg_angles:
            cos_phi = math.cos(phi)
            sin_phi = math.sin(phi)

            Px = self.Rp * (cos_a * cos_phi + sin_a * sin_b * sin_phi)
            Py = self.Rp * (cos_b * sin_phi)
            Pz = z0 + self.Rp * (-sin_a * cos_phi + cos_a * sin_b * sin_phi)

            rho_i = (Px * cos_phi) + (Py * sin_phi)
            z_i = Pz

            theta = self._solve_leg_angle(rho_i, z_i)
            if theta is None:
                return False, None

            delta_theta = theta - self.neutral_angle_rad
            max_delta = math.radians(25.0)
            delta_theta = max(-max_delta, min(max_delta, delta_theta))
            theta = self.neutral_angle_rad + delta_theta

            steps = int(round(delta_theta * self.steps_per_rad))

            motor_angles_rad.append(theta)
            motor_angles_deg.append(math.degrees(theta))
            motor_step_deltas.append(steps)

        return True, {
            "pitch_rad": pitch,
            "roll_rad": roll,
            "pitch_deg": math.degrees(pitch),
            "roll_deg": math.degrees(roll),
            "angles_rad": motor_angles_rad,
            "angles_deg": motor_angles_deg,
            "step_deltas": motor_step_deltas,
            "neutral_angle_deg": math.degrees(self.neutral_angle_rad)
        }
