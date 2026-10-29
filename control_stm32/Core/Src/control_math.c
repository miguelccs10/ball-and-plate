/**
  ******************************************************************************
  * @file    control_math.c
  * @brief   Módulo Matemático de Controle: Filtro de Kalman 2D, Dual PID
  *          e Cinemática Inversa Analítica 3-DoF (Weierstrass)
  *          Plataforma Auto Balanceável Ball and Plate (STM32F446RE)
  *          Autor: Miguel Carlos Costa Sousa — UFG
  ******************************************************************************
  */

#include "system_config.h"

/* ========================================================================== */
/* 1. FILTRO DE KALMAN 1D E 2D COM COMPENSAÇÃO DE LATÊNCIA                    */
/* ========================================================================== */
void Kalman1D_Init(Kalman1D_t *kf, float ts, float q_accel, float r_meas) {
    if (!kf) return;
    kf->ts = ts;
    kf->r_meas = r_meas;
    kf->x = 0.0f;
    kf->v = 0.0f;
    kf->p00 = 0.01f;
    kf->p01 = 0.0f;
    kf->p10 = 0.0f;
    kf->p11 = 0.1f;

    float dt = ts;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    kf->q00 = q_accel * (0.25f * dt4);
    kf->q01 = q_accel * (0.50f * dt3);
    kf->q10 = kf->q01;
    kf->q11 = q_accel * dt2;
}

void Kalman1D_Reset(Kalman1D_t *kf, float init_pos, float init_vel) {
    if (!kf) return;
    kf->x = init_pos;
    kf->v = init_vel;
    kf->p00 = 0.005f;
    kf->p01 = 0.0f;
    kf->p10 = 0.0f;
    kf->p11 = 0.05f;
}

void Kalman1D_Predict(Kalman1D_t *kf, float u_accel, float dt) {
    if (!kf) return;
    float dt2_half = 0.5f * dt * dt;
    kf->x += (kf->v * dt) + (u_accel * dt2_half);
    kf->v += u_accel * dt;

    float p00 = kf->p00, p01 = kf->p01, p10 = kf->p10, p11 = kf->p11;
    kf->p00 = p00 + dt * (p10 + p01) + (dt * dt * p11) + kf->q00;
    kf->p01 = p01 + (dt * p11) + kf->q01;
    kf->p10 = p10 + (dt * p11) + kf->q10;
    kf->p11 = p11 + kf->q11;
}

void Kalman1D_Update(Kalman1D_t *kf, float z_meas) {
    if (!kf) return;
    float y = z_meas - kf->x;
    float s = kf->p00 + kf->r_meas;
    if (s < 1e-7f) s = 1e-7f;
    float s_inv = 1.0f / s;

    float k0 = kf->p00 * s_inv;
    float k1 = kf->p10 * s_inv;

    kf->x += k0 * y;
    kf->v += k1 * y;

    float p00 = kf->p00, p01 = kf->p01;
    kf->p00 = (1.0f - k0) * p00;
    kf->p01 = (1.0f - k0) * p01;
    kf->p10 = kf->p10 - (k1 * p00);
    kf->p11 = kf->p11 - (k1 * p01);
}

void Kalman1D_PredictForward(const Kalman1D_t *kf, float delay_sec, float u_accel, float *x_out, float *v_out) {
    if (!kf) return;
    if (delay_sec <= 0.0f) {
        if (x_out) *x_out = kf->x;
        if (v_out) *v_out = kf->v;
        return;
    }
    float dt2_half = 0.5f * delay_sec * delay_sec;
    if (x_out) *x_out = kf->x + (kf->v * delay_sec) + (u_accel * dt2_half);
    if (v_out) *v_out = kf->v + (u_accel * delay_sec);
}

void Kalman2D_Init(Kalman2D_t *kf2d, float ts, float q_accel, float r_meas, float latency_sec) {
    if (!kf2d) return;
    Kalman1D_Init(&kf2d->kf_x, ts, q_accel, r_meas);
    Kalman1D_Init(&kf2d->kf_y, ts, q_accel, r_meas);
    kf2d->latency_sec = latency_sec;
}

void Kalman2D_Predict(Kalman2D_t *kf2d, float u_accel_x, float u_accel_y) {
    if (!kf2d) return;
    Kalman1D_Predict(&kf2d->kf_x, u_accel_x, kf2d->kf_x.ts);
    Kalman1D_Predict(&kf2d->kf_y, u_accel_y, kf2d->kf_y.ts);
}

void Kalman2D_Update(Kalman2D_t *kf2d, float z_x, float z_y) {
    if (!kf2d) return;
    Kalman1D_Update(&kf2d->kf_x, z_x);
    Kalman1D_Update(&kf2d->kf_y, z_y);
}

void Kalman2D_GetCompensatedState(const Kalman2D_t *kf2d, float u_accel_x, float u_accel_y,
                                 float *x_comp, float *vx_comp, float *y_comp, float *vy_comp) {
    if (!kf2d) return;
    Kalman1D_PredictForward(&kf2d->kf_x, kf2d->latency_sec, u_accel_x, x_comp, vx_comp);
    Kalman1D_PredictForward(&kf2d->kf_y, kf2d->latency_sec, u_accel_y, y_comp, vy_comp);
}

/* ========================================================================== */
/* 2. CONTROLADOR PID DISCRETO COM ANTI-WINDUP CONDICIONAL                    */
/* ========================================================================== */
void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float n_filter,
              float out_min, float out_max, float ts) {
    if (!pid) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->n_filter = n_filter;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->ts = ts;
    PID_Reset(pid);
}

void PID_Reset(PIDController_t *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_meas = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->last_output = 0.0f;
    pid->is_saturated = false;
    pid->has_prev_meas = false;
}

float PID_Update(PIDController_t *pid, float measurement, float setpoint) {
    if (!pid) return 0.0f;

    /* Ação reversa física para ball and plate: d²x/dt² = -C_g * u */
    float error = measurement - setpoint;

    if (!pid->has_prev_meas) {
        pid->prev_meas = measurement;
        pid->prev_error = error;
        pid->has_prev_meas = true;
    }

    float p_term = pid->kp * error;

    /* Derivada calculada sobre a medição para eliminar derivative kick */
    float delta_meas = measurement - pid->prev_meas;
    float d_num = (pid->kd * pid->n_filter * delta_meas) + pid->prev_derivative;
    float d_den = 1.0f + (pid->n_filter * pid->ts);
    float d_term = d_num / d_den;
    pid->prev_derivative = d_term;
    pid->prev_error = error;
    pid->prev_meas = measurement;

    /* Anti-windup condicional (clamping) */
    bool worsen_pos = (pid->last_output >= pid->out_max) && (error > 0.0f);
    bool worsen_neg = (pid->last_output <= pid->out_min) && (error < 0.0f);
    if (!worsen_pos && !worsen_neg) {
        pid->integral += error * pid->ts;
    }
    float i_term = pid->ki * pid->integral;

    float u_raw = p_term + i_term + d_term;
    float u_clamped = u_raw;
    if (u_clamped > pid->out_max) u_clamped = pid->out_max;
    if (u_clamped < pid->out_min) u_clamped = pid->out_min;

    pid->is_saturated = (u_clamped != u_raw);
    pid->last_output = u_clamped;
    return u_clamped;
}

/* ========================================================================== */
/* 3. CINEMÁTICA INVERSA 3-DoF ANALÍTICA (WEIERSTRASS)                        */
/* ========================================================================== */
static float s_leg_angles[3] = { 0.0f, 2.0f * (float)M_PI / 3.0f, 4.0f * (float)M_PI / 3.0f };
static float s_neutral_angle_rad = 0.0f;

static bool solve_leg_angle(float rho_p, float z_p, float *theta_out) {
    float delta_rho = rho_p - GEOM_BASE_RADIUS_M;
    float E = -2.0f * GEOM_CRANK_ARM_M * delta_rho;
    float F = -2.0f * GEOM_CRANK_ARM_M * z_p;
    float G = (delta_rho * delta_rho) + (z_p * z_p) + (GEOM_CRANK_ARM_M * GEOM_CRANK_ARM_M) - (GEOM_ROD_LENGTH_M * GEOM_ROD_LENGTH_M);

    float discr = (E * E) + (F * F) - (G * G);
    if (discr < 0.0f) return false;

    float sqrt_discr = sqrtf(discr);
    float denom = G - E;
    if (fabsf(denom) < 1e-7f) return false;

    float t = (-F + sqrt_discr) / denom;
    *theta_out = 2.0f * atanf(t);
    return true;
}

void Kinematics_Init(void) {
    if (!solve_leg_angle(GEOM_PLATE_RADIUS_M, GEOM_NOMINAL_Z0_M, &s_neutral_angle_rad)) {
        s_neutral_angle_rad = 126.73f * ((float)M_PI / 180.0f);
    }
}

void Kinematics_SetFrontMotor(uint8_t front_motor_id) {
    if (front_motor_id == 1) {
        s_leg_angles[0] = 0.0f;
        s_leg_angles[1] = 2.0f * (float)M_PI / 3.0f;
        s_leg_angles[2] = 4.0f * (float)M_PI / 3.0f;
    } else if (front_motor_id == 2) {
        s_leg_angles[0] = 4.0f * (float)M_PI / 3.0f;
        s_leg_angles[1] = 0.0f;
        s_leg_angles[2] = 2.0f * (float)M_PI / 3.0f;
    } else if (front_motor_id == 3) {
        s_leg_angles[0] = 2.0f * (float)M_PI / 3.0f;
        s_leg_angles[1] = 4.0f * (float)M_PI / 3.0f;
        s_leg_angles[2] = 0.0f;
    }
}

bool Kinematics_ComputeInverse(float pitch_rad, float roll_rad, float z0, KinematicsOutput_t *out) {
    if (!out) return false;

    float pitch = pitch_rad;
    float roll = roll_rad;
    if (pitch > MAX_PLATE_ANGLE_RAD) pitch = MAX_PLATE_ANGLE_RAD;
    if (pitch < -MAX_PLATE_ANGLE_RAD) pitch = -MAX_PLATE_ANGLE_RAD;
    if (roll > MAX_PLATE_ANGLE_RAD) roll = MAX_PLATE_ANGLE_RAD;
    if (roll < -MAX_PLATE_ANGLE_RAD) roll = -MAX_PLATE_ANGLE_RAD;

    float cos_a = cosf(pitch), sin_a = sinf(pitch);
    float cos_b = cosf(roll),  sin_b = sinf(roll);
    float max_delta_rad = MAX_MOTOR_EXCURSION_DEG * ((float)M_PI / 180.0f);

    for (int i = 0; i < 3; i++) {
        float phi = s_leg_angles[i];
        float cos_phi = cosf(phi), sin_phi = sinf(phi);

        float Px = GEOM_PLATE_RADIUS_M * (cos_a * cos_phi + sin_a * sin_b * sin_phi);
        float Py = GEOM_PLATE_RADIUS_M * (cos_b * sin_phi);
        float Pz = z0 + GEOM_PLATE_RADIUS_M * (-sin_a * cos_phi + cos_a * sin_b * sin_phi);

        float rho_i = (Px * cos_phi) + (Py * sin_phi);
        float theta_i = 0.0f;
        if (!solve_leg_angle(rho_i, Pz, &theta_i)) return false;

        float delta_theta = theta_i - s_neutral_angle_rad;
        if (delta_theta > max_delta_rad) delta_theta = max_delta_rad;
        if (delta_theta < -max_delta_rad) delta_theta = -max_delta_rad;

        out->motor_angles_rad[i] = s_neutral_angle_rad + delta_theta;
        out->motor_steps[i] = (int32_t)roundf(delta_theta * STEPS_PER_RADIAN);
    }

    out->pitch_rad = pitch;
    out->roll_rad = roll;
    return true;
}
