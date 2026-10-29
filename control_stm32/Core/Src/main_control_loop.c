/**
  ******************************************************************************
  * @file    main_control_loop.c
  * @brief   Módulo Central de Execução Real-Time (100 Hz), Protocolo UART
  *          com CRC16 e Acionamento Determinístico TMC2209 (TIM3 2 kHz)
  *          Plataforma Auto Balanceável Ball and Plate (STM32F446RE)
  *          Autor: Miguel Carlos Costa Sousa — UFG
  ******************************************************************************
  */

#include "system_config.h"

/* Seções críticas portáteis entre ARM Cortex-M e Ambiente de Testes Host */
#ifdef COMPILE_HOST_TEST
#define ENTER_CRITICAL()
#define EXIT_CRITICAL()
#else
#define ENTER_CRITICAL()    uint32_t primask = __get_PRIMASK(); __disable_irq()
#define EXIT_CRITICAL()     __set_PRIMASK(primask)
#endif

/* ========================================================================== */
/* 1. PROTOCOLO SERIAL UART BINÁRIO (CRC16-CCITT)                             */
/* ========================================================================== */
uint16_t Protocol_CRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void Protocol_Init(ProtocolParser_t *parser) {
    if (!parser) return;
    parser->state = PARSER_WAIT_PREAMBLE_1;
    parser->msg_id = 0;
    parser->payload_len = 0;
    parser->rx_cnt = 0;
    parser->received_crc = 0;
}

bool Protocol_ProcessByteEx(ProtocolParser_t *parser, uint8_t byte, RxPacket_t *out_pkt) {
    if (!parser || !out_pkt) return false;
    out_pkt->type = PACKET_TYPE_NONE;

    switch (parser->state) {
        case PARSER_WAIT_PREAMBLE_1:
            if (byte == PROTOCOL_PREAMBLE_1) parser->state = PARSER_WAIT_PREAMBLE_2;
            break;
        case PARSER_WAIT_PREAMBLE_2:
            parser->state = (byte == PROTOCOL_PREAMBLE_2) ? PARSER_WAIT_MSG_ID : PARSER_WAIT_PREAMBLE_1;
            break;
        case PARSER_WAIT_MSG_ID:
            parser->msg_id = byte;
            parser->state = PARSER_WAIT_LEN;
            break;
        case PARSER_WAIT_LEN:
            parser->payload_len = byte;
            parser->rx_cnt = 0;
            parser->state = (parser->payload_len > 0 && parser->payload_len <= sizeof(parser->rx_buffer))
                            ? PARSER_PAYLOAD : PARSER_WAIT_PREAMBLE_1;
            break;
        case PARSER_PAYLOAD:
            parser->rx_buffer[parser->rx_cnt++] = byte;
            if (parser->rx_cnt >= parser->payload_len) {
                parser->state = PARSER_WAIT_CRC_H;
            }
            break;
        case PARSER_WAIT_CRC_H:
            parser->received_crc = (uint16_t)((uint16_t)byte << 8);
            parser->state = PARSER_WAIT_CRC_L;
            break;
        case PARSER_WAIT_CRC_L:
            parser->received_crc |= byte;
            parser->state = PARSER_WAIT_PREAMBLE_1;

            /* Validação do CRC16 */
            uint8_t crc_buf[36];
            crc_buf[0] = parser->msg_id;
            crc_buf[1] = parser->payload_len;
            memcpy(&crc_buf[2], parser->rx_buffer, parser->payload_len);
            uint16_t computed_crc = Protocol_CRC16(crc_buf, (uint16_t)(parser->payload_len + 2));

            if (computed_crc == parser->received_crc) {
                if (parser->msg_id == MSG_ID_VISION_COORDS && parser->payload_len >= 10) {
                    out_pkt->type = PACKET_TYPE_VISION;
                    out_pkt->data.vision.timestamp_ms = (uint32_t)parser->rx_buffer[0] |
                                                        ((uint32_t)parser->rx_buffer[1] << 8) |
                                                        ((uint32_t)parser->rx_buffer[2] << 16) |
                                                        ((uint32_t)parser->rx_buffer[3] << 24);
                    int16_t raw_x = (int16_t)((uint16_t)parser->rx_buffer[4] | ((uint16_t)parser->rx_buffer[5] << 8));
                    int16_t raw_y = (int16_t)((uint16_t)parser->rx_buffer[6] | ((uint16_t)parser->rx_buffer[7] << 8));
                    out_pkt->data.vision.pos_x_m = (float)raw_x * 0.0001f;
                    out_pkt->data.vision.pos_y_m = (float)raw_y * 0.0001f;
                    uint8_t flags = parser->rx_buffer[8];
                    out_pkt->data.vision.detected = (flags & 0x01) != 0;
                    out_pkt->data.vision.is_yolo   = (flags & 0x02) != 0;
                    out_pkt->data.vision.confidence = parser->rx_buffer[9];
                    return true;
                } else if (parser->msg_id == MSG_ID_SETPOINT_CMD && parser->payload_len >= 4) {
                    out_pkt->type = PACKET_TYPE_SETPOINT;
                    int16_t sp_x_raw = (int16_t)((uint16_t)parser->rx_buffer[0] | ((uint16_t)parser->rx_buffer[1] << 8));
                    int16_t sp_y_raw = (int16_t)((uint16_t)parser->rx_buffer[2] | ((uint16_t)parser->rx_buffer[3] << 8));
                    out_pkt->data.setpoint.sp_x_m = (float)sp_x_raw * 0.0001f;
                    out_pkt->data.setpoint.sp_y_m = (float)sp_y_raw * 0.0001f;
                    return true;
                } else if (parser->msg_id == MSG_ID_SYSTEM_COMMAND && parser->payload_len >= 2) {
                    out_pkt->type = PACKET_TYPE_SYSTEM_CMD;
                    out_pkt->data.system_cmd.cmd_type = parser->rx_buffer[0];
                    out_pkt->data.system_cmd.param    = parser->rx_buffer[1];
                    return true;
                }
            }
            break;
    }
    return false;
}

/* ========================================================================== */
/* 2. DRIVER DE MOTORES TMC2209 E ATUAÇÃO VIA REGISTRADORES BSRR             */
/* ========================================================================== */
void Stepper_Init(StepperSystem_t *sys) {
    if (!sys) return;
    for (int i = 0; i < 3; i++) {
        sys->motors[i].current_step = 0;
        sys->motors[i].target_step  = 0;
        sys->motors[i].dda_acc      = 0.0f;
        sys->motors[i].dir          = 0;
    }
    sys->is_enabled = false;
    sys->pulse_state = 0;
}

void Stepper_EnableAll(StepperSystem_t *sys, bool enable) {
    if (!sys) return;
    sys->is_enabled = enable;
}

void Stepper_SetAllTargets(StepperSystem_t *sys, const int32_t targets[3]) {
    if (!sys || !targets) return;
    ENTER_CRITICAL();
    for (int i = 0; i < 3; i++) {
        int32_t t = targets[i];
        if (t > MAX_STEP_LIMIT) t = MAX_STEP_LIMIT;
        if (t < -MAX_STEP_LIMIT) t = -MAX_STEP_LIMIT;
        sys->motors[i].target_step = t;
    }
    EXIT_CRITICAL();
}

void Stepper_StepInterruptHandler(StepperSystem_t *sys) {
    if (!sys || !sys->is_enabled) return;

    if (sys->pulse_state == 1) {
        sys->pulse_state = 0;
        return;
    }

    int32_t diffs[3];
    int32_t max_diff = 0;
    for (int i = 0; i < 3; i++) {
        diffs[i] = sys->motors[i].target_step - sys->motors[i].current_step;
        int32_t abs_d = (diffs[i] >= 0) ? diffs[i] : -diffs[i];
        if (abs_d > max_diff) max_diff = abs_d;
    }

    if (max_diff == 0) {
        for (int i = 0; i < 3; i++) sys->motors[i].dda_acc = 0.0f;
        return;
    }

    bool step_needed[3] = {false, false, false};
    for (int i = 0; i < 3; i++) {
        if (diffs[i] == 0) continue;
        int32_t abs_d = (diffs[i] >= 0) ? diffs[i] : -diffs[i];
        sys->motors[i].dda_acc += (float)abs_d;
        if (sys->motors[i].dda_acc >= (float)max_diff) {
            sys->motors[i].dda_acc -= (float)max_diff;
            int32_t dir_sign = (diffs[i] > 0) ? 1 : -1;
            sys->motors[i].current_step += dir_sign;
            sys->motors[i].dir = (dir_sign > 0) ? 1 : 0;
            step_needed[i] = true;
        }
    }

    if (step_needed[0] || step_needed[1] || step_needed[2]) {
        sys->pulse_state = 1;
    }
}

/* ========================================================================== */
/* 3. LOOP DETERMINÍSTICO DE CONTROLE (100 Hz) E DEFERRED UART PROCESSING    */
/* ========================================================================== */
static Kalman2D_t        s_kalman;
static PIDController_t   s_pid_x;
static PIDController_t   s_pid_y;
static StepperSystem_t   s_stepper;
static ProtocolParser_t  s_parser;
static KinematicsOutput_t s_kin_out;

static float s_setpoint_x_m = 0.0f;
static float s_setpoint_y_m = 0.0f;
static float s_cmd_pitch_rad = 0.0f;
static float s_cmd_roll_rad  = 0.0f;
static uint32_t s_ticks_since_detection = CONTROL_WATCHDOG_TIMEOUT_TICKS + 1U;

/* Mailboxes protegidas da UART */
static volatile bool s_has_new_measurement = false;
static VisionPacket_t s_last_packet;

static volatile bool s_has_new_setpoint = false;
static SetpointPacket_t s_pending_setpoint;

static volatile bool s_has_new_syscmd = false;
static SystemCmdPacket_t s_pending_syscmd;

void Control_System_Init(void) {
    Kinematics_Init();
    Stepper_Init(&s_stepper);
    Stepper_EnableAll(&s_stepper, true);
    Protocol_Init(&s_parser);
    Kalman2D_Init(&s_kalman, CONTROL_LOOP_TS_SEC, 0.05f, 2.25e-6f, CAMERA_LATENCY_DEFAULT_SEC);
    float max_tilt_rad = MAX_PLATE_ANGLE_DEG * ((float)M_PI / 180.0f);
    PID_Init(&s_pid_x, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD, PID_DEFAULT_FILTER_N,
             -max_tilt_rad, max_tilt_rad, CONTROL_LOOP_TS_SEC);
    PID_Init(&s_pid_y, PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD, PID_DEFAULT_FILTER_N,
             -max_tilt_rad, max_tilt_rad, CONTROL_LOOP_TS_SEC);
}

void Control_Stepper_TimerCallback(void) {
    Stepper_StepInterruptHandler(&s_stepper);
}

void Control_UART_OnByteReceived(uint8_t byte) {
    RxPacket_t rx_pkt;
    if (Protocol_ProcessByteEx(&s_parser, byte, &rx_pkt)) {
        ENTER_CRITICAL();
        if (rx_pkt.type == PACKET_TYPE_VISION) {
            s_last_packet = rx_pkt.data.vision;
            s_has_new_measurement = true;
        } else if (rx_pkt.type == PACKET_TYPE_SETPOINT) {
            s_pending_setpoint = rx_pkt.data.setpoint;
            s_has_new_setpoint = true;
        } else if (rx_pkt.type == PACKET_TYPE_SYSTEM_CMD) {
            s_pending_syscmd = rx_pkt.data.system_cmd;
            s_has_new_syscmd = true;
        }
        EXIT_CRITICAL();
    }
}

void Control_100Hz_TimerCallback(void) {
    VisionPacket_t pkt = {0};
    SetpointPacket_t sp_pkt = {0};
    SystemCmdPacket_t cmd_pkt = {0};
    bool has_new_vision = false;
    bool has_new_sp = false;
    bool has_new_cmd = false;

    ENTER_CRITICAL();
    if (s_has_new_measurement) {
        pkt = s_last_packet;
        s_has_new_measurement = false;
        has_new_vision = true;
    }
    if (s_has_new_setpoint) {
        sp_pkt = s_pending_setpoint;
        s_has_new_setpoint = false;
        has_new_sp = true;
    }
    if (s_has_new_syscmd) {
        cmd_pkt = s_pending_syscmd;
        s_has_new_syscmd = false;
        has_new_cmd = true;
    }
    EXIT_CRITICAL();

    if (has_new_sp) {
        s_setpoint_x_m = sp_pkt.sp_x_m;
        s_setpoint_y_m = sp_pkt.sp_y_m;
    }

    if (has_new_cmd) {
        if (cmd_pkt.cmd_type == CMD_ENABLE) {
            Stepper_EnableAll(&s_stepper, true);
        } else if (cmd_pkt.cmd_type == CMD_DISABLE) {
            Stepper_EnableAll(&s_stepper, false);
        } else if (cmd_pkt.cmd_type == CMD_LEVEL_AND_RESET) {
            s_cmd_pitch_rad = 0.0f;
            s_cmd_roll_rad  = 0.0f;
            PID_Reset(&s_pid_x);
            PID_Reset(&s_pid_y);
            Kalman1D_Reset(&s_kalman.kf_x, 0.0f, 0.0f);
            Kalman1D_Reset(&s_kalman.kf_y, 0.0f, 0.0f);
            s_ticks_since_detection = CONTROL_WATCHDOG_TIMEOUT_TICKS + 1U;
            if (Kinematics_ComputeInverse(0.0f, 0.0f, GEOM_NOMINAL_Z0_M, &s_kin_out)) {
                Stepper_SetAllTargets(&s_stepper, s_kin_out.motor_steps);
            }
            return;
        } else if (cmd_pkt.cmd_type == CMD_SET_FRONT_MOTOR) {
            Kinematics_SetFrontMotor(cmd_pkt.param);
        }
    }

    if (has_new_vision && pkt.detected) {
        s_ticks_since_detection = 0U;
    } else if (s_ticks_since_detection <= CONTROL_WATCHDOG_TIMEOUT_TICKS) {
        s_ticks_since_detection++;
    }

    if (s_ticks_since_detection > CONTROL_WATCHDOG_TIMEOUT_TICKS) {
        s_cmd_pitch_rad = 0.0f;
        s_cmd_roll_rad  = 0.0f;
        PID_Reset(&s_pid_x);
        PID_Reset(&s_pid_y);
        Kalman1D_Reset(&s_kalman.kf_x, 0.0f, 0.0f);
        Kalman1D_Reset(&s_kalman.kf_y, 0.0f, 0.0f);
        if (Kinematics_ComputeInverse(0.0f, 0.0f, GEOM_NOMINAL_Z0_M, &s_kin_out)) {
            Stepper_SetAllTargets(&s_stepper, s_kin_out.motor_steps);
        }
        return;
    }

    float u_accel_x = -CG_ACCEL_CONST * s_cmd_pitch_rad;
    float u_accel_y = -CG_ACCEL_CONST * s_cmd_roll_rad;

    Kalman2D_Predict(&s_kalman, u_accel_x, u_accel_y);
    if (has_new_vision && pkt.detected) {
        Kalman2D_Update(&s_kalman, pkt.pos_x_m, pkt.pos_y_m);
    }

    float x_comp = 0.0f, vx_comp = 0.0f, y_comp = 0.0f, vy_comp = 0.0f;
    Kalman2D_GetCompensatedState(&s_kalman, u_accel_x, u_accel_y, &x_comp, &vx_comp, &y_comp, &vy_comp);

    s_cmd_pitch_rad = PID_Update(&s_pid_x, x_comp, s_setpoint_x_m);
    s_cmd_roll_rad  = PID_Update(&s_pid_y, y_comp, s_setpoint_y_m);

    if (Kinematics_ComputeInverse(s_cmd_pitch_rad, s_cmd_roll_rad, GEOM_NOMINAL_Z0_M, &s_kin_out)) {
        Stepper_SetAllTargets(&s_stepper, s_kin_out.motor_steps);
    }
}

/* ========================================================================== */
/* 4. TESTE UNITÁRIO HOST (COMPILE_HOST_TEST)                                 */
/* ========================================================================== */
#ifdef COMPILE_HOST_TEST
#include <stdio.h>
#include <assert.h>

static void inject_packet(const uint8_t *payload, uint8_t msg_id, uint8_t len) {
    uint8_t buffer[64];
    buffer[0] = PROTOCOL_PREAMBLE_1;
    buffer[1] = PROTOCOL_PREAMBLE_2;
    buffer[2] = msg_id;
    buffer[3] = len;
    memcpy(&buffer[4], payload, len);
    uint16_t crc = Protocol_CRC16(&buffer[2], (uint16_t)(len + 2));
    buffer[4 + len] = (uint8_t)(crc >> 8);
    buffer[5 + len] = (uint8_t)(crc & 0xFF);

    for (uint8_t i = 0; i < (len + 6); i++) {
        Control_UART_OnByteReceived(buffer[i]);
    }
}

int main(void) {
    printf("===============================================================\n");
    printf("TESTE UNITARIO DO MODULO DE CONTROLE HARD REAL-TIME (STM32 C)\n");
    printf("===============================================================\n");

    Control_System_Init();

    KinematicsOutput_t init_kin;
    bool init_ok = Kinematics_ComputeInverse(0.0f, 0.0f, GEOM_NOMINAL_Z0_M, &init_kin);
    assert(init_ok);
    float neutral_deg = init_kin.motor_angles_rad[0] * (180.0f / (float)M_PI);
    printf("[TEST 1] Angulo neutro nominal: %.2f graus\n", neutral_deg);
    assert(fabsf(neutral_deg - 126.73f) < 0.1f);

    uint8_t vision_payload[10] = {
        0x10, 0x27, 0x00, 0x00,
        0x2C, 0x01,             /* X = 300 decimos = 30.0 mm */
        0x00, 0x00,
        0x01,                   /* flags: detected=true */
        95
    };
    inject_packet(vision_payload, MSG_ID_VISION_COORDS, 10);

    for (int step = 0; step < 10; step++) {
        Control_100Hz_TimerCallback();
    }
    printf("[TEST 2] Pitch Cmd: %+.4f rad (%+.2f deg) | Passos M1: %d | M2: %d | M3: %d\n",
           s_cmd_pitch_rad, s_cmd_pitch_rad * (180.0f / (float)M_PI),
           s_kin_out.motor_steps[0], s_kin_out.motor_steps[1], s_kin_out.motor_steps[2]);
    assert(s_cmd_pitch_rad > 0.01f);
    assert(s_kin_out.motor_steps[0] > 0);

    printf("[TEST 3] Testando gerador de pulsos de passo (TIM3)...\n");
    int32_t target_m1 = s_stepper.motors[0].target_step;
    for (int t = 0; t < 100; t++) {
        Control_Stepper_TimerCallback();
    }
    printf("  Motor 1 Current Step: %d (Target: %d)\n", s_stepper.motors[0].current_step, target_m1);
    assert(s_stepper.motors[0].current_step == target_m1);

    printf("[TEST 4] Testando fail-safe watchdog (20 ciclos sem deteccao)...\n");
    for (int step = 0; step < 20; step++) {
        Control_100Hz_TimerCallback();
    }
    printf("  Apos watchdog timeout: Pitch: %.4f | Roll: %.4f | M1 Target: %d\n",
           s_cmd_pitch_rad, s_cmd_roll_rad, s_stepper.motors[0].target_step);
    assert(fabsf(s_cmd_pitch_rad) < 1e-5f);
    assert(fabsf(s_cmd_roll_rad) < 1e-5f);
    assert(s_stepper.motors[0].target_step == 0);

    printf("[TEST 5] Testando recepcao de MSG_ID_SETPOINT_CMD...\n");
    int16_t sp_x_raw = -250;
    int16_t sp_y_raw = 150;
    uint8_t sp_payload[4] = {
        (uint8_t)(sp_x_raw & 0xFF), (uint8_t)((sp_x_raw >> 8) & 0xFF),
        (uint8_t)(sp_y_raw & 0xFF), (uint8_t)((sp_y_raw >> 8) & 0xFF)
    };
    inject_packet(sp_payload, MSG_ID_SETPOINT_CMD, 4);
    Control_100Hz_TimerCallback();
    assert(fabsf(s_setpoint_x_m - (-0.025f)) < 1e-5f);
    assert(fabsf(s_setpoint_y_m - (0.015f)) < 1e-5f);
    printf("  Setpoints decodificados com sucesso: X=%.3f m, Y=%.3f m\n", s_setpoint_x_m, s_setpoint_y_m);

    printf("[TEST 6] Testando comando de sistema CMD_SET_FRONT_MOTOR...\n");
    uint8_t sys_payload[2] = { CMD_SET_FRONT_MOTOR, 3 };
    inject_packet(sys_payload, MSG_ID_SYSTEM_COMMAND, 2);
    Control_100Hz_TimerCallback();
    KinematicsOutput_t test_kin;
    bool kin_ok = Kinematics_ComputeInverse(0.05f, 0.0f, GEOM_NOMINAL_Z0_M, &test_kin);
    assert(kin_ok);
    printf("  Cinematica recalculada com Motor 3 na frente: M1=%d, M2=%d, M3=%d\n",
           test_kin.motor_steps[0], test_kin.motor_steps[1], test_kin.motor_steps[2]);

    printf("\n>>> TODOS OS TESTES UNITARIOS EM C (STM32) PASSARAM COM SUCESSO! <<<\n");
    return 0;
}
#endif
