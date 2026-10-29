/**
  ******************************************************************************
  * @file    system_config.h
  * @brief   Configurações Globais, Estruturas de Dados e Protótipos de Controle
  *          Plataforma Auto Balanceável Ball and Plate 3-DoF (STM32F446RE)
  *          Autor: Miguel Carlos Costa Sousa — UFG
  ******************************************************************************
  */

#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ========================================================================== */
/* CONSTANTES FÍSICAS E TEMPORAIS (EULER-LAGRANGE)                            */
/* ========================================================================== */
#define GRAVITY_MSS                    (9.80665f)
#define CG_ACCEL_CONST                 ((5.0f / 7.0f) * GRAVITY_MSS) /* ~7.00475 m/s² */
#define CONTROL_LOOP_FREQ_HZ           (100.0f)
#define CONTROL_LOOP_TS_SEC            (1.0f / CONTROL_LOOP_FREQ_HZ)
#define CAMERA_LATENCY_DEFAULT_SEC     (0.030f)
#define CONTROL_WATCHDOG_TIMEOUT_TICKS (15U) /* 150 ms @ 100 Hz */

/* ========================================================================== */
/* PARÂMETROS GEOMÉTRICOS DA PLATAFORMA HEXAGONAL 3-DoF (METROS)              */
/* ========================================================================== */
#define GEOM_BASE_RADIUS_M             (0.090f) /* Rb = 9.0 cm */
#define GEOM_PLATE_RADIUS_M            (0.140f) /* Rp = 14.0 cm */
#define GEOM_CRANK_ARM_M               (0.060f) /* r  = 6.0 cm */
#define GEOM_ROD_LENGTH_M              (0.090f) /* L  = 9.0 cm */
#define GEOM_NOMINAL_Z0_M              (0.075f) /* z0 = 7.5 cm */

#define MAX_PLATE_ANGLE_DEG            (15.0f)
#define MAX_PLATE_ANGLE_RAD            (0.261799f)
#define MAX_MOTOR_EXCURSION_DEG        (25.0f)

/* ========================================================================== */
/* ESPECIFICAÇÃO DOS MOTORES NEMA 17 E DRIVERS TMC2209 v2                     */
/* ========================================================================== */
#define MOTOR_FULL_STEPS_PER_REV       (200)
#define TMC2209_MICROSTEPS             (8)
#define TOTAL_STEPS_PER_REV            (MOTOR_FULL_STEPS_PER_REV * TMC2209_MICROSTEPS) /* 1600 */
#define STEPS_PER_RADIAN               ((float)TOTAL_STEPS_PER_REV / (2.0f * (float)M_PI))
#define MAX_STEP_LIMIT                 ((int32_t)((MAX_MOTOR_EXCURSION_DEG * TOTAL_STEPS_PER_REV) / 360.0f))

/* Ganhos padrão do controlador PID */
#define PID_DEFAULT_KP                 (1.85f)
#define PID_DEFAULT_KI                 (0.08f)
#define PID_DEFAULT_KD                 (1.20f)
#define PID_DEFAULT_FILTER_N           (20.0f)

/* Mensagens do Protocolo Serial UART */
#define PROTOCOL_PREAMBLE_1            (0xAA)
#define PROTOCOL_PREAMBLE_2            (0x55)
#define MSG_ID_VISION_COORDS           (0x01)
#define MSG_ID_SETPOINT_CMD            (0x02)
#define MSG_ID_CONTROL_STATE           (0x03)
#define MSG_ID_SYSTEM_COMMAND          (0x04)

#define CMD_ENABLE                     (0x01)
#define CMD_DISABLE                    (0x02)
#define CMD_LEVEL_AND_RESET            (0x03)
#define CMD_SET_FRONT_MOTOR            (0x04)

/* ========================================================================== */
/* ESTRUTURAS DE DADOS: FILTRO DE KALMAN 2D                                   */
/* ========================================================================== */
typedef struct {
    float x;        /* Posição (m) */
    float v;        /* Velocidade (m/s) */
    float p00, p01; /* Matriz de covariância do erro */
    float p10, p11;
    float q00, q01; /* Matriz de covariância do processo */
    float q10, q11;
    float r_meas;   /* Variância do ruído de medição da câmera */
    float ts;       /* Período de amostragem (s) */
} Kalman1D_t;

typedef struct {
    Kalman1D_t kf_x;
    Kalman1D_t kf_y;
    float latency_sec;
} Kalman2D_t;

/* ========================================================================== */
/* ESTRUTURAS DE DADOS: CONTROLADOR PID DISCRETO                              */
/* ========================================================================== */
typedef struct {
    float kp, ki, kd, n_filter;
    float out_min, out_max;
    float ts;
    float integral;
    float prev_error;
    float prev_meas;
    float prev_derivative;
    float last_output;
    bool  is_saturated;
    bool  has_prev_meas;
} PIDController_t;

/* ========================================================================== */
/* ESTRUTURAS DE DADOS: CINEMÁTICA INVERSA 3-DoF                              */
/* ========================================================================== */
typedef struct {
    float pitch_rad;
    float roll_rad;
    float motor_angles_rad[3];
    int32_t motor_steps[3];
} KinematicsOutput_t;

/* ========================================================================== */
/* ESTRUTURAS DE DADOS: ATUADORES DE PASSO TMC2209                            */
/* ========================================================================== */
typedef struct {
    volatile int32_t current_step;
    volatile int32_t target_step;
    float dda_acc;
    uint8_t dir;
} StepperMotor_t;

typedef struct {
    StepperMotor_t motors[3];
    bool is_enabled;
    uint8_t pulse_state;
} StepperSystem_t;

/* ========================================================================== */
/* ESTRUTURAS DE DADOS: PROTOCOLO SERIAL UART                                 */
/* ========================================================================== */
typedef enum {
    PARSER_WAIT_PREAMBLE_1,
    PARSER_WAIT_PREAMBLE_2,
    PARSER_WAIT_MSG_ID,
    PARSER_WAIT_LEN,
    PARSER_PAYLOAD,
    PARSER_WAIT_CRC_H,
    PARSER_WAIT_CRC_L
} ProtocolState_t;

typedef struct {
    ProtocolState_t state;
    uint8_t msg_id;
    uint8_t payload_len;
    uint8_t rx_cnt;
    uint8_t rx_buffer[32];
    uint16_t received_crc;
} ProtocolParser_t;

typedef struct {
    uint32_t timestamp_ms;
    float pos_x_m;
    float pos_y_m;
    bool detected;
    bool is_yolo;
    uint8_t confidence;
} VisionPacket_t;

typedef struct {
    float sp_x_m;
    float sp_y_m;
} SetpointPacket_t;

typedef struct {
    uint8_t cmd_type;
    uint8_t param;
} SystemCmdPacket_t;

typedef enum {
    PACKET_TYPE_NONE = 0,
    PACKET_TYPE_VISION,
    PACKET_TYPE_SETPOINT,
    PACKET_TYPE_SYSTEM_CMD
} PacketType_t;

typedef struct {
    PacketType_t type;
    union {
        VisionPacket_t   vision;
        SetpointPacket_t setpoint;
        SystemCmdPacket_t system_cmd;
    } data;
} RxPacket_t;

/* ========================================================================== */
/* PROTÓTIPOS DAS FUNÇÕES DE CONTROLE MATEMÁTICO                              */
/* ========================================================================== */
void Kalman1D_Init(Kalman1D_t *kf, float ts, float q_accel, float r_meas);
void Kalman1D_Reset(Kalman1D_t *kf, float init_pos, float init_vel);
void Kalman1D_Predict(Kalman1D_t *kf, float u_accel, float dt);
void Kalman1D_Update(Kalman1D_t *kf, float z_meas);
void Kalman1D_PredictForward(const Kalman1D_t *kf, float delay_sec, float u_accel, float *x_out, float *v_out);

void Kalman2D_Init(Kalman2D_t *kf2d, float ts, float q_accel, float r_meas, float latency_sec);
void Kalman2D_Predict(Kalman2D_t *kf2d, float u_accel_x, float u_accel_y);
void Kalman2D_Update(Kalman2D_t *kf2d, float z_x, float z_y);
void Kalman2D_GetCompensatedState(const Kalman2D_t *kf2d, float u_accel_x, float u_accel_y,
                                 float *x_comp, float *vx_comp, float *y_comp, float *vy_comp);

void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float n_filter,
              float out_min, float out_max, float ts);
void PID_Reset(PIDController_t *pid);
float PID_Update(PIDController_t *pid, float measurement, float setpoint);

void Kinematics_Init(void);
void Kinematics_SetFrontMotor(uint8_t front_motor_id);
bool Kinematics_ComputeInverse(float pitch_rad, float roll_rad, float z0, KinematicsOutput_t *out);

/* ========================================================================== */
/* PROTÓTIPOS DAS FUNÇÕES DE ATUAÇÃO E PROTOCOLO                              */
/* ========================================================================== */
void Stepper_Init(StepperSystem_t *sys);
void Stepper_EnableAll(StepperSystem_t *sys, bool enable);
void Stepper_SetAllTargets(StepperSystem_t *sys, const int32_t targets[3]);
void Stepper_StepInterruptHandler(StepperSystem_t *sys);

void Protocol_Init(ProtocolParser_t *parser);
bool Protocol_ProcessByteEx(ProtocolParser_t *parser, uint8_t byte, RxPacket_t *out_pkt);
uint16_t Protocol_CRC16(const uint8_t *data, uint16_t len);

void Control_System_Init(void);
void Control_UART_OnByteReceived(uint8_t byte);
void Control_100Hz_TimerCallback(void);
void Control_Stepper_TimerCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_CONFIG_H */
