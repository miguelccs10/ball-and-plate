#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
Configurações Globais do Sistema Ball and Plate 3-DoF
Constantes Físicas, Hardware, Visão e Parâmetros de Malha Fechada
Autor: Miguel Carlos Costa Sousa — UFG
==============================================================================
"""

import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# ==============================================================================
# MODO OPERACIONAL DO SISTEMA:
#   - 'STANDALONE_RPI': Raspberry Pi 3 executa visão + controle + acionamento pigpio
#   - 'HYBRID_STM32': Raspberry Pi 3 executa visão + telemetria e envia UART para STM32F446RE
# ==============================================================================
DEFAULT_OP_MODE = os.environ.get("OP_MODE", "STANDALONE_RPI")

# ==============================================================================
# CONFIGURAÇÕES DA CÂMERA (USB / V4L2) E GEOMETRIA DA PLACA
# ==============================================================================
CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", 0))
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
TARGET_FPS = 30

# Diâmetro útil da placa superior em milímetros (Rp = 140 mm -> 280 mm)
PLATE_DIAMETER_MM = 280.0

# Orientação e alinhamento visual câmera <-> placa
CAMERA_ROTATION_DEG = 0     # 0, 90, 180 ou 270 graus
CAMERA_INVERT_X = False     # Inversão do eixo X visual
CAMERA_INVERT_Y = False     # Inversão do eixo Y visual
CAMERA_SWAP_XY = False      # Troca eixos X e Y
MOTOR_FRONT_ID = 3          # Motor 3 está montado fisicamente à frente da câmera

# ==============================================================================
# COMUNICAÇÃO SERIAL UART (RPi 3 <-> STM32F446RE)
# ==============================================================================
SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT_S = 0.01

# ==============================================================================
# PARÂMETROS DO CONTROLADOR PID E LIMITES DE SATURAÇÃO
# ==============================================================================
PID_DEFAULT_KP = 1.35
PID_DEFAULT_KI = 0.05
PID_DEFAULT_KD = 0.85
PID_DEFAULT_FILTER_N = 20.0
MAX_PLATE_TILT_DEG = 12.0           # Saturação do comando PID
MAX_KINEMATIC_TILT_DEG = 15.0       # Saturação na cinemática inversa
MAX_MOTOR_EXCURSION_DEG = 25.0      # Excursão máxima dos braços a partir do neutro
MAX_JOG_EXCURSION_DEG = 90.0        # Excursão máxima em calibração/jog

# ==============================================================================
# FILTRO DE KALMAN 2D E COMPENSAÇÃO DE LATÊNCIA
# ==============================================================================
KALMAN_DEFAULT_Q_ACCEL = 0.05       # Densidade espectral do ruído de aceleração
KALMAN_DEFAULT_R_MEAS = 2.25e-6     # Variância de medição da câmera (sigma ~ 1.5 mm)²
KALMAN_DEFAULT_LATENCY_SEC = 0.030  # Compensação de atraso visual padrão (30 ms)

# ==============================================================================
# CALIBRAÇÃO APRILTAG E DETECTOR VISUAL HSV
# ==============================================================================
APRILTAG_DICT_NAME = "DICT_APRILTAG_36h11"
APRILTAG_SIZE_MM = 30.0
APRILTAG_LAYOUT_MM = {
    0: (-110.0, 110.0),
    1: (110.0, 110.0),
    2: (110.0, -110.0),
    3: (-110.0, -110.0),
}
BALL_RADIUS_MM = 20.0
CAMERA_HEIGHT_MM = 450.0

# Detector por Espaço de Cor HSV
HSV_LOWER = (5, 120, 100)
HSV_UPPER = (25, 255, 255)
BALL_MIN_RADIUS_PX = 4
BALL_MAX_RADIUS_PX = 40
BALL_MIN_CIRCULARITY = 0.65
BALL_BLUR_SIZE = 5

# ==============================================================================
# ATUADORES DE PASSO (NEMA 17 + TMC2209)
# ==============================================================================
DEFAULT_MICROSTEPS = 8               # 8 micropassos = 1600 passos/rev (MS1=MS2=GND)
DEFAULT_STEP_DELAY_US = 1400         # Meio-período do pulso STEP -> ~350 passos/s
DEFAULT_INVERT_DIR = [True, False, False]  # M1 invertido por padrão mecânico

# Pinagem pigpio dos TMC2209 no Raspberry Pi 3 (GPIO / BCM)
# Conforme pinagem física real documentada no README.md e web/index.html:
# Motor 1 (0° - Frente)     : DIR: GPIO 17, STEP: GPIO 18, EN: GPIO 27
# Motor 2 (120° - Trás Dir.): DIR: GPIO 24, STEP: GPIO 23, EN: GPIO 22
# Motor 3 (240° - Trás Esq.): DIR: GPIO 25, STEP: GPIO 9,  EN: GPIO 10
PIGPIO_MOTORS = {
    1: {'dir': 17, 'step': 18, 'en': 27, 'name': 'Motor 1 (0° - Frente)'},
    2: {'dir': 24, 'step': 23, 'en': 22, 'name': 'Motor 2 (120° - Trás Dir.)'},
    3: {'dir': 25, 'step': 9,  'en': 10, 'name': 'Motor 3 (240° - Trás Esq.)'},
}

CALIBRATION_FILE = os.path.join(BASE_DIR, "calibration_offsets.json")
