# APLICAÇÃO DE VISÃO COMPUTACIONAL E CONTROLE EM PLATAFORMA AUTO BALANCEÁVEL 3-DoF

**Autor:** Miguel Carlos Costa Sousa  
**Instituição:** Universidade Federal de Goiás (UFG) — Engenharia Mecatrônica  
**Projeto:** Plataforma Ball and Plate 3-DoF (Mecanismo Paralelo Hexagonal 120° / 3-RRS)  

---

## 1. Visão Geral da Arquitetura e Modo Operacional Duplo

O projeto desenvolve uma plataforma mecatrônica auto-balanceável do tipo *Ball and Plate* com 3 graus de liberdade. O sistema possui suporte nativo a **dois modos operacionais intercambiáveis**:

1. **Modo Standalone (Raspberry Pi 3):**
   O Raspberry Pi 3 atua tanto no nível de percepção visual quanto no nível de controle e acionamento direto dos motores de passo via `pigpio` nos pinos GPIO da placa. Este modo é ideal para validação rápida de bancada, testes de integridade mecânica e sintonia dos controladores antes do deployment no microcontrolador.
2. **Modo Híbrido (Raspberry Pi 3 + STM32F446RE):**
   O Raspberry Pi 3 realiza a captura e processamento visual a 30 FPS e transmite as coordenadas via protocolo binário UART determinístico (16 bytes com CRC16) para a placa STM32 Nucleo-F446RE. O microcontrolador Cortex-M4F assume com exclusividade o loop rígido de 100 Hz com temporizadores de hardware para os drivers TMC2209.

```
+-----------------------------------------------------------------------------------+
|                         MODO OPERACIONAL 1: STANDALONE RPi                        |
|                                                                                   |
|  [ Câmera USB QVGA ] ---> [ Raspberry Pi 3 ]                                     |
|                           - Captura V4L2 (< 5ms buffer)                           |
|                           - Detecção HSV / YOLO                                   |
|                           - Filtro de Kalman 2D + Compensação de Latência         |
|                           - Dual PID com Anti-Windup                              |
|                           - Cinemática Inversa 3-DoF                              |
|                           - Acionamento direto via pigpio                         |
|                                   |                                               |
|                                   v                                               |
|                    [ 3x Drivers TMC2209 + NEMA 17 ]                               |
+-----------------------------------------------------------------------------------+
                                     OU
+-----------------------------------------------------------------------------------+
|                         MODO OPERACIONAL 2: HÍBRIDO RPi + STM32                   |
|                                                                                   |
|  [ Câmera USB QVGA ] ---> [ Raspberry Pi 3 ]                                     |
|                           - Captura V4L2 & Detecção                               |
|                           - Dashboard Web & Telemetria                            |
|                           - Transmissão UART Binária (16 bytes, CRC16)            |
|                                   | UART (115200 bps)                             |
|                                   v                                               |
|                           [ STM32F446RE (180 MHz FPU) ]                           |
|                           - Timer Hard Real-Time (100 Hz)                         |
|                           - Kalman 2D + Predição de Latência                      |
|                           - Dual PID + Cinemática Inversa Analítica               |
|                           - Pulsos de Hardware nos pinos STM32                    |
|                                   |                                               |
|                                   v                                               |
|                    [ 3x Drivers TMC2209 + NEMA 17 ]                               |
+-----------------------------------------------------------------------------------+
```

---

## 2. Pinagem de Hardware

### 2.1 Pinos de Hardware Ativos no Raspberry Pi 3 (Modo Standalone)
| Atuador | Sinal DIR | Sinal STEP | Sinal ENABLE (Ativo em Nível Baixo) |
| :--- | :--- | :--- | :--- |
| **Motor 1 (0° - Frente)** | **GPIO 17** | **GPIO 18** | **GPIO 27** |
| **Motor 2 (120° - Trás Dir.)** | **GPIO 24** | **GPIO 23** | **GPIO 22** |
| **Motor 3 (240° - Trás Esq.)** | **GPIO 25** | **GPIO 9** | **GPIO 10** |

### 2.2 Pinos Configuráveis no STM32F446RE (Modo Híbrido — Definidos em `system_config.h`)
| Atuador | Pino DIR | Pino STEP | Pino ENABLE |
| :--- | :--- | :--- | :--- |
| **Motor 1 (0°)** | `PA0` (Morpho CN7) | `PA1` (Morpho CN7) | `PA4` (Morpho CN7) |
| **Motor 2 (120°)** | `PB0` (Morpho CN7) | `PB1` (Morpho CN10) | `PB2` (Morpho CN10) |
| **Motor 3 (240°)** | `PB10` (Morpho CN10) | `PB11` (Morpho CN10) | `PB12` (Morpho CN10) |
| **UART Link** | `USART2_TX` = `PA2` | `USART2_RX` = `PA3` | Baudrate: `115200 bps` |

---

## 3. Estrutura do Repositório (Consolidada)

```text
ball-and-plate/
├── README.md                    # Guia rápido, pinagem e instruções de execução
├── .gitignore                   # Regras de exclusão Git
├── requirements.txt             # Dependências unificadas na raiz
├── config.py                    # Configurações globais e constantes físicas
├── calibration_offsets.json     # Armazenamento permanente dos offsets dos atuadores
├── main.py                      # Ponto de entrada único (Web por padrão, --headless ou --preview)
├── core/                        # Núcleo Python modular puro (3 arquivos)
│   ├── vision.py                # CameraStream + Detectores HSV/YOLO + Geometria 2D
│   ├── control_math.py          # Kalman 2D + Dual PID + Cinemática Inversa 3-DoF
│   └── hardware.py              # MotorController (pigpio + JSON calib) + SerialBridge (UART)
├── web/                         # Interface Web unificada (3 arquivos)
│   ├── server.py                # Servidor HTTP/MJPEG com endpoints consolidados
│   ├── index.html               # Template HTML5 único
│   └── static/                  # dashboard.css e dashboard.js consolidados
├── control_stm32/               # Firmware STM32 C compacto compatível com CubeIDE (3 arquivos)
│   └── Core/
│       ├── Inc/system_config.h  # Pinagem, constantes físicas e estruturas de dados
│       └── Src/
│           ├── control_math.c   # Kalman 2D + Dual PID + Cinemática Inversa
│           └── main_control_loop.c # Parser UART CRC16 + Driver TMC2209 BSRR + Loop 100 Hz
├── tools/                       # Utilitários de bancada e calibração (1 arquivo)
│   └── bench_suite.py           # Testes de hardware, calibração guiada e exportação ONNX
└── docs/                        # Documentação técnica e formalismo unificado (1 arquivo)
    └── THEORY_AND_PROTOCOL.md   # Modelagem Euler-Lagrange, Weierstrass, Kalman e Protocolo UART
```

---

## 4. Como Executar

### 4.1 Iniciar a Aplicação Principal (`main.py`)
No terminal do Raspberry Pi ou estação de desenvolvimento:

```bash
# Execução no Modo Standalone RPi com Dashboard Web (Padrão):
python3 main.py --port 8080 --op-mode STANDALONE_RPI

# Execução no Modo Híbrido com STM32:
python3 main.py --port 8080 --op-mode HYBRID_STM32 --serial-port /dev/ttyACM0

# Modo Headless (sem servidor Web, direto no console):
python3 main.py --headless --op-mode STANDALONE_RPI

# Modo Preview local (janela OpenCV na tela do desktop):
python3 main.py --preview
```

Abra no navegador em qualquer computador da mesma rede:
👉 `http://<IP_DO_RASPBERRY_PI>:8080`

### 4.2 Suíte de Bancada e Calibração (`tools/bench_suite.py`)
```bash
# Menu interativo com 8 testes (diagnóstico de GPIO, limites de passos, calibração interativa):
python3 tools/bench_suite.py

# Exportação do modelo YOLO para formato ONNX otimizado:
python3 tools/bench_suite.py --export-yolo runs/detect/train/weights/best.pt
```

### 4.3 Teste de Unidade do Firmware STM32 no Host
Para validar toda a lógica de controle, cinemática analítica, Kalman 2D, parser UART CRC16 e temporização do firmware STM32 no computador host via GCC:

```bash
gcc -Wall -Wextra -O2 -DCOMPILE_HOST_TEST -Icontrol_stm32/Core/Inc \
    control_stm32/Core/Src/control_math.c \
    control_stm32/Core/Src/main_control_loop.c \
    -lm -o test_control_stm32 && ./test_control_stm32
```

---

## 5. Recursos Disponíveis no Painel Web
- **Ajuste Fino de Ganhos PID:** Controles duplos com sliders e caixas numéricas para $K_p$, $K_i$, $K_d$, filtro $N$ e setpoints $X/Y$, com sincronização instantânea.
- **Filtro de Kalman 2D Interativo:** Ajuste da covariância do processo $Q_a$, ruído da câmera $R$ e slider para compensação antecipatória de atraso visual ($\tau_d$ em ms).
- **Osciloscópio em Tempo Real:** Canvas gráfico traçando em tempo real a resposta dinâmica $X(t)$ vs $X_{\text{sp}}$ e $Y(t)$ vs $Y_{\text{sp}}$.
- **Calibração de Visão:** Ajuste contínuo de limiares HSV com visualizador de máscara binária e seletor entre detector HSV e YOLO.
- **Controle Manual e Diagnóstico:** Sliders manuais de inclinação ($\alpha, \beta$), avanço de micropassos por motor e teste de velocidade.
