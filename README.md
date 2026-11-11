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
├── pytest.ini                   # Configuração da suíte de testes automatizados
├── config.py                    # Configurações globais e constantes físicas
├── calibration_offsets.json     # Armazenamento permanente atômico dos offsets dos atuadores
├── main.py                      # Orquestrador central trifásico (CLI -> Hardware/Persistência -> Web/Visão)
├── core/                        # Núcleo Python modular (4 arquivos)
│   ├── persistence.py           # Persistência transacional atômica (tempfile + fsync + os.replace)
│   ├── vision.py                # CameraStream + AprilTagCalibrator + Detector HSV + Geometria 2D
│   ├── control_math.py          # Kalman 2D + Dual PID + Cinemática Inversa 3-DoF
│   └── hardware.py              # MotorController (pigpio + JSON calib) + SerialBridge (UART)
├── web/                         # Interface Web unificada e executor de bancada (4 arquivos)
│   ├── server.py                # Servidor HTTP/MJPEG com endpoints REST consolidados
│   ├── bench_runner.py          # Executor assíncrono não-bloqueante das rotinas de bancada na Web
│   ├── index.html               # Template HTML5 único com painel completo de depuração e bancada
│   └── static/                  # dashboard.css e dashboard.js consolidados
├── control_stm32/               # Firmware STM32 C compacto compatível com CubeIDE (3 arquivos)
│   └── Core/
│       ├── Inc/system_config.h  # Pinagem, constantes físicas e estruturas de dados
│       └── Src/
│           ├── control_math.c   # Kalman 2D + Dual PID + Cinemática Inversa
│           └── main_control_loop.c # Parser UART CRC16 + Driver TMC2209 BSRR + Loop 100 Hz
├── tools/                       # Utilitários de bancada e calibração CLI
│   └── bench_suite.py           # Testes de hardware e calibração interativa compartilhada com main.py
├── tests/                       # Suíte de testes automatizados (Unitários, Integração e Browser E2E)
│   ├── test_atomic_persistence.py
│   ├── test_lifecycle_phases.py
│   ├── test_calibration_sync.py
│   ├── test_bench_suite_port.py
│   ├── test_web_api.py
│   └── test_browser_dashboard.py
└── docs/                        # Documentação técnica e formalismo unificado (1 arquivo)
    └── THEORY_AND_PROTOCOL.md   # Modelagem Euler-Lagrange, Weierstrass, Kalman e Protocolo UART
```

---

## 4. Ciclo de Vida Trifásico e Como Executar

### 4.1 Pipeline de Inicialização Unificado (`main.py`)
O ponto de entrada único (`main.py`) atua como um **Orquestrador Central** dividido em 3 fases estritas para garantir sincronização absoluta (*Single Source of Truth*) entre a CLI, o hardware e o servidor web:
1. **Fase 1 (CLI Pré-Servidor):** Executa a rotina interativa de verificação e calibração dos ângulos iniciais via terminal (pode ser pulada em automação via `--skip-cli` ou `--headless`).
2. **Fase 2 (Persistência & Hardware):** Aplica automaticamente os offsets salvos em `calibration_offsets.json` (`--apply-calibration` ativo por padrão), aguarda a barreira de prontidão física dos motores (`wait_until_idle`), realiza a calibração homográfica AprilTag com a plataforma nivelada e hidrata o `AppState`.
3. **Fase 3 (Daemon & Servidor Web):** Inicia a ponte UART (se em modo híbrido), sobe o servidor HTTP/MJPEG (`run_dashboard_server`) e entra no `vision_loop` a 30 FPS.

No terminal do Raspberry Pi ou estação de desenvolvimento:

```bash
# Execução padrão no Modo Standalone RPi (Fase 1 CLI interativa -> Fase 2 Offsets -> Fase 3 Web):
python3 main.py --port 8080 --op-mode STANDALONE_RPI

# Iniciar diretamente o servidor Web pulando o menu CLI interativo (aplicando a calibração salva):
python3 main.py --port 8080 --op-mode STANDALONE_RPI --skip-cli

# Iniciar sem aplicar os offsets salvos do JSON:
python3 main.py --port 8080 --no-calibration

# Execução no Modo Híbrido com STM32:
python3 main.py --port 8080 --op-mode HYBRID_STM32 --serial-port /dev/ttyACM0

# Modo Headless (sem servidor Web e sem CLI interativa, direto no loop de controle):
python3 main.py --headless --op-mode STANDALONE_RPI

# Modo Preview local (janela OpenCV na tela do desktop):
python3 main.py --preview
```

Abra no navegador em qualquer computador da mesma rede:
👉 `http://<IP_DO_RASPBERRY_PI>:8080`

### 4.2 Suíte de Bancada e Calibração Standalone (`tools/bench_suite.py`)
As ferramentas de bancada estão integradas tanto na Fase 1 do `main.py` quanto na interface Web, mas também podem ser executadas isoladamente via CLI:
```bash
# Menu interativo completo de bancada (limites cardeais, trajetórias, diagnóstico GPIO e calibração):
python3 tools/bench_suite.py

# Abrir diretamente a rotina interativa de calibração dos ângulos iniciais:
python3 tools/bench_suite.py --calibrate
```

### 4.3 Suíte de Testes Automatizados (Python & Browser E2E)
Para validar a persistência atômica (`tempfile` + `os.fsync` + `os.replace`), o ciclo de vida trifásico do orquestrador, a sincronização de telemetria, as rotinas de bancada na API e a renderização no navegador:

```bash
pytest -v
```

### 4.4 Teste de Unidade do Firmware STM32 no Host
Para validar toda a lógica de controle, cinemática analítica, Kalman 2D, parser UART CRC16 e temporização do firmware STM32 no computador host via GCC:

```bash
gcc -Wall -Wextra -O2 -DCOMPILE_HOST_TEST -Icontrol_stm32/Core/Inc \
    control_stm32/Core/Src/control_math.c \
    control_stm32/Core/Src/main_control_loop.c \
    -lm -o test_control_stm32 && ./test_control_stm32
```

---

## 5. Recursos Disponíveis no Painel Web Consolidado
- **Tabela de Telemetria e Calibração dos 3 Atuadores:** Exibição em tempo real sincronizada com `calibration_offsets.json` contendo Pinos GPIO, Passo Relativo (`current_steps`), Offset Salvo (`calib_offsets`), Passo Absoluto e Ângulo Físico Total (°) de cada braço.
- **Calibração e Jog Multiescala na Web:** Seleção de motor individual (`M1`, `M2`, `M3`) ou `TODOS` simultaneamente, com botões de ajuste fino ($\pm 1$ passo), médio ($\pm 5$ passos), grosso ($\pm 20$ passos), entrada direta em passos ou graus ($\pm\Delta^\circ$), gravação atômica do neutro em JSON e reset de fábrica.
- **Suíte de Testes de Bancada Assíncrona (`BenchRunner`):** Execução não-bloqueante direto pelo navegador de **Teste de Amplitude Cardeal ($\pm 12^\circ$)**, **Trajetórias Programadas (Círculo, Triângulo, Quadrado, Figura-8 / Lissajous)**, **Diagnóstico Elétrico GPIO** e **Teste de Repetibilidade de Nivelamento**, com barra de progresso e parada imediata.
- **Ajuste Fino de Ganhos PID & Persistência:** Controles duplos com sliders e caixas numéricas para $K_p$, $K_i$, $K_d$, filtro $N$ e setpoints $X/Y$, com opção de salvar o perfil de controle em disco (`control_profile.json`).
- **Filtro de Kalman 2D Interativo:** Ajuste da covariância do processo $Q_a$, ruído da câmera $R$ e slider para compensação antecipatória de atraso visual ($\tau_d$ em ms).
- **Osciloscópio em Tempo Real:** Canvas gráfico traçando em tempo real a resposta dinâmica $X(t)$ vs $X_{\text{sp}}$ e $Y(t)$ vs $Y_{\text{sp}}$.
- **Calibração Homográfica AprilTag & Visão HSV:** Ajuste contínuo de limiares HSV, máscara binária, orientação câmera-motor, calibração planar AprilTag (36h11) com compensação de paralaxe e persistência atômica do perfil visual (`vision_profile.json`).
