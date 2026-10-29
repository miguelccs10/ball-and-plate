# Fundamentos Teóricos, Modelagem Matemática e Protocolo de Comunicação
## Plataforma Auto Balanceável Ball and Plate 3-DoF (RPi 3 / STM32F446RE)
**Autor:** Miguel Carlos Costa Sousa — Universidade Federal de Goiás (UFG)

---

## 1. Modelagem Dinâmica Não-Linear (Euler-Lagrange)

A dinâmica do sistema esfera-placa é descrita pelas equações de Euler-Lagrange para um corpo rígido esférico que rola sem deslizar sobre um plano com inclinação biaxial ($\alpha$ em torno do eixo $Y$ e $\beta$ em torno do eixo $X$):

### 1.1. Propriedades Inerciais da Esfera
Para uma esfera homogênea de massa $m$ e raio $r_b$, o momento de inércia em torno do seu centro de massa é dado por:
$$I_b = \frac{2}{5} m r_b^2$$

A energia cinética total $T$ engloba a translação da esfera e sua rotação pura (rolamento sem escorregamento):
$$T = \frac{1}{2} m (\dot{x}^2 + \dot{y}^2) + \frac{1}{2} I_b (\omega_x^2 + \omega_y^2)$$

Com a restrição cinemática de não-escorregamento $\omega_x = \frac{\dot{y}}{r_b}$ e $\omega_y = -\frac{\dot{x}}{r_b}$:
$$T = \frac{1}{2} m (\dot{x}^2 + \dot{y}^2) + \frac{1}{2} \left(\frac{2}{5} m r_b^2\right) \left(\frac{\dot{x}^2 + \dot{y}^2}{r_b^2}\right) = \frac{7}{10} m (\dot{x}^2 + \dot{y}^2)$$

### 1.2. Equações Diferenciais de Movimento
Para ângulos de inclinação pequenos ($\sin\alpha \approx \alpha$, $\cos\alpha \approx 1$):
$$\ddot{x} = -\frac{5}{7} g \sin\alpha \approx -C_g \cdot \alpha$$
$$\ddot{y} = -\frac{5}{7} g \sin\beta \approx -C_g \cdot \beta$$

onde a constante efetiva de gravidade reduzida é:
$$C_g = \frac{5}{7} g = \frac{5}{7} \times 9.80665 \approx 7.00475\text{ m/s}^2$$

---

## 2. Cinemática Inversa Analítica 3-DoF (Base Hexagonal)

A estrutura mecânica utiliza 3 atuadores de passo NEMA 17 dispostos radialmente a 120° na base com braços e bielas articuladas ligadas à placa superior:

### 2.1. Dimensões Físicas Nominais
* Raio dos eixos dos motores na base ($R_b$): $9.0\text{ cm} = 0.090\text{ m}$
* Raio das rótulas na placa superior ($R_p$): $14.0\text{ cm} = 0.140\text{ m}$
* Braço da manivela motorizada ($r$): $6.0\text{ cm} = 0.060\text{ m}$
* Haste articulada / biela ($L$): $9.0\text{ cm} = 0.090\text{ m}$
* Altura neutra de operação ($z_0$): $7.5\text{ cm} = 0.075\text{ m}$
* Resolução angular padrão: $1600\text{ passos/revolução}$ (1/8 micropasso TMC2209)
* Ângulo neutro nominal dos braços: $\theta_{\text{neutro}} \approx 126.73^\circ$

### 2.2. Solução Trigonométrica Exata via Substituição de Weierstrass
Para cada perna $i \in \{1, 2, 3\}$, a posição da rótula superior sob inclinação $(\alpha, \beta)$ projeta-se no plano radial com coordenadas $(\rho_i, z_i)$.
A restrição de comprimento da biela $L$ resulta na equação trigonométrica:
$$E \cos\theta_i + F \sin\theta_i = G$$

com:
$$E = -2 r (\rho_i - R_b)$$
$$F = -2 r z_i$$
$$G = (\rho_i - R_b)^2 + z_i^2 + r^2 - L^2$$

Aplicando a substituição $t = \tan(\theta_i / 2)$:
$$\theta_i = 2 \arctan\left(\frac{-F + \sqrt{E^2 + F^2 - G^2}}{G - E}\right)$$

---

## 3. Filtro de Kalman Discreto 2D e Compensação de Latência

### 3.1. Modelo em Espaço de Estados Discreto (Duplo Integrador)
Para cada eixo cartesiano ($X$ e $Y$), o vetor de estados é $\mathbf{x}[k] = [p[k], v[k]]^T$:
$$\mathbf{x}[k] = \mathbf{F} \mathbf{x}[k-1] + \mathbf{B} u[k-1] + \mathbf{w}[k]$$
$$\mathbf{F} = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}, \quad \mathbf{B} = \begin{bmatrix} \frac{1}{2} T_s^2 \\ T_s \end{bmatrix}, \quad \mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

### 3.2. Compensação Antecipatória de Latência Visual ($30\text{ ms}$)
Devido ao tempo de captura do sensor CMOS e inferência visual ($\tau \approx 30\text{ ms}$), a posição estimada é projetada à frente:
$$x_{\text{comp}} = x + v \cdot \tau + \frac{1}{2} u \cdot \tau^2$$
$$v_{\text{comp}} = v + u \cdot \tau$$

---

## 4. Protocolo Serial UART Binário Determinístico (CRC16-CCITT)

A comunicação RPi 3 $\leftrightarrow$ STM32F446RE opera a $115200\text{ bps}$ (8N1). Cada pacote de telemetria visual possui $16\text{ bytes}$:

| Byte Offset | Campo | Tipo | Descrição |
| :--- | :--- | :--- | :--- |
| **0** | `PREAMBLE_1` | `uint8_t` | `0xAA` (sincronismo de quadro) |
| **1** | `PREAMBLE_2` | `uint8_t` | `0x55` (sincronismo de quadro) |
| **2** | `MSG_ID` | `uint8_t` | `0x01` (Vision), `0x02` (Setpoint), `0x04` (Command) |
| **3** | `LEN` | `uint8_t` | Tamanho útil em bytes (`0x0A` para visão) |
| **4–7** | `TIMESTAMP` | `uint32_t` | Timestamp em milissegundos |
| **8–9** | `POS_X` | `int16_t` | Posição $X$ em décimos de milímetro ($0.1\text{ mm}$) |
| **10–11** | `POS_Y` | `int16_t` | Posição $Y$ em décimos de milímetro ($0.1\text{ mm}$) |
| **12** | `FLAGS` | `uint8_t` | Bit 0: Detected, Bit 1: YOLO/HSV |
| **13** | `CONF` | `uint8_t` | Confiança ($0\text{ a }100\%$) |
| **14–15** | `CRC16` | `uint16_t` | Polinômio `0x1021`, inicial `0xFFFF`, Big-Endian |

### 4.1. Tipos de Mensagem Suportados
* `0x01` — **`MSG_VISION_COORDS`** (RPi $\to$ STM32): Coordenadas e flags de detecção a 30 Hz.
* `0x02` — **`MSG_SETPOINT_CMD`** (RPi $\to$ STM32): Setpoints desejados $(X, Y)$ em décimos de milímetro.
* `0x04` — **`MSG_SYSTEM_COMMAND`** (RPi $\to$ STM32): Comandos imperativos:
  - `CMD_ENABLE` (`1`): energiza drivers TMC2209.
  - `CMD_DISABLE` (`2`): desenergiza bobinas (modo livre).
  - `CMD_LEVEL_AND_RESET` (`3`): nivela placa em repouso e reseta estimadores.
  - `CMD_SET_FRONT_MOTOR` (`4`): define o motor físico posicionado em frente à câmera (`param` = 1, 2 ou 3).
