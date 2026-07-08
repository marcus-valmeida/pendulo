# 🚁 Controle de Aeropêndulo com STM32 (Blue Pill)

Este projeto implementa um sistema de controle de tempo real para um Aeropêndulo (um pêndulo acionado por um motor com hélice). O sistema utiliza um microcontrolador **STM32F103C8T6 (Blue Pill)** para manter a haste em ângulos pré-determinados através de uma malha fechada de controle que combina **Feedforward de Gravidade** e um **Controlador PID (Tipo B)**.

## 🗂️ Mapa do Repositório

A estrutura do projeto foi desenvolvida utilizando a extensão PlatformIO. O código está modularizado dividindo as responsabilidades de Hardware, Lógica de Aplicação e Controle.

```text
PENDULO/
│
├── lib/                        # Bibliotecas de terceiros (Drivers I2C)
│   ├── MPU6050/                # Driver do acelerômetro/giroscópio
│   │   ├── MPU6050.c
│   │   └── MPU6050.h
│   └── SSD1306/                # Driver do display OLED
│       ├── ssd1306.c
│       └── ssd1306.h
│
├── src/                        # Código-fonte do projeto
│   ├── aeropendulo.c / .h      # Lógica de negócio, conversão de ângulos e display
│   ├── hardware.c / .h         # Configuração HAL (Clocks, Timers, I2C, ADC, GPIO)
│   ├── main.c                  # Ponto de entrada, loop principal e seleção de modos
│   ├── motor.c / .h            # Controle do driver MOSFET HW-517 via PWM
│   └── pid.c / .h              # Matemática do controlador PID e Feedforward
│
└── platformio.ini              # Configurações de compilação da placa STM32

```

## 🔌 Arquitetura de Hardware e Conexões

Para evitar colisões no barramento de comunicação e garantir um tempo de execução perfeito, o hardware foi dividido de forma estratégica:

* **Motor (HW-517):** Pino **PA6** (PWM gerado pelo Timer 3 a 1 kHz).
* **Encoder Incremental:** Pinos **PA0 e PA1** (Leitura por hardware via Timer 2 em quadratura).
* **Sensor MPU6050:** Pinos **PB6 e PB7** (I2C1 - Uso exclusivo e rápido na interrupção).
* **Display OLED:** Pinos **PB10 e PB11** (I2C2 - Uso lento no loop principal).
* **Potenciômetro:** Pino **PA4** (ADC1 - Usado para testes e definição de setpoint manual).

## 🧠 Lógica e Matemática do Controle

O coração deste projeto não é um simples PID de livro didático. Ele foi adaptado para lidar com a física não-linear da gravidade e proteger o hardware contra travamentos.

### 1. Tempo Real Estrito (50 Hz)

A matemática de controle exige que o tempo entre as amostragens ($DT$) seja constante. O cálculo do PID **não** roda no loop `while(1)`. Ele é executado exclusivamente dentro da interrupção de hardware do Timer 4 (`TIM4_IRQHandler`), garantindo uma frequência cravada de **50 Hz** ($DT = 20ms$), imune aos atrasos de desenho da tela OLED.

### 2. O Feedforward (Compensação da Gravidade)

A força da gravidade atua no pêndulo de forma trigonométrica (mínima em 0°, máxima em 90°). Usar apenas o PID para vencer isso gera lentidão. O sistema usa um modelo levantado em bancada:


$$V = \frac{\sin(\theta)}{C}$$


Onde $C$ é a constante de calibração do motor. O controlador calcula qual PWM é necessário apenas para "sustentar" o pêndulo no ar no ângulo alvo. Esse valor serve como base.

### 3. PID Tipo B (Derivada na Medição)

A parcela PID apenas corrige o resíduo que o Feedforward não acertou.

* **Proporcional ($K_p$):** Atua sobre o erro atual.
* **Integral ($K_i$) com Anti-windup:** Limitado para não acumular erros gigantescos, serve apenas para zerar o erro estacionário.
* **Derivativo ($K_d$) sobre o Ângulo:** O termo derivativo não reage ao Erro, mas sim à velocidade da haste física. Isso evita que o motor dê um "tranco" (Derivative Kick) quando o usuário muda o alvo bruscamente, atuando puramente como um amortecedor contra perturbações externas ("tapas").

## ⚖️ Calibração Automática do Encoder (Sensor Fusion)

Encoders incrementais são excelentes e rápidos, mas não sabem onde é o "Zero" quando a placa é ligada. Para resolver isso, usamos o **MPU6050** (Acelerômetro) na inicialização:

1. Ao ligar, o MPU6050 lê a gravidade e calcula o ângulo real da haste (Pitch) em repouso.
2. Esse ângulo é convertido em pulsos de encoder.
3. O valor é salvo como um `offset_inicial`.
A partir daí, o MPU6050 é ignorado para o controle principal e o sistema passa a confiar na velocidade e precisão absurda do Encoder, mas agora com uma referência absoluta confiável!

## 🎮 Modos de Operação

O sistema possui dois modos que podem ser alternados no arquivo `main.c` mudando a constante `MODO_ATUAL`:

* **`MODO_MALHA_ABERTA`:** Desliga a matemática do PID. Aplica um PWM bruto, contínuo e fixo (ex: `PWM_45_GRAUS`) diretamente ao motor. Útil para extrair a constante $C$ do motor e levantar a curva de tensão vs. ângulo em regime permanente.
* **`MODO_MALHA_FECHADA`:** O PID assume o controle. O display passa a mostrar os dados de telemetria e o sistema aciona seus gatilhos de segurança (reduzindo a potência em ângulos > 130° e cortando o motor em ângulos perigosos). O alvo pode ser definido no próprio código (`SETPOINT_CODIGO`) ou variado em tempo real girando o potenciômetro físico (`SETPOINT_POTENCIOMETRO`).
