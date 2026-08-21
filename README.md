# Aeropêndulo com STM32 — Controle de Ângulo por PID

Projeto PIBIC de Engenharia Elétrica. Um braço articulado acionado por
motor/hélice é levado a um ângulo de referência e mantido nele por um
**controlador PID puro**, rodando numa **STM32F103C8 (Blue Pill)** a 50 Hz.

O memorial de projeto — identificação da planta, sintonia, ensaios e limitações
— está em **[`docs/projeto_pid.md`](docs/projeto_pid.md)**.

---

## Como o controle funciona

**1. Medição do ângulo.** Encoder incremental em quadratura lido por hardware
pelo TIM2 (1440 contagens/volta = 0,25°/pulso). Como encoder incremental não
sabe onde é o zero, no boot o **MPU6050** lê a gravidade, calcula o ângulo de
repouso do braço e grava esse valor como offset — depois disso o acelerômetro
sai do laço e o controle usa só o encoder.

**2. Tempo real estrito.** O PID **não** roda no `while(1)`: roda dentro da
interrupção do TIM4, a 50 Hz exatos (`DT = 20 ms`). Isso é o que garante que o
termo integral tenha um `DT` confiável, imune ao tempo de escrita no display
I²C. O laço principal só cuida do OLED e da telemetria.

**3. Identificação da planta.** A resposta ao degrau em malha aberta é de 2ª
ordem subamortecida (ζ ≈ 0,13–0,17), não uma curva em "S". ζ e ωₙ foram
extraídos por **decremento logarítmico** em cada ponto de operação. A queda de
ωₙ com o ângulo (8,38 → 5,83 rad/s) confirma a linearização
`ωₙ = √(mgL·cos θ₀ / I)`.

**4. Sintonia por Ziegler-Nichols Método 2.** O Método 1 (curva de reação) foi
descartado por estar fora da faixa de validade — devolve `L` negativo, e daí
`Kp` negativo, numa planta subamortecida. O Método 2 calcula `Ku` e `Tu` a
partir do modelo identificado; projeta-se no pior ponto da faixa (30°, menor
`Ku`) para cobrir todos os ângulos com **um único jogo de ganhos, sem
escalonamento**:

| | Kp | Ki | Kd | N | DT |
|---|---|---|---|---|---|
| Linha *some overshoot* | 0,8138 | 2,3589 | 0,1870 | 50 | 20 ms |

Verificado em frequência contra todos os pontos identificados: margem de fase
acima de **93°** e margem de ganho acima de **13,9 dB** em toda a faixa.

**5. Detalhes de implementação que importam.** Derivativo sobre a **medição**
(`−Kd·θ̇`) e não sobre o erro, para não dar chute de PWM a cada mudança de
setpoint; filtro do derivativo com `N·DT = 1` (o polo cai em zero — derivativo
mais rápido possível sem oscilar); anti-windup por **clamping dinâmico**
(Åström & Hägglund), em que o integral é limitado pela folga que P e D deixam
até os limites do atuador.

**6. Segurança.** Acima de 130° a saída é reduzida; acima de 160° o motor
desliga e o integral zera; acima de 170° o sistema trava e exige reset.

**Resultado:** partida monotônica e sem oscilação, erro de regime abaixo de 2°
em toda a faixa de 30° a 100°, e recuperação de perturbações manuais fortes
(até 131°) sem oscilação sustentada.

---

## Modos de operação

Trocados em `src/main.c` pela constante `MODO_ATUAL`:

* **`MODO_MALHA_ABERTA`** — aplica um PWM fixo direto no motor, sem PID. Usado
  para levantar a curva PWM × ângulo e os ensaios de degrau da identificação.
* **`MODO_MALHA_FECHADA`** — o PID assume. O alvo vem do código
  (`SETPOINT_CODIGO`, constante `ALVO_GRAUS`) ou do potenciômetro em PA4
  (`SETPOINT_POTENCIOMETRO`).

---

## Ligações

| Componente | Pinos | Observação |
|---|---|---|
| Encoder | PA0 (A), PA1 (B) | TIM2 em modo encoder, quadratura x4 |
| MPU6050 | PB6 (SCL), PB7 (SDA) | I2C1, uso rápido só no boot |
| Display OLED | PB10 (SCL), PB11 (SDA) | I2C2 separado — o I²C do OLED é lento |
| Motor (HW-517) | PA6 | PWM de 1 kHz pelo TIM3, escala 0–1000 |
| Potenciômetro | PA4 | ADC1, setpoint manual de −90° a +90° |
| Telemetria | PA9 (TX) | UART1 a 115200 bps |
| LED | PC13 | onboard, pisca a cada volta do laço |

---

## Estrutura

```text
pendulo/
├── docs/
│   ├── projeto_pid.md            # memorial: identificação, sintonia, ensaios
│   └── TCC_Alexsandro_Barros_2019.pdf
├── lib/                          # drivers de terceiros
│   ├── MPU6050/
│   └── SSD1306/
├── src/
│   ├── main.c                    # modos de operação e laço principal
│   ├── hardware.c / .h           # clock, GPIO, timers, I2C, ADC, UART, IRQ 50 Hz
│   ├── pid.c / .h                # controlador
│   ├── motor.c / .h              # driver de PWM do HW-517
│   ├── aeropendulo.c / .h        # ângulo, display e telemetria
│   ├── coleta_degrau.py          # grava a telemetria UART em CSV
│   ├── painel_angulo.py          # painel de ângulo em tela cheia
│   ├── identifica_zn.py          # identificação da planta + ganhos ZN
│   ├── analise_margem_multiponto.py  # margem de fase por ponto de operação
│   ├── simula_partida_degrau.py  # simula o degrau de partida com a lógica de pid.c
│   ├── Telemetria/               # CSVs dos ensaios
│   └── Graficos/                 # gráficos gerados
└── platformio.ini
```

---

## Como usar

```bash
pio run                # compila
pio run -t upload      # grava via ST-Link

python3 src/coleta_degrau.py                            # grava um ensaio em CSV
python3 src/identifica_zn.py identificar <csv> --pwm 450 # identifica a planta
python3 src/identifica_zn.py projetar                    # Ku/Tu e ganhos ZN
python3 src/analise_margem_multiponto.py --pid-do-firmware  # margem de estabilidade
```

Antes de qualquer ensaio, seguir o protocolo da seção 7 de
[`docs/projeto_pid.md`](docs/projeto_pid.md) — em especial: **soltar o braço e
ligar a fonte do motor antes ou junto com a placa**, senão o integrador carrega
com o motor parado e o braço dá um salto na partida.
