# Projeto e Sintonia do Controlador PID — Aeropêndulo

**Projeto PIBIC — Engenharia Elétrica**

Documento de projeto do controlador: identificação da planta, sintonia por
Ziegler-Nichols Método 2, configuração implementada em `src/pid.c` e resultados
de bancada.

---

## 1. Objetivo e bancada

Controlar o ângulo de um aeropêndulo (braço articulado acionado por
motor/hélice) de modo que ele alcance e mantenha um ângulo de referência,
usando **PID puro de ganho único**.

| Item | Descrição |
|---|---|
| Microcontrolador | STM32F103C8 (Blue Pill), PlatformIO + HAL |
| Sensor de ângulo | Encoder incremental em quadratura, TIM2, 1440 contagens/volta (0,25°/pulso) |
| Referência de zero | MPU6050 (acelerômetro) — calibra o offset do encoder no boot |
| Atuador | Motor DC + hélice, driver HW-517 (MOSFET), PWM 0–1000 em PA6 (TIM3) |
| Taxa de controle | 50 Hz exatos, por interrupção do TIM4 (`DT = 20 ms` garantido) |
| Telemetria | UART1 (PA9) → `coleta_degrau.py` → CSV |

O laço de controle roda **dentro da interrupção**, não no `while(1)`. É isso que
garante que o termo integral use sempre `DT = 20 ms`; com o cálculo no laço
principal o `DT` variaria com o tempo de atualização do display I²C.

### 1.1 Restrição de projeto

O controlador deve ser **PID puro, com ganho único e fixo**, sintonizado por um
método formal da disciplina (Ziegler-Nichols) e justificável academicamente.

**Não é permitido:** feedforward, escalonamento de ganhos (*gain scheduling*),
rampa/perfil de setpoint, banda morta, chaveamento de modo.
**É permitido:** Kp, Ki, Kd, o filtro N do derivativo, derivativo sobre a
medição, anti-windup do integrador e os limites de segurança.

---

## 2. Identificação da planta

### 2.1 Ensaios de malha aberta

Degrau de PWM aplicado com o motor em repouso (`MODO_MALHA_ABERTA` em
`main.c`), telemetria gravada até o braço estabilizar.

A resposta **não é uma curva em "S"** — é de 2ª ordem subamortecida. Por isso os
parâmetros vieram do **decremento logarítmico**
(`metodo_2a_ordem_decremento_log` em `identifica_zn.py`), que extrai ζ e ωₙ da
razão entre amplitudes de picos sucessivos:

| Ângulo | PWM | ζ | ωₙ (rad/s) | K (°/PWM) |
|---|---|---|---|---|
| 15° | 120 | não resolvido | — | ~0,098 |
| 30° | 210 | 0,130 | 8,38 | 0,136 |
| 45° | 295 | 0,146 | 8,14 | 0,129 |
| 60° | 410 | 0,171 | 7,70 | 0,139 |
| 75° | 470 | 0,144 | 6,32 | 0,156 |
| 90° | 500 | 0,165 | 5,83 | 0,198 |

**Resultado físico confirmado:** ωₙ cai monotonicamente com o ângulo
(8,38 → 5,83 rad/s), exatamente como prevê a linearização
`ωₙ = √(mgL·cos θ₀ / I)` — o "efeito mola" da gravidade enfraquece conforme o
braço sobe. É a validação de que a identificação está correta.

### 2.2 Por que Ziegler-Nichols Método 2

O **Método 1** (curva de reação / tangente) pressupõe resposta ao degrau
monotônica em "S", de um processo essencialmente de 1ª ordem com atraso. A
planta é subamortecida (ζ ≈ 0,13–0,17), e os dois ajustes de 1ª ordem
implementados em `identifica_zn.py` devolveram **`L` negativo em todos os
ensaios**, o que produz `Kp` negativo. O Método 1 está **fora da faixa de
validade** para esta planta.

O **Método 2** (ganho crítico / oscilação sustentada) não tem essa restrição:
precisa apenas do ponto em que a fase do processo cruza −180°, que existe e é
bem definido aqui. `identifica_zn.py projetar` calcula `Ku` e `Tu` analiticamente a
partir do modelo identificado, sem precisar levar a bancada à instabilidade.

### 2.3 Atraso de laço (`L`)

O `L` usado no projeto é **0,11 s**: o retentor de ordem zero da amostragem de
50 Hz mais o atraso de empuxo da hélice. É o valor que `identifica_zn.py
projetar` usa, e é ele que reproduz exatamente os ganhos gravados no `pid.c`.

Os valores de 0,70 a 1,53 s que aparecem nos CSVs de identificação **não são**
atraso da planta — são o intervalo entre a telemetria começar a gravar e a
fonte de bancada ser ligada na mão, somado à aceleração da hélice partindo do
motor parado. Todo projeto feito contra esse `L` sai ultraconservador.

### 2.4 Calibração estática (PWM × ângulo)

Levantada em malha aberta, usada para dimensionar `INTEGRAL_MAX`:

| Ângulo | 15° | 30° | 45° | 60° | 75° | 90° |
|---|---|---|---|---|---|---|
| PWM | 137 | 220 | 320 | 400 | 450 | 500 |

---

## 3. Sintonia — Ziegler-Nichols Método 2

Com `L = 0,11 s`, `Ku` e `Tu` foram calculados em cada ponto de operação
(`identifica_zn.py projetar`):

| Ângulo | 30° | 45° | 60° | 75° | 90° |
|---|---|---|---|---|---|
| `Ku` | **2,47** | 2,99 | 3,40 | 2,96 | 2,87 |
| `Tu` (s) | **0,690** | 0,701 | 0,723 | 0,865 | 0,907 |

Projeta-se no **pior caso** (menor `Ku`: 30°), o que cobre toda a faixa com um
único jogo de ganhos — sem escalonamento.

Aplicando a linha **"some overshoot"** da tabela de Ziegler-Nichols:

```
Kp = 0,33  · Ku = 0,8138
Ti = 0,50  · Tu = 0,345 s   →   Ki = Kp/Ti = 2,3589
Td = 0,333 · Tu = 0,230 s   →   Kd = Kp·Td = 0,1870
```

**Por que "some overshoot" e não a linha clássica:** a linha clássica
(`Kp = 0,60·Ku`) fica instável a 90° se o atraso de laço real for maior que o
suposto — 17% de overshoot em `L = 0,15 s` e 75% em `L = 0,20 s`. A linha
"some overshoot" não passa de 0% de overshoot até `L = 0,30 s`, o que dá
margem para a incerteza de `L`.

---

## 4. Implementação (`src/pid.c`)

### 4.1 Derivativo sobre a medição e filtro N

O derivativo é calculado sobre a **medição** (`−Kd·θ̇`), não sobre o erro, para
não dar um chute de PWM a cada mudança de setpoint. O filtro é um passa-baixa
de 1ª ordem discreto:

```c
deriv_estado += N * DT * (deriv_angulo - deriv_estado);
```

com polo em `z = 1 − N·DT`. **Só é estável para `0 < N·DT < 2`.**

| N | N·DT | polo | Recuperação de peteleco (30°) | Ondulação do PWM em regime |
|---|---|---|---|---|
| 20 | 0,40 | 0,60 | 5,48 s | 5 (pp) |
| 30 | 0,60 | 0,40 | 4,72 s | 7 |
| **50** | **1,00** | **0,00** | **4,02 s** | **13** |
| 80 | 1,60 | −0,60 | 3,36 s | 30 |
| 100 | 2,00 | −1,00 | 14,98 s | **374** |

**`N = 50` é o ponto ótimo.** Em `N·DT = 1` o polo cai exatamente em zero e o
filtro degenera num **atraso puro de uma amostra** — é o derivativo mais rápido
possível sem nenhuma oscilação. Acima disso o ganho de velocidade é pequeno e a
ondulação cresce rápido.

`N = 100` é o pior valor possível: `N·DT = 2,0` exatos põem o polo em `z = −1`,
o filtro fica no limite de estabilidade e oscila em 25 Hz (frequência de
Nyquist) sem nunca amortecer.

### 4.2 Anti-windup

Quase todo o PWM de sustentação vem do integral (no regime permanente
`Kp·erro → 0`), então o integrador precisa chegar a ~450 para segurar 75°.

```c
integral += Ki * erro * DT;
integral  = saturar(integral, PWM_MIN - (P + D), PWM_MAX - (P + D));  // clamp dinâmico
integral  = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);           // teto absoluto
```

O **clamp dinâmico** limita o integral pela folga que P e D deixam até os
limites do atuador: sozinho, ele nunca consegue empurrar a saída para fora de
[0, 1000]. É a forma recomendada por Åström & Hägglund.

`INTEGRAL_MAX = 620` é o teto absoluto, dimensionado pela calibração da seção
2.4 com folga para queda de tensão da fonte. **Este valor define o ângulo máximo
alcançável:** com 550 o braço parava em ~80° para alvos de 90° ou mais, porque a
saída ficava travada em `550 + Kp·erro`. Com 620 os alvos de 90° e 100° passaram
a ser alcançados (seção 5).

### 4.3 Estágios de segurança

Válidos em módulo (positivo e negativo):

| Faixa | Ação |
|---|---|
| 130° ≤ \|θ\| < 150° | Saída × 0,90 |
| 150° ≤ \|θ\| < 160° | Saída × 0,90 × 0,80 (os fatores se acumulam) |
| \|θ\| ≥ 160° | Desliga o motor e zera o integral |
| \|θ\| ≥ 170° | Trava por desequilíbrio (exige reset da placa) |

### 4.4 Margem de estabilidade

Verificação em frequência dos ganhos implementados contra **todos** os pontos
identificados (`analise_margem_multiponto.py --pid-do-firmware`, atraso de laço
de 0,11 s, aproximação de Padé de 3ª ordem):

| Ponto | 30° | 45° | 60° | 75° | 90° |
|---|---|---|---|---|---|
| Margem de ganho (dB) | 13,9 | 15,3 | 16,2 | 18,6 | 18,4 |
| Margem de fase (°) | 93,8 | 93,5 | 93,6 | 94,0 | 94,9 |

O pior caso é 45°, com 93,5° de margem de fase e 15,3 dB de margem de ganho —
folga confortável em toda a faixa, coerente com a partida sem oscilação medida
em bancada. A margem alta é consequência direta da linha *some overshoot*, que
usa apenas 33% do ganho crítico.

---

## 5. Resultados de bancada

Degrau de partida do repouso (~8°) até o alvo, seguido de perturbações manuais
("petelecos"). Ensaios de 10 a 12/08/2026.

| Alvo | Pico na partida | 95% do alvo | Entra em ±2% | Regime final | Erro |
|---|---|---|---|---|---|
| 30° | 30,5° (+1,7%) | 8,3 s | 10,6 s | 29,97° | −0,03° |
| 45° | 46,3° (+2,8%) | 8,4 s | 10,7 s | 45,42° | +0,42° |
| 60° | sem overshoot | 2,0 s | 9,7 s | 58,22° | −1,78° |
| 75° | 72,5° (não ultrapassou) | 2,3 s | — | 76,22° | +1,22° |
| 90° | — | ~5,2 s | — | 89,69° | −0,31° |
| 100° | — | ~11,4 s | — | 99,72° | −0,28° |

Os ensaios de 90° e 100° (12/08, `INTEGRAL_MAX = 620`) confirmam que o teto do
integral era o que limitava o ângulo máximo — o erro de regime permanece abaixo
de 0,4° em ambos.

**Rejeição de perturbação** (peteleco manual, braço já estabilizado):

| Alvo | Picos atingidos | Tempo para voltar ao alvo |
|---|---|---|
| 30° | 52,5° / 66,5° | 4,0 s / 7,0 s |
| 45° | 74,8° / 86,0° | 11,0 s / 5,1 s |
| 60° | 95,2° / 111,5° | 3,9 s / 11,0 s |
| 75° | 105,8° / 131,0° | 2,5 s / 7,7 s |

A partida é **monotônica e sem oscilação**, e o braço volta ao alvo depois de um
peteleco forte (que chega a levá-lo a 131°) sem entrar em oscilação sustentada.
Esse é o comportamento que valida o projeto.

---

## 6. Limitações conhecidas

### 6.1 Pico de partida se a malha rodar com o motor sem energia

O MCU é alimentado pelo ST-Link/USB e começa a executar `PID_Atualizar()` no
boot, antes de a fonte do motor ser ligada. Nesse intervalo o braço está parado,
o erro fica cravado no valor do alvo, e o integral sobe a `Ki · erro ≈ 140
PWM/s` até bater no teto. Quando a fonte é ligada, o motor recebe um degrau em
malha aberta do valor de `INTEGRAL_MAX`.

Num ensaio de 60° com os ganhos antigos (`Kp = 0,0953`, `INTEGRAL_MAX = 900`)
isso levou o braço a **117°**. O mesmo vale se o braço for segurado com a
malha já rodando.

Com PID puro e setpoint em degrau isso é **inevitável e está correto** — o
integrador precisa chegar a ~430 para sustentar 60°, e se o braço não pode se
mover ele vai lá de qualquer jeito. **A correção é de procedimento**, seção 7.

### 6.2 Recuperação de peteleco é limitada pelo atuador, não pelos ganhos

O termo D realimenta `−Kd·θ̇`, e em tese `ζ_efetivo = ζ_planta + K·ωₙ·Kd/2`.
Chegar a `ζ = 1` exigiria `Kd ≈ 1,5` (a tabela ZN dá 0,187). Mas isso só vale se
o atuador conseguir responder na frequência de ressonância da planta (~1,2 Hz).
O empuxo da hélice tem atraso próprio (τ), e com τ acima de ~0,1 s o termo D
chega defasado — `Kd` grande **piora** a recuperação em vez de melhorar
(verificado em simulação: `Kd = 0,40` recupera em 4,3 s contra 3,3 s de
`Kd = 0,19`).

Ou seja: com esta hélice e esta taxa de 50 Hz, nenhum ajuste de PID leva a
resposta a distúrbio ao amortecimento crítico. É um limite de hardware, não de
sintonia — e é um resultado válido para o relatório, não uma deficiência.

---

## 7. Protocolo de ensaio

1. **Soltar o braço antes de energizar.** Nunca segurar o braço com a malha
   rodando (seção 6.1).
2. **Ligar a fonte do motor antes ou junto com a placa.** O intervalo entre o
   boot do MCU e a energização do motor é integrado como erro.
3. Iniciar `coleta_degrau.py` e deixar rodar sem interferência até o braço ficar
   visivelmente parado por 15–20 s.
4. **Nenhum toque no braço ou na hélice** durante a fase de partida. Aplicar
   petelecos só depois de o ângulo estabilizar, e anotar o instante.
5. Cada ensaio é salvo com timestamp próprio
   (`telemetria_<ALVO>_FECH_AAAAMMDD_HHMMSS.csv`), sem sobrescrever anteriores.

---

## 8. Arquivos do projeto

| Arquivo | Função |
|---|---|
| `src/pid.c` / `src/pid.h` | Controlador PID |
| `src/main.c` | Seleção de modo (malha aberta / fechada) e alvo |
| `src/hardware.c` | Periféricos e interrupção de 50 Hz (TIM4) |
| `src/motor.c` | Driver do PWM |
| `src/aeropendulo.c` | Leitura de ângulo, display e telemetria |
| `src/coleta_degrau.py` | Captura a telemetria UART em CSV |
| `src/painel_angulo.py` | Mostra o ângulo em tela cheia durante o ensaio |
| `src/identifica_zn.py` | Identificação da planta e ganhos por ZN Método 2 |
| `src/analise_margem_multiponto.py` | Margem de fase por ponto de operação |
| `src/simula_partida_degrau.py` | Simulação do degrau de partida com a lógica de `pid.c` |
