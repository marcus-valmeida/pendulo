# Projeto e Sintonia do Controlador PID — Aeropêndulo

**Projeto PIBIC — Engenharia Elétrica**
Última atualização: 2026-08-11

Documento oficial de projeto do controlador. Registra a identificação da
planta, todas as estratégias de sintonia testadas, a configuração validada em
bancada e as limitações conhecidas.

---

## 1. Objetivo e bancada

Controlar o ângulo de um aeropêndulo (braço articulado acionado por
motor/hélice) de modo que ele alcance e mantenha um ângulo de referência na
faixa de 30° a 75°, usando **PID puro de ganho único**.

| Item | Descrição |
|---|---|
| Microcontrolador | STM32F103C8 (Blue Pill), PlatformIO + HAL |
| Sensor de ângulo | Encoder incremental em quadratura, TIM2, 1440 pulsos/volta (0,25°/pulso) |
| Referência de zero | MPU6050 (acelerômetro) — calibra o offset do encoder no boot |
| Atuador | Motor DC + hélice, driver HW-517 (MOSFET), PWM 0–1000 em PA6 (TIM3) |
| Taxa de controle | 50 Hz exatos, por interrupção do TIM4 (`DT = 20 ms` garantido) |
| Telemetria | UART1 (PA9) → `coleta_degrau.py` → CSV |

O laço de controle roda **dentro da interrupção**, não no `while(1)`. Isso é o
que garante que o termo integral use sempre `DT = 20 ms`; com o cálculo no
laço principal o `DT` variaria com o tempo de atualização do display I²C.

### 1.1 Restrição de projeto

Determinação do orientador: o controlador deve ser **PID puro, com ganho único
e fixo**, sintonizado por um método formal da disciplina (Ziegler-Nichols) e
justificável academicamente.

**Não é permitido:** feedforward, escalonamento de ganhos (*gain scheduling*),
rampa/perfil de setpoint, banda morta, chaveamento de modo.
**É permitido:** Kp, Ki, Kd, o filtro N do derivativo, derivativo sobre a
medição (forma "Tipo B"), anti-windup do integrador e os limites de segurança.

Toda a seção 3 deve ser lida com essa restrição em mente — várias tentativas
foram descartadas por ela, não por desempenho.

---

## 2. Identificação da planta

### 2.1 Ensaios de malha aberta

Degrau de PWM aplicado com o motor em repouso (`MODO_MALHA_ABERTA` em
`main.c`), telemetria gravada até o braço estabilizar.

A resposta **não é uma curva em "S"** — é de 2ª ordem subamortecida. Os dois
métodos de 1ª ordem do Método 1 de Ziegler-Nichols (tangente e mínimos
quadrados) devolveram `L` negativo em todos os quatro ensaios, o que produz
`Kp` negativo. **O Método 1 está fora da faixa de validade para esta planta.**

Os parâmetros vieram do **decremento logarítmico**
(`metodo_2a_ordem_decremento_log` em `identifica_zn.py`), que extrai ζ e ωₙ da
razão entre amplitudes de picos sucessivos.

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

### 2.2 Duas armadilhas encontradas nos dados

1. **Coluna `L` inválida.** Os ensaios registravam `L = 0,70` a `1,53 s`, o
   que foi inicialmente lido como atraso de transporte da planta. Não é: é o
   tempo entre a telemetria começar a gravar e a fonte de bancada ser ligada
   na mão, somado ao tempo que a hélice leva para acelerar **partindo do motor
   parado**. Todo projeto feito contra esse `L` sai ultraconservador.

2. **Segunda dinâmica lenta.** Todos os ensaios de 45° a 90° mostram deriva de
   1° a 4° ao longo de dezenas de segundos (provável aquecimento do motor sob
   carga). Está muito abaixo da banda de qualquer PID razoável aqui, então não
   afeta estabilidade, mas torna os valores de `K` aproximados.

### 2.3 Calibração estática (PWM × ângulo)

Levantada em malha aberta, usada para dimensionar `INTEGRAL_MAX`:

| Ângulo | 15° | 30° | 45° | 60° | 75° | 90° |
|---|---|---|---|---|---|---|
| PWM | 137 | 220 | 320 | 400 | 450 | 500 |

---

## 3. Tentativas realizadas

| # | Data | Estratégia | Resultado | Situação |
|---|---|---|---|---|
| 1 | — | PID + **feedforward** + **escalonamento de ganhos** | Funcionava, mas fora da formulação clássica | ❌ Vetado pelo orientador |
| 2 | 04/08 | **Método 1 ZN** (curva de reação) | `L` negativo nos 4 ensaios → `Kp` negativo | ❌ Método inválido para planta subamortecida |
| 3 | 04/08 | **Cancelamento de polos**, projeto em 90°, `INTEGRAL_MAX=300` | Erro de regime crescente: 45° ficava 5° abaixo, 60° ficava 18° abaixo | ❌ Teto do integral baixo demais |
| 4 | 04/08 | Idem, projeto em 75°, MF 70°, `INTEGRAL_MAX=900`<br>`Kp=0,0953 Ki=2,0912 Kd=0,0524` | Erro de regime corrigido, mas overshoot de partida de **104% (60°)** e **107% (75°)**, com ciclo-limite de 100° pico a pico em 75° | ❌ Descartado |
| 5 | 05/08 | Redução de `INTEGRAL_MAX` para 600 + anti-windup condicional | Atenuou, não resolveu. Descobriu-se que a condição era **código morto**: a saída nunca chegava a 1000, então o ramo de proteção nunca executava | ⚠️ Diagnóstico correto, correção insuficiente |
| 6 | 05/08 | Diagnóstico da causa raiz do overshoot de partida | O PID roda desde o boot do MCU, com o motor sem energia: o integral vai ao teto antes de a fonte ser ligada, e o motor recebe um **degrau em malha aberta de 600 PWM** | ✅ Explicado — vira regra de procedimento (seção 6) |
| 7 | 10/08 | **Método 2 ZN** com `L` corrigido (0,10 s), linha *some overshoot* | Ver seção 4 | ✅ **Configuração atual** |
| 8 | 10/08 | Anti-windup por **clamping dinâmico** (Åström & Hägglund) | Substitui a condição morta da tentativa 5: o integral é limitado pela folga que P e D deixam até os limites do atuador | ✅ Em uso |
| 9 | 11/08 | Correção do filtro derivativo: `N = 100 → 50` | `N·DT = 2,0` punha o filtro no limite de estabilidade | ✅ Em uso — ver 4.2 |
| 10 | 11/08 | Partida em malha aberta + pré-carga do integrador | Funcionava, mas o PWM de partida escolhido por faixa de ângulo é escalonamento disfarçado, e o sistema passava até 30 s em malha aberta após o reset | ❌ Removido |

---

## 4. Configuração validada

### 4.1 Ganhos — Ziegler-Nichols Método 2

Com o `L` real (0,10 s = ZOH de 50 Hz + atraso de empuxo) em vez do `L` falso
da seção 2.2, `Ku` e `Tu` foram recalculados em cada ponto de operação
(`projeto_zn_metodo2.py`):

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

### 4.2 Filtro do derivativo

O derivativo é implementado sobre a **medição** (Tipo B, `−Kd·θ̇`), não sobre
o erro, para não dar um chute de PWM a cada mudança de setpoint. O filtro é um
passa-baixa de 1ª ordem discreto:

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
possível sem nenhuma oscilação. Acima disso o ganho de velocidade é pequeno e
a ondulação cresce rápido.

**`N = 100` (usado até 11/08) é o pior valor possível:** `N·DT = 2,0` exatos
põem o polo em `z = −1`, ou seja, o filtro fica no limite de estabilidade e
oscila em 25 Hz (frequência de Nyquist) sem nunca amortecer. Um único pulso do
encoder faz o termo D alternar ±4,7 de PWM indefinidamente. É o único valor da
tabela que **piora até a recuperação de distúrbio**, porque a própria
oscilação segura o braço fora da faixa de ±2%.

### 4.3 Anti-windup

Quase todo o PWM de sustentação vem do integral (no regime permanente
`Kp·erro → 0`), então o integrador precisa chegar a ~450 para segurar 75°.

```c
integral += Ki * erro_atual * DT;
integral  = saturar(integral, PWM_MIN - (P + D), PWM_MAX - (P + D));  // clamp dinâmico
integral  = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);           // teto absoluto
```

O **clamp dinâmico** limita o integral pela folga que P e D deixam até os
limites do atuador: sozinho, ele nunca consegue empurrar a saída para fora de
[0, 1000]. É a forma recomendada por Åström & Hägglund, e substitui a
integração condicional da tentativa 5, que era código morto.

`INTEGRAL_MAX = 550` é o teto absoluto — dimensionado a partir da calibração
da seção 2.3 (~500 para 90°, com folga para queda de bateria). **Ver a
limitação da seção 5.1.**

### 4.4 Estágios de segurança

Válidos em módulo (positivo e negativo):

| Faixa | Ação |
|---|---|
| \|θ\| ≥ 130° | Reduz a saída em 20% |
| \|θ\| ≥ 150° | Desliga o motor e zera o integral |
| \|θ\| ≥ 170° | Trava por desequilíbrio (exige reset) |

---

## 5. Resultados de bancada

Ensaios de 10 e 11/08/2026 com a configuração da seção 4. Degrau de partida do
repouso (~8°) até o alvo, seguido de perturbações manuais ("petelecos").

| Alvo | Pico na partida | 95% do alvo | Entra em ±2% | Regime final | Erro |
|---|---|---|---|---|---|
| 30° | 30,5° (+1,7%) | 8,3 s | 10,6 s | 29,97° | −0,03° |
| 45° | 46,3° (+2,8%) | 8,4 s | 10,7 s | 45,42° | +0,42° |
| 60° | sem overshoot | 2,0 s | 9,7 s | 58,22° | −1,78° |
| 75° | 72,5° (não ultrapassou) | 2,3 s | — | 76,22° | +1,22° |

**Rejeição de perturbação** (peteleco manual, braço já estabilizado):

| Alvo | Picos atingidos | Tempo para voltar ao alvo |
|---|---|---|
| 30° | 52,5° / 66,5° | 4,0 s / 7,0 s |
| 45° | 74,8° / 86,0° | 11,0 s / 5,1 s |
| 60° | 95,2° / 111,5° | 3,9 s / 11,0 s |
| 75° | 105,8° / 131,0° | 2,5 s / 7,7 s |

**Comparação com a tentativa 4** (cancelamento de polos, 05/08):

| | Tentativa 4 | Configuração atual |
|---|---|---|
| Overshoot de partida (60°) | 104% | ~0% |
| Overshoot de partida (75°) | 107% | ~0% |
| Ciclo-limite em 75° | 100° pico a pico | ausente |
| Erro de regime | até 18° | < 2° |

A partida é **monotônica e sem oscilação**, e o braço volta ao alvo depois de
um peteleco forte (que chega a levá-lo a 131°) sem entrar em oscilação
sustentada. Esse é o comportamento que valida o projeto.

---

## 6. Limitações conhecidas

### 6.1 Alvos de 90° ou mais não são alcançados

**Observado em bancada (11/08):** com alvo de 90° o braço estabiliza em ~80°;
com alvo de 110° estabiliza mais alto, mas ainda abaixo do alvo. Como o braço
*fisicamente* passa de 90° no ensaio de 110°, não se trata de limite mecânico
nem de falta de empuxo.

**Causa provável: `INTEGRAL_MAX = 550`.** Um PID com ação integral tem erro de
regime nulo por construção — se sobra erro, o integrador está sendo bloqueado.
O integral satura em 550, e a saída total fica travada em
`550 + Kp·erro ≈ 558`, seja qual for o alvo. O braço então para no ângulo que
558 de PWM sustenta. Isso explica os dois sintomas de uma vez: o valor final é
estável (não oscila) e sobe um pouco quando o alvo sobe, porque `Kp·erro`
cresce.

**Como confirmar (1 minuto de bancada):** com o alvo em 90°, ler o PWM no
display OLED depois que o braço estabilizar.

| PWM no display | Conclusão |
|---|---|
| ~550–560 | Confirmado: o teto do integral é o limitante |
| ~1000 | O atuador está saturado — limite físico de empuxo ou queda da fonte |

**Se confirmado**, a correção é remover o teto artificial: `INTEGRAL_MAX` é
redundante com o clamp dinâmico da seção 4.3, que já impede o integral de
levar a saída para fora de [0, 1000]. Elevá-lo a 1000 (ou remover a linha)
mantém o anti-windup correto e continua sendo PID puro.

**Contrapartida a avaliar antes:** um teto mais alto piora o pico de partida
descrito na seção 6.2 — o integral tem mais espaço para carregar antes de o
braço se mover. As duas coisas precisam ser testadas juntas.

### 6.2 Pico de partida se a malha rodar com o motor sem energia

O MCU é alimentado pelo ST-Link/USB e começa a executar `PID_Atualizar()` no
boot, antes de a fonte do motor ser ligada. Nesse intervalo o braço está
parado, o erro fica cravado no valor do alvo, e o integral sobe a
`Ki · erro ≈ 140 PWM/s` até bater no teto. Quando a fonte é ligada, o motor
recebe um degrau em malha aberta de 550 PWM.

No ensaio de 60° de 05/08 isso levou o braço a **117°** (a simulação reproduz:
117,7° contra 117,25° medidos). O mesmo vale se o braço for segurado com a
malha já rodando.

Com PID puro e setpoint em degrau isso é **inevitável e está correto** — o
integrador precisa chegar a ~430 para sustentar 60°, e se o braço não pode se
mover ele vai lá de qualquer jeito. **A correção é de procedimento**, ver
seção 7.

### 6.3 Recuperação de peteleco é limitada pelo atuador, não pelos ganhos

O termo D realimenta `−Kd·θ̇`, e em tese `ζ_efetivo = ζ_planta + K·ωₙ·Kd/2`.
Chegar a `ζ = 1` exigiria `Kd ≈ 1,5` (a tabela ZN dá 0,187). Mas isso só vale
se o atuador conseguir responder na frequência de ressonância da planta
(~1,2 Hz). O empuxo da hélice tem atraso próprio (τ), e com τ acima de ~0,1 s
o termo D chega defasado — `Kd` grande **piora** a recuperação em vez de
melhorar (verificado em simulação: `Kd = 0,40` recupera em 4,3 s contra 3,3 s
de `Kd = 0,19`).

Ou seja: com esta hélice e esta taxa de 50 Hz, nenhum ajuste de PID leva a
resposta a distúrbio ao amortecimento crítico. É um limite de hardware, não de
sintonia — e é um resultado válido para o relatório, não uma deficiência.

Para melhorar seria preciso: medir τ com um degrau de PWM **em torno do ponto
de operação** (ex.: 400 → 500) em vez de partir do motor parado; e, se τ ficar
abaixo de ~0,07 s, aí sim vale subir `Kd`.

---

## 7. Protocolo de ensaio

1. **Soltar o braço antes de energizar.** Nunca segurar o braço com a malha
   rodando (seção 6.2).
2. **Ligar a fonte do motor antes ou junto com a placa.** O intervalo entre o
   boot do MCU e a energização do motor é integrado como erro.
3. Iniciar `coleta_degrau.py` e deixar rodar sem interferência até o braço
   ficar visivelmente parado por 15–20 s.
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
| `src/identifica_zn.py` | Identificação (tangente, mínimos quadrados, decremento logarítmico) |
| `src/projeto_zn_metodo2.py` | Cálculo de `Ku`/`Tu` e dos ganhos pelo Método 2 |
| `src/analise_margem_multiponto.py` | Margem de fase por ponto de operação |
| `src/simula_partida_degrau.py` | Simulação do degrau de partida com a lógica discreta de `pid.c` |
| `src/simulacao_nyquist.py` | Diagrama de Nyquist |

---

## 9. Próximos passos

1. Confirmar a causa do teto de 80° (seção 6.1) lendo o PWM no display.
2. Se confirmado, testar `INTEGRAL_MAX = 1000` avaliando junto o efeito no
   pico de partida.
3. Incluir o PWM aplicado na telemetria UART — hoje o CSV só tem
   tempo/ângulo/alvo, e foi essa lacuna que gerou o `L` falso da seção 2.2.
4. Medir o τ do empuxo com degrau em torno do ponto de operação (seção 6.3).
5. Repetir cada ensaio de identificação 2–3 vezes por ângulo e usar a média de
   ζ/ωₙ, em vez de uma única medição por ponto.
