# Sintonia do PID — Aeropêndulo (PIBIC)

Última atualização: 2026-08-04

## 1. Onde estávamos

O controlador em `src/pid.c` apresentava oscilação forte em malha fechada
(registrada em `src/telemetria_aeropendulo.csv`: oscilações de 25° a 80° em
torno de um alvo de 60°, estabilizando depois em ~53° em vez de 60°).

## 2. Causas identificadas no código

### 2.1 Filtro derivativo instável (bug, corrigido — pendente de re-teste)

```c
deriv_estado += N * DT * (deriv_angulo - deriv_estado);
```

É a discretização de Euler de um filtro passa-baixa `D(s) = N/(s+N)`. Com
`alpha = N*DT`, a recorrência só é bem-comportada para `alpha << 1`, e só é
numericamente estável para `alpha < 2`. Com `DT = 0.02s` (loop a 50 Hz):

| N   | alpha = N·DT | Situação |
|-----|--------------|----------|
| 10 (valor original) | 0.20 | estável, filtro correto |
| 100 (testado) | 2.00 | fronteira exata da instabilidade |
| 200 (estava em uso) | 4.00 | instável — diverge geometricamente |

Simulação numérica confirmou: só com o ruído de quantização do encoder
(±0.25°, pêndulo parado), `N=200` faz `deriv_estado` divergir de dezenas para
~10²² em 40 ciclos (0.8s), alternando de sinal — produzindo chutes de PWM
gigantes e alternados no motor. Isso por si só explica boa parte da
oscilação observada, independente de qualquer ganho Kp/Ki/Kd escolhido.
**Ação**: usar `N` entre 8 e 20 quando o PID for retestado.

### 2.2 Feedforward e escalonamento de ganhos removidos (não é mais um caminho a seguir)

Um diff anterior (não commitado) havia removido o feedforward
(`calcular_base_ff`) e o escalonamento de ganhos (`calcular_fator_ganho`)
presentes no commit `7270df6`, subindo `Ki` de 0.30 para 16.0.

**Decisão do orientador**: feedforward e escalonamento de ganhos por faixa de
ângulo são tratados como gambiarra — fora da formulação clássica de PID do
Ogata. A partir daqui o controlador deve ser **PID puro, ganho único e fixo**,
sintonizado por um método formal.

### 2.3 Perda de dados de identificação (corrigido)

`coleta_degrau.py` salvava sempre no mesmo nome de arquivo (`mode='w'`),
sobrescrevendo ensaios anteriores. **Corrigido**: agora salva com timestamp
(`telemetria_AAAAMMDD_HHMMSS.csv`), então cada ensaio fica preservado.

## 3. Material externo (outro chat) — o que é aproveitável e o que não é

Parte da sintonia sugerida em outra conversa (fora deste projeto) foi
conferida contra os dados reais do repositório:

- Ganhos do TCC do colega (`Kp=1.9, Ki=9.694, Kd=0.0788, N=843.7`) —
  **conferem** com o PDF original.
- Fórmulas do Método 2 de Ziegler-Nichols (`Kp=0.6Ku`, `Ti=0.5Pu`,
  `Td=0.125Pu`) — **corretas**, batem com o material da disciplina.
- Um `Ki=47.10` citado como causa da oscilação — **não existe** em nenhum
  commit nem no código atual.
- Uma tabela de "ganhos extraídos de CSV" para 60° e 75° (picos de 81.25° e
  119.75°, tempos de pico específicos) — **nenhum desses valores aparece**
  em `telemetria_aeropendulo.csv` (que só cobre 9s de um ensaio de malha
  *fechada* com alvo=60°, nunca passando de 65.75°). Não verificável.
- A tabela final de "escalonamento de ganhos" por faixa de ângulo —
  reintroduz gain scheduling, o que o próprio orientador já havia
  descartado no mesmo texto. Contradição interna.

**Regra adotada**: nenhum ganho entra no código ou no relatório sem uma
fonte verificável no repositório (CSV de um ensaio real, ou fórmula
localizável no material da disciplina).

## 4. Revisão dos 4 PDFs da disciplina

### 4.1 `4_AcoesBasicasControle-2.pdf` (slides 122-131) — Método 1 (curva de reação)

Modelo: `H(s) = K/(Ts+1) · e^(-Ls)`.

Dois jeitos de extrair K, T, L de uma resposta ao degrau:

1. **Método da tangente** (slides 123-125): traça a reta tangente no ponto
   de inflexão (derivada máxima) da curva; a interseção com o eixo do tempo
   dá `L`, e `T` é o tempo (menos `L`) para atingir 63.2% do valor final.
2. **Método dos mínimos quadrados** (slides 128-131, eq. 190-203): integra a
   curva de resposta em vários instantes crescentes e resolve por mínimos
   quadrados para `K`, `L·K` e `T` — bem mais robusto a ruído do que ler a
   tangente a olho.

Tabela de ganhos (Método 1, PID):
```
Kp = 1.2 * T / (K * L)
Ti = 2L         ->  Ki = Kp / Ti
Td = 0.5L       ->  Kd = Kp * Td
```

### 4.2 `7_BodeeNyquist.pdf` (slides 94-102) — Método 2 (ganho crítico)

Confirma a tabela já usada antes:
```
Kp = 0.6 * Ku
Ti = 0.5 * Pu   ->  Ki = Kp / Ti
Td = 0.125 * Pu ->  Kd = Kp * Td
```
Também apresenta o método do relé como alternativa mais segura a subir Kp
manualmente até a oscilação sustentada (evita chegar perto da instabilidade
sem controle da amplitude).

### 4.3 `6_LugardasRaizes.pdf` (slides 85-92) — método adicional disponível

Existe também um método de projeto analítico via lugar das raízes
(alocação do polo dominante `s0 = -ζωn + jωn√(1-ζ²)` e cálculo de Kp/Ki por
condição de ângulo e módulo). Não é gain scheduling — usa ganho único fixo
para um polo de malha fechada desejado. Fica registrado como alternativa
válida caso o Método 1 não dê uma resposta satisfatória, mas não é o
caminho escolhido agora.

### 4.4 `5_Estabilidade.pdf`

Sem menção direta a sintonia de PID; conteúdo de critérios de estabilidade
(Routh-Hurwitz), contexto geral, não diretamente aplicado ainda.

## 5. Caminho escolhido: Método 1, com o que já existe no código

O firmware já tem um modo de malha aberta (`MODO_MALHA_ABERTA` em
`main.c`) e `coleta_degrau.py` já sabe capturar e salvar a curva. Não é
necessário arriscar o ensaio de ganho crítico (que exige chegar perto da
instabilidade) — o Método 1 é mais seguro e reaproveita a infraestrutura
existente.

### O que já foi feito no código (2026-08-04)

- `src/main.c`: `MODO_ATUAL` alterado para `MODO_MALHA_ABERTA`; o degrau de
  teste (`angulo_m_a`) definido como `PWM_60_GRAUS` (317) — mesmo alvo já
  usado em `PID_SetAlvo(60.0f)` em malha fechada, para identificar a planta
  no ponto onde ela realmente vai operar.
- `src/coleta_degrau.py`: salva cada ensaio com nome único
  (`telemetria_AAAAMMDD_HHMMSS.csv`).
- `src/identifica_zn.py` (novo): lê o CSV do ensaio de malha aberta, estima
  K, T, L pelos dois métodos do slide 4 (tangente e mínimos quadrados),
  compara os dois, calcula os ganhos pela tabela do Método 1, e plota o
  ajuste sobre os dados reais para inspeção visual. Validado com dados
  sintéticos (K, T, L conhecidos + ruído de quantização de 0.25°): erro
  menor que 2.5% na recuperação dos três parâmetros.

### Alerta técnico

Ganho puro do Método 1 tende a ficar agressivo quando `T/L` é grande (atraso
pequeno frente à constante de tempo) — situação esperada aqui, já que o
aeropêndulo não tem grande atraso de transporte (diferente de processos
térmicos, onde Ziegler-Nichols Método 1 foi pensado originalmente). Depois
de calcular Kp/Ki/Kd, **checar as margens de estabilidade** com
`src/simulacao_nyquist.py` (atualizando a planta com o K, T, L
recém-identificados) antes de gravar os ganhos em `pid.c`.

## 6. Execução: o que é manual e o que é automático

- **Manual, obrigatório** (não há como eu fazer isso remotamente — não há
  hardware conectado a este ambiente): compilar e gravar o firmware com
  PlatformIO, conectar a placa fisicamente, alimentar o motor, rodar
  `coleta_degrau.py` observando o pêndulo, e interromper (Ctrl+C) quando a
  curva estabilizar. É um teste físico com motor/hélice girando — precisa
  de alguém presente para segurança.
- **Automático, a partir do CSV gerado**: assim que o arquivo
  `telemetria_AAAAMMDD_HHMMSS.csv` existir em `src/`, basta avisar (ou
  apontar o nome do arquivo) que eu rodo
  `python3 src/identifica_zn.py <arquivo> --pwm 317` e trago o resultado —
  K, T, L, os ganhos calculados e o gráfico de validação.

## 6.1 Protocolo de ensaio (aprendido na prática)

O primeiro ensaio a 15° (PWM=120) saiu contaminado: a fonte foi ligada, e
depois disso houve toques manuais na hélice durante a captura, produzindo um
segundo pico (~21° em t≈14s) que não é dinâmica da planta, é perturbação
externa. Dado descartado para fins de identificação.

**Regra a partir de agora**: nenhum toque no braço/hélice depois que a
placa for ligada. Iniciar `coleta_degrau.py` o quanto antes após ligar a
fonte (o degrau é aplicado assim que o firmware entra no `main()`), e deixar
rodar sem interferência até ficar visivelmente parado por 15-20s no final.

## 6.2 Resultados reais de identificação (ensaios de bancada)

Todos os ensaios seguiram o protocolo da secao 6.1 (sem toque apos ligar).
Os dois metodos de 1a ordem (tangente e minimos quadrados) nao se aplicam
bem a essa planta — a resposta real e de 2a ordem subamortecida (oscila,
nao e uma curva em "S" monotona), entao os ganhos abaixo vem do metodo do
decremento logaritmico (`metodo_2a_ordem_decremento_log` em
`identifica_zn.py`), que extrai zeta e omega_n direto da razao entre
amplitudes pico-vale sucessivas — imune a deriva lenta de regime.

| Angulo | PWM | zeta | omega_n (rad/s) | L (s) | K (graus/PWM) | Observacao |
|--------|-----|------|------------------|-------|---------------|------------|
| 15°    | 120 | nao resolvido (T mais rapido que a amostragem) | - | ~1.35 | ~0.098 | regiao rigida (perto de 0°); resposta quase instantanea apos o atraso |
| 30°    | 210 | 0.130 | 8.38 | 0.70 | 0.136 | cauda assentou bem em ~55-60s |
| 45°    | 295 | 0.146 | 8.14 | 1.00 | 0.129 (incerto) | cauda ainda variando -0.64° no fim do ensaio (83s) — K impreciso |
| 60°    | 410 | 0.171 | 7.70 | 0.85 | 0.139 (incerto) | cauda ainda variando -1.08° no fim (51s) |
| 75°    | 470 | **0.073 (suspeito)** | 7.16 | 1.40 | 0.151 (incerto) | zeta quebra a tendencia crescente — provavel contaminacao do 2o ciclo pela deriva lenta (cauda variou -1.48°); nao usar isoladamente sem re-ensaio |
| 90°    | 500 | 0.165 | 5.83 | 1.53 | 0.198 (bem incerto) | cauda ainda variando +3.61° no fim (46s) — K pouco confiavel |

**Tendencia fisica confirmada**: `omega_n` cai de forma monotonica com o
angulo (8.38 -> 8.14 -> 7.70 -> 7.16 -> 5.83 rad/s), exatamente como
esperado pela linearizacao `omega_n = sqrt(mgL*cos(theta0)/I)` — o "efeito
mola" da gravidade enfraquece conforme o braco sobe. Esse resultado bate
com a teoria e e um bom ponto para o relatorio.

O ponto de 75° quebra a tendencia de `zeta` crescente (0.130, 0.146, 0.171,
**0.073**, 0.165) — suspeita-se de contaminacao do decremento logaritmico
pela deriva lenta (que nesse ensaio ja apareceu forte no 2o ciclo de
oscilacao, distorcendo a razao pico-vale). Tratar como outlier ate re-testar.

A deriva lenta (aviso de cauda) apareceu em **todos** os ensaios de 45° a
90°, e so nao apareceu no de 30° — quanto maior o PWM/angulo, mais forte e
mais lenta essa segunda dinamica (provavel aquecimento do motor sob carga
sustentada). Os valores de `K` de 45° em diante devem ser tratados como
aproximados.

Descoberta real (nao mais estimativa de fora): a planta tem uma segunda
dinamica, bem mais lenta (dezenas de segundos), sobreposta a oscilacao
mecanica rapida — provavelmente aquecimento do motor ou acomodacao da
fonte sob carga sustentada. Essa dinamica lenta esta muito abaixo da
frequencia de cruzamento de qualquer PID razoavel para este sistema
(ωn≈8 rad/s De oscilacao rapida vs. dezenas de segundos da deriva lenta),
entao nao deve afetar margem de estabilidade — mas explica porque o
metodo de 1a ordem (Metodo 1 de Ziegler-Nichols) nunca convergia direito
nos dados reais.

`ζ≈0.13-0.15` medido e mais baixo (mais oscilatorio) do que os valores
~0.28-0.34 citados antes por uma fonte externa nao verificavel — agora
temos numero real, de bancada, com metodo documentado.

## 6.3 Primeira rodada de testes com o candidato (projeto em 90°) — problemas encontrados

Gravamos `Kp=0.098, Ki=1.73, Kd=0.051` (projeto em 90°, MF alvo 60°) e
`N=15` e testamos em malha fechada (30°, 45°, 60°) e reensaiamos o 75° em
malha aberta. Resultado:

- **75° reensaiado**: `zeta=0.144, omega_n=6.32, L=1.07, K=0.156` —
  confirma que o `zeta=0.073` da rodada anterior era mesmo defeituoso
  (contaminado pela deriva lenta). Tabela da secao 6.2 atualizada.
- **Erro de regime crescente com o angulo** (30° rastreia bem; 45° fica
  ~5° abaixo do alvo; 60° fica ~18° abaixo). Causa: com `Kp` tao pequeno
  (cancelamento de polos), o proporcional quase nao contribui no regime
  permanente — praticamente todo o PWM de sustentacao tem que vir do
  integral, mas `INTEGRAL_MAX=300` (calibrado para o `Ki=16` antigo) trava
  o integral antes de fechar o erro em alvos grandes (60° precisa de
  ~410 PWM de regime, acima do teto de 300).
- **Oscilacao grande apos um disturbio manual ("peteleco")**: a resposta ao
  degrau em si fica razoavel, mas um flick forte no braco excita uma
  oscilacao bem maior do que o esperado. Isso e uma fragilidade conhecida
  do cancelamento de polos: se `zeta`/`omega_n` identificados nao forem
  exatos, a cancelacao fica imperfeita e o modo natural (levemente
  amortecido) da planta "vaza" e toca quando excitado por um disturbio
  abrupto, mesmo que o rastreamento da referencia pareca bem comportado.

### Correcoes aplicadas

- **Ponto de projeto excluindo o 90°** (ensaio pouco confiavel — K "bem
  incerto" na tabela 6.2): novo pior caso e o 75° reensaiado
  (`omega_n=6.32`, o mais baixo entre os pontos confiaveis).
- **Margem de fase alvo subida de 60° para 70°**: mais conservador, para
  dar folga a imprecisao da cancelacao de polos (Kp/Ki/Kd menores e menos
  agressivos). Isso reduz a severidade do "toque" apos disturbio, mas nao
  elimina — a causa raiz e a incerteza do modelo, nao so a margem.
- **`INTEGRAL_MAX` subido de 300 para 900** (de 1000 de PWM_MAX, deixando
  ~100 de folga para P+D): corrige o erro de regime, ja que agora quase
  todo o esforco de sustentacao vem do integral.

Novos ganhos gravados: `Kp=0.0953, Ki=2.0912, Kd=0.0524`.

**Limitacao registrada para o relatorio**: o cancelamento de polos e
sensivel a precisao do modelo. Uma melhoria futura seria repetir cada
ensaio de identificacao (2-3 vezes por angulo) e usar a media de
zeta/omega_n, em vez de uma unica medicao por ponto.

## 6.4 Por que o "peteleco" ainda oscila — e por que Kd/N não resolvem

Pergunta natural: aumentar `Kd` (mais amortecimento) ou `N` (derivativo
menos filtrado, mais "rapido") reduziria a oscilacao apos um disturbio
manual forte? Testado numericamente (impulso de disturbio na planta de
75° identificada, simulando o peteleco, com `control.impulse_response`):

**Variando so Kd** (Kp/Ki fixos nos valores gravados):

| Kd | Margem de fase | Pico do impulso | Assentamento |
|----|-----------------|------------------|--------------|
| 0.026 | 70.0° | 0.760 | 6.6s |
| 0.052 (atual) | 70.0° | 0.748 | 5.9s |
| 0.105 (2x) | 70.1° | 0.725 | 6.0s |
| 0.210 (4x) | 70.2° | 0.680 | 5.8s |

Quadruplicar `Kd` reduz o pico em menos de 10%.

**Variando a agressividade total do projeto** (Kp/Ki/Kd juntos, via margem
de fase alvo — banda do controlador `wc` sobe conforme a margem alvo cai):

| MF alvo | wc (rad/s) | Pico do impulso | Assentamento |
|---------|------------|------------------|--------------|
| 70°  | 0.33 | 0.748 | 5.9s |
| 45°  | 0.73 | 0.735 | 6.7s |
| 15° (bem agressivo) | 1.22 | 0.720 | 15.0s (pior) |

**Variando N** (com derivativo filtrado de verdade na simulacao, nao mais
ideal): de N=15 ate N=49 (quase no limite de instabilidade, `N*DT=0.98`),
pico e assentamento praticamente nao mudam (0.755 -> 0.750), e batem quase
exatamente com o derivativo ideal sem filtro nenhum (0.748).

**Causa raiz**: a planta oscila naturalmente em `omega_n≈6.3 rad/s`, mas o
atraso de transporte (`L≈1.07s` em 75°) limita a banda alcancavel do
controlador a no maximo ~1.2 rad/s antes da margem de fase desmoronar
(cada rad/s de banda "custa" `L*wc` radianos de atraso de fase). O loop de
realimentacao e, na melhor das hipoteses, ~5x mais lento que a oscilacao
que precisaria amortecer — qualquer acao de controle (P, I ou D, filtrada
ou nao) tem que atravessar esse atraso antes de afetar a planta de novo,
entao by the time ela "chega", a oscilacao natural ja rodou varios ciclos
sozinha. Isso e um limite classico de banda-vs-atraso (Bode/Nyquist), nao
um problema de sintonia — nenhuma combinacao estavel de Kp/Ki/Kd/N supera
isso.

O que se observa no peteleco e, na pratica, o amortecimento **natural** da
propria planta (`zeta≈0.13-0.17`) decaindo sozinho — o PID nao contribui
de forma relevante nesse transiente especifico, nem para o bem nem para o
mal.

### O que ajudaria de verdade

1. **Amortecimento fisico** (atrito/dashpot no eixo) — aumenta o `zeta` da
   planta diretamente, unica forma real de acelerar essa dinamica.
2. **Reduzir o atraso `L`**, se parte dele vier de algo corrigivel (vale
   investigar a origem — resposta do motor/helice, filtragem, etc.).
3. **Aceitar como caracteristica fisica documentada**: se o rastreamento
   de referencia (sem disturbio artificial forte) estiver bom, o tempo de
   recuperacao de um peteleco pode ser uma limitacao inerente da planta,
   nao uma falha de projeto — e um resultado real e defensavel para o
   relatorio (limite classico banda-atraso), nao uma deficiencia a
   esconder.

## 7. Próximos passos

1. Compilar e gravar o firmware (`MODO_MALHA_ABERTA` já ativado).
2. Rodar `coleta_degrau.py` no ensaio de degrau (PWM=317, alvo ~60°).
3. Rodar `identifica_zn.py` sobre o CSV gerado — obter K, T, L e os ganhos
   do Método 1.
4. Checar margens de estabilidade com `simulacao_nyquist.py` usando o
   modelo identificado.
5. Gravar `Kp`, `Ki`, `Kd` em `pid.c` (ganho único, sem feedforward, sem
   escalonamento) e corrigir `N` para a faixa estável (8-20).
6. Testar em malha fechada e comparar com o `telemetria_aeropendulo.csv`
   anterior.
