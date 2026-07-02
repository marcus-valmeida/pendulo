/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Controlador PID de posicao para o Aeropendulo - implementacao.
  *          LE do aeropendulo.c e ESCREVE no motor.c.
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "pid.h"
#include "aeropendulo.h"
#include "motor.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern ADC_HandleTypeDef hadc1;

/* ============================ AJUSTES DO PID ==============================
 * Ganhos iniciais do TCC. Ajuste na bancada conforme o comportamento.
 * ------------------------------------------------------------------------- */
static const float Kp = 4.0f;     // proporcional — forca proporcional ao erro
static const float Ki = 1.0f;     // integral — elimina erro de regime
static const float Kd = 1.5f;     // derivativo — freia perto do alvo (anti-supapo)
static const float N  = 50.0f;    // filtro do derivativo (evita ruido no D)

static const float DT = 0.020f;   // 20 ms = 50 Hz (casa com o HAL_Delay do main)

/* ======================= POTENCIA DE SUSTENTACAO ========================= */
static const float POTENCIA_BASE = 380.0f;  // PWM base p/ sustentar (0-1000)

/* =========================== LIMITES DE SAIDA =========================== */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;

/* ================= LIMITE DE AUTORIDADE DO PID (NOVO) ====================
 * Este e o ponto-chave para evitar o "360 graus" por supapo.
 * O PID (P+I+D) NUNCA pode empurrar mais que este limite alem da base.
 * Assim, mesmo com erro gigante, o motor nao recebe um chute violento —
 * a potencia fica sempre proxima da media que ja funcionava bem.
 * ------------------------------------------------------------------------- */
static const float LIMITE_ACAO_PID = 300.0f;  // teto do |P+I+D| (ajuste fino)

/* ======================= RAMPA SUAVE DO SETPOINT ======================== */
static const float RAMPA_SETPOINT = 1.0f;   // graus por ciclo (1.0 * 50Hz = 50 graus/s)

/* ================ ZONA DE PARTIDA — ANTI-SUPAPO INICIAL (NOVO) ===========
 * Durante os primeiros ciclos, seguramos o integral para ele nao acumular
 * e dar aquele "chute" inicial (o leve supapo que voce notou na partida).
 * ------------------------------------------------------------------------- */
static const float INTEGRAL_MAX = 200.0f;   // teto do acumulo integral (era PWM_MAX)

/* ====================== ESTAGIOS DE SEGURANCA (modulo) ===================
 * ALTERADO conforme pedido:
 *   |ang| >= 150  -> corta 20% da potencia (antes era 120)
 *   |ang| >= 170  -> desliga o motor        (antes era 150)
 *   |ang| >= 180  -> trava por desequilibrio
 * ------------------------------------------------------------------------- */
static const float LIM_REDUZIR   = 150.0f;  // corta 20% (era 120)
static const float LIM_DESLIGAR  = 170.0f;  // desliga    (era 150)
static const float LIM_TRAVAR    = 180.0f;  // desequilibrio — trava ate reset
static const float FATOR_REDUCAO = 0.80f;   // mantem 80% (corta 20%)

/* ===================== MAPEAMENTO DO POTENCIOMETRO ======================= */
static const float POT_ANG_MIN = -90.0f;
static const float POT_ANG_MAX =  90.0f;
static const float ADC_MAX     = 4095.0f;

/* --------------------------- Estado interno ------------------------------ */
static FonteSetpoint fonte_setpoint = SETPOINT_CODIGO;
static float alvo_final    = 0.0f;
static float alvo_rampa    = 0.0f;
static float integral      = 0.0f;
static float deriv_estado  = 0.0f;
static float erro_anterior = 0.0f;
static float erro_atual    = 0.0f;
static int32_t saida_pwm   = 0;
static uint8_t travado     = 0;
/* USER CODE END PV */

/* Prototipos privados ------------------------------------------------------*/
static float ler_setpoint_potenciometro(void);
static float saturar(float valor, float minimo, float maximo);

/* --------------------------- Inicializacao ------------------------------- */
void PID_Init(void) {
    alvo_final    = 0.0f;
    alvo_rampa    = 0.0f;
    integral      = 0.0f;
    deriv_estado  = 0.0f;
    erro_anterior = 0.0f;
    erro_atual    = 0.0f;
    saida_pwm     = 0;
    travado       = 0;
}

/* ----------------------- Configuracao da fonte --------------------------- */
void PID_SetFonte(FonteSetpoint fonte) {
    fonte_setpoint = fonte;
}

/* ------------------------- Define o alvo (codigo) ------------------------ */
void PID_SetAlvo(float angulo_alvo) {
    alvo_final = angulo_alvo;
}

/* --------------- Le o potenciometro e mapeia para -90..+90 --------------- */
static float ler_setpoint_potenciometro(void) {
    float adc_val = 0.0f;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_val = (float)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    float faixa = POT_ANG_MAX - POT_ANG_MIN;
    return POT_ANG_MIN + (adc_val / ADC_MAX) * faixa;
}

/* ---------------------------- Saturacao ---------------------------------- */
static float saturar(float valor, float minimo, float maximo) {
    if (valor > maximo) return maximo;
    if (valor < minimo) return minimo;
    return valor;
}

/* ====================== CICLO PRINCIPAL DO PID =========================== */
void PID_Atualizar(void) {
    // ---- 0. Se travou por desequilibrio, motor desligado ate reset ----
    if (travado) {
        Motor_Parar();
        saida_pwm = 0;
        return;
    }

    // ---- 1. Le o angulo atual (encoder ja calibrado) ----
    float angulo_atual = Aeropendulo_LerAngulo();
    float ang_abs = fabsf(angulo_atual);

    // ---- 2. SEGURANCA em modulo (vale p/ + e -) ----
    // Estagio 3 — desequilibrio: passou de 180. Trava.
    if (ang_abs >= LIM_TRAVAR) {
        Motor_Parar();
        saida_pwm = 0;
        integral  = 0.0f;
        travado   = 1;
        return;
    }
    // Estagio 2 — passou de 170. Desliga (nao trava).
    if (ang_abs >= LIM_DESLIGAR) {
        Motor_Parar();
        saida_pwm = 0;
        integral  = 0.0f;
        return;
    }

    // ---- 3. Rampa suave do setpoint ----
    if (fonte_setpoint == SETPOINT_POTENCIOMETRO) {
        alvo_final = ler_setpoint_potenciometro();
    }
    float delta_alvo = alvo_final - alvo_rampa;
    if (delta_alvo >  RAMPA_SETPOINT) delta_alvo =  RAMPA_SETPOINT;
    if (delta_alvo < -RAMPA_SETPOINT) delta_alvo = -RAMPA_SETPOINT;
    alvo_rampa += delta_alvo;

    // ---- 4. Erro ----
    erro_atual = alvo_rampa - angulo_atual;

    // ---- 5. Proporcional ----
    float P = Kp * erro_atual;

    // ---- 6. Integral com anti-windup reforcado ----
    integral += Ki * erro_atual * DT;
    integral = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX); // teto menor = menos supapo
    float I = integral;

    // ---- 7. Derivativo com filtro (N) ----
    float deriv_erro = (erro_atual - erro_anterior) / DT;
    deriv_estado += N * DT * (deriv_erro - deriv_estado);
    float D = Kd * deriv_estado;
    erro_anterior = erro_atual;

    // ---- 8. Acao de controle do PID ----
    float acao_pid = P + I + D;

    // ---- 9. LIMITE DE AUTORIDADE (NOVO) — resolve o "360 graus" ----
    // Nao importa quao grande fique o erro: o PID nunca empurra alem
    // deste teto. Isso impede o chute violento que rodava o pendulo.
    acao_pid = saturar(acao_pid, -LIMITE_ACAO_PID, LIMITE_ACAO_PID);

    // ---- 10. Soma com a base de sustentacao ----
    float saida = acao_pid + POTENCIA_BASE;

    // ---- 11. Saturacao final (0 a 1000) ----
    saida = saturar(saida, PWM_MIN, PWM_MAX);

    // ---- 12. Estagio 1 de seguranca — reduz 20% acima de 150 ----
    if (ang_abs >= LIM_REDUZIR) {
        saida *= FATOR_REDUCAO;
    }

    // ---- 13. Aplica no motor ----
    saida_pwm = (int32_t)saida;
    Motor_SetPWM(saida_pwm);
}

/* ---------------------------- Getters ------------------------------------ */
float   PID_GetAlvo(void)    { return alvo_rampa; }
float   PID_GetErro(void)    { return erro_atual; }
int32_t PID_GetSaida(void)   { return saida_pwm; }
uint8_t PID_GetTravado(void) { return travado; }
