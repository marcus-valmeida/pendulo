/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Controlador PID (Tipo B) + Feedforward + Escalonamento de Ganhos.
  *          - Feedforward (base_ff): estima o PWM que sustenta o alvo.
  *          - PID: corrige o residuo (P, I, D sobre o angulo).
  *          - ESCALONAMENTO: Kp e Kd sao reduzidos para alvos pequenos, onde
  *            o feedforward contribui pouco e os ganhos plenos (calibrados
  *            para alvos grandes, tipo 110) causam overshoot. Para alvos
  *            grandes, o fator fica em 1.0 (ganho pleno, como ja esta bom).
  *          Executado a 50 Hz EXATOS pela interrupcao do TIM4 (DT confiavel).
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
/* ============================ GANHOS BASE DO PID ==========================
 * Estes sao os ganhos PLENOS — usados integralmente para alvos grandes
 * (perto de ALVO_REF_GANHO). Para alvos menores, sao escalonados (ver
 * calcular_fator_ganho() mais abaixo).
 * ------------------------------------------------------------------------- */
static const float Kp = 1.40f;
static const float Ki = 0.30f;
static const float Kd = 0.87f;
static const float N  = 10.0f;
static const float DT = 0.020f;   // 20 ms = 50 Hz (cravado pela interrupcao)

/* ===================== ESCALONAMENTO DE GANHOS (NOVO) =====================
 * fator = |alvo| / ALVO_REF_GANHO, limitado entre FATOR_MIN e 1.0
 *   - alvo = 110 (ou mais)     -> fator = 1.0  (ganho pleno, como ja bom)
 *   - alvo = 45                -> fator ~ 0.45 (ganho reduzido, menos overshoot)
 *   - alvo pequeno (perto de 0)-> fator = FATOR_MIN (nunca fica fraco demais)
 * Aplicado em Kp e Kd (o que mais causa overshoot). Ki fica fixo — ele so
 * afeta o erro de regime, nao a velocidade da resposta.
 * ------------------------------------------------------------------------- */
static const float ALVO_REF_GANHO = 100.0f;  // angulo onde o ganho fica pleno
static const float FATOR_MIN      = 0.60f;   // piso do fator (nunca fica mole demais)

/* ================= FEEDFORWARD (base pelos dados de bancada) =============
 * sen(theta) = C * V  =>  V = sen(theta)/C  =>  PWM = V * PWM_POR_VOLT
 * ------------------------------------------------------------------------- */
static const float C_CALIBRACAO = 0.68f;
static const float PWM_POR_VOLT = 1000.0f / 4.7f;

/* =========================== LIMITES DE SAIDA =========================== */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;

/* ================ ANTI-WINDUP DO INTEGRAL =============================== */
static const float INTEGRAL_MAX = 100.0f;

/* ====================== ESTAGIOS DE SEGURANCA (modulo) ===================
 * Faixas: 130-149 reduz, 150-169 desliga, >=170 trava. Valem p/ + e -.
 * ------------------------------------------------------------------------- */
static const float LIM_REDUZIR   = 130.0f;
static const float LIM_DESLIGAR  = 150.0f;
static const float LIM_TRAVAR    = 170.0f;
static const float FATOR_REDUCAO = 0.80f;

/* ===================== MAPEAMENTO DO POTENCIOMETRO ======================= */
static const float POT_ANG_MIN = -90.0f;
static const float POT_ANG_MAX =  90.0f;
static const float ADC_MAX     = 4095.0f;

/* --------------------------- Estado interno ------------------------------ */
static FonteSetpoint fonte_setpoint = SETPOINT_CODIGO;
static float alvo_final      = 0.0f;
static float integral        = 0.0f;
static float erro_atual      = 0.0f;
static float angulo_anterior = 0.0f;
static float deriv_estado    = 0.0f;
static int32_t saida_pwm     = 0;
static uint8_t travado       = 0;
static uint8_t primeiro_ciclo = 1;
/* USER CODE END PV */

/* Prototipos privados ------------------------------------------------------*/
static float ler_setpoint_potenciometro(void);
static float saturar(float valor, float minimo, float maximo);
static float calcular_base_ff(float angulo_alvo);
static float calcular_fator_ganho(float angulo_alvo);

/* --------------------------- Inicializacao ------------------------------- */
void PID_Init(void) {
    alvo_final      = 0.0f;
    integral        = 0.0f;
    erro_atual      = 0.0f;
    angulo_anterior = 0.0f;
    deriv_estado    = 0.0f;
    saida_pwm       = 0;
    travado         = 0;
    primeiro_ciclo  = 1;
}

/* ----------------------- Configuracao da fonte --------------------------- */
void PID_SetFonte(FonteSetpoint fonte) {
    fonte_setpoint = fonte;
}

/* ------------------------- Define o alvo (codigo) ------------------------ */
void PID_SetAlvo(float angulo_alvo) {
    alvo_final = angulo_alvo;
}

/* ------------- Feedforward: PWM aproximado que sustenta o alvo ----------- */
static float calcular_base_ff(float angulo_alvo) {
    float rad = fabsf(angulo_alvo) * (3.14159265f / 180.0f);
    float tensao = sinf(rad) / C_CALIBRACAO;
    float pwm = tensao * PWM_POR_VOLT;
    return saturar(pwm, 0.0f, PWM_MAX);
}

/* ------------- Fator de escalonamento de ganho (NOVO) --------------------
 * Alvos pequenos -> fator menor (Kp/Kd mais suaves, menos overshoot).
 * Alvos grandes (>= ALVO_REF_GANHO) -> fator 1.0 (ganho pleno, ja bom). */
static float calcular_fator_ganho(float angulo_alvo) {
    float fator = fabsf(angulo_alvo) / ALVO_REF_GANHO;
    return saturar(fator, FATOR_MIN, 1.0f);
}

/* ------------- Le o potenciometro e mapeia para -90 a +90 ---------------- */
static float ler_setpoint_potenciometro(void) {
    float adc_val = 0.0f;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
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

/* ====================== CICLO PRINCIPAL =================================== */
void PID_Atualizar(void) {

    if (travado) {
        Motor_Parar();
        saida_pwm = 0;
        return;
    }

    float angulo_atual = Aeropendulo_LerAngulo();
    float ang_abs = fabsf(angulo_atual);

    // ---- SEGURANCA POR FAIXAS (da mais grave para a mais leve) ----
    if (ang_abs >= LIM_TRAVAR) {
        Motor_Parar();
        saida_pwm = 0;
        integral  = 0.0f;
        travado   = 1;
        return;
    }
    if (ang_abs >= LIM_DESLIGAR) {
        Motor_Parar();
        saida_pwm = 0;
        integral  = 0.0f;
        return;
    }

    // ---- Setpoint (codigo ou potenciometro) ----
    float alvo = alvo_final;
    if (fonte_setpoint == SETPOINT_POTENCIOMETRO) {
        alvo = ler_setpoint_potenciometro();
    }

    // ---- Fator de escalonamento (depende do tamanho do alvo) ----
    float fator_ganho = calcular_fator_ganho(alvo);
    float Kp_eff = Kp * fator_ganho;
    float Kd_eff = Kd * fator_ganho;

    // ---- Erro ----
    erro_atual = alvo - angulo_atual;

    // ---- Proporcional (com ganho escalonado) ----
    float P = Kp_eff * erro_atual;

    // ---- Integral com anti-windup (ganho fixo) ----
    integral += Ki * erro_atual * DT;
    integral = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    float I = integral;

    // ---- Derivativo TIPO B: sobre o ANGULO, com ganho escalonado ----
    float D = 0.0f;
    if (!primeiro_ciclo) {
        float deriv_angulo = (angulo_atual - angulo_anterior) / DT;
        deriv_estado += N * DT * (deriv_angulo - deriv_estado);
        D = -Kd_eff * deriv_estado;
    }
    angulo_anterior = angulo_atual;
    primeiro_ciclo  = 0;

    // ---- Feedforward: base calculada a partir do alvo (dados de bancada) --
    float base_ff = calcular_base_ff(alvo);

    // ---- Saida = feedforward (empurrao) + PID escalonado ----
    float saida = base_ff + P + I + D;
    saida = saturar(saida, PWM_MIN, PWM_MAX);

    // ---- Faixa 130 a 149 : reduz 20% ----
    if (ang_abs >= LIM_REDUZIR) {
        saida *= FATOR_REDUCAO;
    }

    saida_pwm = (int32_t)saida;
    Motor_SetPWM(saida_pwm);
}

/* ---------------------------- Getters ------------------------------------ */
float   PID_GetAlvo(void)    { return alvo_final; }
float   PID_GetErro(void)    { return erro_atual; }
int32_t PID_GetSaida(void)   { return saida_pwm; }
uint8_t PID_GetTravado(void) { return travado; }
