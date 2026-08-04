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
 * Ganhos calculados por cancelamento de polos sobre o modelo de 2a ordem +
 * atraso identificado na bancada (decremento logaritmico — ver
 * docs/diagnostico_oscilacao_pid.md, secao 6.2/6.3).
 * Ponto de projeto: 75 graus (pior caso confiavel de omega_n; o ensaio de
 * 90 graus foi excluido por baixa confiabilidade). Margem de fase alvo
 * 70 graus (mais conservadora que os 60 graus originais, para dar folga
 * ao fato de que a cancelacao de polos depende de zeta/omega_n medidos
 * com incerteza — um disturbio forte tipo "peteleco" pode excitar o modo
 * natural da planta se a cancelacao nao for perfeita).
 * Validado contra os pontos de 30/45/60/75 via analise_margem_multiponto.py:
 * margem de fase minima 70 graus (no proprio ponto de projeto), sobe ate
 * ~79 graus em 30 graus — nenhum escalonamento de ganho necessario.
 * ------------------------------------------------------------------------- */
static const float Kp = 0.0953f;
static const float Ki = 2.0912f;
static const float Kd = 0.0524f;
static const float N  = 15.0f;    // N*DT=0.3 — filtro estavel (ver docs, secao 2.1)
static const float DT = 0.020f;   // 20 ms = 50 Hz
    
/* =========================== LIMITES DE SAIDA =========================== */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;

/* ================ ANTI-WINDUP DO INTEGRAL ===============================
 * Kp agora e pequeno (cancelamento de polos), entao o proporcional quase
 * nao contribui no regime permanente — quase todo o PWM de sustentacao
 * (ate ~500 em 90 graus, pela calibracao de malha aberta) tem que vir do
 * integral. Um teto baixo aqui (o antigo 300, calibrado para o Ki=16
 * anterior) trava o integral antes de fechar o erro em alvos grandes —
 * foi a causa do erro de regime visto nos ensaios de 45/60 graus.
 * ------------------------------------------------------------------------- */
static const float INTEGRAL_MAX = 900.0f;

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

    // ---- Erro ----
    erro_atual = alvo - angulo_atual;

    // ---- Proporcional (com ganho escalonado) ----
    float P = Kp * erro_atual;

    // ---- Integral com anti-windup (ganho fixo) ----
    integral += Ki * erro_atual * DT;
    integral = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    float I = integral;

    // ---- Derivativo TIPO B: sobre o ANGULO, com ganho escalonado ----
    float D = 0.0f;
    if (!primeiro_ciclo) {
        float deriv_angulo = (angulo_atual - angulo_anterior) / DT;
        deriv_estado += N * DT * (deriv_angulo - deriv_estado);
        D = -Kd * deriv_estado;
    }
    angulo_anterior = angulo_atual;
    primeiro_ciclo  = 0;

    // ---- Saida = feedforward (empurrao) + PID escalonado ----
    float saida = P + I + D;
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
