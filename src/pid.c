/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Controlador PID (Tipo B) + Feedforward de posicao para o Aeropendulo.
  *          - Feedforward (base_ff): estima o PWM que sustenta o alvo, a
  *            partir dos dados de bancada (sen(theta) = C * V). Da o
  *            "empurrao inicial" para o pendulo nao partir do zero.
  *          - PI: corrige o residuo que o feedforward nao acerta sozinho.
  *          - D: sobre o ANGULO, amortece oscilacao e o "tapa".
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
/* ============================ GANHOS DO PID ==============================
 * Kp e Ki mantidos (seus valores ajustados). D adicionado para amortecer
 * a oscilacao em angulos maiores e o balanco apos o "tapa".
 * ------------------------------------------------------------------------- */
static const float Kp = 1.20f;    // proporcional (corrige residuo)
static const float Ki = 0.15f;    // integral (zera erro estacionario)
static const float Kd = 0.40f;    // derivativo (amortecimento) — comece baixo
static const float N  = 10.0f;    // filtro do derivativo (evita ruido)
static const float DT = 0.020f;   // 20 ms = 50 Hz (cravado pela interrupcao)

/* ================= FEEDFORWARD (base pelos dados de bancada) =============
 * sen(theta) = C * V  =>  V = sen(theta)/C  =>  PWM = V * PWM_POR_VOLT
 * C medido na bancada. Da o PWM aproximado que sustenta o angulo alvo.
 * ------------------------------------------------------------------------- */
static const float C_CALIBRACAO = 0.68f;          // constante medida
static const float PWM_POR_VOLT = 1000.0f / 4.7f; // fonte de 4.7V -> PWM

/* =========================== LIMITES DE SAIDA =========================== */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;

/* ================ ANTI-WINDUP DO INTEGRAL ===============================
 * Menor que sem feedforward: como a base ja entrega a maior parte, o
 * integral so precisa de uma margem pequena para aparar o residuo.
 * ------------------------------------------------------------------------- */
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
static float angulo_anterior = 0.0f;   // para a derivada do angulo (Tipo B)
static float deriv_estado    = 0.0f;   // estado do filtro do derivativo
static int32_t saida_pwm     = 0;
static uint8_t travado       = 0;
static uint8_t primeiro_ciclo = 1;     // evita derivada gigante no 1o ciclo
/* USER CODE END PV */

/* Prototipos privados ------------------------------------------------------*/
static float ler_setpoint_potenciometro(void);
static float saturar(float valor, float minimo, float maximo);
static float calcular_base_ff(float angulo_alvo);
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
float tensao = sinf(rad) / C_CALIBRACAO;   // V necessaria
float pwm = tensao * PWM_POR_VOLT;         // converte para PWM
return saturar(pwm, 0.0f, PWM_MAX);
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
if (ang_abs >= LIM_TRAVAR) {                 // >= 170 : trava
Motor_Parar();
        saida_pwm = 0;
        integral  = 0.0f;
        travado   = 1;
return;
    }
if (ang_abs >= LIM_DESLIGAR) {               // 150 a 169 : desliga
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
// ---- Proporcional ----
float P = Kp * erro_atual;
// ---- Integral com anti-windup ----
    integral += Ki * erro_atual * DT;
    integral = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
float I = integral;
// ---- Derivativo TIPO B: sobre o ANGULO, com filtro e sinal negativo ----
// Reage so ao movimento fisico do pendulo (imune a salto de setpoint).
float D = 0.0f;
if (!primeiro_ciclo) {
float deriv_angulo = (angulo_atual - angulo_anterior) / DT;
        deriv_estado += N * DT * (deriv_angulo - deriv_estado);
        D = -Kd * deriv_estado;   // sinal negativo: freia o movimento
    }
    angulo_anterior = angulo_atual;
    primeiro_ciclo  = 0;
// ---- Feedforward: base calculada a partir do alvo (dados de bancada) --
float base_ff = calcular_base_ff(alvo);
// ---- Saida = feedforward (empurrao) + PID (ajuste fino) ----
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
