/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Controlador PID (Tipo B) puro — ganho unico e fixo, sem
  *          feedforward e sem escalonamento de ganhos.
  *          Ganhos por Ziegler-Nichols Metodo 2, linha "some overshoot".
  *          Memorial de calculo e ensaios: docs/diagnostico_oscilacao_pid.md
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
 * Ziegler-Nichols Metodo 2 no pior ponto da faixa (30 graus: Ku=2.47,
 * Tu=0.690 s), linha "some overshoot". Deducao na secao 4 do diagnostico.
 * ------------------------------------------------------------------------- */
static const float Kp = 0.8138f;  // 0.33 * Ku
static const float Ki = 2.3589f;  // Kp / Ti,  Ti = 0.50  * Tu
static const float Kd = 0.1870f;  // Kp * Td,  Td = 0.333 * Tu
static const float DT = 0.020f;   // 20 ms = 50 Hz

/* ===================== FILTRO DO DERIVATIVO (N) ==========================
 * Passa-baixa de 1a ordem com polo em z = 1 - N*DT; so e estavel se N*DT < 2.
 * N*DT = 1 poe o polo em zero: derivativo mais rapido possivel sem oscilar.
 * NAO usar N=100 (N*DT = 2 exatos, oscila em 25 Hz para sempre).
 * ------------------------------------------------------------------------- */
static const float N  = 50.0f;    // N*DT = 1.0 — atraso puro de 1 amostra

/* =========================== LIMITES DE SAIDA =========================== */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;

/* ================ ANTI-WINDUP DO INTEGRAL ===============================
 * Quase todo o PWM de sustentacao vem do integral (Kp*erro tende a zero no
 * regime), entao o teto tem que cobrir o PWM do maior alvo desejado.
 * ATENCAO: este teto limita o angulo maximo alcancavel — ver secao 6 do
 * diagnostico (alvos de 90 graus ou mais param abaixo do alvo por causa dele).
 * ------------------------------------------------------------------------- */
static const float INTEGRAL_MAX = 550.0f;

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

    // ---- Proporcional ----
    float P = Kp * erro_atual;

    // ---- Derivativo TIPO B: sobre o ANGULO ----
    float D = 0.0f;
    if (!primeiro_ciclo) {
        float deriv_angulo = (angulo_atual - angulo_anterior) / DT;
        deriv_estado += N * DT * (deriv_angulo - deriv_estado);
        D = -Kd * deriv_estado;
    }
    angulo_anterior = angulo_atual;
    primeiro_ciclo  = 0;

    // ---- Integral com anti-windup (clamping dinamico) ----
    // Limitado pela folga que P e D deixam ate os limites do atuador.
    integral += Ki * erro_atual * DT;
    integral  = saturar(integral, PWM_MIN - (P + D), PWM_MAX - (P + D));
    integral  = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    // ---- Saida ----
    float saida = saturar(P + integral + D, PWM_MIN, PWM_MAX);

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
