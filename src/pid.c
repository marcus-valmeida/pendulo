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
// Handler do ADC (potenciometro) — definido em main.c.
extern ADC_HandleTypeDef hadc1;

/* ============================ AJUSTES DO PID ==============================
 * Ganhos iniciais vindos do TCC (planta identica, motor 5V/2A).
 * Ajuste fino na bancada: se oscilar, baixe Kp ou suba Kd.
 * ------------------------------------------------------------------------- */
static const float Kp = 2.6f;     // proporcional — forca proporcional ao erro
static const float Ki = 1.0f;     // integral — elimina erro de regime
static const float Kd = 0.9f;     // derivativo — freia perto do alvo (anti-supapo)
static const float N  = 50.0f;    // filtro do derivativo (evita ruido no D)

// Periodo do ciclo de controle em segundos (deve casar com o HAL_Delay do main).
static const float DT = 0.020f;   // 20 ms = 50 Hz

/* ======================= POTENCIA DE SUSTENTACAO =========================
 * Mesmo parado no alvo, a gravidade puxa o pendulo. Precisamos de uma
 * potencia base para "flutuar". Ajuste conforme o teste de malha aberta:
 * o PWM em que a helice comecou a levantar o pendulo.
 * ------------------------------------------------------------------------- */
static const float POTENCIA_BASE = 380.0f;  // ponto de partida (0-1000)

/* =========================== LIMITES DE SAIDA ============================
 * PWM do projeto vai de 0 a 1000 (nao 0-255 como no Arduino do TCC).
 * ------------------------------------------------------------------------- */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;

/* ======================= RAMPA SUAVE DO SETPOINT =========================
 * O alvo nao "pula" para o valor final — ele caminha devagar ate la.
 * Assim a partida e suave, como na rampa do teste de malha aberta.
 * ------------------------------------------------------------------------- */
static const float RAMPA_SETPOINT = 0.5f;   // graus por ciclo (0.5 * 50Hz = 25 graus/s)

/* ====================== ESTAGIOS DE SEGURANCA (modulo) ===================
 * Valem para os dois lados (+ e -) usando fabsf(angulo).
 * ------------------------------------------------------------------------- */
static const float LIM_REDUZIR   = 120.0f;  // corta 20% da potencia
static const float LIM_DESLIGAR  = 150.0f;  // desliga o motor
static const float LIM_TRAVAR    = 180.0f;  // desequilibrio — trava ate reset
static const float FATOR_REDUCAO = 0.80f;   // mantem 80% (corta 20%)

/* ===================== MAPEAMENTO DO POTENCIOMETRO =======================
 * Potenciometro multivoltas 5k:
 *   0    ohm (ADC ~0)    -> -90 graus
 *   4.7k ohm (ADC ~4095) -> +90 graus
 * ------------------------------------------------------------------------- */
static const float POT_ANG_MIN = -90.0f;
static const float POT_ANG_MAX =  90.0f;
static const float ADC_MAX     = 4095.0f;

/* --------------------------- Estado interno ------------------------------ */
static FonteSetpoint fonte_setpoint = SETPOINT_CODIGO;
static float alvo_final   = 0.0f;   // alvo desejado (destino da rampa)
static float alvo_rampa   = 0.0f;   // alvo instantaneo (segue a rampa)
static float integral     = 0.0f;   // acumulador do termo integral
static float deriv_estado = 0.0f;   // estado do filtro derivativo
static float erro_anterior = 0.0f;  // erro do ciclo anterior
static float erro_atual   = 0.0f;   // erro deste ciclo (para getter)
static int32_t saida_pwm  = 0;      // PWM aplicado (para getter)
static uint8_t travado    = 0;      // 1 = travado por desequilibrio
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
        adc_val = (float)HAL_ADC_GetValue(&hadc1);   // 0 a 4095
    }
    HAL_ADC_Stop(&hadc1);

    // Regra de tres linear: 0 -> -90 ; 4095 -> +90
    float faixa = POT_ANG_MAX - POT_ANG_MIN;         // 180 graus
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
    // ---- 0. Se travou por desequilibrio, motor fica desligado ate reset ----
    if (travado) {
        Motor_Parar();
        saida_pwm = 0;
        return;
    }

    // ---- 1. Le o angulo atual do sistema (encoder ja calibrado) ----
    float angulo_atual = Aeropendulo_LerAngulo();

    // ---- 2. SEGURANCA (avaliada em modulo, vale p/ + e -) ----
    float ang_abs = fabsf(angulo_atual);

    // Estagio 3 — desequilibrio: passou de 180. Trava tudo.
    if (ang_abs >= LIM_TRAVAR) {
        Motor_Parar();
        saida_pwm = 0;
        integral  = 0.0f;   // limpa memoria para nao "chutar" ao destravar
        travado   = 1;
        return;
    }

    // Estagio 2 — passou de 150. Desliga o motor (mas nao trava).
    if (ang_abs >= LIM_DESLIGAR) {
        Motor_Parar();
        saida_pwm = 0;
        integral  = 0.0f;
        return;
    }

    // ---- 3. Rampa suave do setpoint (o alvo caminha devagar) ----
    // Escolhe a fonte do alvo final.
    if (fonte_setpoint == SETPOINT_POTENCIOMETRO) {
        alvo_final = ler_setpoint_potenciometro();
    }
    // Move alvo_rampa em direcao a alvo_final, no maximo RAMPA_SETPOINT por ciclo.
    float delta_alvo = alvo_final - alvo_rampa;
    if (delta_alvo >  RAMPA_SETPOINT) delta_alvo =  RAMPA_SETPOINT;
    if (delta_alvo < -RAMPA_SETPOINT) delta_alvo = -RAMPA_SETPOINT;
    alvo_rampa += delta_alvo;

    // ---- 4. Calcula o erro ----
    erro_atual = alvo_rampa - angulo_atual;

    // ---- 5. Termo Proporcional ----
    float P = Kp * erro_atual;

    // ---- 6. Termo Integral com anti-windup ----
    // So acumula se a saida nao estiver saturada (evita "supapo" por acumulo).
    integral += Ki * erro_atual * DT;
    // Limita o integral para nao explodir.
    integral = saturar(integral, -PWM_MAX, PWM_MAX);
    float I = integral;

    // ---- 7. Termo Derivativo com filtro (N) ----
    // Filtro passa-baixa no derivativo: suaviza ruido do encoder.
    // deriv_estado segue a derivada do erro de forma filtrada.
    float deriv_erro = (erro_atual - erro_anterior) / DT;
    deriv_estado += N * DT * (deriv_erro - deriv_estado);
    float D = Kd * deriv_estado;
    erro_anterior = erro_atual;

    // ---- 8. Soma PID + potencia base de sustentacao ----
    float saida = P + I + D + POTENCIA_BASE;

    // ---- 9. Saturacao final (0 a 1000) ----
    saida = saturar(saida, PWM_MIN, PWM_MAX);

    // ---- 10. Estagio 1 de seguranca — reducao de 20% acima de 120 ----
    if (ang_abs >= LIM_REDUZIR) {
        saida *= FATOR_REDUCAO;   // mantem 80% da potencia calculada
    }

    // ---- 11. Aplica no motor ----
    saida_pwm = (int32_t)saida;
    Motor_SetPWM(saida_pwm);
}

/* ---------------------------- Getters ------------------------------------ */
float   PID_GetAlvo(void)    { return alvo_rampa; }
float   PID_GetErro(void)    { return erro_atual; }
int32_t PID_GetSaida(void)   { return saida_pwm; }
uint8_t PID_GetTravado(void) { return travado; }
