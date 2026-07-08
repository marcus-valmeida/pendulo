/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Controlador PID de posicao para o Aeropendulo - implementacao.
  *          Calibrado com os dados reais de bancada (malha aberta):
  *            PWM 160 -> ~30 graus | PWM 225 -> ~45 | PWM 275 -> ~60
  *          O sistema e sensivel: pouca tensao move muito o angulo, e o
  *          limite fisico de equilibrio fica por volta de 85-90 graus.
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
 * Calculados por alocacao de polos a partir dos dados de bancada:
 *   - Modelo 1a ordem equivalente: H(s) = 0.684/(s + 3.49)
  * A razao efetiva (Ki*DT)/Kp ~ 0.05 (baixa, como esperado da teoria).
 * -------------------------------------------------------------------------  */
static const float Kp = 3.0f;                                                   // proporcional
static const float Ki = 8.0f;                                                   // integral (zera erro estacionario)
 
static const float DT = 0.020f;                                                 // 20 ms = 50 Hz (cravado pela interrupcao)

/* ================= POTENCIA BASE POR FEEDFORWARD =========================
 * Em vez de uma base fixa alta (que empurrava o pendulo longe demais),
 * usamos os dados de bancada: sen(theta) = C * V, com C ~ 0.68.
 * A funcao calcular_base_ff() devolve o PWM aproximado que sustenta o
 * angulo alvo. O PID so faz o ajuste fino em cima disso.
 * ------------------------------------------------------------------------- 
static const float C_CALIBRACAO   = 0.68f;   // constante medida na bancada
static const float TENSAO_FONTE   = 4.7f;    // tensao real da sua fonte
static const float PWM_POR_VOLT   = 1000.0f / 4.7f;  // converte V -> PWM*/

/* =========================== LIMITES DE SAIDA ===========================   */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;

/* ================= LIMITE DE AUTORIDADE DO PID ==========================
 * REDUZIDO: como o equilibrio se perde perto de 90 graus, o PID nao pode
 * ter autoridade para empurrar o pendulo muito alem do necessario, senao
 * ele passa dos 90 e gira. Antes era 300; agora bem menor.
 * -------------------------------------------------------------------------  
static const float LIMITE_ACAO_PID = 150.0f;

/* ======================= RAMPA SUAVE DO SETPOINT ======================== 
static const float RAMPA_SETPOINT = 0.5f;   // graus por ciclo (25 graus/s)*/

/* ================ ANTI-WINDUP DO INTEGRAL =============================== 
 * Limita o acumulo do integral para nao "estourar" e causar supapo.
 * ------------------------------------------------------------------------- */
static const float INTEGRAL_MAX = 400.0f;

/* ====================== ESTAGIOS DE SEGURANCA (modulo) ===================
 * Usa FAIXAS, nao pontos exatos — caso houver um overshoot
 *   130 a 149  -> reduz 20% da potencia
 *   150 a 169  -> desliga o motor
 *   >= 170     -> trava ate reset
 * ------------------------------------------------------------------------- */
static const float LIM_REDUZIR   = 170.0f;   // reduz potencia perto do limite
static const float LIM_DESLIGAR  = 170.0f;   // passou do equilibrio — desliga
static const float LIM_TRAVAR    = 170.0f;  // girou — trava ate reset
static const float FATOR_REDUCAO = 0.80f;

/* ===================== MAPEAMENTO DO POTENCIOMETRO ======================= */
static const float POT_ANG_MIN = -90.0f;
static const float POT_ANG_MAX =  90.0f;
static const float ADC_MAX     = 4095.0f;

/* --------------------------- Estado interno ------------------------------ */
static float alvo_final   = 0.0f;
static float integral     = 0.0f;
static float erro_atual   = 0.0f;
static int32_t saida_pwm  = 0;
static uint8_t travado    = 0;
/* USER CODE END PV */

/* Prototipos privados ------------------------------------------------------*/
static float ler_setpoint_potenciometro(void);
static float saturar(float valor, float minimo, float maximo);
//static float calcular_base_ff(float angulo_alvo);

/* --------------------------- Inicializacao ------------------------------- */
void PID_Init(void) {
    alvo_final  = 0.0f;
    integral    = 0.0f;
    erro_atual  = 0.0f;
    saida_pwm   = 0;
    travado     = 0;
}

/* ----------------------- Configuracao da fonte --------------------------- */
void PID_SetFonte(FonteSetpoint fonte) {
    fonte_setpoint = fonte;
}

/* ------------------------- Define o alvo (codigo) ------------------------ */
void PID_SetAlvo(float angulo_alvo) {
    alvo_final = angulo_alvo;
}

/* ----------- Feedforward: PWM aproximado que sustenta o alvo -------------
 * Da tabela de bancada: sen(theta) = C * V  =>  V = sen(theta) / C
 * Depois converte a tensao em PWM. Como o pendulo pede modulo de forca
 * tanto para + quanto para -, usamos fabsf no angulo.
 * ------------------------------------------------------------------------- 
static float calcular_base_ff(float angulo_alvo) {
    float rad = fabsf(angulo_alvo) * (3.14159265f / 180.0f);
    float tensao = sinf(rad) / C_CALIBRACAO;    // V necessaria
    float pwm = tensao * PWM_POR_VOLT;          // converte para PWM
    return saturar(pwm, 0.0f, PWM_MAX);
}

/* --------------- Le o potenciometro e mapeia para -90..+90 --------------- 
static float ler_setpoint_potenciometro(void) {
    float adc_val = 0.0f;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        adc_val = (float)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    float faixa = POT_ANG_MAX - POT_ANG_MIN;
    return POT_ANG_MIN + (adc_val / ADC_MAX) * faixa;
}*/

/* ---------------------------- Saturacao ---------------------------------- */
static float saturar(float valor, float minimo, float maximo) {
    if (valor > maximo) return maximo;
    if (valor < minimo) return minimo;
    return valor;
}

/* ====================== CICLO PRINCIPAL DO PI ===========================
 * Chamado a 50 Hz EXATOS pela interrupcao do TIM4 (ver hardware.c).
 * Por rodar em interrupcao, o DT e sempre 20ms — os calculos do integral
 * ficam corretos independente do que o loop principal esteja fazendo.
 * ------------------------------------------------------------------------- */
void PID_Atualizar(void) {
    if (travado) {
        Motor_Parar();
        saida_pwm = 0;
        return;
    }
 
    float angulo_atual = Aeropendulo_LerAngulo();
    float ang_abs = fabsf(angulo_atual);
 
    // ---- SEGURANCA POR FAIXAS (em modulo, vale p/ + e -) ----
    // Faixas (nao pontos): um overshoot que pule por cima de um limite
    // ainda cai na faixa certa. Ordem: da mais grave para a mais leve.
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
 
    // ---- Saida do PI ----
    float saida = P + I;
    saida = saturar(saida, PWM_MIN, PWM_MAX);
 
    // ---- Faixa 130 a 149 : reduz 20% da potencia ----
    // (acima de 150 o codigo ja saiu antes, no bloco de desligar/travar,
    //  entao aqui so chega quem esta na faixa 130-149)
    if (ang_abs >= LIM_REDUZIR) {
        saida *= FATOR_REDUCAO;
    }
 
    saida_pwm = (int32_t)saida;
    Motor_SetPWM(saida_pwm);
}

/* ---------------------------- Getters ------------------------------------ */
float   PID_GetAlvo(void)    { return alvo_rampa; }
float   PID_GetErro(void)    { return erro_atual; }
int32_t PID_GetSaida(void)   { return saida_pwm; }
uint8_t PID_GetTravado(void) { return travado; }
