/* pid.c — PID puro (derivativo sobre a medicao), ganho unico e fixo, sem
 * feedforward e sem escalonamento. Sintonia por Ziegler-Nichols Metodo 2,
 * linha "some overshoot". Roda a 50 Hz exatos na interrupcao do TIM4.
 * Memorial de calculo e ensaios: docs/projeto_pid.md */

#include "pid.h"
#include "aeropendulo.h"
#include "motor.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;

/* ============================== GANHOS ===================================
 * Ziegler-Nichols Metodo 2 no pior ponto da faixa (30 graus: Ku = 2,47 e
 * Tu = 0,690 s), linha "some overshoot" — deducao na secao 3 do projeto.
 * ------------------------------------------------------------------------ */
static const float Kp = 0.8138f;  // 0,33 * Ku
static const float Ki = 2.3589f;  // Kp / Ti,  Ti = 0,50  * Tu
static const float Kd = 0.1870f;  // Kp * Td,  Td = 0,333 * Tu
static const float DT = 0.020f;   // 20 ms = 50 Hz

/* Filtro do derivativo: polo em z = 1 - N*DT, estavel so se N*DT < 2.
 * N*DT = 1 poe o polo em zero — o mais rapido sem oscilar.
 * NAO usar N = 100 (N*DT = 2 exatos, oscila em 25 Hz para sempre). */
static const float N = 50.0f;

/* Quase todo o PWM de sustentacao vem do integral (Kp*erro tende a zero no
 * regime), entao INTEGRAL_MAX define o maior alvo alcancavel: 620 sustenta
 * ate 100 graus (ensaios de 12/08). */
static const float PWM_MIN = 0.0f;
static const float PWM_MAX = 1000.0f;
static const float INTEGRAL_MAX = 620.0f;

/* Seguranca, em modulo: 130 reduz 10%, 150 reduz mais 20% (acumulam),
 * 160 desliga o motor e zera o integral, 170 trava ate o reset. */
static const float LIM_REDUZIR1   = 130.0f;
static const float LIM_REDUZIR2   = 150.0f;
static const float LIM_DESLIGAR   = 160.0f;
static const float LIM_TRAVAR     = 170.0f;
static const float FATOR_REDUCAO1 = 0.90f;
static const float FATOR_REDUCAO2 = 0.80f;

/* Mapeamento do potenciometro (ADC de 12 bits -> graus). */
static const float POT_ANG_MIN = -90.0f;
static const float POT_ANG_MAX =  90.0f;
static const float ADC_MAX     = 4095.0f;

/* Estado interno ---------------------------------------------------------- */
static FonteSetpoint fonte_setpoint = SETPOINT_CODIGO;
static float alvo_final      = 0.0f;
static float integral        = 0.0f;
static float angulo_anterior = 0.0f;
static float deriv_estado    = 0.0f;
static int32_t saida_pwm     = 0;
static uint8_t travado       = 0;
static uint8_t primeiro_ciclo = 1;

static float ler_setpoint_potenciometro(void);
static float saturar(float valor, float minimo, float maximo);

void PID_Init(void) {
    alvo_final      = 0.0f;
    integral        = 0.0f;
    angulo_anterior = 0.0f;
    deriv_estado    = 0.0f;
    saida_pwm       = 0;
    travado         = 0;
    primeiro_ciclo  = 1;
}

void PID_SetFonte(FonteSetpoint fonte) {
    fonte_setpoint = fonte;
}

void PID_SetAlvo(float angulo_alvo) {
    alvo_final = angulo_alvo;
}

static float ler_setpoint_potenciometro(void) {
    float adc_val = 0.0f;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        adc_val = (float)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return POT_ANG_MIN + (adc_val / ADC_MAX) * (POT_ANG_MAX - POT_ANG_MIN);
}

static float saturar(float valor, float minimo, float maximo) {
    if (valor > maximo) return maximo;
    if (valor < minimo) return minimo;
    return valor;
}

/* ====================== UM CICLO DO CONTROLADOR ==========================
 * Chamado pela interrupcao do TIM4 a cada 20 ms.
 * ------------------------------------------------------------------------ */
void PID_Atualizar(void) {
    if (travado) {
        Motor_Parar();
        saida_pwm = 0;
        return;
    }

    float angulo_atual = Aeropendulo_LerAngulo();
    float ang_abs      = fabsf(angulo_atual);

    /* Seguranca, da faixa mais grave para a mais leve. */
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

    float alvo = (fonte_setpoint == SETPOINT_POTENCIOMETRO)
               ? ler_setpoint_potenciometro()
               : alvo_final;

    float erro = alvo - angulo_atual;
    float P    = Kp * erro;

    /* Derivativo sobre o ANGULO (nao sobre o erro): evita chute de PWM a cada
     * mudanca de setpoint e atua como amortecedor de perturbacao. */
    float D = 0.0f;
    if (!primeiro_ciclo) {
        float deriv_angulo = (angulo_atual - angulo_anterior) / DT;
        deriv_estado += N * DT * (deriv_angulo - deriv_estado);
        D = -Kd * deriv_estado;
    }
    angulo_anterior = angulo_atual;
    primeiro_ciclo  = 0;

    /* Anti-windup por clamping dinamico: o integral e limitado pela folga que
     * P e D deixam ate os limites do atuador, e depois pelo teto absoluto. */
    integral += Ki * erro * DT;
    integral  = saturar(integral, PWM_MIN - (P + D), PWM_MAX - (P + D));
    integral  = saturar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    float saida = saturar(P + integral + D, PWM_MIN, PWM_MAX);

    if (ang_abs >= LIM_REDUZIR1) saida *= FATOR_REDUCAO1;
    if (ang_abs >= LIM_REDUZIR2) saida *= FATOR_REDUCAO2;

    saida_pwm = (int32_t)saida;
    Motor_SetPWM(saida_pwm);
}

float   PID_GetAlvo(void)    { return alvo_final; }
int32_t PID_GetSaida(void)   { return saida_pwm; }
uint8_t PID_GetTravado(void) { return travado; }
