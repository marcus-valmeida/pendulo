/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Controle do motor/helice via driver HW-517 (MOSFET + PWM).
  *          A main.c chama as funcoes deste arquivo.
  *          O hardware (TIM3 e ADC1) ja esta configurado por hardware.c.
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "motor.h"
#include "aeropendulo.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Handlers DEFINIDOS em main.c — referenciados aqui (privado ao modulo).
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

// Limite de seguranca do teste automatico (graus).
static const float ANGULO_LIMITE = 40.0f;

// Incremento de PWM por ciclo na rampa automatica.
// Com loop a cada 20ms, leva ~20s para ir de 0 a 1000 (0% a 100%).
static const float INCREMENTO_RAMPA = 1.0f;

// PWM maximo permitido (resolucao do timer configurada no hardware.c).
static const int32_t PWM_MAXIMO = 1000;

// Estado do teste automatico.
static float   pwm_automatico   = 0.0f;
static uint8_t teste_finalizado = 0;
/* USER CODE END PV */

/* -------------------- Funcao base: envia PWM ao motor --------------------- */
void Motor_SetPWM(int32_t pwm) {
    // Saturacao — protege o timer de receber valores fora da faixa.
    if (pwm > PWM_MAXIMO) pwm = PWM_MAXIMO;
    if (pwm < 0)          pwm = 0;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm);
}

/* ----------------------------- Parada de emergencia ------------------------ */
void Motor_Parar(void) {
    Motor_SetPWM(0);
}

/* ------------------------ MODO 1: Teste manual (potenciometro) ------------ */
void Motor_TesteManual(void) {
    // Le o potenciometro via ADC1 (canal ligado ao PA4).
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1); // 0 a 4095 (12 bits)

        // Converte a leitura (0-4095) para a escala do PWM (0-1000).
        uint32_t pwm_alvo = (adc_val * (uint32_t)PWM_MAXIMO) / 4095;

        Motor_SetPWM((int32_t)pwm_alvo);
    }
    HAL_ADC_Stop(&hadc1);
}

/* --------------------- MODO 2: Teste automatico (rampa) ------------------- */
void Motor_TesteAutomatico(void) {
    // Le o angulo atual do encoder (funcao ja existente no aeropendulo).
    float angulo_atual = Aeropendulo_LerAngulo();

    if (teste_finalizado == 0) {
        if (angulo_atual < ANGULO_LIMITE) {
            // Ainda nao atingiu o limite — sobe a potencia devagar.
            pwm_automatico += INCREMENTO_RAMPA;
            Motor_SetPWM((int32_t)pwm_automatico);
        } else {
            // Atingiu ou ultrapassou o limite de seguranca — corta tudo.
            Motor_Parar();
            teste_finalizado = 1;
        }
    } else {
        // Teste ja finalizado — mantem o motor desligado.
        Motor_Parar();
    }
}

/* ------------------- Reinicia o teste automatico --------------------------- */
void Motor_ResetarTesteAuto(void) {
    pwm_automatico   = 0.0f;
    teste_finalizado = 0;
    Motor_Parar();
}
