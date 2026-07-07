/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Controle do motor/helice via driver HW-517 (MOSFET + PWM).
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "motor.h"
#include "aeropendulo.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

static const int32_t PWM_MAXIMO = 1000;
// Tensao nominal de alimentacao do motor (usada so para exibir a "tensao").
static const float TENSAO_ALIMENTACAO = 4.7f;
/* USER CODE END PV */

/* -------------------- Funcao base: envia PWM ao motor --------------------- */
void Motor_SetPWM(int32_t pwm) {
    if (pwm > PWM_MAXIMO) pwm = PWM_MAXIMO;
    if (pwm < 0)          pwm = 0;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm);
}

/* ----------------------------- Parada de emergencia ------------------------ */
void Motor_Parar(void) {
    Motor_SetPWM(0);
}

/* ------------------- Conversao PWM (0-1000) -> tensao (V) ----------------- */
float Motor_PWM_Para_Tensao(int32_t pwm) {
    if (pwm > PWM_MAXIMO) pwm = PWM_MAXIMO;
    if (pwm < 0)          pwm = 0;
    return ((float)pwm / (float)PWM_MAXIMO) * TENSAO_ALIMENTACAO;
}

/* ------------------------ MODO Teste manual (potenciometro) ------------ */
void Motor_TesteManual(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);                            // le o potenciometro (0-4095)
        uint32_t pwm_alvo = (adc_val * (uint32_t)PWM_MAXIMO) / 4095;            // converte para 0-1000
        Motor_SetPWM((int32_t)pwm_alvo);                                        // aplica direto no motor
    }
    HAL_ADC_Stop(&hadc1);
}
