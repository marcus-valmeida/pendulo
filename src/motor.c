/* motor.c — acionamento do motor/helice pelo driver HW-517 (MOSFET + PWM). */

#include "motor.h"
#include "hardware.h"

static const int32_t PWM_MAXIMO = 1000;

/* Tensao nominal da fonte do motor — usada so para exibir a tensao media
 * equivalente no display durante os ensaios de malha aberta. */
static const float TENSAO_ALIMENTACAO = 4.7f;

void Motor_SetPWM(int32_t pwm) {
    if (pwm > PWM_MAXIMO) pwm = PWM_MAXIMO;
    if (pwm < 0)          pwm = 0;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)pwm);
}

void Motor_Parar(void) {
    Motor_SetPWM(0);
}

float Motor_PWM_Para_Tensao(int32_t pwm) {
    if (pwm > PWM_MAXIMO) pwm = PWM_MAXIMO;
    if (pwm < 0)          pwm = 0;
    return ((float)pwm / (float)PWM_MAXIMO) * TENSAO_ALIMENTACAO;
}

/* Teste manual de bancada: le o potenciometro (PA4) e aplica direto no motor,
 * sem passar pelo PID. Nao e chamado pelo main — usar so para testar o driver. */
void Motor_TesteManual(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_val  = HAL_ADC_GetValue(&hadc1);                  // 0-4095
        uint32_t pwm_alvo = (adc_val * (uint32_t)PWM_MAXIMO) / 4095;   // 0-1000
        Motor_SetPWM((int32_t)pwm_alvo);
    }
    HAL_ADC_Stop(&hadc1);
}
