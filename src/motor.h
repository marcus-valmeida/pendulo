/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Controle do motor/helice via driver HW-517 (MOSFET + PWM).
  *
  *  LIGACOES:
  *    Potenciometro 5K : Terminal 1 -> GND | Terminal 2 (wiper) -> PA4 (ADC)
  *                       Terminal 3 -> 3.3V
  *    HW-517 (driver)  : TRIG/PWM -> PA6 | GND -> GND da Blue Pill
  *                       VIN+/VIN- -> fonte do motor - VIN+ parte de cima e VIN- parte de baixo
  *                       OUT+/OUT- -> motor - OUT+ parte de cima (fio vermelho) e OUT- parte de baixo (fio azul)
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * Handlers de periferico criados na main.c.
 * ------------------------------------------------------------------------- */
extern TIM_HandleTypeDef htim3;                                                 // Timer 3 — gera o PWM para o HW-517
extern ADC_HandleTypeDef hadc1;                                                 // ADC1 — le o potenciometro

/* ---------------------------------------------------------------------------
 * API do Motor - funcoes publicas disponiveis para o sistema.
 * ---------------------------------------------------------------------------*/

/* Define a potencia do motor diretamente (0 a 1000 = 0% a 100%).             */
void Motor_SetPWM(int32_t pwm);

/* Desliga o motor imediatamente (seguranca). */
void Motor_Parar(void);

/* Converte o PWM (0-1000) na tensao media equivalente aplicada ao motor.     */
float Motor_PWM_Para_Tensao(int32_t pwm);

/*Le o potenciometro e aplica a potencia correspondente ao motor.   */
void Motor_TesteManual(void);

#endif /* MOTOR_H */
