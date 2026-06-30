/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Controle do motor/helice via driver HW-517 (MOSFET + PWM).
  *          Aqui ficam as "assinaturas" das funcoes que controlam o motor.
  *          A logica (leitura do potenciometro, rampa, seguranca) fica em motor.c.
  *
  *  LIGACOES:
  *    Potenciometro 5K : Terminal 1 -> GND | Terminal 2 (wiper) -> PA4 (ADC)
  *                       Terminal 3 -> 3.3V
  *    HW-517 (driver)  : TRIG/PWM -> PA6 | GND -> GND da Blue Pill
  *                       VIN+/VIN- -> fonte do motor | OUT+/OUT- -> motor
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * Handlers de periferico criados na main.c.
 * ------------------------------------------------------------------------- */
extern TIM_HandleTypeDef htim3;   // Timer 3 — gera o PWM para o HW-517
extern ADC_HandleTypeDef hadc1;   // ADC1 — le o potenciometro

/* ---------------------------------------------------------------------------
 * API do Motor - funcoes publicas disponiveis para o sistema.
 * ------------------------------------------------------------------------- */

/* Define a potencia do motor diretamente (0 a 1000 = 0% a 100%). */
void Motor_SetPWM(int32_t pwm);

/* Desliga o motor imediatamente (seguranca). */
void Motor_Parar(void);

/* MODO 1 — Le o potenciometro e aplica a potencia correspondente ao motor.
 * Chame esta funcao repetidamente no loop para controle manual em tempo real. */
void Motor_TesteManual(void);

/* MODO 2 — Sobe a potencia automaticamente e devagar (rampa).
 * Corta a tensao para zero assim que o angulo do encoder passar de 40 graus.
 * Chame esta funcao repetidamente no loop. */
void Motor_TesteAutomatico(void);

/* Reseta o teste automatico para poder rodar de novo sem regravar a placa. */
void Motor_ResetarTesteAuto(void);

#endif /* MOTOR_H */
