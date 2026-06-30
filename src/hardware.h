/**
  ******************************************************************************
  * @file    hardware.h
  * @brief   Configuracao de hardware do Aeropendulo - declaracoes.
  *          Aqui ficam os prototipos das funcoes que configuram o chip.
  ******************************************************************************
  */
#ifndef HARDWARE_H
#define HARDWARE_H

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * Handlers dos perifericos.
 * DEFINIDOS em main.c — referenciados (extern) por hardware.c e aeropendulo.c
 * ------------------------------------------------------------------------- */
extern TIM_HandleTypeDef htim2;   // Timer 2 — modo Encoder (PA0/PA1)
extern I2C_HandleTypeDef hi2c1;   // I2C1 — display OLED (PB6/PB7)
extern TIM_HandleTypeDef htim3;   // Timer 3 — PWM para o driver HW-517 (PA6)
extern ADC_HandleTypeDef hadc1;   // ADC1 — leitura do potenciometro (PA4)

/* ---------------------------------------------------------------------------
 * API de hardware — unica funcao que a main.c precisa chamar.
 * Internamente ela chama clock, GPIO, TIM2 e I2C na ordem correta.
 * ------------------------------------------------------------------------- */

/* Inicializa todo o hardware do chip (clock, LED, encoder, I2C). */
void Hardware_Init(void);

/* Chamada em caso de erro critico — trava o sistema com interrupcoes off. */
void Error_Handler(void);

#endif /* HARDWARE_H */
