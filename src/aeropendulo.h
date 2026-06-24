#ifndef AEROPENDULO_H
#define AEROPENDULO_H

/* Includes necessários para as definições de hardware e display */
#include "stm32f1xx_hal.h"
#include "ssd1306.h"

/* Constantes */
#define ENCODER_RESOLUCAO 1440.0f  /* 360 pulsos * 4 (quadratura TI12) */

/* * Variáveis Globais (extern)
 * O 'extern' avisa a main.c que essas variáveis existem no aeropendulo.c
 */
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

/* Protótipos das funções de inicialização */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_I2C1_Init(void);
void Error_Handler(void);

#endif /* AEROPENDULO_H */