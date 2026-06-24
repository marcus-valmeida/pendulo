/**
  ******************************************************************************
  * @file    aeropendulo.c
  * @brief   Camada de aplicacao do Aeropendulo - logica e calculos.
  *        
  ******************************************************************************
  */
 
/* USER CODE BEGIN Includes */
#include "aeropendulo.h"
#include "ssd1306.h"
#include <stdio.h>
/* USER CODE END Includes */
 
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Handlers definidos na main.c - referenciados aqui (privado ao .c)
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

// Resolucao do encoder: 360 pulsos por volta x 4 (quadratura TI12) = 1440.
static const float RESOLUCAO_ENCODER = 1440.0f;

// Buffer de texto para montar as linhas do display.
static char linha[24];

/* USER CODE END PV */
