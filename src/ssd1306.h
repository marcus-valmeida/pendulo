/**
  ******************************************************************************
  * @file    ssd1306.h
  * @brief   Driver minimo para display OLED SSD1306 (128x64) via I2C
  *          Para STM32 HAL. Coloque em Core/Inc.
  ******************************************************************************
  */
#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f1xx_hal.h"
#include <stddef.h>

/* Endereco I2C do SSD1306. A maioria usa 0x3C (deslocado = 0x78).
   Alguns modulos usam 0x3D. Se a tela nao acender, teste 0x3D. */
#define SSD1306_I2C_ADDR   (0x3C << 1)

#define SSD1306_WIDTH      128
#define SSD1306_HEIGHT     64

/* Inicializa o display. Passe o ponteiro do seu handler I2C. */
void SSD1306_Init(I2C_HandleTypeDef *hi2c);

/* Limpa o buffer (nao atualiza a tela ate chamar UpdateScreen). */
void SSD1306_Clear(void);

/* Posiciona o cursor em (x, y) pixels. */
void SSD1306_SetCursor(uint8_t x, uint8_t y);

/* Escreve um caractere / string no buffer (fonte 7x10). */
void SSD1306_WriteChar(char ch);
void SSD1306_WriteString(const char *str);

/* Envia o buffer para o display (torna visivel o que foi escrito). */
void SSD1306_UpdateScreen(void);

#endif /* SSD1306_H */