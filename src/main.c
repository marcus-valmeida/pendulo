/**
  ******************************************************************************
  * @file           : main.c  
  * @brief          : Leitura de Encoder NPN (Modo Hardware) com Sanity Check
  *
  * COMO USAR ESTE TESTE:
  * - O LED (PC13) vai piscar RAPIDO (100ms) -> 5 piscadas por segundo.
  * - Se apos gravar o LED piscar RAPIDO, a gravacao funcionou perfeitamente.
  * - Se o LED continuar piscando LENTO, a gravacao falhou e o chip esta 
  * rodando um firmware antigo ou de fabrica.
  *
  * LIGACOES:
  * Encoder:  Fase A -> PA0 | Fase B -> PA1 | VCC -> 5V | GND -> GND
  * (pull-up interno ativado; encoder NPN coletor aberto)
  * OLED:     SCL -> PB6 | SDA -> PB7 | VCC -> 3.3V | GND -> GND
  * LED:      onboard PC13 (heartbeat)
  ******************************************************************************
  */

/* Inclui o seu módulo criado */
#include "aeropendulo.h"
#include <stdio.h>

// Mantém o tempo rodando para o HAL_Delay funcionar
// (Mantido na main pois é uma interrupção de sistema)
void SysTick_Handler(void) {
    HAL_IncTick(); 
}

int main(void) {
    /* 1. Inicializações do Hardware (funções puxadas de aeropendulo.h) */
    HAL_Init();
    SystemClock_Config();   
 
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
 
    /* 2. Inicialização dos Periféricos Específicos */
    /* Liga o Timer 2 em modo Encoder */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
 
    /* Inicializa o display OLED */
    SSD1306_Init(&hi2c1);
    SSD1306_Clear();
    SSD1306_SetCursor(0, 40);
    SSD1306_WriteString("ENCODER OLED");
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("INICIANDO...");
    SSD1306_UpdateScreen();
    HAL_Delay(800);
 
    /* 3. Variáveis Locais do Loop */
    int16_t  posicao_encoder = 0;
    float    angulo = 0.0f;
    char     linha[24];
 
    /* 4. Loop Principal da Aplicação */
    while (1) {
        /* Le o contador de hardware (cast int16_t para permitir valor negativo) */
        posicao_encoder = -(int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        
        /* Note que usamos a constante definida no seu aeropendulo.h */
        angulo = ((float)posicao_encoder * 360.0f) / ENCODER_RESOLUCAO;
 
        /* Atualiza o display */
        SSD1306_Clear();
 
        SSD1306_SetCursor(0, 40);
        SSD1306_WriteString("== ENCODER ==");
 
        SSD1306_SetCursor(0, 20);
        sprintf(linha, "CNT: %d", posicao_encoder);
        SSD1306_WriteString(linha);
 
        SSD1306_SetCursor(0, 0);
        sprintf(linha, "ANG: %.1f", angulo);
        SSD1306_WriteString(linha);
 
        SSD1306_UpdateScreen();
 
        /* Heartbeat */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}