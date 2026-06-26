/**
  ******************************************************************************
  * @file           : main.c  
  * @brief          : Calibração de Encoder com MPU6050 (Foco no Pitch)
  * Zero do MPU = 90° do Encoder
  ******************************************************************************
  */

#include "aeropendulo.h"
#include "MPU6050.h"
#include <stdio.h>
#include <math.h>

void SysTick_Handler(void) { 
    HAL_IncTick(); 
}

int main(void) {
    HAL_Init();
    SystemClock_Config();   
 
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
 
    SSD1306_Init(&hi2c1);
    SSD1306_Clear();
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("LIGANDO SENSOR...");
    SSD1306_UpdateScreen();
    
    MPU6050_init();
    HAL_Delay(500); 

    float Ax, Ay, Az;
    float pitch_inicial = 0.0f;
    float angulo_inicial = 0.0f;
    int16_t offset_encoder = 0;

    /* --- 1. CALIBRAÇÃO INICIAL DO PITCH --- */
    MPU6050_Read_Accel(&Ax, &Ay, &Az);

    /* Calcula o Pitch (Inclinação Frontal/Traseira usando o eixo X)
     * Na posição reta, isso resultará em 0 graus.
     */
    pitch_inicial = atan2(-Ax, sqrt((Ay * Ay) + (Az * Az))) * (180.0f / 3.14159265f);
    
    /* Transforma o "0" do MPU em "90" para o sistema. */
    angulo_inicial = pitch_inicial + 90.0f;
    
    /* Converte esse novo ângulo inicial para pulsos do encoder */
    offset_encoder = (int16_t)((angulo_inicial * ENCODER_RESOLUCAO) / 360.0f);

    /* Inicia o Encoder */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
 
    int16_t posicao_bruta = 0;
    int16_t posicao_calibrada = 0;
    float pitch_live = 0.0f;
    char linha[24];
 
    float angulo_bruto = 0.0f;
    float angulo_calibrado = 0.0f;

    while (1) {
        /* A. Leitura do Encoder Físico (Bruto) */
        posicao_bruta = -(int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        angulo_bruto = ((float)posicao_bruta * 360.0f) / ENCODER_RESOLUCAO;

        /* B. Aplicação da Calibração (Offset do MPU embute os +90 graus) */
        posicao_calibrada = posicao_bruta + offset_encoder;
        angulo_calibrado = ((float)posicao_calibrada * 360.0f) / ENCODER_RESOLUCAO;

        /* C. Lê o MPU6050 continuamente para monitoramento (opcional, só para teste) */
        MPU6050_Read_Accel(&Ax, &Ay, &Az);
        pitch_live = atan2(-Ax, sqrt((Ay * Ay) + (Az * Az))) * (180.0f / 3.14159265f);

        /* D. Atualiza o OLED */
        SSD1306_Clear();
        
        SSD1306_SetCursor(0, 0);
        sprintf(linha, "PITCH: %.1f", pitch_live);
        SSD1306_WriteString(linha); 
        

        /* Mostra o valor REAL (Físico) do hardware do encoder */
        SSD1306_SetCursor(0, 20);
        sprintf(linha, "BRUTO: %.1f", angulo_bruto); 
        SSD1306_WriteString(linha);
        
        /* Mostra o valor CALIBRADO (Com a referência de 90° do MPU) */
        SSD1306_SetCursor(0, 40);
        sprintf(linha, "CALIB: %.1f", angulo_calibrado); 
        SSD1306_WriteString(linha);

        SSD1306_UpdateScreen();
 
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(50); 
    }
}