/**
  ******************************************************************************
  * @file    aeropendulo.c
  * @brief   Camada de aplicacao do Aeropendulo - logica e calculos.
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "aeropendulo.h"
#include "ssd1306.h"
#include "MPU6050.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

static const float RESOLUCAO_ENCODER = 1440.0f;

// Offset calculado uma unica vez na inicializacao pelo MPU.
// Depois disso nunca e alterado.
static int16_t offset_inicial = 0;

static char linha[24];
/* USER CODE END PV */

/* Prototipos das funcoes privadas ------------------------------------------*/
static float calcular_pitch(float Ax, float Ay, float Az);

/* -------- Funcao privada: calcula pitch a partir dos dados do MPU --------- */
static float calcular_pitch(float Ax, float Ay, float Az) {
    float pitch_bruto = atan2f(-Ax, Az) * (180.0f / 3.14159265f);
    float pitch_corrigido = -(pitch_bruto + 180.0f);

    if (pitch_corrigido >  180.0f) pitch_corrigido -= 360.0f;
    if (pitch_corrigido < -180.0f) pitch_corrigido += 360.0f;

    return pitch_corrigido;
}

/* ------------------------- Inicializacao do sistema ----------------------- */
void Aeropendulo_Init(void) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    SSD1306_Init(&hi2c1);
    SSD1306_Clear();
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("AEROPENDULO");
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("INICIANDO...");
    SSD1306_UpdateScreen();
    HAL_Delay(800);
}

/* ---------- Inicializacao do MPU + calibracao do encoder ------------------ */
void Aeropendulo_InitMPU(void) {
    float Ax, Ay, Az;

    SSD1306_Clear();
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("CALIBRANDO...");
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("NAO MEXA!");
    SSD1306_UpdateScreen();

    MPU6050_init();
    HAL_Delay(500);

    // Le o angulo real do MPU com o pendulo parado.
    MPU6050_Read_Accel(&Ax, &Ay, &Az);
    float angulo_mpu = calcular_pitch(Ax, Ay, Az);

    // Converte o angulo do MPU em pulsos do encoder.
    // Esse valor sera somado a cada leitura do encoder como offset inicial.
    offset_inicial = (int16_t)((angulo_mpu * RESOLUCAO_ENCODER) / 360.0f);

    // Confirmacao no display.
    SSD1306_Clear();
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("PRONTO!");
    SSD1306_SetCursor(0, 20);
    sprintf(linha, "OFF: %d", offset_inicial);
    SSD1306_WriteString(linha);
    SSD1306_UpdateScreen();
    HAL_Delay(1000);
}

/* ----------------------- Leitura bruta do encoder ------------------------- */
int16_t Aeropendulo_LerContagem(void) {
    // Aplica o offset calculado na inicializacao.
    // Depois da calibracao, o encoder parte do angulo real medido pelo MPU.
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2) + offset_inicial;
}

/* ------------------- Conversao de contagem para angulo -------------------- */
float Aeropendulo_LerAngulo(void) {
    int16_t contagem = Aeropendulo_LerContagem();
    return ((float)contagem * 360.0f) / RESOLUCAO_ENCODER;
}

/* ---------------------- Leitura do pitch pelo MPU ------------------------- */
float Aeropendulo_LerPitch(void) {
    float Ax, Ay, Az;
    MPU6050_Read_Accel(&Ax, &Ay, &Az);
    return calcular_pitch(Ax, Ay, Az);
}

/* ------------------ Atualiza leitura e mostra no display ------------------ */
void Aeropendulo_Atualizar(void) {
    // Encoder ja parte do angulo correto (offset aplicado em LerContagem).
    int16_t contagem = Aeropendulo_LerContagem();
    float   angulo   = ((float)contagem * 360.0f) / RESOLUCAO_ENCODER;

    // MPU somente para monitoramento visual — nao altera nada apos o inicio.
    float pitch = Aeropendulo_LerPitch();

    SSD1306_Clear();

    SSD1306_SetCursor(0, 0);
    sprintf(linha, "MPU: %.1f    ", pitch);
    SSD1306_WriteString(linha);

    SSD1306_SetCursor(0, 20);
    sprintf(linha, "CNT: %d      ", contagem);
    SSD1306_WriteString(linha);

    SSD1306_SetCursor(0, 40);
    sprintf(linha, "ANG: %.1f GR  ", angulo);
    SSD1306_WriteString(linha);

    SSD1306_UpdateScreen();
}