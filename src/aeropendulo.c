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
extern I2C_HandleTypeDef hi2c2;

// Resolucao: 360 pulsos/volta x 4 (quadratura TI12) = 1440 contagens/volta.
static const float RESOLUCAO_ENCODER = 1440.0f;

// Offset calculado uma unica vez na inicializacao pelo MPU.
static int16_t offset_inicial = 0;

static char linha[24];
/* USER CODE END PV */

/* Prototipos privados ------------------------------------------------------*/
static float calcular_pitch(float Ax, float Ay, float Az);

/* -------- Funcao privada: calcula pitch a partir dos dados do MPU --------- */
static float calcular_pitch(float Ax, float Ay, float Az) {
    // Sensor de cabeca para baixo: desloca -180 para 0 e corrige o sentido.
    float pitch_bruto = atan2f(-Ax, Az) * (180.0f / 3.14159265f);
    float pitch_corrigido = -(pitch_bruto + 180.0f);

    if (pitch_corrigido >  180.0f) pitch_corrigido -= 360.0f;
    if (pitch_corrigido < -180.0f) pitch_corrigido += 360.0f;

    return pitch_corrigido;
}

/* ------------------------- Inicializacao do sistema ----------------------- */
void Aeropendulo_Init(void) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    SSD1306_Init(&hi2c2);
    SSD1306_Clear();
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("AEROPENDULO");
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("INICIANDO...");
    SSD1306_UpdateScreen();
    HAL_Delay(800);
}

/* ---------- Inicializacao do MPU + calibracao do encoder ----------------- */
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

    // Converte o angulo do MPU em pulsos e guarda como offset inicial.
    offset_inicial = (int16_t)((angulo_mpu * RESOLUCAO_ENCODER) / 360.0f);

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
    // Aplica o offset calculado na inicializacao (encoder parte do angulo real).
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

/* ------------- Mostra o ensaio de MALHA ABERTA (tensao x angulo) --------- */
void Aeropendulo_MostrarMalhaAberta(int32_t pwm, float tensao) {
    float angulo = Aeropendulo_LerAngulo();
    float mpu    = Aeropendulo_LerPitch();

    SSD1306_Clear();

    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("MALHA ABERTA");

    SSD1306_SetCursor(0, 16);
    sprintf(linha, "TENSAO: %.2fV", tensao);
    SSD1306_WriteString(linha);

    SSD1306_SetCursor(0, 32);
    sprintf(linha, "ANG:    %.1f", angulo);
    SSD1306_WriteString(linha);

    SSD1306_SetCursor(0, 48);
    sprintf(linha, "MPU:    %.1f", mpu);
    SSD1306_WriteString(linha);

    SSD1306_UpdateScreen();
}

void Aeropendulo_TransmitirTelemetria(uint32_t tempo_ms, float angulo_real, float alvo) {
    char buffer_tx[64];
    
    // Quebra o angulo real (float -> int + decimal)
    int int_ang = (int)angulo_real;
    int dec_ang = (int)((fabsf(angulo_real) - abs(int_ang)) * 100);
    
    // Quebra o alvo (float -> int + decimal)
    int int_alvo = (int)alvo;
    int dec_alvo = (int)((fabsf(alvo) - abs(int_alvo)) * 100);

    // Formato CSV: tempo, angulo_real, alvo
    sprintf(buffer_tx, "%lu,%d.%02d,%d.%02d\r\n", tempo_ms, int_ang, dec_ang, int_alvo, dec_alvo);
    Hardware_EnviarTexto(buffer_tx);
}

/* ------------------ Mostra os dados do controle no display --------------- */
void Aeropendulo_MostrarControle(float alvo, float angulo, float mpu,
                                 int32_t pwm, uint8_t travado) {
    SSD1306_Clear();

    SSD1306_SetCursor(0, 0);
    if (travado) {
        SSD1306_WriteString("! DESEQUILIBRIO !");
    } else {
        sprintf(linha, "ALVO: %.1f", alvo);
        SSD1306_WriteString(linha);
    }

    SSD1306_SetCursor(0, 16);
    sprintf(linha, "ANG:  %.1f", angulo);
    SSD1306_WriteString(linha);

    SSD1306_SetCursor(0, 32);
    sprintf(linha, "MPU:  %.1f", mpu);
    SSD1306_WriteString(linha);

    SSD1306_SetCursor(0, 48);
    sprintf(linha, "PWM:  %ld", pwm);
    SSD1306_WriteString(linha);

    SSD1306_UpdateScreen();
}
