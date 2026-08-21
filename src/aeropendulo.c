/* aeropendulo.c — camada de aplicacao: angulo, display OLED e telemetria. */

#include "aeropendulo.h"
#include "hardware.h"
#include "ssd1306.h"
#include "MPU6050.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// Resolucao: 360 pulsos/volta x 4 (quadratura TI12) = 1440 contagens/volta.
static const float RESOLUCAO_ENCODER = 1440.0f;

// Offset calculado uma unica vez na inicializacao pelo MPU.
static int16_t offset_inicial = 0;

/* Buffer unico reaproveitado por todas as telas do OLED. */
static char linha[24];

/* Pitch a partir do acelerometro do MPU6050. */
static float calcular_pitch(float Ax, float Ay, float Az) {
    // Sensor de cabeca para baixo: desloca -180 para 0 e corrige o sentido.
    float pitch_bruto = atan2f(-Ax, Az) * (180.0f / 3.14159265f);
    float pitch_corrigido = -(pitch_bruto + 180.0f);

    if (pitch_corrigido >  180.0f) pitch_corrigido -= 360.0f;
    if (pitch_corrigido < -180.0f) pitch_corrigido += 360.0f;

    return pitch_corrigido;
}

/* Encoder ligado + tela de abertura. --------------------------------------- */
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

/* MPU6050 + calibracao do zero do encoder pelo angulo de repouso. ---------- */
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

int16_t Aeropendulo_LerContagem(void) {
    // Aplica o offset calculado na inicializacao (encoder parte do angulo real).
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim2) + offset_inicial;
}

float Aeropendulo_LerAngulo(void) {
    int16_t contagem = Aeropendulo_LerContagem();
    return ((float)contagem * 360.0f) / RESOLUCAO_ENCODER;
}

float Aeropendulo_LerPitch(void) {
    float Ax, Ay, Az;
    MPU6050_Read_Accel(&Ax, &Ay, &Az);
    return calcular_pitch(Ax, Ay, Az);
}

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

/* Linha CSV "tempo_ms,angulo,alvo" pela UART. Os floats sao quebrados em
 * inteiro + centesimos porque o %f do newlib-nano custa caro no laco. */
void Aeropendulo_TransmitirTelemetria(uint32_t tempo_ms, float angulo_real, float alvo) {
    char buffer_tx[64];

    int int_ang  = (int)angulo_real;
    int dec_ang  = (int)((fabsf(angulo_real) - abs(int_ang)) * 100);
    int int_alvo = (int)alvo;
    int dec_alvo = (int)((fabsf(alvo) - abs(int_alvo)) * 100);

    sprintf(buffer_tx, "%lu,%d.%02d,%d.%02d\r\n", tempo_ms, int_ang, dec_ang, int_alvo, dec_alvo);
    Hardware_EnviarTexto(buffer_tx);
}

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
