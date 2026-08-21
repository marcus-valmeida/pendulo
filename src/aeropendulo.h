/* aeropendulo.h — leitura de angulo, display OLED e telemetria. */
#ifndef AEROPENDULO_H
#define AEROPENDULO_H

#include "stm32f1xx_hal.h"

/* Liga o encoder e mostra a tela de abertura. */
void Aeropendulo_Init(void);

/* Inicializa o MPU6050 e calibra o zero do encoder pelo angulo de repouso. */
void Aeropendulo_InitMPU(void);

/* Contagem bruta do encoder, em pulsos (pode ser negativa). */
int16_t Aeropendulo_LerContagem(void);

/* Angulo do braco em graus — esta e a medida usada pelo controle. */
float Aeropendulo_LerAngulo(void);

/* Pitch em graus lido pelo MPU6050 — so referencia visual apos o boot. */
float Aeropendulo_LerPitch(void);

/* Tela do ensaio de malha aberta (PWM, tensao, angulo, MPU). */
void Aeropendulo_MostrarMalhaAberta(int32_t pwm, float tensao);

/* Tela do controle em malha fechada; travado = 1 mostra o aviso de falha. */
void Aeropendulo_MostrarControle(float alvo, float angulo, float mpu,
                                 int32_t pwm, uint8_t travado);

/* Envia "tempo_ms,angulo,alvo" pela UART (lido por src/coleta_degrau.py). */
void Aeropendulo_TransmitirTelemetria(uint32_t tempo_ms, float angulo_real, float alvo);

#endif /* AEROPENDULO_H */
