/**
  ******************************************************************************
  * @file    aeropendulo.h
  * @brief   Camada de aplicacao do Aeropendulo - declaracoes das funcoes.
  ******************************************************************************
  */
#ifndef AEROPENDULO_H
#define AEROPENDULO_H

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * API do Aeropendulo - funcoes publicas.
 * ------------------------------------------------------------------------- */

/* Inicializa encoder e display OLED (tela de abertura). */
void Aeropendulo_Init(void);

/* Inicializa o MPU6050 e calibra o encoder pelo angulo inicial. */
void Aeropendulo_InitMPU(void);

/* Le o contador bruto do encoder (em pulsos). Pode ser negativo. */
int16_t Aeropendulo_LerContagem(void);

/* Converte a contagem do encoder em angulo (graus). */
float Aeropendulo_LerAngulo(void);

/* Retorna o pitch atual lido pelo MPU6050 (graus). */
float Aeropendulo_LerPitch(void);

/* Mostra o ensaio de MALHA ABERTA: tensao, angulo e MPU a partir do PWM      */
void Aeropendulo_MostrarMalhaAberta(int32_t pwm, float tensao);

/* Mostra os dados do controle em MALHA FECHADA (PID) no display OLED.
 *   alvo, angulo, mpu, pwm.
 *   travado - 1 se o sistema travou por desequilibrio */
void Aeropendulo_MostrarControle(float alvo, float angulo, float mpu,
                                 int32_t pwm, uint8_t travado);

#endif /* AEROPENDULO_H */
