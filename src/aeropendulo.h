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

/* Retorna quantas voltas o encoder deu (com casas decimais).
 * Ex: 1.25 = uma volta e um quarto. Pode ser negativo. */
float Aeropendulo_LerVoltas(void);

/* Mostra no display SOMENTE: angulo do MPU, angulo do encoder e voltas.
 * Usado para validar a calibracao antes de ligar outras partes do sistema. */
void Aeropendulo_MostrarSensores(void);

/* Mostra os dados do controle no display OLED.
 * Recebe os valores prontos (nao conhece o PID nem o motor por dentro):
 *   alvo    - angulo desejado (graus)
 *   angulo  - angulo atual do encoder (graus)
 *   mpu     - angulo do MPU (graus)
 *   pwm     - tensao/PWM aplicado no motor (0-1000)
 *   travado - 1 se o sistema travou por desequilibrio */
void Aeropendulo_MostrarControle(float alvo, float angulo, float mpu,
                                 int32_t pwm, uint8_t travado);

#endif /* AEROPENDULO_H */
