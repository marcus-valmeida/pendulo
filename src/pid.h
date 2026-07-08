/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Controlador PID de posicao para o Aeropendulo.
  *          Lê do aeropendulo.c (angulo) e ESCREVE no motor.c (PWM).
  *
  *  ESTRATEGIA:
  *    - PID completo (P + I + D) para evitar os oscilações.
  *    - Seguranca em 3 estagios (em modulo, vale p/ + e -):
  *        |ang| >= 150  -> corta 20% da potencia atual
  *        |ang| >= 170  -> desliga o motor
  *        |ang| >= 180  -> trava por desequilibrio (exige reset)
  *    - Setpoint via codigo OU via potenciometro (0 ohm=-90, 4.7k=+90).
  ******************************************************************************
  */

#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * Fonte do setpoint (angulo desejado).
 * -------------------------------------------------------------------------  */
typedef enum {
    SETPOINT_CODIGO = 0,   // alvo definido por PID_SetAlvo() no codigo
    SETPOINT_POTENCIOMETRO // alvo lido do potenciometro (PA4)
} FonteSetpoint;

/* ---------------------------------------------------------------------------
 * API do controlador.
 * -------------------------------------------------------------------------  */

/* Inicializa o PID (zera memoria interna). Chamar uma vez no main. */
void PID_Init(void);

/* Escolhe se o alvo vem do codigo ou do potenciometro.                       */
void PID_SetFonte(FonteSetpoint fonte);

/* Define o angulo alvo em graus (usado quando a fonte e SETPOINT_CODIGO).    */
void PID_SetAlvo(float angulo_alvo);

/* Executa um ciclo do controlador. Chamar periodicamente no loop (ex: 20ms).
 * Le o angulo atual, calcula o PID, aplica seguranca e comanda o motor.      */
void PID_Atualizar(void);

/* Getters para exibir no display.                                            */
float   PID_GetAlvo(void);       // angulo alvo atual (graus)
float   PID_GetErro(void);       // erro atual (graus)
int32_t PID_GetSaida(void);      // PWM aplicado no motor (0-1000)
uint8_t PID_GetTravado(void);    // 1 se travou por desequilibrio (>180)

#endif /* PID_H */
