/* pid.h — controlador PID de posicao. Le o angulo em aeropendulo.c e
 * escreve o PWM em motor.c. Roda na interrupcao de 50 Hz do TIM4. */
#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h"

/* De onde vem o angulo desejado. */
typedef enum {
    SETPOINT_CODIGO = 0,    // definido por PID_SetAlvo()
    SETPOINT_POTENCIOMETRO  // lido do potenciometro em PA4
} FonteSetpoint;

/* Zera a memoria interna do controlador. Chamar uma vez, antes de ligar a
 * interrupcao de controle. */
void PID_Init(void);

void PID_SetFonte(FonteSetpoint fonte);
void PID_SetAlvo(float angulo_alvo);

/* Um ciclo do controlador: le o angulo, calcula PID, aplica os estagios de
 * seguranca e comanda o motor. Chamado pela interrupcao de 50 Hz. */
void PID_Atualizar(void);

/* Getters para o display e a telemetria. */
float   PID_GetAlvo(void);     // alvo atual (graus)
int32_t PID_GetSaida(void);    // PWM aplicado (0-1000)
uint8_t PID_GetTravado(void);  // 1 se travou por desequilibrio

#endif /* PID_H */
