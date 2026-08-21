/* motor.h — driver HW-517 (MOSFET + PWM).
 * TRIG/PWM -> PA6, GND -> GND da Blue Pill.
 * VIN+/VIN- na fonte do motor, OUT+/OUT- no motor (OUT+ = fio vermelho). */
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

/* Define a potencia do motor (0 a 1000 = 0% a 100%). Satura fora da faixa. */
void Motor_SetPWM(int32_t pwm);

/* Desliga o motor imediatamente (seguranca). */
void Motor_Parar(void);

/* Converte o PWM (0-1000) na tensao media equivalente no motor, para display. */
float Motor_PWM_Para_Tensao(int32_t pwm);

/* Teste manual: potenciometro (PA4) comandando o PWM direto, sem PID. */
void Motor_TesteManual(void);

#endif /* MOTOR_H */
