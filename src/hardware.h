/* hardware.h — perifericos: clock, GPIO, timers, I2C, ADC e UART. */
#ifndef HARDWARE_H
#define HARDWARE_H

#include "stm32f1xx_hal.h"

/* Handlers definidos em main.c e usados por todos os modulos. */
extern TIM_HandleTypeDef  htim2;   // encoder (PA0/PA1)
extern TIM_HandleTypeDef  htim3;   // PWM do motor (PA6)
extern TIM_HandleTypeDef  htim4;   // base de tempo do controle (50 Hz)
extern I2C_HandleTypeDef  hi2c1;   // MPU6050 (PB6/PB7)
extern I2C_HandleTypeDef  hi2c2;   // OLED (PB10/PB11)
extern ADC_HandleTypeDef  hadc1;   // potenciometro (PA4)
extern UART_HandleTypeDef huart1;  // telemetria (PA9)

/* Inicializa todos os perifericos na ordem correta. Unica chamada do main. */
void Hardware_Init(void);

/* Envia uma string pela UART1 (bloqueante, timeout de 100 ms). */
void Hardware_EnviarTexto(const char *texto);

/* Liga a interrupcao de 50 Hz que executa PID_Atualizar(). Chamar depois de
 * PID_Init() — antes disso o controlador ainda nao tem estado valido. */
void Hardware_IniciarControle50Hz(void);

/* Falha critica de hardware: trava o sistema com as interrupcoes desligadas. */
void Error_Handler(void);

#endif /* HARDWARE_H */
