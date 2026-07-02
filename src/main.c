/**
  ******************************************************************************
  * @file    main.c
  * @brief   Ponto de entrada do Aeropendulo — controle PID de posicao.
  *          Apenas coordena o sistema: cada modulo cuida do seu papel.
  *
  *  LIGACOES:
  *    Encoder        : Fase A -> PA0 | Fase B -> PA1 | VCC -> 5V   | GND -> GND
  *    OLED + MPU6050 : SCL   -> PB6 | SDA   -> PB7  | VCC -> 3.3V | GND -> GND
  *    Potenciometro  : Term1 -> GND | Term2 (wiper) -> PA4 | Term3 -> 3.3V
  *                     (0 ohm = -90 graus | 4.7k = +90 graus)
  *    HW-517 (motor) : TRIG/PWM -> PA6 | GND -> GND da Blue Pill
  *    LED            : onboard PC13 (heartbeat)
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "hardware.h"
#include "aeropendulo.h"
#include "motor.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;   // Timer 2 — modo Encoder (PA0/PA1)
I2C_HandleTypeDef hi2c1;   // I2C1 — OLED e MPU6050 (PB6/PB7)
TIM_HandleTypeDef htim3;   // Timer 3 — PWM para o HW-517 (PA6)
ADC_HandleTypeDef hadc1;   // ADC1 — leitura do potenciometro (PA4)
/* USER CODE END PV */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SysTick_Handler(void) {
    HAL_IncTick();
}
/* USER CODE END 0 */

/* ---------------------------------------------------------------------------
 * Ponto de entrada do programa.
 * ------------------------------------------------------------------------- */
int main(void) {
    HAL_Init();
    Hardware_Init();         // clock, GPIO, TIM2, I2C, TIM3 (PWM), ADC1
    Aeropendulo_Init();      // liga encoder e prepara o display
    Aeropendulo_InitMPU();   // inicializa MPU6050 e calibra o encoder
    PID_Init();              // zera a memoria do controlador

    /* -----------------------------------------------------------------
     * ESCOLHA A FONTE DO ALVO:
     * ------------------------------------------------------------- */
    // OPCAO A — alvo fixo definido no codigo:
    PID_SetFonte(SETPOINT_CODIGO);
    PID_SetAlvo(30.0f);        // buscar 30 graus

    // OPCAO B — alvo controlado pelo potenciometro (requer PA4 soldado):
    // PID_SetFonte(SETPOINT_POTENCIOMETRO);

    while (1) {
        PID_Atualizar();     // le angulo, calcula PID, aplica seguranca, comanda motor

        // Mostra no display: alvo, angulo atual, angulo do MPU e o PWM aplicado.
        Aeropendulo_MostrarControle(PID_GetAlvo(),
                                    Aeropendulo_LerAngulo(),
                                    Aeropendulo_LerPitch(),
                                    PID_GetSaida(),
                                    PID_GetTravado());

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   // heartbeat
        HAL_Delay(20);                            // 50 Hz — casa com DT do PID
    }
}
