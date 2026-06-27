/**
  ******************************************************************************
  * @file    main.c
  * @brief   Ponto de entrada do Aeropendulo.
  *          Este arquivo apenas coordena o sistema — sem configuracao de
  *          hardware e sem logica de aplicacao. Tudo delegado aos modulos.
  *
  *  LIGACOES:
  *    Encoder : Fase A -> PA0 | Fase B -> PA1 | VCC -> 5V   | GND -> GND
  *    OLED    : SCL   -> PB6 | SDA   -> PB7  | VCC -> 3.3V | GND -> GND
  *    MPU6050 : SCL   -> PB6 | SDA   -> PB7  | VCC -> 3.3V | GND -> GND
  *    LED     : onboard PC13 (heartbeat — pisca a cada 50ms)
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "hardware.h"
#include "aeropendulo.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Handlers DEFINIDOS aqui — referenciados (extern) por hardware.c e aeropendulo.c
TIM_HandleTypeDef htim2;   // Timer 2 — modo Encoder (PA0/PA1)
I2C_HandleTypeDef hi2c1;   // I2C1 — display OLED (PB6/PB7)
/* USER CODE END PV */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Mantem o tempo do sistema rodando para HAL_Delay funcionar.
void SysTick_Handler(void) {
    HAL_IncTick();
}
/* USER CODE END 0 */

/* ---------------------------------------------------------------------------
 * Ponto de entrada do programa.
 * -------------------------------------------------------------------------- */
int main(void) {
    HAL_Init();                  // inicializa a HAL (obrigatorio ser o primeiro)
    Hardware_Init();             // configura clock, GPIO, TIM2, I2C
    Aeropendulo_Init();          // liga encoder e prepara o display
    Aeropendulo_InitMPU();       // inicializa MPU6050 e calcula calibracao
 
    while (1) {
        Aeropendulo_Atualizar();                    // le, calcula e exibe no OLED
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);     // heartbeat — programa vivo
        HAL_Delay(50);                              // 20 Hz de atualizacao
    }
}
