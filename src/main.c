#include "stm32f1xx_hal.h"

void GPIO_Init(void);

void SysTick_Handler(void) {
    HAL_IncTick(); // Diz para a HAL que o tempo passou
}

int main(void) {
    // Inicializa a HAL e liga o relógio de 1ms (SysTick)
    HAL_Init();

    // Inicializa o pino PC13
    GPIO_Init();

    while (1) {
        // Inverte o estado do LED
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        
        // Agora o delay vai funcionar e não vai mais travar!
        HAL_Delay(1000);
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}