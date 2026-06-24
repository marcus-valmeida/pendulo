#include "aeropendulo.h"


 
TIM_HandleTypeDef htim2;
I2C_HandleTypeDef hi2c1;

/* -------------------------------- LED PC13 -------------------------------- */
void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();                        // Habilita a porta C.
 
    GPIO_InitTypeDef GPIO_InitStruct = {0}; // Cria formulário de configuração.
    GPIO_InitStruct.Pin = GPIO_PIN_13;       // Define o pino 13 (LED onboard).
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Define como saida push-pull.
    GPIO_InitStruct.Pull = GPIO_NOPULL;            // Sem pull-up ou pull-down.
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Define a velocidade baixa.
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); // Entrega o formulário para o HAL.
}

/* ---------------------- TIM2 Encoder (PA0=A, PA1=B) ----------------------- */
void MX_TIM2_Init(void) {
    TIM_Encoder_InitTypeDef sConfig = {0};       // Cria formulário do Encoder.
    TIM_MasterConfigTypeDef sMasterConfig = {0};   // Cria formulário do Timer.
 
    __HAL_RCC_TIM2_CLK_ENABLE();                         // Habilita o Timer 2.
    __HAL_RCC_GPIOA_CLK_ENABLE();                        // Habilita a porta A.
 
    htim2.Instance = TIM2;                          // Define o uso do Timer 2.
    htim2.Init.Prescaler = 0;                                 // Sem prescaler.
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;         // Contador crescente.
    htim2.Init.Period = 65535;              // Contador de 16 bits (0 a 65535).
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Sem divisão do clock.
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;// Sem preload.
 
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;  // Ler fase A e B, encoder x4.
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING; // Borda de subida referencia.
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI; // Conexão direta do pino.
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;         // Sem prescaler no canal 1.
    sConfig.IC1Filter = 10; // Filtro de 10 ciclos de clock para reduzir ruído.
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING; // Borda de subida referencia.  
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI; // Conexão direta do pino.
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;         // Sem prescaler no canal 2.
    sConfig.IC2Filter = 10; // Filtro de 10 ciclos de clock para reduzir ruído.   
    
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
 
    GPIO_InitTypeDef GPIO_InitStruct = {0}; 
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;    
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
}
 
/* --------------------- Clock 72 MHz (HSE 8MHz x PLL9) --------------------- */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; 
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; 
 
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;                    
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;     
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                    
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;        
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;                
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
 
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}
 
/* -------------------- I2C1 (PB6=SCL, PB7=SDA) p/ OLED --------------------- */
void MX_I2C1_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;               
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;                    
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}
 
void Error_Handler(void) {
    __disable_irq();
    while (1) { }
}