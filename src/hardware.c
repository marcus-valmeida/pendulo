/**
  ******************************************************************************
  * @file    hardware.c
  * @brief   Configuracao de hardware do Aeropendulo - implementacao.
  *          Contem toda a inicializacao do chip: clock, GPIO, TIM2 e I2C1.
  *          TIM3 (PWM do motor) e ADC1 (leitura do potenciometro).
  *          A main.c chama apenas Hardware_Init().
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "hardware.h"
/* USER CODE END Includes */

/* ---------------------------------------------------------------------------
 * Funcoes privadas (static) — visivel apenas dentro deste arquivo.
 * -------------------------------------------------------------------------- */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

/* ---------------------------------------------------------------------------
 * Hardware_Init — ponto de entrada publico.
 * -------------------------------------------------------------------------- */
void Hardware_Init(void) {
    SystemClock_Config();   // 1. clock em 72 MHz (base de tudo)
    MX_GPIO_Init();         // 2. LED PC13
    MX_I2C1_Init();         // 3. I2C1 para o OLED (PB6/PB7)
    MX_TIM2_Init();         // 4. TIM2 modo Encoder (PA0/PA1)
    MX_TIM3_Init();         // 5. TIM3 modo PWM para o motor (PA6)
    MX_ADC1_Init();         // 6. ADC1 para o potenciometro (PA4)

}

/* --------------------- Clock 72 MHz (HSE 8MHz x PLL9) --------------------- */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Cria formulário de configuração do oscilador.
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Cria formulário de configuração do clock.

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // Usa o oscilador externo (HSE).
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;                    // Liga o HSE (8 MHz externo).
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;     // Sem divisão do HSE (8 MHz).
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                    // Liga o HSI (8 MHz interno, usado para PLL)
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                // Liga o PLL (Phase-Locked Loop) para multiplicar a frequência.
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;        // Usa o HSE como fonte do PLL.
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;                // Multiplica a frequência do HSE por 9 (8 MHz x 9 = 72 MHz).
    // Entrega o formulário de configuração do oscilador para o HAL.
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    // Reconfigura os 4 clocks do sistema: HCLK, SYSCLK, PCLK1 e PCLK2.
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   // Usa o PLL como fonte do SYSCLK (72 MHz).
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          // Sem divisão do AHB (HCLK = 72 MHz).
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;           // Divide o HCLK por 2 para o APB1 (PCLK1 = 36 MHz).
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           // Sem divisão do APB2 (PCLK2 = 72 MHz).
    // Entrega o formulário de configuração do clock para o HAL.
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

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

/* -------------------- I2C1 (PB6=SCL, PB7=SDA) para OLED ------------------- */
static void MX_I2C1_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7; // PB6=SCL, PB7=SDA.
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;          // I2C exige open drain.
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;             // 400 kHz (fast mode).
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/* ------------------- TIM3 PWM (PA6 = CH1) para o HW-517 -------------------
 * Frequencia do PWM: 1 kHz (dentro da faixa 0-20kHz aceita pelo HW-517)
 * Resolucao: 1000 passos (ARR = 999), entao o duty cycle vai de 0 a 1000.
 * Clock APB1 Timer = 72 MHz (TIM3 esta no barramento APB1 x2)
 *   Prescaler = 72 - 1   -> conta a 1 MHz (1 us por tick)
 *   Period    = 1000 - 1 -> estoura a cada 1000 us = 1 ms = 1 kHz
 * --------------------------------------------------------------------------*/
static void MX_TIM3_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
 
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
 
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 72 - 1;     // 72 MHz / 72 = 1 MHz
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 1000 - 1;   // 1 MHz / 1000 = 1 kHz
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
 
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;                  // comeca em 0% (motor desligado)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
 
    // PA6 em modo Alternate Function Push-Pull (saida do PWM).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
    // Inicia o PWM no canal 1 — ja comeca rodando, mas com duty cycle 0.
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}
 
/* ------------------- ADC1 (PA4) para o potenciometro 5K -------------------
 * Leitura de 12 bits (0 a 4095). Terminal 2 (wiper) do potenciometro no PA4.
 * --------------------------------------------------------------------------*/
static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
 
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
 
    // PA4 em modo entrada analogica (sem pull-up/down, sem digital).
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
 
    sConfig.Channel      = ADC_CHANNEL_4;        // PA4 = canal ADC 4
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* ----------------------------- Error Handler ------------------------------ */
void Error_Handler(void) {
    // Desliga as interrupcoes e trava — indica falha critica de hardware.
    __disable_irq();
    while (1) { }
}
