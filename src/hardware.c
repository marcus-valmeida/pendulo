/* hardware.c — perifericos do STM32F103C8. A main.c chama so Hardware_Init();
 * a ordem interna importa (clock primeiro). Aqui tambem fica a interrupcao do
 * TIM4 que executa o PID a 50 Hz. */

#include "hardware.h"
#include "pid.h"

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);

void Hardware_Init(void) {
    SystemClock_Config();    // 72 MHz — base de tempo de todo o resto
    MX_GPIO_Init();          // LED PC13
    MX_I2C1_Init();          // MPU6050  (PB6/PB7)
    MX_I2C2_Init();          // OLED     (PB10/PB11)
    MX_TIM2_Init();          // encoder  (PA0/PA1)
    MX_TIM3_Init();          // PWM do motor (PA6)
    MX_ADC1_Init();          // potenciometro (PA4)
    MX_TIM4_Init();          // base de tempo do controle (50 Hz)
    MX_USART1_UART_Init();   // telemetria (PA9)
}

/* Clock: HSE de 8 MHz x PLL9 = 72 MHz. HCLK 72, PCLK1 36, PCLK2 72 MHz. */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* LED onboard PC13 — pisca a cada volta do laco principal. */
static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* TIM2 em modo encoder por hardware (PA0 = fase A, PA1 = fase B).
 * TI12 conta as 4 bordas do par A/B: 360 pulsos/volta viram 1440 contagens. */
static void MX_TIM2_Init(void) {
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 10;                  // filtro anti-ruido, em ciclos de clock
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 10;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* I2C1 (PB6 = SCL, PB7 = SDA) — barramento exclusivo do MPU6050. */
static void MX_I2C1_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;    // open drain: exigido pelo I2C
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;        // fast mode
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/* I2C2 (PB10 = SCL, PB11 = SDA) — barramento do OLED, separado do MPU para
 * que a escrita lenta na tela nao atrase a leitura do sensor. */
static void MX_I2C2_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 400000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
}

/* TIM3 canal 1 (PA6) — PWM de 1 kHz para o HW-517.
 *   Prescaler 72-1  -> conta a 1 MHz (1 us por tick)
 *   Period    1000-1 -> estoura a cada 1 ms, e o duty vai de 0 a 1000. */
static void MX_TIM3_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 72 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 1000 - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;                   // comeca com o motor desligado
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

/* ADC1 canal 4 (PA4) — potenciometro de setpoint, 12 bits (0 a 4095). */
static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;   // converte so quando mandado
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    sConfig.Channel      = ADC_CHANNEL_4;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/* TIM4 — base de tempo do controle: 72 MHz / 7200 / 200 = 50 Hz exatos. */
static void MX_TIM4_Init(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 7200 - 1;    // 10 kHz
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 200 - 1;     // 10 kHz / 200 = 50 Hz
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }

    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* UART1 (PA9 = TX) — telemetria para o PC a 115200 bps, so transmissao. */
static void MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

void Hardware_EnviarTexto(const char *texto) {
    uint16_t tamanho = 0;
    while (texto[tamanho] != '\0') { tamanho++; }

    HAL_UART_Transmit(&huart1, (uint8_t *)texto, tamanho, 100);
}

void Hardware_IniciarControle50Hz(void) {
    HAL_TIM_Base_Start_IT(&htim4);
}

void TIM4_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim4);
}

/* Aqui e onde o controle realmente roda: a cada 20 ms, sem depender do laco
 * principal nem do tempo de escrita no display. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        PID_Atualizar();
    }
}

/* Falha critica de inicializacao: desliga interrupcoes e trava. */
void Error_Handler(void) {
    __disable_irq();
    while (1) { }
}
