/**
  ******************************************************************************
  * @file           : main.c  
  * @brief          : Leitura de Encoder NPN (Modo Hardware) com Sanity Check
  *
  * COMO USAR ESTE TESTE:
  * - O LED (PC13) vai piscar RAPIDO (100ms) -> 5 piscadas por segundo.
  * - Se apos gravar o LED piscar RAPIDO, a gravacao funcionou perfeitamente.
  * - Se o LED continuar piscando LENTO, a gravacao falhou e o chip esta 
  * rodando um firmware antigo ou de fabrica.

  *  LIGACOES:
  *    Encoder:  Fase A -> PA0 | Fase B -> PA1 | VCC -> 5V | GND -> GND
  *              (pull-up interno ativado; encoder NPN coletor aberto)
  *    OLED:     SCL -> PB6 | SDA -> PB7 | VCC -> 3.3V | GND -> GND
  *    LED:      onboard PC13 (heartbeat)

  ******************************************************************************
  */

/* USER CODE BEGIN Includes */  
// Inclui o arquivo de configuração do HAL para o STM32F1 
#include "stm32f1xx_hal.h"
#include "ssd1306.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Cria o "handler" para controlar o Timer 2
TIM_HandleTypeDef htim2;
I2C_HandleTypeDef hi2c1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
// Declara as funções de inicialização.
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
void MX_I2C1_Init(void);
void Error_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Mantém o tempo rodando para o HAL_Delay funcionar
void SysTick_Handler(void) {
    HAL_IncTick(); 
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    SystemClock_Config();   /* agora configura 72 MHz corretamente */
 
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
 
    /* Liga o Timer 2 em modo Encoder */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
 
    /* Inicializa o display OLED */
    SSD1306_Init(&hi2c1);
    SSD1306_Clear();
    SSD1306_SetCursor(0, 40);
    SSD1306_WriteString("ENCODER OLED");
    SSD1306_SetCursor(0, 20);
    SSD1306_WriteString("INICIANDO...");
    SSD1306_UpdateScreen();
    HAL_Delay(800);
 
    int16_t  posicao_encoder = 0;
    float    angulo = 0.0f;
    char     linha[24];
    const float RESOLUCAO = 1440.0f;  /* 360 pulsos * 4 (quadratura TI12) */
 
    while (1) {
        /* Le o contador de hardware (cast int16_t para permitir valor negativo) */
        posicao_encoder = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        angulo = ((float)posicao_encoder * 360.0f) / RESOLUCAO;
 
        /* Atualiza o display */
        SSD1306_Clear();
 
        SSD1306_SetCursor(0, 40);
        SSD1306_WriteString("== ENCODER ==");
 
        SSD1306_SetCursor(0, 20);
        sprintf(linha, "CNT: %d", posicao_encoder);
        SSD1306_WriteString(linha);
 
        SSD1306_SetCursor(0, 0);
        sprintf(linha, "ANG: %.1f", angulo);
        SSD1306_WriteString(linha);
 
        SSD1306_UpdateScreen();
 
        /* Heartbeat */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
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
    // Entrega o formulário de configuração do Encoder para o HAL. 
    // Se falhar, chama o Error_Handler.  
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }

    // Configura o Timer 2 como mestre.
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
 
    /* PA0/PA1 em modo Alternate Function Input (correto para timer) */
    GPIO_InitTypeDef GPIO_InitStruct = {0}; // Cria formulário de configuração.
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;    // Define os pinos 0 e 1.
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;// Define como entrada (modo timer).
    GPIO_InitStruct.Pull = GPIO_PULLUP;// Ativa pull-up interno, coletor aberto.
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Entrega o formulário para o HAL.
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
 
/* -------------------- I2C1 (PB6=SCL, PB7=SDA) p/ OLED --------------------- */
void MX_I2C1_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;               /* I2C = open drain */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;                    /* 400 kHz (fast mode) */
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