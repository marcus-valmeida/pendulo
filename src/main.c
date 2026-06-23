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
// Declara as funções de inicialização do GPIO e do Timer 2
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
    SSD1306_SetCursor(0, 0);
    SSD1306_WriteString("Encoder OLED");
    SSD1306_SetCursor(0, 16);
    SSD1306_WriteString("Iniciando...");
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
 
        SSD1306_SetCursor(0, 0);
        SSD1306_WriteString("== ENCODER ==");
 
        SSD1306_SetCursor(0, 20);
        sprintf(linha, "Contagem: %d", posicao_encoder);
        SSD1306_WriteString(linha);
 
        SSD1306_SetCursor(0, 40);
        sprintf(linha, "Angulo: %.1f", angulo);
        SSD1306_WriteString(linha);
 
        SSD1306_UpdateScreen();
 
        /* Heartbeat */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}
 
/* ----------------- Clock 72 MHz (HSE 8MHz x PLL9) ----------------- */
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
 
/* ----------------- LED PC13 ----------------- */
void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
 
/* ----------------- I2C1 (PB6=SCL, PB7=SDA) p/ OLED ----------------- */
void MX_I2C1_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
 
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;       /* I2C = open drain */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;               /* 400 kHz (fast mode) */
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}
 
/* ----------------- TIM2 Encoder (PA0=A, PA1=B) ----------------- */
void MX_TIM2_Init(void) {
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
 
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
 
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;   /* quadratura x4 */
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 10;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 10;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }
 
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
 
    /* PA0/PA1 em modo Alternate Function Input (correto para timer) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;    /* <-- corrigido (era INPUT) */
    GPIO_InitStruct.Pull = GPIO_PULLUP;           /* pull-up p/ encoder NPN */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
 
void Error_Handler(void) {
    __disable_irq();
    while (1) { }
}
