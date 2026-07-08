/**
  ******************************************************************************
  * @file    main.c
  * @brief   Ponto de entrada do Aeropendulo.
  *          Escolha do modo de operacao (malha aberta ou malha fechada/PID)
  *
  *    ARQUITETURA COM INTERRUPCAO:
  *    O controle roda dentro da interrupcao do TIM4, a 50 Hz EXATOS. 
  *    Isso garante que o calculo do integral use sempre DT=20ms.
  *
  *  LIGACOES:
  *    Encoder        : Fase A -> PA0 | Fase B -> PA1 | VCC -> 5V   | GND -> GND
  *    MPU6050        : SCL   -> PB6 | SDA   -> PB7  | VCC -> 3.3V | GND -> GND
  *    OLED           : PB10 = SCL, PB11 = SDA | VCC -> 3.3V | GND -> GND
  *    Potenciometro  : Term1 -> GND | Term2 (wiper) -> PA4 | Term3 -> 3.3V
  *                     (0 ohm = -90 graus | 4.7k = +90 graus)
  *    HW-517 (motor) : TRIG/PWM -> PA6 | GND -> GND da Blue Pill
  *    LED            : onboard PC13
  ******************************************************************************
  */

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
#include "hardware.h"
#include "aeropendulo.h"
#include "motor.h"
#include "pid.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;                                                        // Timer 2 — modo Encoder (PA0/PA1)
I2C_HandleTypeDef hi2c1;                                                        // I2C1 — OLED e MPU6050 (PB6/PB7)
TIM_HandleTypeDef htim3;                                                        // Timer 3 — PWM para o HW-517 (PA6)
ADC_HandleTypeDef hadc1;                                                        // ADC1 — leitura do potenciometro (PA4)
TIM_HandleTypeDef htim4;                                                        // Interrupcao de controle 50 Hz
I2C_HandleTypeDef hi2c2;                                                        // I2C2 — OLED (PB10/PB11)
/* USER CODE END PV */

// -----------------------------------------------------------------------
// SELETOR DE MODO — muda o comportamento entre malha aberta e fechada.
// -----------------------------------------------------------------------
typedef enum {
    MODO_MALHA_ABERTA,                                                          // Controle via variação do PWM.
    MODO_MALHA_FECHADA                                                          // Controlador PID busca e mantem um angulo alvo.
} ModoOperacao;
 
static const ModoOperacao MODO_ATUAL = MODO_MALHA_FECHADA;
 
// Aplicando a fórmula: C ≈ (0.710 + 0.671 + 0.677 + 0.664) / 4 = 0.680
static const int32_t PWM_15_GRAUS = 87;
static const int32_t PWM_30_GRAUS = 160;
static const int32_t PWM_45_GRAUS = 225;
static const int32_t PWM_60_GRAUS = 275;
static const int32_t PWM_75_GRAUS = 317;
static const int32_t PWM_90_GRAUS = 330;
// Usado somente em MODO_MALHA_ABERTA: PWM fixo aplicado ao motor (0-1000).
static const int32_t PWM_TESTE_MALHA_ABERTA = 100;

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SysTick_Handler(void) {
    HAL_IncTick();
}
/* USER CODE END 0 */

/* ---------------------------------------------------------------------------
 * Ponto de entrada do programa.
 * ---------------------------------------------------------------------------*/
int main(void) {
    HAL_Init();
    Hardware_Init();                                                            // clock, GPIO, TIM2, I2C, TIM3, ADC1
    Aeropendulo_Init();                                                         // liga encoder e prepara o display
    Aeropendulo_InitMPU();                                                      // inicializa MPU6050 e calibra o encoder

    if (MODO_ATUAL == MODO_MALHA_FECHADA) {
        PID_Init();

        /* --------------------------------------------------------------------
        * ESCOLHA A FONTE DO ALVO:
        * ------------------------------------------------------------------- */
        // OPCAO A — alvo fixo definido no codigo:
        PID_SetFonte(SETPOINT_CODIGO);
        PID_SetAlvo(30.0f);                                                     // Insere manualmente o alvo
        // OPCAO B — alvo controlado pelo potenciometro (requer PA4 soldado):
        // PID_SetFonte(SETPOINT_POTENCIOMETRO);                                // Ângulo via potenciometro

        // Liga a interrupcao de controle SO em malha fechada.
        Hardware_IniciarControle50Hz();

    }

    while (1) {

        if (MODO_ATUAL == MODO_MALHA_ABERTA) {
            // --------- ENSAIO DE MALHA ABERTA ---------
            Motor_SetPWM(PWM_45_GRAUS);
            float tensao = Motor_PWM_Para_Tensao(PWM_45_GRAUS);
            Aeropendulo_MostrarMalhaAberta(PWM_45_GRAUS, tensao);

        } else {
            // --------- MALHA FECHADA (PID) ---------
            // Mostra as informações direto no display.
            Aeropendulo_MostrarControle(PID_GetAlvo(),
                                        Aeropendulo_LerAngulo(),
                                        Aeropendulo_LerPitch(),
                                        PID_GetSaida(),
                                        PID_GetTravado());
        }

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(50);                                                         
    }
}
