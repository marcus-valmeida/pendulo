/* main.c — ponto de entrada do Aeropendulo.
 * O controle nao roda neste laco: roda na interrupcao do TIM4 a 50 Hz exatos
 * (hardware.c), para que o integral use sempre DT = 20 ms. O while(1) cuida
 * so do display OLED e da telemetria UART.
 * Mapa de ligacoes: README.md. */

#include "stm32f1xx_hal.h"
#include "hardware.h"
#include "aeropendulo.h"
#include "motor.h"
#include "pid.h"

/* Handlers dos perifericos — declarados como extern em hardware.h. */
TIM_HandleTypeDef  htim2;   // Encoder (PA0/PA1)
TIM_HandleTypeDef  htim3;   // PWM do motor (PA6)
TIM_HandleTypeDef  htim4;   // Base de tempo do controle (50 Hz)
I2C_HandleTypeDef  hi2c1;   // MPU6050 (PB6/PB7)
I2C_HandleTypeDef  hi2c2;   // OLED (PB10/PB11)
ADC_HandleTypeDef  hadc1;   // Potenciometro (PA4)
UART_HandleTypeDef huart1;  // Telemetria para o PC (PA9)

/* ---------------------------------------------------------------------------
 * MODO DE OPERACAO — trocar aqui e regravar.
 *   MALHA_ABERTA  : aplica PWM_MALHA_ABERTA fixo no motor (ensaio de degrau).
 *   MALHA_FECHADA : PID persegue ALVO_GRAUS.
 * ------------------------------------------------------------------------- */
typedef enum { MODO_MALHA_ABERTA, MODO_MALHA_FECHADA } ModoOperacao;

static const ModoOperacao MODO_ATUAL = MODO_MALHA_FECHADA;

/* Alvo de malha fechada, em graus. Faixa ensaiada: 30 a 100. */
static const float ALVO_GRAUS = 100.0f;

/* Degrau de malha aberta (0-1000). Calibracao PWM x angulo: docs/projeto_pid.md */
static const int32_t PWM_MALHA_ABERTA = 450;

void SysTick_Handler(void) {
    HAL_IncTick();
}

int main(void) {
    HAL_Init();
    Hardware_Init();        // clock, GPIO, TIM2/3/4, I2C1/2, ADC1, UART1
    Aeropendulo_Init();     // encoder + tela de abertura
    Aeropendulo_InitMPU();  // MPU6050 e calibracao do zero do encoder

    if (MODO_ATUAL == MODO_MALHA_FECHADA) {
        PID_Init();
        PID_SetFonte(SETPOINT_CODIGO);   // ou SETPOINT_POTENCIOMETRO (exige PA4)
        PID_SetAlvo(ALVO_GRAUS);
        Hardware_IniciarControle50Hz();  // so aqui a interrupcao do PID e ligada
    } else {
        Motor_SetPWM(PWM_MALHA_ABERTA);
    }

    uint32_t tempo_inicial = HAL_GetTick();

    while (1) {
        uint32_t tempo_atual  = HAL_GetTick() - tempo_inicial;
        float    angulo_real  = Aeropendulo_LerAngulo();

        if (MODO_ATUAL == MODO_MALHA_ABERTA) {
            Aeropendulo_TransmitirTelemetria(tempo_atual, angulo_real, 0.0f);

            // O I2C do OLED e lento e derruba a taxa de amostragem da UART;
            // na identificacao a telemetria importa mais que a tela.
            static uint8_t contador_oled = 0;
            if ((contador_oled++ % 8) == 0) {
                Aeropendulo_MostrarMalhaAberta(PWM_MALHA_ABERTA,
                                               Motor_PWM_Para_Tensao(PWM_MALHA_ABERTA));
            }
        } else {
            Aeropendulo_TransmitirTelemetria(tempo_atual, angulo_real, PID_GetAlvo());
            Aeropendulo_MostrarControle(PID_GetAlvo(), angulo_real,
                                        Aeropendulo_LerPitch(),
                                        PID_GetSaida(), PID_GetTravado());
        }

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(MODO_ATUAL == MODO_MALHA_ABERTA ? 10 : 50);
    }
}
