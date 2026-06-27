/**
  ******************************************************************************
  * @file    aeropendulo.h
  * @brief   Camada de aplicacao do Aeropendulo - declaracoes das funcoes.
  *          Aqui ficam as "assinaturas" das funcoes que a main.c pode chamar.
  *          A logica (calculos, leitura de sensores) fica em aeropendulo.c.
  ******************************************************************************
  */
#ifndef AEROPENDULO_H
#define AEROPENDULO_H

#include "stm32f1xx_hal.h"

/* ---------------------------------------------------------------------------
 * API do Aeropendulo - funcoes publicas disponiveis para o sistema.
 * ------------------------------------------------------------------------- */
 
/* Inicializa encoder e display OLED (tela de abertura). */
void Aeropendulo_Init(void);
 
/* Inicializa o MPU6050 e calcula o offset de calibracao do encoder. */
void Aeropendulo_InitMPU(void);
 
/* Le o contador bruto do encoder (em pulsos). Pode ser negativo. */
int16_t Aeropendulo_LerContagem(void);
 
/* Converte a contagem do encoder em angulo bruto (graus). */
float Aeropendulo_LerAngulo(void);
 
/* Retorna o pitch atual lido pelo MPU6050 (graus). */
float Aeropendulo_LerPitch(void);

/* Retorna o angulo fusionado pelo filtro complementar (o mais confiavel). */
float Aeropendulo_LerAnguloFusao(void);
 
/* Le sensores, calcula e mostra PITCH, BRUTO e CALIB no display. */
void Aeropendulo_Atualizar(void);
 
#endif /* AEROPENDULO_H */