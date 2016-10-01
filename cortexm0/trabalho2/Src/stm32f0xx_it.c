/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
/* Codigo acrescentado por Marcos A. Stemmer */
/* Escreve no LCD de Lab-processadores 6JK-2016-2 usando interrupcao do SysTick */
#define TAMFILA 64
#define EN_0 (GPIOA->BRR=0x40)
#define EN_1 (GPIOA->BSRR=0x40)
#define RS_0 (GPIOA->BRR=0x80)
#define RS_1 (GPIOA->BSRR=0x80)
volatile uint32_t miliseg;

 /* Prototipos exportados */
void LCDputc(int c);
void LCDputs(char *txt);
void LCDinit(void);

void Pulsa_EN(void)
{
	EN_1;
	__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();
	EN_0;
}

/* Escreve nas portas GPIO para escrever no LCD */
/* Interface de dados de 4 bits */
/*********************
 * D7 = GPIOB_6
 * D6 = GPIOB_5
 * D5 = GPIOB_4
 * D4 = GPIOB_3
 * RS = GPIOA_7
 * EN = GPIOA_6
 * *******************/
void LCD_escreve(int c)
{
	/** Escrita fisica no LCD **/
	GPIOB->BRR = 0x78;
	GPIOB->BSRR = (c >> 1) & 0x78;
	Pulsa_EN();	// Envia bits 7 a 4
	GPIOB->BRR = 0x78;
	GPIOB->BSRR = (c << 3) & 0x78;	// D4 na PA_8
	Pulsa_EN();	// Envia bits 3 a 0
}

/* O bit 8 dos caracteres e' usado para comando */
volatile uint16_t LCDfila[TAMFILA];		// Memoria para a fila
volatile int LCD_ini = 0;	// Inicio da fila: Local de atendimento
volatile int LCD_fim = 0;	// Fim da fila: Local de chegada
volatile int pula_it = 50000;	// Pula interrupcoes: (aumenta intervalo)

/* Retorna o numero de lugares ocupados na fila */
int LCDnbytes(void)
{
	return ((LCD_fim - LCD_ini) & (TAMFILA-1));
}

/* Coloca caractere no fim da fila */
void LCDputc(int c)
{
	while(TAMFILA - LCDnbytes() < 2);	// Espera ter pelo menos 2 lugares na fila
	LCDfila[LCD_fim] = c;			// Coloca o caractere na fila
	LCD_fim = ((LCD_fim + 1) & (TAMFILA-1));	// Incrementa fim da fila
}

/* Escreve string */
void LCDputs(char *txt)
{
	while(*txt) LCDputc((*txt++) & 0xff);
}

void pulsarEnable()
{	
	GPIOA->BSRR = (1 << 6); /* Liga LCD_EN */
	__nop();
	__nop();
	GPIOA->BRR = (1 << 6); /* Desliga LCD_EN */
}


void LCDputchar(int c)
{
	HAL_Delay(5);
	GPIOB->BRR = (0x0f << 3); // Limpa bits 6, 5, 4 e 3. *"(0x0f << 3)" é equivalente a atribuir 0x78 a BRR. 
	GPIOB->BSRR = (c >> 1) & (0x0f << 3); // Envia bits 7, 6, 5 e 4
	pulsarEnable();
	GPIOB->BRR = (0x0f << 3); // Limpa bits 6, 5, 4 e 3. *"(0x0f << 3)" é equivalente a atribuir 0x78 a BRR. 
	GPIOB->BSRR = (c << 3) & (0x0f << 3); // Envia bits 3210 
	pulsarEnable();
}

void LCDcomando(int c)
{
	HAL_Delay(4);
	GPIOA->BRR = (1 << 7); // LCD_RS = 0; => Comando
	LCDputchar(c);
	GPIOA->BSRR = (1 << 7); // LCD_rs = 1
}

/* Inicializa LCD */
void LCD_init(void)
{
	pula_it = 500;		// So atende quando a inicializacao esta' pronta
	LCD_fim=LCD_ini=0;	// Inicializa fila: Vazia.
	EN_0;			// Sinal EN normamente em zero
	LCDputc(0x128);
	LCDputc(0x128);
	LCDputc(0x128);	// Configura LCD com interface de 4 bits
	LCDputc(0x101);	// Limpa
	LCDputc(0x10c);	// Cursor oculto
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	/* Este e' o procedimento de atendimento no inicio da fila	 */
	/* feito na interrupcao de tempo SysTick a cada 1 milisegundo*/
	int c;
	miliseg++;
	if(pula_it-- <= 0) {	// Se pula_it>0 nao pega caractere
		pula_it = 3;
		if(LCD_fim != LCD_ini) {	// So' envia se tem caractere na fila
			c = LCDfila[LCD_ini];		// Pega o caractere
			LCD_ini = (LCD_ini + 1) & (TAMFILA-1);	// Incrementa o inicio da fila
			if(c & 0x100) { RS_0; pula_it = 8; }	// Se bit8 ligado e' comando
			else RS_1;	// Bit 8 desligado: caractere
			LCD_escreve(c);
		}
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/