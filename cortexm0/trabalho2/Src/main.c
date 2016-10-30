/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "usart.h"
#include "gpio.h"
#include "ctype.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

int CAPS = 0, SHIFT = 0, CTRL = 0, NUMLOCK = 0, linhaDeBaixo = 0, posicao = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void LCDputc(int c);
void LCDputs(char *txt);
void LCD_init(void);
void pulsarEnable();
void LCDputchar(int c);
void LCDcomando(int c);
void UART2puts(char *txt);
int kbd_getchar(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern volatile uint32_t miliseg;

#define LE_CLK ((GPIOA->IDR >> 8) & 1) 
/*  
IDR significa Input Data Register (Leitura dos 16 bits da porta). Ele é
um barramento de mais de um bit que carrega diversas informações e, dentre 
elas, o bit de clock, o qual está na oitava posição, por isso o deslocamento.
A operação de AND com 1 é usada para filtrar o ultimo bit. 
Essa filtragem é feita pra garantir que ele ta lendo somente
o clock, e nada mais. 
*/

#define LE_DADO  ((GPIOA->IDR >> 10) & 1)

int le_bit()
{
	while(LE_CLK == 0);  // Espera subir CLK
	while(LE_CLK == 1);  // Espera descer CLK
	return LE_DADO; // Pega o bit de dado 
}

int le_scan(void)
{
	int scan = 0;
	int mask;
	
	while(le_bit() != 0); /* Espera start bit: deve ser zero */
		
	for(mask = 1; mask < 0x100; mask <<= 1)
		if(le_bit()) scan |= mask; /* Se for 1, junta com a máscara, senao não. */
	
	mask = le_bit(); /* Le bit de paridade */
	
	while(le_bit() != 1); /* O stop bit deve ser 1 */
	
	return scan;
}

int kbd_getchar(void)
{
	int sc = le_scan();
	int tecla; 
	//sc
	switch(sc)
	{
		// Parte principal
		case 0x0E:
			tecla = 39;
			break;
		case 0x16:
			tecla = 49;
			break;
		case 0x1E:
			tecla = 50;
			break;
		case 0x26:
			tecla = 51;
			break;
		case 0x25:
			tecla = 52;
			break;
		case 0x2E:
			tecla = 53;
			break;
		case 0x36:
			tecla = 54;
			break;
		case 0x3D:
			tecla = 55;
			break;
		case 0x3E:
			tecla = 56;
			break;
		case 0x46:
			tecla = 57;
			break;
		case 0x45:
			tecla = 48;
			break;
		case 0x4E:
			tecla = 45;
			break;
		case 0x55:
			tecla = 61;
			break;
		case 0x66:
			if (posicao == 16)
			{
				LCDcomando(0x8F);
				LCDputc(0x20);
				LCDcomando(0x10);
				posicao--;
			}
			else if (posicao > 0)
			{
				LCDcomando(0x10);
				LCDputc(0x20);
				LCDcomando(0x10);
				posicao--;
			}
			tecla = -1; // BACKSPACE
			break;
		case 0x0D:
			tecla = -1; // TAB
			break;
		case 0x15:
			tecla = 113;
			break;
		case 0x1D:
			tecla = 119;
			break;
		case 0x24:
			tecla = 101;
			break;
		case 0x2D:
			tecla = 114;
			break;
		case 0x2C:
			tecla = 116;
			break;
		case 0x35:
			tecla = 121;
			break;
		case 0x3C:
			tecla = 117;
			break;
		case 0x43:
			tecla = 105;
			break;
		case 0x44:
			tecla = 111;
			break;
		case 0x4D:
			tecla = 112;
			break;
		case 0x54:
			tecla = 180;
			break;
		case 0x5B:
			tecla = 91;
			break;
		case 0x5A:
			if (!linhaDeBaixo) 
			{
				linhaDeBaixo = 1;
				LCDcomando(0xC0);
				posicao = 16;
			}
			tecla = -1; // ENTER
			break;
		case 0x58:
			if (CAPS) CAPS = 0;
			else CAPS = 1;
			tecla = -1; // CAPS LOCK
			break;
		case 0x1C:
			tecla = 97;
			break;
		case 0x1B:
			tecla = 115;
			break;
		case 0x23:
			tecla = 100;
			break;
		case 0x2B:
			tecla = 102;
			break;
		case 0x34:
			tecla = 103;
			break;
		case 0x33:
			tecla = 104;
			break;
		case 0x3B:
			tecla = 106;
			break;
		case 0x42:
			tecla = 107;
			break;
		case 0x4B:
			tecla = 108;
			break;
		case 0x4C:
			tecla = 128;
			break;
		case 0x52:
			tecla = 126;
			break;
		case 0x5D:
			tecla = 93;
			break;
		case 0x12:
			SHIFT = 1;
			tecla = -1; // SHIFT ESQUERDO
			break;
		case 0x61:
			tecla = 92;
			break;
		case 0x1A:
			tecla = 122;
			break;
		case 0x22:
			tecla = 120;
			break;
		case 0x21:
			tecla = 99;
			break;
		case 0x2A:
			tecla = 118;
			break;
		case 0x32:
			tecla = 98;
			break;
		case 0x31:
			tecla = 110;
			break;
		case 0x3A:
			tecla = 109;
			break;
		case 0x41:
			tecla = 44;
			break;
		case 0x49:
			tecla = 46;
			break;
		case 0x4A:
			tecla = 59;
			break;
		case 0x51:
			tecla = 47;
			break;
		case 0x59:
			SHIFT = 1;
			tecla = -1; // SHIFT DIREITO
			break;
		case 0x14:
			CTRL = 1;
			tecla = -1; // CTRL ESQUERDO
			break;
		case 0x11:
			tecla = -1; // ALT ESQUERDO
			break;
		case 0x29:
			tecla = 32;
			break;
		case 0xE011:
			tecla = -1; // ALT DIREITO
			break;
		// Teclas de função
		case 0x76:
			tecla = 27; // ESC	
			break;
		// Teclado numérico
		case 0x70:
			tecla = 48;
			break;
		case 0x71:
			tecla = 46;
			break;
		case 0x69:
			tecla = 49;
			break;		
		case 0x72:
			tecla = 50;
			break;		
		case 0x7A:
			tecla = 51;
			break;		
		case 0x6B:
			tecla = 52;
			break;		
		case 0x73:
			tecla = 53;
			break;		
		case 0x74:
			tecla = 54;
			break;		
		case 0x6C:
			tecla = 55;
			break;		
		case 0x75:
			tecla = 56;
			break;		
		case 0x7D:
			tecla = 57;
			break;		
		case 0x79:
			tecla = 43;
			break;		
		case 0x7B:
			tecla = 45;
			break;		
		case 0x7C:
			tecla = 42;
			break;		
		case 0x77:
			if (NUMLOCK) NUMLOCK = 0;
			else NUMLOCK = 1;
			tecla = -1; // NUM LOCK
			break;
		// Brake code]
		case 0xE0:
			sc = le_scan();
			if (sc == 0xF0) 
			{
				sc = le_scan();
				switch (sc)
				{
					case 0x14:
						CTRL = 0;
						break;
				}
			}
			else if (sc == 0x4A) tecla = 47;
			else if (sc == 0x5A)
			{				
				if (!linhaDeBaixo) 
				{
					linhaDeBaixo = 1;
					LCDcomando(0xC0);
					posicao = 16;
				}
				tecla = -1; // ENTER
			}
			else if (sc == 0x14)
			{
				CTRL = 1;
				tecla = -1;
			}
			break;
		case 0xF0:
			sc = le_scan();
			if ((sc == 0x12) || (sc == 0x59))	SHIFT = 0;
			if (sc == 0x14) CTRL = 0;
			tecla = -1; // BRAKE CODE
			break;
	}
	if (tecla != -1)
	{
		if((SHIFT || CAPS) && isalpha(tecla)) tecla -= 32;
		else if (CTRL) tecla -= 96;
		else if (SHIFT)
		{
			switch(sc)
			{
				case 0x16:
					tecla = 33;
					break;
				case 0x1E:
					tecla = 64;
					break;
				case 0x26:
					tecla = 35;
					break;
				case 0x25:
					tecla = 36;
					break;
				case 0x2E:
					tecla = 37;
					break;
				case 0x36:
					tecla = 168;
					break;
				case 0x3D:
					tecla = 38;
					break;
				case 0x3E:
					tecla = 42;
					break;
				case 0x46:
					tecla = 40;
					break;
				case 0x45:
					tecla = 41;
					break;
				case 0x0E:
					tecla = 34;
					break;
				case 0x4E:
					tecla = 95;
					break;
				case 0x55:
					tecla = 43;
					break;
				case 0x54:
					tecla = 96;
					break;
				case 0x5B:
					tecla = 123;
					break;
				case 0x52:
					tecla = 94;
					break;
				case 0x5D:
					tecla = 125;
					break;
				case 0x41:
					tecla = 60;
					break;
				case 0x49:
					tecla = 62;
					break;
				case 0x4A:
					tecla = 58;
					break;
				case 0x51:
					tecla = 63;
					break;
			}
		}
	}
	return tecla;
}

/* USER CODE END 0 */

int main(void)
{
	int min,seg;
	char texto[80];
  /* USER CODE BEGIN 1 */

	int sc; // Scan code
	char ASCII;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  LCD_init();
	UART2puts("\r\Iniciada a USART\r\n");
	miliseg = min =seg =0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int debug = 0;
	
  while (1)
  {		
		if (debug)
		{
			sc = le_scan();				
			sprintf(texto, "%02X ", sc);
			UART2puts(texto);
			LCDputs(texto);
		}
		else
		{
			sc = kbd_getchar();
			if (sc != -1)
			{
				ASCII = sc;
				sprintf(texto, "%c", ASCII);
				UART2puts(texto);
				LCDputs(texto);
				if (posicao < 15) posicao++;
				else if (posicao < 31)
				{
					if (posicao == 15) 
					{
						LCDcomando(0xC0);
						linhaDeBaixo = 1;
					}
					posicao++;
				}
				else
				{
					LCDcomando(0x80);
					linhaDeBaixo = 0;
					posicao = 0;
				}
			}
		}
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/