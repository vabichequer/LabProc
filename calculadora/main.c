/**************************************************************************
 * Laboratorio de processadores II-FENG-PUCRS - 2016-2
 * Prof. Marcos Augusto Stemmer
 * Modificacoes: Vicenzo Abichequer e Nicolas Nascimento
 * Programa kit STM32F429 Discovery usando gcc e libopencm3
 * Programa original: lcd-touch: teste do touch screen
 * Programa final: calculadora, T4 de Laboratorio de processadores
****************************************************************************/

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <stdio.h>
#include "charset.c"

#define CPUCLK 168000000
#define DEBUG1 0

#include "uart.h"
#include "sdram.h"
#include "ili9341.h"
#include "i2c3_touch.h"

struct txtinfo stxt;

const int xinicial = 20, yinicial = 100, step = 50, tamanhoDaTecla = 48;

void clock_setup(void);

/******* Interrupcao de ticks do relogio **********/
volatile uint32_t miliseg;
/* Chamada pelo atendimento da interrupcao do timer */
void sys_tick_handler(void)
{
	miliseg++;
}

void msleep(uint32_t t)
{
	t += miliseg;
	while(t > miliseg);
}

void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	/* Enable GPIOG and GPIOA clock. */
	RCC_AHB1ENR |= RCC_AHB1ENR_IOPGEN | RCC_AHB1ENR_IOPAEN | RCC_AHB1ENR_IOPCEN;
	GPIOG_MODER |= (1 << 26);	/* GPIOG_13 como saida */
	/* Configura SysTick para gerar 1000 interrupcoes/segundo */
	systick_set_reload(CPUCLK/1000-1);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	systick_interrupt_enable();
}

/* Ideias de modificacao:
	*****************
	*				*
	*				*
	*				*
	*				*
	*				*
	*A	B	C	D	*
	*E	F	G	H	*
	*I	J	K	L	*
	*M	N	O	P	*
	*****************

	* Cada numero eh um quadrado, ai detectamos a entrada pela area na tela
	* Criar estrutura tecla, que guarda suas dimensoes e etc
*/

void desenha_teclado(void)
{
	int i, j;
	int xt, yt; // x e y das teclas
	/* 
		16 teclas, com margens de 20px em uma tela de 240px de largura, sobra 200 para desenho.
		Logo, 50 px de step. 48px de largura nas teclas, um de folga em cada lado pra nao grudar tudo.
		Alem disso, o teclado comecara no pixel 100, na vertical.
	*/
	for(i = 0; i < 4; i++)
	{
		for(j = 0; j < 4; j++)
		{
			xt = xinicial + step * j;
			yt = yinicial + step * i;
			lcd_retangulo(xt, yt, xt+tamanhoDaTecla, yt+tamanhoDaTecla, 0);
		}
	}
}

/* 
	Funcao que detecta toque na tela 
	Porta GPIOA, no bit 15, detecta atividade no touch screen. Caso tiver atividade, ele valera 0, senao 1.
*/

int le_touch1(void)
{
	int x, y, z;
	int i, j;
	
	if(GPIOA_IDR & (1 << 15)) return 0;
	
	(void)touch_read_xy(&x, &y, &z);
	i = (y - yinicial) / step;
	j = (x - xinicial) / step;
	
	return (4 * i + j + 'A'); // retorna a letra da tecla
}

int main(void)
{
	int k = 80;
	int teclaPressionada;
	clock_setup();	/* Configura clock e Sys Tick */
	usart_setup();	/* Configura USART1 */
	sdram_init();	/* Inicializa SDRAM de 8 MByte em 0xd0000000 */
	stxt.font = dos8x16;
	stxt.altura = 16; // caracteres tem 16 bits de altura
	/* Inicializa LCD e touch-screen com calibracao */
	touch_calibra(&stxt); // Configura e inicializa LCD, I2C e chama a rotina de calibracao do touch screen
	stxt.corfrente = 0; // Escrever texto preto
	stxt.tamanho=1; // Tamanho pequeno (pode ser 1, 2 ou 3)
	stxt.coluna=4; // 
	stxt.linha=4; // escreve 40 pixels abaixo do inicio
	mprintf(lcd_putchar, "Calculadora");
	desenha_teclado();
	lcd_show_frame();
	while (1) {
		if(GPIOA_IDR & 1) {	/* Aperto no botao azul: limpa a tela */
			if(k == 0) { 
				lcd_retangulo(0,0,239,319,0xffff);
				lcd_show_frame();
				k = 80;
				}
			}
		else	if(k) k--;
		/* Detecta atividade no touch-screen */
		teclaPressionada = le_touch1();
		if (teclaPressionada != 0) U1putchar(teclaPressionada);
	}
	return 0;
}
