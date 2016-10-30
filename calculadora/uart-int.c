/**************************************************************************
 * Laboratorio de processadores II-FENG-PUCRS 2016-2
 * Programa kit STM32F429 Discovery usando gcc e libopencm3
 * uart1-int.c: Configura a interface serial UART1 com interrupcao
 * Escrito por Marcos Augusto Stemmer
 *************************************************************************/
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include "uart.h"

char Fila_rx[TAM_FILA_RX];
volatile int rx_fim;		/* Local de chegada */
volatile int rx_ini;		/* Local de atendimento */

/* Atendimento da interrupcao da usart1 (necessario que tenha este nome) */
void usart1_isr(void)
{
uint32_t sr;
int i;
sr = USART_SR(USART1);
while(sr & USART_SR_RXNE) {	/* Detecta caractere recebido */
	Fila_rx[rx_fim] = USART_DR(USART1);	/* Le o coloca no fim da fila */
	i = (rx_fim + 1);
	if( i >= TAM_FILA_RX) i = 0;
	rx_fim = i;
	sr = USART_SR(USART1);
	}
}

/* Escreve um caractre na UART1 (direto, sem interrupcao) */
void U1putchar(int c)
{
	while((USART_SR(USART1) & USART_SR_TXE) == 0);
	USART_DR(USART1) = (c & 0xff);
}

/* Numero de caracteres na fila RX */
int U1recebeu(void)
{
	return ((rx_fim - rx_ini) % TAM_FILA_RX);
}

/* Pega caractere retirando do inicio da fila de recebimento */
int U1getchar(void)
{
	int c;
	while ( !U1recebeu());
	c = Fila_rx[rx_ini];
	rx_ini = (rx_ini + 1) % TAM_FILA_RX;
	return c & 0xff;
}

/* Configura a USART1 com interrupcao por recebimento */
void usart_setup(void)
{
/* Necessario habilitar GPIO clock usado na USART */
	rcc_periph_clock_enable(RCC_GPIOA);
 /* GPIOA-9 = USART1-TX;   GPIOA-10 = USART1-RX; em nivel CMOS. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
 /* No STM32F429i a USART e' a funcao alternativa 7 dos pinos 9 e 10 do GPIOA */
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
	rcc_periph_clock_enable(RCC_USART1);
	usart_set_baudrate(USART1, 19200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
	/* Habilita interrupcao da USART1 no NVIC */
	nvic_enable_irq(NVIC_USART1_IRQ);
	/* Habilita interrupcao RXNEIE na USART */
	rx_ini = rx_fim = 0;
	USART_CR1(USART1) |= USART_CR1_RXNEIE ;
}

/* Envia uma mensagem no LCD */
void U1puts(char *txt)
{
while(*txt) U1putchar(*txt++);
}

/* Entrada de uma linha de texto */
void U1gets(char *txt, int nmax)
{
int k;	/* Numero de caracteres armazenados */
int c;	/* caractere lido */
k=0;
do	{
	c=U1getchar();
	if(c==0x7f) c=8;
	U1putchar(c);
	if(c == '\r') U1putchar('\n');
	/* se for backspace retira um caractere do buffer */
	if(c==8) {
		if(k)	{ 
			k--; U1putchar(' '); 
			U1putchar(8); 
			}
		}
	else if(k<nmax) txt[k++]=c;
	} while(c!='\n' && c!='\r');
txt[k-1]='\0';
}

/* Le um numero especificando a base de numeracao */
int U1getnum(char *prompt, unsigned base)
{
char buf[32];
U1puts(prompt);
U1gets(buf,32);
return xatoi(buf, base);
}

/* Converte string para numero especificando a base */
unsigned xatoi(char *str, unsigned base)
{
unsigned x,d, s;
x=0; s=0;
while((d = *str++) < '0') s |= (d=='-');
while(d >= '0') {;
	if(d >= 'a') d -= 'a' - 'A';
	d -= (d > '9')? 'A'-10: '0'; 
	if(d >= base) break;
	x = x*base + d;
	d = *str++;
	};
return s? -x: x;
}
