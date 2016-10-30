/**************************************************************************
 * Laboratorio de processadores II-FENG-PUCRS 2016-2
 * Programa kit STM32F429 Discovery usando gcc e libopencm3
 * ili9341b.c  Funcoes de baixo nivel para inicializar o display ILI9341 por SPI
 * Escrito por Marcos Augusto Stemmer
 * Link:
 * https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf
 **********************************************************************/

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "sdram.h"	/* Prototipos das funcoes de sdram */
#include "ili9341.h"	/* Nomes dos regs do controlados de LCD ili9341 */

/* Interface SPI5 usada no LCD */
#define LCD_SPI SPI5
#define SPI5_LCD_CS_0	(GPIOC_BSRR=(1<<18))
#define SPI5_LCD_CS_1	(GPIOC_BSRR=(1<<2))
#define LCD_DATA_CMD_0	(GPIOD_BSRR=(1<<29))
#define LCD_DATA_CMD_1	(GPIOD_BSRR=(1<<13))

/* Estrutura de dados que tem o contecto da escrita no lcd */
struct txtinfo *ptxinfo;
/***************************************************************************
 * Configura as portas e a interface SPI5 usada para comunicar com o display
 * PF7	SPI5_SCK		Clock do SPI5
 * PF9	SPI5_MOSI		Escreve dado SPI5
 * PC1	SPI5_MEMS_L3GD20	Chip Select do Acelerometro MEMS_L3GD20
 * PC2	SPI5_LCD_CS		Chip Select do display LCD-TFT
 * PD13	LCD_DATA_CMD		Data/Command do LCD-TFT
 * *************************************************************************/
void spi5_lcd_init(void)
{
/* Ativa portas GPIOC, GPIOD e GPIOF usadas para comunicar com o display */
	rcc_periph_clock_enable(RCC_GPIOC | RCC_GPIOD | RCC_GPIOF);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO13);
/* GPIOF poinos 9=LCD_MOSI e 7=LCD_SCK como AF5: interfce SPI do LCD */
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7 | GPIO9);
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO9);
/* Configura a interface SPI5 */
	rcc_periph_clock_enable(RCC_SPI5);
	spi_init_master(LCD_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_4,
				SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
				SPI_CR1_CPHA_CLK_TRANSITION_1,
				SPI_CR1_DFF_8BIT,
				SPI_CR1_MSBFIRST);
	spi_enable_ss_output(LCD_SPI);
	spi_enable(LCD_SPI);
}

/* Comando para o ili9341 */
void ili_cmd(int cmd)
{
	SPI5_LCD_CS_0;
	(void) spi_xfer(LCD_SPI, cmd);
	SPI5_LCD_CS_1;
}

/* 1 byte de dados */
void ili_data8(int data)
{
	LCD_DATA_CMD_1;
	SPI5_LCD_CS_0;
	(void) spi_xfer(LCD_SPI, data);
	SPI5_LCD_CS_1;
	LCD_DATA_CMD_0;
}

/* 16 bits (2 bytes) de dados */
void ili_data16(int data)
{
	LCD_DATA_CMD_1;
	SPI5_LCD_CS_0;
	(void) spi_xfer(LCD_SPI,(data >> 8) & 0xff);
	(void) spi_xfer(LCD_SPI,(data & 0xff));
	SPI5_LCD_CS_1;
	LCD_DATA_CMD_0;
}

/* Multiplos dados
 * pd = aponta para a sequencia
 * nd = numero de bytes */
void ili_mdata(const uint8_t *pd, int nd)
{
	LCD_DATA_CMD_1;
	SPI5_LCD_CS_0;
	while(nd--) { (void) spi_xfer(LCD_SPI, *pd++); }
	SPI5_LCD_CS_1;
	LCD_DATA_CMD_0;
}

static const uint8_t pos_gamma_args[] = { 
	0x0F, 0x29, 0x24, 0x0C, 0x0E, 
	0x09, 0x4E, 0x78, 0x3C, 0x09, 
	0x13, 0x05, 0x17, 0x11, 0x00} ;
	
static const uint8_t neg_gamma_args[] = { 
	0x00, 0x16, 0x1B, 0x04, 0x11, 
	0x07, 0x31, 0x33, 0x42, 0x05, 
	0x0C, 0x0A, 0x28, 0x2F, 0x0F };

/* Sequencia de comandos de inicializacao do lcd grafico */
void ili_9341_init(void)
{
	LCD_DATA_CMD_0;
	ili_cmd(LCD_FRMCTR1); ili_data16(0x001b);/*0xB1 Frame Rate Control (In Normal Mode) */
	ili_cmd(LCD_POWER1); ili_data8(0x10);	/*0xC0 Power Control 1 */
	ili_cmd(LCD_POWER2); ili_data8(0x10);	/*0xC1 Power Control 2 */
	ili_cmd(LCD_VCOM1); ili_data16(0x4515);	/*0xC5 VCOM Control 1 */
	ili_cmd(LCD_VCOM2); ili_data8(0x90);	/*0xC7 VCOM Control 7 */
	ili_cmd(LCD_MAC); 	/*0x36 Memory Access Control register*/
	ili_data8(0xc8);	/*com 0x08 fica de cabeca para baixo */
	ili_cmd(LCD_RGB_INTERFACE); ili_data8(0xc2);	/*0xB0 RGB Interface Signal	*/
	ili_cmd(LCD_PIXEL_FORMAT); ili_data8(0x55);	/*0x3A Pixel Format: 16 bit/pix */
	ili_cmd(LCD_DFC);		/*0xB6 Display Function */
	ili_data16(0x0aa7);ili_data16(0x2704);
	msleep(50);
	ili_cmd(LCD_GAMMA); ili_data8(1);	/*0x26 Gamma register */
	ili_cmd(LCD_PGAMMA);	/*0xE0 Positive Gamma Correction */
	ili_mdata(pos_gamma_args, 15);
	ili_cmd(LCD_NGAMMA); 	/*0xE1 Negative Gamma Correction */
	ili_mdata(neg_gamma_args, 15);
	ili_cmd(LCD_SLEEP_OUT);		/*0x11*/
	msleep(200);
	ili_cmd(LCD_DISPLAY_ON);	/*0x29*/
	msleep(20);
}

/* Transfere dados da memoria FRAME_BUFFER para o display */
void lcd_show_frame(void)
{
	ili_cmd(LCD_COLUMN_ADDR);	/* 0x2a Column Address*/
	ili_data16(0);			/* x1 */
	ili_data16(LCD_WIDTH-1);	/* x2 */
	ili_cmd(LCD_PAGE_ADDR);		/* 0x2b Row Address */
	ili_data16(0);			/* y1 */
	ili_data16(LCD_HIGHT-1);	/* y2 */
	ili_cmd(LCD_GRAM);		/* 0x2C Write RAM */
	ili_mdata((const uint8_t *)FRAME_BUFFER, 2*240*320);
}

/******************************************************************
 * Copia do FRAME_BUFFER para o LCD os dados de uma area retangular
 * ****************************************************************/
void lcd_show_rect(int x1, int y1, int x2, int y2)
{
	int nw, y;
	uint8_t *pm;
	if(x1 > x2) { y = x1; x1 = x2; x2 = y; }
	if(y1 > y2) { y = y1; y1 = y2; y2 = y; }
	if(x1 < 0) x1 = 0;
	if(y1 < 0) y1 = 0;
	if(x2 >= LCD_WIDTH) x2 = LCD_WIDTH - 1;
	if(y2 >= LCD_HIGHT) y2 = LCD_HIGHT - 1;
	ili_cmd(LCD_COLUMN_ADDR);	/* 0x2a Column Address*/
	ili_data16(x1);
	ili_data16(x2);
	ili_cmd(LCD_PAGE_ADDR);		/* 0x2b Row Address */
	ili_data16(y1);
	ili_data16(y2);
	ili_cmd(LCD_GRAM);		/* 0x2C Write RAM */
	pm = (uint8_t *)FRAME_BUFFER + 2*y1 * LCD_WIDTH + 2*x1;
	nw = 2*(x2 - x1 + 1);
	for(y = y1; y <= y2; y++) {
		ili_mdata(pm, nw);
		pm += 2*LCD_WIDTH;
	}
}

/* inicializa o display */
void lcd_init(struct txtinfo *txi)
{
	ptxinfo = txi;
	spi5_lcd_init();
	ili_9341_init();
	lcd_retangulo(0,0,239,319, txi->corfundo);
	lcd_show_frame();
}

/* Converte componentes de cor RGB para 16 bits tipo 565 */
/* Como a interface e' big endian, os bytes ficam invertidos: */
/* G2G1G0B4B3B2B1B0 R4R3R2R1R0G5G4G3	*/
int lcd_cor565(int r, int g, int b)
{
return (((g & 0x1c) << 11) | ((b & 0xF8) << 5) |
	(r & 0xf8) | ((g & 0xe0) >> 5));
}

/* Usa o reg Memory Access Control Reg. para virar a imagem de cabeca para baixo */
void lcd_upsdown(void)
{
	static int pos = 0x08;
	ili_cmd(LCD_MAC); 	/*0x36 Memory Access Control register*/
	ili_data8(pos);	/*com 0x08 fica de cabeca para baixo */
	pos ^= 0xc0;
}	
	
/* Desenha um retangulo na memoria FRAME_BUFFER */
void lcd_retangulo(int x1, int y1, int x2, int y2, int cor)
{
	uint16_t *p;
	int x, y;
	if(x2 < x1) { x = x1; x1 = x2; x2 = x; }
	if(y2 < y1) { y = y1; y1 = y2; y2 = y; }
	if(x1 >= LCD_WIDTH || y1 >= LCD_HIGHT) return;
	if(x1 < 0) x1 = 0;
	if(y1 < 0) y1 = 0;
	if(x2 >= LCD_WIDTH) x2 = LCD_WIDTH - 1;
	if(y2 >= LCD_HIGHT) y2 = LCD_HIGHT - 1;
	p = FRAME_BUFFER + LCD_WIDTH*y1;
	for(y = y1; y <= y2; y++) {
		for(x = x1; x <= x2; x++){
			p[x] = cor;
			}
		p += LCD_WIDTH;
	}
}

/* ************************************
 * Faz um ponto escrevendo no FRAME BUFFER
 * A cor do ponto e'	ptxinfo->corfrente
 * O tamanho e'		ptxinfo->tamanho 
 **************************************/
void lcd_dot(int x, int y)
{
	uint16_t *p;
	int cor;
	if(x > LCD_WIDTH-2 || y > LCD_HIGHT-2 || x < 0 || y < 0) return;
	p = FRAME_BUFFER + x + y * LCD_WIDTH;
	cor = ptxinfo->corfrente;
	switch(ptxinfo->tamanho) {
		case 1: *p = ptxinfo->corfrente;
			break;
		case 2: *p = p[1]= p[LCD_WIDTH]=p[LCD_WIDTH+1]= cor;
			break;
		default:
		case 3: /*p[-LCD_WIDTH]=p[-1]=p[0]=p[1]=p[LCD_WIDTH]=cor;
			break;*/
		case 4: 
			*p++=cor; *p++=cor;*p++=cor;*p++=cor;
			if(y)	{
				p -= (LCD_WIDTH+3);
				*p++=cor;*p++=cor;
				p += (2*LCD_WIDTH - 3);
				}
			*p++=cor; *p++=cor;*p++=cor;*p++=cor;
			p += (LCD_WIDTH - 3);
			*p++=cor; *p++=cor;
			break;
		}
}

/* ************************************************
 * Escreve um caractere no display grafico
 * Usa a estrutura txinfo para informar o contexto 
 **************************************************/
void lcd_putchar(int c)
{
	int x, y, k, lc, m;
	y = ptxinfo->linha;
	switch(c) {
		case '\r': ptxinfo->coluna = 4; break;
		case '\n': ptxinfo->linha += (ptxinfo->altura + 4)*ptxinfo->tamanho;
			break;
	}
	if(c < 0x20) return;
	for(k = ptxinfo->altura; k--; y += ptxinfo->tamanho) {
		x = ptxinfo->coluna;
		lc = ptxinfo->font[ptxinfo->altura * (c - 0x1f) - k];
		for(m = 0x80; m; m >>= 1) {
			if(m & lc) lcd_dot(x, y);
			x += ptxinfo->tamanho;
			}
		}
	ptxinfo->coluna += 9*ptxinfo->tamanho;
}

/*********************************************
Algoritmo de Bresenham para desenhar um circulo
[http://en.wikipedia.org/wiki/Midpoint_circle_algorithm]
*************************************************/
void plot4points(int cx, int cy, int x, int y)
{
	lcd_dot(cx + x, cy + y);
	if (x) lcd_dot(cx - x, cy + y);
	if (y) lcd_dot(cx + x, cy - y);
	lcd_dot(cx - x, cy - y);
}

/* Circulo com centro em (cx,cy)  */
void lcd_circulo(int cx, int cy, int raio)
{
int erro = -raio;
int x = raio;
int y = 0;
while (x >= y) {
	plot4points(cx, cy, x, y);
	plot4points(cx, cy, y, x);
	erro += y++;
	erro += y;
	if (erro >= 0) {
		erro -= (--x);
		erro -= x;
		}
	}
}

/* Desenha uma linha reta */
void lcd_linha(int x1, int y1, int x2, int y2)
{
int dx, dy, sx, sy;
int erro;
sx = sy =1;
dx = x2-x1;
if(dx < 0) { dx = -dx; sx=-1; }
dy = y2-y1;
if(dy < 0) { dy = -dy; sy=-1; }
if(dx > dy) {
	erro = dx/2;
	do	{
		lcd_dot(x1,y1);
		if(x1 == x2) break;
		x1 += sx;
		erro -= dy;
		if(erro < 0) { y1 += sy; erro += dx; }
		} while(1);
	}
else	{
	erro = dy/2;
	do	{
		lcd_dot(x1,y1);
		if(y1 == y2) break;
		y1 += sy;
		erro -= dx;
		if(erro < 0) { x1 += sx; erro += dy; }
		} while(1);
	}
}

/*********************************
 * lcd_flood
 * Preenche uma area substituindo os pixels da cor inicial
 * por pixels da nova cor
 * ******************************/
#define NPONTOS 20000

/* Variaveis globais da funcao lcd_flood */
struct sponto {
	uint16_t x, y;
} *pontos;

int npontos, cori, corf;
uint16_t *ppix;

/* Usado pela funcao lcd_flood */
int novoponto(int x, int y)
{
	int ui;
	if(x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HIGHT) return -1;
	ui = x+y*LCD_WIDTH;
	if(ppix[ui] != cori || npontos > NPONTOS-2) return -1;
	ppix[ui]=corf;
	pontos[npontos].x = x;
	pontos[npontos].y = y;
	npontos++;
	return npontos;
}

/* Usado pela funcao lcd_flood */
int mataponto(int k)
{
	if(k < npontos) {
		pontos[k].x = pontos[npontos].x;
		pontos[k].y = pontos[npontos].y;
	}
	return --npontos;
}

/****************************************************************
 * Enche uma area da imagem trocando os pixels da mesma cor do
 * ponto inicial pela cor dada como parametro
 * *************************************************************/
void lcd_flood(int x, int y, int cor)
{
	int k;
	int ix, iy;
	int ui;
	pontos = (struct sponto *)(FRAME_BUFFER + 320*240);
	corf=cor;
	ppix = (uint16_t *)FRAME_BUFFER;
	npontos = 1;
	pontos[0].x = x;
	pontos[0].y = y;
	ui = x+y*LCD_WIDTH;
	cori = ppix[ui];
	ppix[ui] = cor;
	do	{
		for(k = 0; k < npontos; k++) {
			ix = pontos[k].x; iy = pontos[k].y;
			novoponto(ix-1, iy); novoponto(ix+1, iy);
			novoponto(ix-1, iy-1); novoponto(ix+1, iy+1);
			novoponto(ix+1, iy-1); novoponto(ix-1, iy+1);
			novoponto(ix, iy-1); novoponto(ix, iy+1);
			mataponto(k);
		}
	}while(npontos);
}
