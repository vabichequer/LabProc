/**************************************************************************
 * Laboratorio de processadores II-FENG-PUCRS 2016-2
 * Programa kit STM32F429 Discovery usando gcc e libopencm3
 * Leitura do Touch Screen
 * Escrito por Marcos Augusto Stemmer
*********************************************************************
 * Touch Screen STMPE811 conectado ao I2c3 nos pinos
 * PA8=SCL3	PC9=SDA3	PA15=TP_INT1 (interrupcao do STMP811)
 * Referencias:
 * www.st.com/resource/en/datasheet/CD00186725.pdf
 * www.st.com/resource/en/application_note/cd00203648.pdf
*********************************************************************/
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include "uart.h"
#include "ili9341.h"
#include "i2c3_touch.h"

#define _DEBUG

uint32_t hi2c = I2C3;
extern volatile uint32_t miliseg;
#define STMP811_DEV_ADDRESS 0x41
/***********************************
 * Inicializa a interface I2C3
 * PA8=SCL3	PC9=SDA3
 * *********************************/
void i2c3_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOA | RCC_GPIOC);
/* Pinos PA8=SCL3 PC9=SDA3 do I2C3 como AF4 */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);
	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);
	rcc_periph_clock_enable(RCC_I2C3);
	i2c_peripheral_disable(I2C3);
	i2c_reset(I2C3);
	i2c_set_standard_mode(I2C3);
	i2c_enable_ack(I2C3);
	i2c_set_dutycycle(I2C3, I2C_CCR_DUTY_DIV2); /* default, no need to do this really */
	i2c_set_clock_frequency(I2C3, I2C_CR2_FREQ_24MHZ);
	/* divisor ccr = 120 = 24MHz / (100kHz * 2) */
	i2c_set_ccr(I2C3, 120);
	i2c_set_trise(I2C3, 10);
	i2c_peripheral_enable(I2C3);
}

/* Funcao auxiliar para ajustar o seletor de registrador (uso interno apenas) */
int i2c3_set_reg(uint8_t reg)
{
	uint32_t reg32;
	miliseg=0;
	while ((I2C_SR2(hi2c) & I2C_SR2_BUSY)) {
		if(miliseg >= 100) return -1;
	}

	i2c_send_start(hi2c);
	/* Espera terminar start-bit em modo mestre */
	miliseg=0;
	while (!((I2C_SR1(hi2c) & I2C_SR1_SB)
		& (I2C_SR2(hi2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))){
		if(miliseg >= 100) return -2;
	};

	i2c_send_7bit_address(hi2c, STMP811_DEV_ADDRESS, I2C_WRITE);

	/* Espera enviar endereco */
	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_ADDR)){
		if(miliseg >= 100) return -3;
	};

	reg32 = I2C_SR2(hi2c);
	(void)reg32;
	/* Escreve numero do reg */
	i2c_send_data(hi2c, reg);
	miliseg=0;
	while (!(I2C_SR1(hi2c) & (I2C_SR1_BTF))){
		if(miliseg >= 100) return -4;
	};
	return 0;
}

/***************************************************/
/* Escreve em um registrador de 8 bits do STMP811 */
/***************************************************/
int i2c3_write_reg8(uint8_t reg, uint8_t val)
{
	if(i2c3_set_reg(reg)) return -1;
	i2c_send_data(hi2c, val);
	miliseg=0;
	while (!(I2C_SR1(hi2c) & (I2C_SR1_BTF | I2C_SR1_TxE))){
		if(miliseg >= 100) return -5;
	};
	i2c_send_stop(hi2c);
	return 0;
}

/******************************************/
/* Le um registrador de 8 bits do STMP811 */
/******************************************/
int i2c3_read_reg8(uint8_t reg)
{
	uint32_t reg32, result;
	if(i2c3_set_reg(reg)) return -1;
	i2c_send_start(hi2c);
	/* Espera start-bit em modo mestre */
	miliseg=0;
	while (!((I2C_SR1(hi2c) & I2C_SR1_SB)
		& (I2C_SR2(hi2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))){
		if(miliseg >= 100) return -10;
	};
	i2c_send_7bit_address(hi2c, STMP811_DEV_ADDRESS, I2C_READ);
	/* Espera transferir o endereco */
	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_ADDR)){
		if(miliseg >= 100) return -11;
	};

	i2c_disable_ack(hi2c);

	/* Le e descarta, para sair da condicao de endereco */
	reg32 = I2C_SR2(hi2c);
	(void)reg32;
	i2c_send_stop(hi2c);

	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_RxNE)){
		if(miliseg >= 100) return -12;
	};
	result = i2c_get_data(hi2c);

	i2c_enable_ack(hi2c);
	I2C_SR1(hi2c) &= ~I2C_SR1_AF;
	return (int)result;
}

/*******************************************/
/* Le um registrador de 16 bits do STMP811 */
/*******************************************/
int i2c3_read_reg16(uint8_t reg)
{
	uint32_t reg32, result;
	if(i2c3_set_reg(reg)) return -1;
	i2c_send_start(hi2c);
	/* Espera start-bit em modo mestre */
	miliseg=0;
	while (!((I2C_SR1(hi2c) & I2C_SR1_SB)
		& (I2C_SR2(hi2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))){
		if(miliseg >= 100) return -17;
	};

	i2c_send_7bit_address(hi2c, STMP811_DEV_ADDRESS, I2C_READ);
	/* Espera transferir o endereco */
	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_ADDR)){
		if(miliseg >= 100) return -18;
	};
	i2c_enable_ack(hi2c);
	/* Le e descarta, para sair da condicao de endereco */
	reg32 = I2C_SR2(hi2c);
	(void)reg32;
	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_RxNE)){
		if(miliseg >= 100) return -19;
	};
	
	result = (i2c_get_data(hi2c) << 8);	/* Le primeiro byte */

	i2c_disable_ack(hi2c);	/* Ultimo byte deve ser lido com nack */
	i2c_send_stop(hi2c);
	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_RxNE)){
		if(miliseg >= 100) return -20;
	};
	result |= i2c_get_data(hi2c);		/* Junta com o segundo byte */

	i2c_enable_ack(hi2c);
	I2C_SR1(hi2c) &= ~I2C_SR1_AF;
	return (int)result;
}

/*****************************************
 * Le uma sequencia de dados
 * ***************************************/
int i2c3_read_array(uint8_t reg, uint8_t *data, int nb)
{
	uint32_t reg32;
	int k;
	if(i2c3_set_reg(reg)) return -1;
	i2c_send_start(hi2c);
	miliseg=0;
	while (!((I2C_SR1(hi2c) & I2C_SR1_SB)
		& (I2C_SR2(hi2c) & (I2C_SR2_MSL | I2C_SR2_BUSY)))){
		if(miliseg >= 100) return -17;
	};

	i2c_send_7bit_address(hi2c, STMP811_DEV_ADDRESS, I2C_READ);
	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_ADDR)){
		if(miliseg >= 100) return -18;
	};

	i2c_enable_ack(hi2c);
	reg32 = I2C_SR2(hi2c);
	(void)reg32;
	miliseg=0;
	while (!(I2C_SR1(hi2c) & I2C_SR1_RxNE)){
		if(miliseg >= 100) return -19;
	};
	k = 0;
	while(nb--) {
		if(nb == 0) {
			i2c_disable_ack(hi2c);
			i2c_send_stop(hi2c);
		} else	i2c_enable_ack(hi2c);

		miliseg=0;
		while (!(I2C_SR1(hi2c) & I2C_SR1_RxNE)){
			if(miliseg >= 100) return -19;
		};
		data[k++] = i2c_get_data(hi2c);
	}
	return 0;
}

/*********************************************
 * Parte especifica do touch-screen
 * *******************************************/
struct regset {
	uint8_t reg,val;
};

/*******************************************************
 * Sequencia de bytes de inicializacao do touch-screen 
 * Retorna 0 se ok; -1 se teve erro;	
********************************************************/
int touch_init(void)
{
	int k;
	static const struct regset reginit[]={
		{0x04,0x0c},	/* Turn on touch-screen and ADC */
		{0x0a,0x02},	/* Enable interrupts: TOUCH_DETECT and FIFO_THRESHOLD */
		{0x20,0x48},	/* Select sample time: 72 Clocks */
		{0x21,0x01},	/* Select ADC Clock speed: 3.25 MHz */
		{0x17,0x00},	/* Select I/O pins to primary function */
		{0x41,0x9a},	/* Select 500us touch detect delay */
		{0x4a,0x05},	/* FIFO interrupt threshold: 5 points */
		{0x4b,0x01},	/* Clear FIFO */
		{0x4b,0x00},	/* Set FIFO back to normal operation */
		{0x56,0x07},	/* Set Z data format */
		{0x58,0x00},	/* Maximum current=20mA, If value=01 then maxcurrent = 50mA */
		{0x40,0x01},	/* Oparate in XYZ mode */
		{0x0b,0xff},	/* Clear interrupt flags */
		{0x09,0x01}};	/* Enable touch-detect interrupt */
	i2c3_init();
	k = i2c3_read_reg16(0);	/* Ask for device ID (should be 0x0811) */
	if(k != 0x0811) return -1;
	for(k = 0; k < 14; k++) {
		if(i2c3_write_reg8(reginit[k].reg, reginit[k].val)) return -1;
	}
	return 0;
}


/************************************************************************
 * Leitura crua do ADC do touch-screen sem mapear para coordenadas do LCD
 * Entra aqui quando tem pelo menos 4 pontos lidos
 *************************************************************************/
int touch_read_raw_xy(int *x, int *y, int *z)
{
	int intflag, k, k4, kmax;
	int size;
	uint8_t data[32];
	int rx[8], ry[8], rz[8];
	int mx, my, mz, x2, y2, max;
	intflag = i2c3_read_reg8(0x0b);
	size = -1;
	if((intflag & 0x02) ) {
		size = i2c3_read_reg8(0x4c);
		if(size <= 0) return -1;
		if(size > 8) size = 8;
		i2c3_read_array(0xd7, data, 4*size);
		for(k=0; k < size; k++) {
			k4=4*k;
			rx[k] = (data[k4]<<4) | ((data[k4+1]>>4) & 0x0f);
			ry[k] = ((data[k4+1] << 8) & 0xf00) | data[k4+2];
			rz[k] = data[k4+3];	/* rz indica a pressao do toque */
		}
		i2c3_write_reg8(0x0b, 0x02);	/* limpa indicador de int de pontos recebidos */
	}
	if((intflag & 0x01)) {	/* limpa indicador de int de toque */
		i2c3_write_reg8(0x0b, 0x01);
	}
/* A segur um procedimento para eliminar pontos errados (que se desviam muito da media) */
	if(size < 3) {	/* Nao fazer se tem menos de 3 pontos */
		*x = rx[0]; *y = ry[0]; *z = rz[0];
		return size;
	}
	/* Soma os pontos para calcular a media */
	mx = rx[0]; my = ry[0]; mz = rz[0];
	for(k=1; k < size; k++) {
		mx += rx[k];
		my += ry[k];
		mz += rz[k];
	}
	/* Determina o maximo desvio da media */
	max = kmax = 0; 
	for(k = 0; k < size; k++){
		x2 = (rx[k] - mx/size);
		y2 = (ry[k] - my/size);
		k4 = x2*x2 + y2*y2;
		if(k4 > max) { max = k4; kmax = k; }
	}
	/* Elimina da media o ponto de maior desvio */
	if(max > 200) {
		mx -= rx[kmax];
		my -= ry[kmax];
		mz -= rz[kmax];
		size--;
		if(size > 2) {
			max = kmax = 0; 
			for(k = 0; k < size; k++){
				x2 = (rx[k] - mx/size);
				y2 = (ry[k] - my/size);
				k4 = x2*x2 + y2*y2;
				if(k4 > max) { max = k4; kmax = k; }
				}
			if(max > 200) {
				mx -= rx[kmax];
				my -= ry[kmax];
				mz -= rz[kmax];
				size--;
				}
			}
		}
	*x = mx/size; *y = my/size; *z = mz/size;
	return size;
}

/* As variaveis globais a srguir sao coeficientes da transformacao de coordenadas */
int multix=0x4000, multiy=0x4000, somax=0, somay=0;

enum nome_estado { INICIAL, CALIBRA1, CALIBRA2, PRONTO };

/* Prototipo d funcao msleep que deve esperar t milisegundos */
void msleep(uint32_t t);

/* Distancia dos pontos de calibracao ate a borda da tela */
#define CAL_CX1	30
#define CAL_CY1 40
#define CAL_CX2 (LCD_WIDTH-40)
#define CAL_CY2 (LCD_HIGHT-40)

/***************************************************
 * Inicializa o LCD com procedimento 
 * de calibracao do touch screen
 * *************************************************/
int touch_calibra(struct txtinfo *ptxt)
{
	int k2, estado;
	int tx1=-1,ty1=-1;
	int tx2=-1,ty2=-1;
	int ix, iy, iz;
	ptxt->tamanho = 1;
	ptxt->tamanho = 1;
	ptxt->corfundo = lcd_cor565(0xff, 0xff,0xdf);
	ptxt->corfrente = 0;
	ptxt->linha = 140;
	ptxt->coluna = 14;
	lcd_init(ptxt);
	if(touch_init()) {
		mprintf(lcd_putchar,"Erro");
		return -1;
	}
	mprintf(lcd_putchar,"Toque no alvo para");
	ptxt->linha = 158;
	ptxt->coluna = 4;
	mprintf(lcd_putchar,"calibrar o Touch Screen");
	ptxt->tamanho=3;
	lcd_dot(CAL_CX1, CAL_CY1);	/* Desenha o primeiro alvo */
	lcd_circulo(CAL_CX1, CAL_CY1, 8);
	ptxt->tamanho=1;
	lcd_show_frame();
	k2 = 60; iz = 0;
	estado = INICIAL;
	while (1) {
		if((GPIOA_IDR & (1 << 15)) == 0) {
			touch_read_raw_xy( &ix, &iy, &iz);
			if(estado == INICIAL) estado = CALIBRA1;
			k2 = 60;
		} else if(k2) k2--;
		if(!k2 && estado == CALIBRA1) {	/* Registra o primeiro ponto de calibracao */
			tx1 = ix;
			ty1 = iy;
			estado = CALIBRA2;
			k2=60000;
			ptxt->tamanho=3;
			/* Apaga primeiro alvo */
			lcd_retangulo(CAL_CX1-10, CAL_CY1-10, 
				      CAL_CX1 + 10, CAL_CY1 + 10, ptxt->corfundo);
			lcd_show_rect(CAL_CX1-10, CAL_CY1-10, CAL_CX1 + 10, CAL_CY1 + 10);
			/* Desenha o segundo alvo */
			lcd_dot(CAL_CX2, CAL_CY2);
			lcd_circulo(CAL_CX2, CAL_CY2, 8);
			lcd_show_rect(	CAL_CX2 - 10, CAL_CY2 - 10,
					CAL_CX2 + 10, CAL_CY2 + 10);
			ptxt->tamanho=1;

		}
		if(!k2 && estado == CALIBRA2) {	/* Registra o segundo ponto de calibracao */
			tx2 = ix;
			ty2 = iy;
			/* Se os pontos estiverem fora do esperado, adota valores fixos */
			if(tx1 < 2800 || tx1 > 3500) tx1 = 3150;
			if(ty1 > 1000 || ty1 < 400 ) ty1 = 800;
			if(tx2 > 1000 || tx2 < 400 ) tx2 = 700;
			if(ty1 > 3800 || ty2 < 3000) ty2 = 3461;
			k2=80000;
#ifdef _DEBUG
			mprintf(U1putchar, "x1=%-4d y1=%d\r\nx2=%-4d y2=%d\r\n", tx1,ty1,tx2,ty2);
#endif
			/* Calcula os parametros da transformacao linear */
			multix = ((CAL_CX2 - CAL_CX1) << 12)/(tx2 - tx1);
			multiy = ((CAL_CY2 - CAL_CY1) << 12)/(ty2 - ty1);
			somax = CAL_CX1 - ((tx1 * multix) >> 12);
			somay = CAL_CY1 - ((ty1 * multiy) >> 12);
			estado = PRONTO;
			lcd_retangulo(0,0, LCD_WIDTH-1, LCD_HIGHT - 1, 0xffff);
			/*
			ptxt->tamanho = 1;
			ptxt->linha = 10;
			ptxt->coluna = 4;
			mprintf(lcd_putchar,"Touch-screen ativado\r\n");
			*/
			lcd_show_frame();
			break;
		}
		msleep(1);	/* ?? precisa deste tempo para funcionar ??? */
	}
return 0;
}

/***************************************************
 * Le local do toque em coordenadas do LCD
 * Recebe apontadores para x, y, z
 * O parametro z indica a pressao sobre a tela
 * ************************************************/
int touch_read_xy(int *x, int *y, int *z)
{
	int ix, iy, n;
	n = touch_read_raw_xy( &ix, &iy, z);
	/* Mapeia para coordenadas do LCD */
	*x = ((multix * ix) >> 12) + somax;
	*y = ((multiy * iy) >> 12) + somay;
	if(*x < 0) *x = 0;
	if(*y < 0) *y = 0;
	return n;
}
