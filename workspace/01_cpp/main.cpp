#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "delay.h"
#include "STBM_UART.h"

#define	_BV(n)		( 1 << (n) )

int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	STBM_UART serial1(2,57600);
	
	SystemInit();
	NVIC_SetVectorTable(0x3000, 0);	// Set ISR vector table offset for DFU use
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
	AFIO->MAPR = _BV(26);	// Disable JTAG
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = _BV(13) | _BV(15);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	serial1.init();
	
	while(1){
		GPIOA->ODR = _BV(13);
		delay_ms(1000);
		GPIOA->ODR = _BV(15);
		delay_ms(1000);
		serial1.send_data('a');
		delay_ms(1000);
	}
}

