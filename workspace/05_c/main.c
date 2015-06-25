	#include "stm32f10x.h"
	#include "stm32f10x_conf.h"
	#include "stbm_delay.h"
	#include "stbm_mpu9250.h"
	#include "stbm_mpu.h"
	
	// FreeRTOS
	#include "FreeRTOSConfig.h"
	#include "FreeRTOS.h"
	#include "queue.h"
	#include "semphr.h"
	#include "task.h"

#define	_BV(n)		( 1 << (n) )

int main(void)
{
	// 変数宣言
	uint8_t i = 0;
	int16_t accel_raw[3];
	int16_t accel_raw_old[3];
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// ここからプログラム
	SystemInit();
	NVIC_SetVectorTable(0x3000, 0);	// Set ISR vector table offset for DFU use
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
	AFIO->MAPR = _BV(26);	// Disable JTAG
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = _BV(13) | _BV(15);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	xbee_init(57600);
	delay_ms(10);
	
	while(1){
		xbee_send_data(i++);
		GPIOA->ODR = _BV(13);
		delay_ms(500);
		GPIOA->ODR = _BV(15);
		delay_ms(500);
		//mpu_get_accel_reg(accel_raw);
		//if(accel_raw_old[2] != accel_raw[2]){
		//	GPIOA->ODR = _BV(15);
		//}
		//for(int i=0; 2; i++){
		//	accel_raw_old[i] = accel_raw[i];
		//}
		//delay_ms(200);
	}
}

