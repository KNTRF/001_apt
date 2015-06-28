/** charset=UTF-8 **/

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stbm_delay.h"
#include "stbm_uart.h"
#include "stbm_mpu9250.h"
#include "stbm_mpu.h"
#include "tsprintf.h"
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

// FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define	_BV(n)		( 1 << (n) )

#define LED1_ON      GPIO_SetBits(GPIOA, GPIO_Pin_13)
#define LED2_ON      GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define LED1_OFF     GPIO_ResetBits(GPIOA, GPIO_Pin_13)
#define LED2_OFF     GPIO_ResetBits(GPIOA, GPIO_Pin_15)



const char Message_A[] = "A: This is message A.\n";
const char Message_B[] = "B: This is message B.\n";
const char Message_C[] = "C: This is message C.\n";
const char Message_D[] = "D: This is message D.\n";
const char Message_E[] = "E: This is message E.\n";


int main(void)
{
	// 変数宣言
	uint8_t i = 0;
	uint8_t wai = 0;
	int8_t result = 0;
	int16_t accel_raw[3] = {0,0,0};
	int16_t gyro_raw[3] = {0,0,0};
	//int16_t accel_raw_old[3];
	GPIO_InitTypeDef GPIO_InitStructure;
	char st[30];
	
	// ここからプログラム
	SystemInit();
	
	// XBee DMA通信 割り込みハンドラ設定
	xbee_nvic_configuration();
	
	NVIC_SetVectorTable(0x3000, 0);	// Set ISR vector table offset for DFU use
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
	AFIO->MAPR = _BV(26);	// Disable JTAG
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//GPIO_InitStructure.GPIO_Pin = _BV(13) | _BV(15);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	xbee_init();
	delay_ms(100);
	mpu9250_init();
	delay_ms(100);
	mpu_init();
	delay_ms(100);
	mpu_who_am_i(&wai);
	tsprintf(st, "who am i : 0x%2X\n", wai);
	xbee_send_string(st);
	delay_ms(100);
	tsprintf(st, "ax: %d, ay: %d, az: %d\n", accel_raw[0], accel_raw[1], accel_raw[2]);
	xbee_send_string(st);
	delay_ms(100);
	tsprintf(st, "gx: %d, gy: %d, gz: %d\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
	xbee_send_string(st);
		
	
	while(1){
		//xbee_send_data(i++);
		if(i==0){
			LED1_ON;
			LED2_OFF;
			i++;
		} else {
			LED1_OFF;
			LED2_ON;
			i = 0;
		}
		delay_ms(1000);
		
		//Send message successively
		//xbee_send_string(Message_A);
		//xbee_send_string(Message_B);
		//xbee_send_string(Message_C);
		//xbee_send_string(Message_D);
		//xbee_send_string(Message_E);
		//xbee_send_string("\r\n");
		//delay_ms(1000);
		result =mpu_get_accel_reg(accel_raw);
		delay_ms(10);
		tsprintf(st, "ax: %d, ay: %d, az: %d   ", accel_raw[0], accel_raw[1], accel_raw[2]);
		xbee_send_string(st);
		delay_ms(10);
		//tsprintf(st, "result=%d\n", result);
		//xbee_send_string(st);
		result = mpu_get_gyro_reg(gyro_raw);
		delay_ms(10);
		tsprintf(st, "gx: %d, gy: %d, gz: %d\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
		xbee_send_string(st);
		delay_ms(10);
		//if(accel_raw_old[2] != accel_raw[2]){
		//	GPIOA->ODR = _BV(15);
		//}
		//for(int i=0; 2; i++){
		//	accel_raw_old[i] = accel_raw[i];
		//}
		//delay_ms(200);
	}
}




