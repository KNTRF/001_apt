/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "stbm_mpu9250.h"


void mpu9250_init(){
	/* Supply APB2 clock */
	RCC_APB2PeriphClockCmd(SPI1_RCC | SPI1_GPIO_RCC , ENABLE);
	
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Configure SPI1 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);
	
	/* Configure SPI1_NSS: xCS : output push-pull */
	GPIO_InitStructure.GPIO_Pin = SPI1_NSS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);
	
	/* SPI1 configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE);
}

void mpu9250_select_cs(){
	GPIO_ResetBits(SPI1_PORT, SPI1_NSS_PIN);
}

void mpu9250_deselect_cs(){
	GPIO_SetBits(SPI1_PORT, SPI1_NSS_PIN);
}

uint8_t mpu9250_send_byte(uint8_t byte){
	// Loop while DR register in not emplty
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	// Send byte through the SPI1 peripheral
	SPI_I2S_SendData(SPI1, byte);
	// Wait to receive a byte
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	// Return the byte read from the SPI bus
	return SPI_I2S_ReceiveData(SPI1);
}

uint8_t mpu9250_read(uint8_t addr, uint8_t reg_addr){
	uint8_t dummy = 0;
	uint8_t data = 0;
	
	mpu9250_select_cs();
	mpu9250_send_byte(0x80 | reg_addr);
	data = mpu9250_send_byte(dummy);
	mpu9250_deselect_cs();
	
	return data;
}

uint8_t mpu9250_reads(uint8_t addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	uint32_t i = 0;
	uint8_t dummy = 0x00;
	
	mpu9250_select_cs();
	mpu9250_send_byte(0x80 | reg_addr);
	while(i < len){
		data[i++] = mpu9250_send_byte(dummy);
	}
	mpu9250_deselect_cs();
	
	return 0;
}

uint8_t mpu9250_write(uint8_t addr, uint8_t reg_addr, uint8_t data){
	mpu9250_select_cs();
	mpu9250_sent_byte(reg_addr);
	mpu9250_sent_byte(data);
	mpu9250_deselect_cs();
	
	return 0;
}

uint8_t mpu9250_writes(uint8_t addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	uint32_t i = 0;
	
	mpu9250_select_cs();
	mpu9250_sent_byte(reg_addr);
	while(i < len){
		mpu9250_send_byte(data[i++]);
	}
	mpu9250_deselect_cs();
	
	return 0;
}

/*
//////////////////////////////////////////////////////////////////////////
//init
void MPU9250_Init(void)
{
	SPIx_Init(pMPU9250);
}

int MPU9250_SPIx_Write(u8 addr, u8 reg_addr, u8 data){
	CHIP_SELECT(pMPU9250);
	MPU9250_SPIx_SendByte(reg_addr);
	MPU9250_SPIx_SendByte(data);
	CHIP_DESELECT(pMPU9250);
	return 0;
}

int MPU9250_SPIx_Writes(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	CHIP_SELECT(pMPU9250);
	MPU9250_SPIx_SendByte(reg_addr);
	while(i < len){
		MPU9250_SPIx_SendByte(data[i++]);
	}
	CHIP_DESELECT(pMPU9250);
	return 0;
}

u8 MPU9250_SPIx_Read(u8 addr, u8 reg_addr)
{
	u8 dummy = 0;
	u8 data = 0;

	CHIP_SELECT(pMPU9250);
	MPU9250_SPIx_SendByte(0x80 | reg_addr);
	data = MPU9250_SPIx_SendByte(dummy);
	CHIP_DESELECT(pMPU9250);
	return data;
}

int MPU9250_SPIx_Reads(u8 addr, u8 reg_addr, u8 len, u8* data){
	u32 i = 0;
	u8 dummy = 0x00;

	CHIP_SELECT(pMPU9250);
	MPU9250_SPIx_SendByte(MPU9250_I2C_READ | reg_addr);
	while(i < len){
		data[i++] = MPU9250_SPIx_SendByte(dummy);
	}
	CHIP_DESELECT(pMPU9250);
	return 0;
}

int MPU9250_AK8963_SPIx_Read(u8 akm_addr, u8 reg_addr, u8* data) {
	u8 status = 0;
	u32 timeout = 0;

	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	Delay_Ms(1);

	do {
		if (timeout++ > 50){
			return -2;
		}
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

int MPU9250_AK8963_SPIx_Reads(u8 akm_addr, u8 reg_addr, u8 len, u8* data){
	u8 index = 0;
	u8 status = 0;
	u32 timeout = 0;
	u8 tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DI, 1, data + index);
		Delay_Ms(1);
		index++;
	}
	return 0;
}

int MPU9250_AK8963_SPIx_Write(u8 akm_addr, u8 reg_addr, u8 data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	tmp = reg_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
	Delay_Ms(1);
	tmp = data;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, &tmp);
	Delay_Ms(1);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	Delay_Ms(1);

	do {
		if (timeout++ > 50)
			return -2;

		MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

int MPU9250_AK8963_SPIx_Writes(u8 akm_addr, u8 reg_addr, u8 len, u8* data)
{
	u32 timeout = 0;
	uint8_t status = 0;
	u8 tmp = 0;
	u8 index = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);

	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_DO, 1, data + index);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(MPU9250_SPIx_ADDR, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do {
			if (timeout++ > 50)
				return -2;
			MPU9250_SPIx_Reads(MPU9250_SPIx_ADDR, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}
*/
