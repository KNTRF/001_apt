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
#include "stbm_delay.h"


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

uint8_t mpu9250_read(uint8_t reg_addr){
	uint8_t dummy = 0;
	uint8_t data = 0;
	
	mpu9250_select_cs();
	mpu9250_send_byte(0x80 | reg_addr);
	data = mpu9250_send_byte(dummy);
	mpu9250_deselect_cs();
	
	return data;
}

uint8_t mpu9250_reads(uint8_t reg_addr, uint8_t len, uint8_t* data){
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

uint8_t mpu9250_write(uint8_t reg_addr, uint8_t data){
	uint8_t temp;
	
	mpu9250_select_cs();
	mpu9250_send_byte(reg_addr);
	temp = mpu9250_send_byte(data);
	mpu9250_deselect_cs();
	
	return temp;
}

uint8_t mpu9250_writes(uint8_t reg_addr, uint8_t len, uint8_t* data){
	uint32_t i = 0;
	
	mpu9250_select_cs();
	mpu9250_send_byte(reg_addr);
	while(i < len){
		mpu9250_send_byte(data[i++]);
	}
	mpu9250_deselect_cs();
	
	return 0;
}


uint8_t mpu9250_ak8963_read(uint8_t akm_addr, uint8_t reg_addr, uint8_t* data) {
	uint8_t status = 0;
	uint32_t timeout = 0;

	mpu9250_writes(MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	delay_ms(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	mpu9250_writes(MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	delay_ms(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	mpu9250_writes(MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	delay_ms(1);

	do {
		if (timeout++ > 50){
			return -2;
		}
		mpu9250_reads(MPU9250_I2C_MST_STATUS, 1, &status);
		delay_ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	mpu9250_reads(MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

uint8_t mpu9250_ak8963_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	uint8_t index = 0;
	uint8_t status = 0;
	uint32_t timeout = 0;
	uint8_t tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	mpu9250_writes(MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);
	while(index < len){
		tmp = reg_addr + index;
		mpu9250_writes(MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay_ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		mpu9250_writes(MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay_ms(1);

		do {
			if (timeout++ > 50){
				return -2;
			}
			mpu9250_reads(MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(2);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		mpu9250_reads(MPU9250_I2C_SLV4_DI, 1, data + index);
		delay_ms(1);
		index++;
	}
	return 0;
}

uint8_t mpu9250_ak8963_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data){
	uint32_t timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;

	tmp = akm_addr;
	mpu9250_writes(MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);
	tmp = reg_addr;
	mpu9250_writes(MPU9250_I2C_SLV4_REG, 1, &tmp);
	delay_ms(1);
	tmp = data;
	mpu9250_writes(MPU9250_I2C_SLV4_DO, 1, &tmp);
	delay_ms(1);
	tmp = MPU9250_I2C_SLV4_EN;
	mpu9250_writes(MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	delay_ms(1);

	do {
		if (timeout++ > 50)
			return -2;

		mpu9250_reads(MPU9250_I2C_MST_STATUS, 1, &status);
		delay_ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}

uint8_t mpu9250_ak8963_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data){
	uint32_t timeout = 0;
	uint8_t status = 0;
	uint8_t tmp = 0;
	uint8_t index = 0;

	tmp = akm_addr;
	mpu9250_writes(MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	delay_ms(1);

	while(index < len){
		tmp = reg_addr + index;
		mpu9250_writes(MPU9250_I2C_SLV4_REG, 1, &tmp);
		delay_ms(1);
		mpu9250_writes(MPU9250_I2C_SLV4_DO, 1, data + index);
		delay_ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		mpu9250_writes(MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		delay_ms(1);

		do {
			if (timeout++ > 50)
				return -2;
			mpu9250_reads(MPU9250_I2C_MST_STATUS, 1, &status);
			delay_ms(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}


// ---------------------------------------------------------------------------------------------------------------------------------

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
low pass filter value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ 
BITS_DLPF_CFG_5HZ 
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/
#define MPU_InitRegNum 17
 
uint8_t mpu9250_init2(int sample_rate_div,int low_pass_filter){
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        {0x80, MPUREG_PWR_MGMT_1},     // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},     // Enable Acc & Gyro
        {low_pass_filter, MPUREG_CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18, MPUREG_GYRO_CONFIG},    // +-2000dps
        {0x08, MPUREG_ACCEL_CONFIG},   // +-4G
        {0x09, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, MPUREG_INT_PIN_CFG},    //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay
		
        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte
		
        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
        
    };
	
    spi.format(8,0);
    spi.frequency(1000000);
 
    for(i=0; i<MPU_InitRegNum; i++) {
        mpu9250_write(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        delay_ms(1);  //I2C must slow down the write speed, otherwise it won't work
    }
 
    set_acc_scale(2);
    set_gyro_scale(250);
    
    //AK8963_calib_Magnetometer();  //Can't load this function here , strange problem?
    return 0;
}
/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu9250_spi::set_acc_scale(int scale){
    unsigned int temp_scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale);
    
    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;   
    }
    temp_scale=WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}
 
 
/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right range for the
gyroscopes. Suitable ranges are:
BITS_FS_250DPS
BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu9250_spi::set_gyro_scale(int scale){
    unsigned int temp_scale;
    WriteReg(MPUREG_GYRO_CONFIG, scale);
    switch (scale){
        case BITS_FS_250DPS:
            gyro_divider=131;
        break;
        case BITS_FS_500DPS:
            gyro_divider=65.5;
        break;
        case BITS_FS_1000DPS:
            gyro_divider=32.8;
        break;
        case BITS_FS_2000DPS:
            gyro_divider=16.4;
        break;   
    }
    temp_scale=WriteReg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);
    switch (temp_scale){
        case BITS_FS_250DPS:
            temp_scale=250;
        break;
        case BITS_FS_500DPS:
            temp_scale=500;
        break;
        case BITS_FS_1000DPS:
            temp_scale=1000;
        break;
        case BITS_FS_2000DPS:
            temp_scale=2000;
        break;   
    }
    return temp_scale;
}
 
 
/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu9250 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/
unsigned int mpu9250_spi::whoami(){
    unsigned int response;
    response=WriteReg(MPUREG_WHOAMI|READ_FLAG, 0x00);
    return response;
}
 
 
/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void mpu9250_spi::read_acc()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }
    
}
 
/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/
void mpu9250_spi::read_rot()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyroscope_data[i]=data/gyro_divider;
    }
 
}
 
/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data. 
returns the value in °C
-----------------------------------------------------------------------------------------------*/
void mpu9250_spi::read_temp(){
    uint8_t response[2];
    int16_t bit_data;
    float data;
    ReadRegs(MPUREG_TEMP_OUT_H,response,2);
 
    bit_data=((int16_t)response[0]<<8)|response[1];
    data=(float)bit_data;
    Temperature=(data/340)+36.53;
    deselect();
}
 
/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/
void mpu9250_spi::calib_acc()
{
    uint8_t response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    set_acc_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);
 
    ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0]=((response[0]&11100000)>>3)|((response[3]&00110000)>>4);
    calib_data[1]=((response[1]&11100000)>>3)|((response[3]&00001100)>>2);
    calib_data[2]=((response[2]&11100000)>>3)|((response[3]&00000011));
 
    set_acc_scale(temp_scale);
}
uint8_t mpu9250_spi::AK8963_whoami(){
    uint8_t response;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
 
    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    wait(0.001);
    response=WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C 
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C 
 
    return response;
}
void mpu9250_spi::AK8963_calib_Magnetometer(){
    uint8_t response[3];
    float data;
    int i;
 
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83); //Read 3 bytes from the magnetometer
 
    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    wait(0.001);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00);    //Read I2C 
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C 
    for(i=0; i<3; i++) {
        data=response[i];
        Magnetometer_ASA[i]=((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
}
void mpu9250_spi::AK8963_read_Magnetometer(){
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;
 
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 6 bytes from the magnetometer
 
    wait(0.001);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        Magnetometer[i]=data*Magnetometer_ASA[i];
    }
}
void mpu9250_spi::read_all(){
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;
 
    //Send I2C command at first
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); //Read 7 bytes from the magnetometer
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
 
    //wait(0.001);
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
    //Get accelerometer value
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accelerometer_data[i]=data/acc_divider;
    }
    //Get temperature
    bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
    data=(float)bit_data;
    Temperature=((data-21)/333.87)+21;
    //Get gyroscop value
    for(i=4; i<7; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyroscope_data[i-4]=data/gyro_divider;
    }
    //Get Magnetometer value
    for(i=7; i<10; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        Magnetometer[i-7]=data*Magnetometer_ASA[i-7];
    }
}
 
/*-----------------------------------------------------------------------------------------------
                                SPI SELECT AND DESELECT
usage: enable and disable mpu9250 communication bus
-----------------------------------------------------------------------------------------------*/
void mpu9250_spi::select() {
    //Set CS low to start transmission (interrupts conversion)
    cs = 0;
}
void mpu9250_spi::deselect() {
    //Set CS high to stop transmission (restarts conversion)
    cs = 1;
}


