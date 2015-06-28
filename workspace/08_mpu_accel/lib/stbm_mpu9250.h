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

#ifndef _STBM_MPU9250_H
#define _STBM_MPU9250_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include "stm32f10x_conf.h"


//#define MPU9250_I2C_MST_CTRL            0x24
//#define MPU9250_I2C_SLV0_ADDR           0x25
//#define MPU9250_I2C_SLV0_REG            0x26
//#define MPU9250_I2C_SLV0_CTRL           0x27
//#define MPU9250_I2C_SLV1_ADDR           0x28
//#define MPU9250_I2C_SLV1_REG            0x29
//#define MPU9250_I2C_SLV1_CTRL           0x2A
//#define MPU9250_I2C_SLV2_ADDR           0x2B
//#define MPU9250_I2C_SLV2_REG            0x2C
//#define MPU9250_I2C_SLV2_CTRL           0x2D
//#define MPU9250_I2C_SLV3_ADDR           0x2E
//#define MPU9250_I2C_SLV3_REG            0x2F
//#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36

#define MPU9250_I2C_SLV4_EN 0x80
#define MPU9250_I2C_SLV4_DONE 0x40
#define MPU9250_I2C_SLV4_NACK 0x10
//
#define MPU9250_I2C_IF_DIS (0x10)
#define MPU9250_I2C_MST_EN (0x20)
//#define MPU9250_FIFO_RST (0x04)
//#define MPU9250_FIFO_ENABLE (0x40)

//#define MPU9250_SPIx_ADDR 0x00
#define MPU9250_I2C_READ 0x80
//#define MPU9250_SPIx_BR_CLEAR_MASK 0xFFC7


// SPI1
#define  SPI1_PORT                       GPIOA
#define  SPI1_RCC                        RCC_APB2Periph_SPI1
#define  SPI1_GPIO_RCC                   RCC_APB2Periph_GPIOA
#define  SPI1_NSS_PIN                    GPIO_Pin_4
#define  SPI1_SCK_PIN                    GPIO_Pin_5
#define  SPI1_MISO_PIN                   GPIO_Pin_6
#define  SPI1_MOSI_PIN                   GPIO_Pin_7

// functions
void    mpu9250_init(void);
void    mpu9250_select_cs(void);
void    mpu9250_deselect_cs(void);
uint8_t mpu9250_send_byte(uint8_t byte);
uint8_t mpu9250_read(uint8_t reg_addr);
uint8_t mpu9250_reads(uint8_t reg_addr, uint8_t len, uint8_t* data);
uint8_t mpu9250_write(uint8_t reg_addr, uint8_t data);
uint8_t mpu9250_writes(uint8_t reg_addr, uint8_t len, uint8_t* data);
uint8_t mpu9250_ak8963_read(uint8_t akm_addr, uint8_t reg_addr, uint8_t* data);
uint8_t mpu9250_ak8963_reads(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data);
uint8_t mpu9250_ak8963_write(uint8_t akm_addr, uint8_t reg_addr, uint8_t data);
uint8_t mpu9250_ak8963_writes(uint8_t akm_addr, uint8_t reg_addr, uint8_t len, uint8_t* data);



#ifdef __cplusplus
}
#endif

#endif /* _STBM_MPU9250_H */