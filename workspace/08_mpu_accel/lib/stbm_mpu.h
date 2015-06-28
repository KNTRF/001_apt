#ifndef _STBM_MPU_H
#define _STBM_MPU_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
//#include "stm32f10x_conf.h"
#include "stbm_mpu9250.h"

#define USE_AK8963

#define BIT_I2C_MST_VDDIO     0x80
#define BIT_FIFO_EN           0x40
#define BIT_DMP_EN            0x80
#define BIT_FIFO_RST          0x04
#define BIT_DMP_RST           0x08
#define BIT_FIFO_OVERFLOW     0x10
#define BIT_DATA_RDY_EN       0x01
#define BIT_DMP_INT_EN        0x02
#define BIT_MOT_INT_EN        0x40
#define BITS_FSR              0x18
#define BITS_LPF              0x07
#define BITS_HPF              0x07
#define BITS_CLK              0x07
#define BIT_FIFO_SIZE_1024    0x40
#define BIT_FIFO_SIZE_2048    0x80
#define BIT_FIFO_SIZE_4096    0xC0
#define BIT_RESET             0x80	// 01 00 00 00
#define BIT_SLEEP             0x40	// 00 10 00 00
#define BIT_S0_DELAY_EN       0x01
#define BIT_S2_DELAY_EN       0x04
#define BITS_SLAVE_LENGTH     0x0F
#define BIT_SLAVE_BYTE_SW     0x40
#define BIT_SLAVE_GROUP       0x10
#define BIT_SLAVE_EN          0x80
#define BIT_I2C_READ          0x80
#define BITS_I2C_MASTER_DLY   0x1F
#define BIT_AUX_IF_EN         0x20
#define BIT_ACTL              0x80
#define BIT_LATCH_EN          0x20
#define BIT_ANY_RD_CLR        0x10
#define BIT_BYPASS_EN         0x02
#define BITS_WOM_EN           0xC0
#define BIT_LPA_CYCLE         0x20
#define BIT_STBY_XA           0x20
#define BIT_STBY_YA           0x10
#define BIT_STBY_ZA           0x08
#define BIT_STBY_XG           0x04
#define BIT_STBY_YG           0x02
#define BIT_STBY_ZG           0x01
#define BIT_STBY_XYZA         BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA
#define BIT_STBY_XYZG         BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG

#ifdef USE_AK8963
#define SUPPORTS_AK89xx_HIGH_SENS   0x10
#define AK89xx_FSR                  4915

#define AKM_REG_WHOAMI          0x00
#define AKM_REG_ST1             0x02
#define AKM_REG_HXL             0x03
#define AKM_REG_ST2             0x09
#define AKM_REG_CNTL            0x0A
#define AKM_REG_ASTC            0x0C
#define AKM_REG_ASAX            0x10
#define AKM_REG_ASAY            0x11
#define AKM_REG_ASAZ            0x12
#define AKM_DATA_READY          0x01
#define AKM_DATA_OVERRUN        0x02
#define AKM_OVERFLOW            0x80
#define AKM_DATA_ERROR          0x40
#define AKM_BIT_SELF_TEST       0x40
#define AKM_POWER_DOWN          0x00 | SUPPORTS_AK89xx_HIGH_SENS
#define AKM_SINGLE_MEASUREMENT  0x01 | SUPPORTS_AK89xx_HIGH_SENS
#define AKM_FUSE_ROM_ACCESS     0x0F | SUPPORTS_AK89xx_HIGH_SENS
#define AKM_MODE_SELF_TEST      0x08 | SUPPORTS_AK89xx_HIGH_SENS
#define AKM_WHOAMI              0x48
#endif

#define MPU6500_WHO_AM_I       0x75
#define MPU6500_RATE_DIV       0x19
#define MPU6500_LPF            0x1A
#define MPU6500_PROD_ID        0x0C
#define MPU6500_USER_CTRL      0x6A
#define MPU6500_FIFO_EN        0x23
#define MPU6500_GYRO_CFG       0x1B
#define MPU6500_ACCEL_CFG      0x1C
#define MPU6500_ACCEL_CFG2     0x1D
#define MPU6500_LP_ACCEL_ODR   0x1E
#define MPU6500_MOTION_THR     0x1F
#define MPU6500_MOTION_DUR     0x20
#define MPU6500_FIFO_COUNT_H   0x72
#define MPU6500_FIFO_R_W       0x74
#define MPU6500_RAW_GYRO       0x43
#define MPU6500_RAW_ACCEL      0x3B
#define MPU6500_TEMP           0x41
#define MPU6500_INT_ENABLE     0x38
#define MPU6500_DMP_INT_STATUS 0x39
#define MPU6500_INT_STATUS     0x3A
#define MPU6500_ACCEL_INTEL    0x69
#define MPU6500_PWR_MGNT_1     0x6B
#define MPU6500_PWR_MGNT_2     0x6C
#define MPU6500_INT_PIN_CFG    0x37
#define MPU6500_MEM_R_W        0x6F
#define MPU6500_ACCEL_OFFS     0x77
#define MPU6500_I2C_MST        0x24
#define MPU6500_BANK_SEL       0x6D
#define MPU6500_MEM_START_ADDR 0x6E
#define MPU6500_PRGM_START_H   0x70

#define AK8963_S0_ADDR         0x25
#define AK8963_S0_REG          0x26
#define AK8963_S0_CTRL         0x27
#define AK8963_S1_ADDR         0x28
#define AK8963_S1_REG          0x29
#define AK8963_S1_CTRL         0x2A
#define AK8963_S4_CTRL         0x34
#define AK8963_S0_DO           0x63
#define AK8963_S1_DO           0x64
#define AK8963_I2C_DELAY_CTRL  0x67
#define AK8963_RAW_COMPASS     0x49

#define MPU6500_GYRO_FSR_250DPS    0
#define MPU6500_GYRO_FSR_500DPS    1
#define MPU6500_GYRO_FSR_1000DPS   2
#define MPU6500_GYRO_FSR_2000DPS   3

#define MPU6500_ACCEL_FSR_2G    0
#define MPU6500_ACCEL_FSR_4G    1
#define MPU6500_ACCEL_FSR_8G    2
#define MPU6500_ACCEL_FSR_16G   3

#define MPU6500_LPF_256HZ    0
#define MPU6500_LPF_188HZ    1
#define MPU6500_LPF_98HZ     2
#define MPU6500_LPF_42HZ     3
#define MPU6500_LPF_20HZ     4
#define MPU6500_LPF_10HZ     5
#define MPU6500_LPF_5HZ      6
#define MPU6500_LPF_2HZ      7
#define MPU6500_LPF_2100HZ   8

#define MAX_COMPASS_SAMPLE_RATE 100

#define MPU_X_GYRO        0x40
#define MPU_Y_GYRO        0x20
#define MPU_Z_GYRO        0x10
#define MPU_XYZ_GYRO      MPU_X_GYRO | MPU_Y_GYRO | MPU_Z_GYRO
#define MPU_XYZ_ACCEL     0x08
#define MPU_XYZ_COMPASS   0x01

/* When entering motion interrupt mode, the driver keeps track of the
* previous state so that it can be restored at a later time.
* TODO: This is tacky. Fix it.
*/
struct motion_int_cache_s {
	unsigned short gyro_fsr;
	unsigned char accel_fsr;
	unsigned short lpf;
	unsigned short sample_rate;
	unsigned char sensors_on;
	unsigned char fifo_sensors;
	unsigned char dmp_on;
};

struct mpu9250_st_s{
	uint16_t gyro_fsr;
	uint16_t accel_fsr;
	uint8_t sensors;					/* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
	uint8_t lpf;						/* Matches config register. */
	uint8_t clk_src;
	uint16_t sample_rate;				/* Sample rate, NOT rate divider. */
	uint8_t fifo_enable;				/* Matches fifo_en register. */
	uint8_t int_enable;					/* Matches int enable register. */
	uint8_t bypass_mode;				/* 1 if devices on auxiliary I2C bus appear on the primary. */
	uint8_t accel_half;					/* 1 if half-sensitivity.
										* NOTE: This doesn't belong here, but everything else in hw_s is const,
										* and this allows us to save some precious RAM.
										*/
	uint8_t lp_accel_mode;				/* 1 if device in low-power accel-only mode. */
	uint8_t int_motion_only;			/* 1 if interrupts are only triggered on motion events. */
	struct motion_int_cache_s cache;
	uint8_t active_low_int;				/* 1 for active low interrupts. */
	uint8_t latched_int;				/* 1 for latched interrupts. */
	uint8_t dmp_on;						/* 1 if DMP is enabled. */
	uint8_t dmp_loaded;					/* Ensures that DMP will only be loaded once. */
	uint16_t dmp_sample_rate;			/* Sampling rate used when DMP is enabled. */
	
#ifdef USE_AK8963
	uint16_t compass_sample_rate;		/* Compass sample rate. */
	uint8_t compass_addr;
	int16_t mag_sens_adj[3];
#endif
};

/* Clock sources. */
enum {
	INV_CLK_INTERNAL = 0,
	INV_CLK_PLL,
	NUM_CLK
};

uint8_t mpu_init(void);
uint8_t mpu_set_gyro_fsr(uint16_t fsr);
uint8_t mpu_set_accel_fsr(uint16_t fsr);
uint8_t mpu_set_lpf(uint16_t lpf);
uint8_t mpu_set_sample_rate(uint16_t rate);
uint8_t mpu_set_compass_sample_rate(uint16_t rate);
uint8_t mpu_set_bypass(uint8_t bypass_on);
uint8_t mpu_set_sensors(uint8_t sensors);
uint8_t mpu_set_int_latched(uint8_t enable);
uint8_t mpu_get_accel_reg(int16_t *data);
uint8_t mpu_get_gyro_reg(int16_t *data);
uint8_t mpu_who_am_i(uint8_t *data);

#ifdef USE_AK8963
uint8_t setup_compass(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _STBM_MPU_H */



