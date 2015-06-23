#include "stbm_mpu.h"
#include "stbm_mpu9250.h"
#include "stbm_delay.h"
#include <math.h>


static struct mpu9250_st_s mpu9250_st;


uint8_t mpu_init(){
	uint8_t data[6];

	/* Reset device. */
	data[0] = BIT_RESET;
	if (mpu9250_writes(MPU6500_PWR_MGNT_1, 1, data))
		return -1;
	delay_ms(100);

	/* Wake up chip. */
	data[0] = 0x00;
	if (mpu9250_writes(MPU6500_PWR_MGNT_1, 1, data))
		return -1;

	/* user control */
	data[0] = MPU9250_I2C_IF_DIS | MPU9250_I2C_MST_EN;
	if (mpu9250_writes(MPU6500_USER_CTRL, 1, data))
		return -1;
	
	mpu9250_st.accel_half = 0;
	
	
	/* MPU6500 shares 4kB of memory between the DMP and the FIFO. Since the
	* first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
	*/
	data[0] = BIT_FIFO_SIZE_1024 | 0x8;
	if (mpu9250_writes(MPU6500_ACCEL_CFG2, 1, data))
		return -1;
	
	
	/* Set to invalid values to ensure no I2C writes are skipped. */
	//mpu9250_st.sensors = 0xFF;
	//mpu9250_st.gyro_fsr = 0xFF;
	//mpu9250_st.accel_fsr = 0xFF;
	//mpu9250_st.lpf = 0xFF;
	//mpu9250_st.sample_rate = 0xFFFF;
	//mpu9250_st.fifo_enable = 0xFF;
	//mpu9250_st.bypass_mode = 0xFF;
	//mpu9250_st.compass_sample_rate = 0xFFFF;
	/* mpu_set_sensors always preserves this setting. */
	mpu9250_st.clk_src = INV_CLK_PLL;
	/* Handled in next call to mpu_set_bypass. */
	mpu9250_st.active_low_int = 1;
	mpu9250_st.latched_int = 0;
	mpu9250_st.int_motion_only = 0;
	mpu9250_st.lp_accel_mode = 0;
	//とりあえずマスク
	//memset(&mpu9250_st.cache, 0, sizeof(mpu9250_st.cache));
	mpu9250_st.dmp_on = 0;
	mpu9250_st.dmp_loaded = 0;
	mpu9250_st.dmp_sample_rate = 0;
	
	// ジャイロフルスケール設定
	if (mpu_set_gyro_fsr(2000))
		return -1;
	
	// 加速度フルスケール設定
	if (mpu_set_accel_fsr(2))
		return -1;
	
	// LPF設定
	if (mpu_set_lpf(42))
		return -1;
	
	// サンプルレート設定
	if (mpu_set_sample_rate(100))
		return -1;
	
	// ひとまずマスク
	/*
	if (mpu_configure_fifo(0))
		return -1;
	
	if (int_param)
		reg_int_cb(int_param);
	*/

#ifdef USE_AK8963
	if (setup_compass())
		return -1;
	
	if (mpu_set_compass_sample_rate(100))
		return -1;
#else
	/* Already disabled by setup_compass. */
	if (mpu_set_bypass(0))
		return -1;
#endif
	/*
	mpu_set_sensors(0);
	*/
	return 0;
}

uint8_t mpu_set_gyro_fsr(uint16_t fsr){
	uint8_t data;
	
	// センサー使用しない場合は終了
	if (!(mpu9250_st.sensors))
		return -1;
	
	// 保存値と同じならスキップ
	if (mpu9250_st.gyro_fsr == fsr)
		return 0;
	
	// レジスタ書き込み値に変換(3,4bitなので3bitシフト)
	switch (fsr) {
		case 250:
			data = MPU6500_GYRO_FSR_250DPS << 3;
			break;
		case 500:
			data = MPU6500_GYRO_FSR_500DPS << 3;
			break;
		case 1000:
			data = MPU6500_GYRO_FSR_1000DPS << 3;
			break;
		case 2000:
			data = MPU6500_GYRO_FSR_2000DPS << 3;
			break;
		default:
			return -1;
	}
	
	// レジスタ書き込み
	if (mpu9250_writes(MPU6500_GYRO_CFG, 1, &data))
		return -1;
	
	// 値の保存
	mpu9250_st.gyro_fsr = fsr;
	
	return 0;
}

uint8_t mpu_set_accel_fsr(uint16_t fsr){
	uint8_t data;
	
	// センサー使用しない場合は終了
	if (!(mpu9250_st.sensors))
		return -1;
	
	// 保存値と同じならスキップ
	if (mpu9250_st.accel_fsr == fsr)
		return 0;
	
	// レジスタ書き込み値に変換(3,4bitなので3bitシフト)
	switch (fsr) {
		case 2:
			data = MPU6500_ACCEL_FSR_2G << 3;
			break;
		case 4:
			data = MPU6500_ACCEL_FSR_4G << 3;
			break;
		case 8:
			data = MPU6500_ACCEL_FSR_8G << 3;
			break;
		case 16:
			data = MPU6500_ACCEL_FSR_16G << 3;
			break;
		default:
			return -1;
	}
	
	// レジスタ書き込み
	if (mpu9250_writes(MPU6500_ACCEL_CFG, 1, &data))
		return -1;
	
	// 値の保存
	mpu9250_st.accel_fsr = fsr;
	
	return 0;
}

uint8_t mpu_set_lpf(uint16_t lpf){
	uint8_t data;
	
	// センサー使用しない場合は終了
	if (!(mpu9250_st.sensors))
		return -1;
	
	// 保存値と同じならスキップ
	if (mpu9250_st.lpf == lpf)
		return 0;
	
	// レジスタ書き込み値に変換
	if (lpf >= 188)
		data = MPU6500_LPF_188HZ;
	else if (lpf >= 98)
		data = MPU6500_LPF_98HZ;
	else if (lpf >= 42)
		data = MPU6500_LPF_42HZ;
	else if (lpf >= 20)
		data = MPU6500_LPF_20HZ;
	else if (lpf >= 10)
		data = MPU6500_LPF_10HZ;
	else
		data = MPU6500_LPF_5HZ;
	
	// レジスタ書き込み
	if (mpu9250_writes(MPU6500_LPF, 1, &data))
		return -1;
	
	// 値の保存
	mpu9250_st.lpf = lpf;
	
	return 0;
}

uint8_t mpu_set_sample_rate(uint16_t rate){
	uint8_t data;
	
	// センサー使用しない場合は終了
	if (!(mpu9250_st.sensors))
		return -1;
	
	if (mpu9250_st.dmp_on){
		return -1;
	} else {
		// ローパワーモード？とりあえずマスク
		#if 0
		if (mpu9250_st.lp_accel_mode) {
			if (rate && (rate <= 40)) {
				/* Just stay in low-power accel mode. */
				mpu_lp_accel_mode(rate);
				return 0;
			}
			/* Requested rate exceeds the allowed frequencies in LP accel mode,
			* switch back to full-power mode.
			*/
			mpu_lp_accel_mode(0);
		}
		#endif
		
		
		// 上下限カット
		if (rate < 4)
			rate = 4;
		else if (rate > 1000)
			rate = 1000;
		
		// レジスタ書き込み値に変換
		data = 1000 / rate - 1;
		
		// レジスタ書き込み
		if (mpu9250_writes(MPU6500_RATE_DIV, 1, &data))
			return -1;
		
		// 値の保存
		mpu9250_st.sample_rate = 1000 / (1 + data);
		
	#ifdef USE_AK8963
		mpu_set_compass_sample_rate(min(mpu9250_st.compass_sample_rate, MAX_COMPASS_SAMPLE_RATE));
	#endif
		
		/* Automatically set LPF to 1/2 sampling rate. */
		mpu_set_lpf(mpu9250_st.sample_rate >> 1);
		return 0;
	}
}

uint8_t mpu_set_compass_sample_rate(uint16_t rate){
#ifdef USE_AK8963
	uint8_t div;
	
	// エラー処理
	if (!rate || rate > mpu9250_st.sample_rate || rate > MAX_COMPASS_SAMPLE_RATE)
		return -1;
	
	// レジスタ書き込み値変換
	div = mpu9250_st.sample_rate / rate - 1;
	
	// レジスタ書き込み
	if (mpu9250_writes(AK8963_S4_CTRL, 1, &div))
		return -1;
	
	// 値の保存
	mpu9250_st.compass_sample_rate = mpu9250_st.sample_rate / (div + 1);
	
	return 0;
#else
	return -1;
#endif
}

#if 0
uint8_t mpu_configure_fifo(uint8_t sensors)
{
	uint8_t prev;
	uint8_t result = 0;

	/* Compass data isn't going into the FIFO. Stop trying. */
	sensors &= ~AK8963_XYZ_COMPASS;

	if (mpu9250_st.dmp_on)
		return 0;
	else {
		if (!(mpu9250_st.sensors))
			return -1;
		prev = mpu9250_st.fifo_enable;
		mpu9250_st.fifo_enable = sensors & mpu9250_st.sensors;
		if (mpu9250_st.fifo_enable != sensors)
			/* You're not getting what you asked for. Some sensors are
			* asleep.
			*/
			result = -1;
		else
			result = 0;
		if (sensors || mpu9250_st.lp_accel_mode)
			set_int_enable(1);
		else
			set_int_enable(0);
		if (sensors) {
			if (mpu_reset_fifo()) {
				mpu9250_st.fifo_enable = prev;
				return -1;
			}
		}
	}

	return result;
}
#endif

#ifdef USE_AK8963
/* This initialization is similar to the one in ak8975.c. */
static uint8_t setup_compass(void){
	uint8_t data[4], akm_addr;
	int result;
	
	mpu_set_bypass(0);
	/* Find compass. Possible addresses range from 0x0C to 0x0F. */
	for (akm_addr = 0x0C; akm_addr <= 0x0F; akm_addr++) {
		//result = i2c_read(akm_addr, AKM_REG_WHOAMI, 1, data);
		result = mpu9250_ak8963_reads(akm_addr, AKM_REG_WHOAMI, 1, data);
		if (!result && (data[0] == AKM_WHOAMI))
			break;
	}
	
	if (akm_addr > 0x0F) {
		/* TODO: Handle this case in all compass-related functions. */
		log_e("Compass not found.\n");
		return -1;
	}
	
	mpu9250_st.compass_addr = akm_addr;
	
	data[0] = AKM_POWER_DOWN;
	if (mpu9250_ak8963_writes(mpu9250_st.compass_addr, AKM_REG_CNTL, 1, data))
		return -1;
	delay_ms(1);
	data[0] = AKM_FUSE_ROM_ACCESS;
	if (mpu9250_ak8963_writes(mpu9250_st.compass_addr, AKM_REG_CNTL, 1, data))
		return -1;
	delay_ms(1);
	
	/* Get sensitivity adjustment data from fuse ROM. */
	if (mpu9250_ak8963_reads(mpu9250_st.compass_addr, AKM_REG_ASAX, 3, data))
		return -1;
	//bug?why plus 128? but must sub 128 from datasheet
	mpu9250_st.mag_sens_adj[0] = (long)data[0] + 128;
	mpu9250_st.mag_sens_adj[1] = (long)data[1] + 128;
	mpu9250_st.mag_sens_adj[2] = (long)data[2] + 128;
	
	data[0] = AKM_POWER_DOWN;
	if (mpu9250_ak8963_writes(mpu9250_st.compass_addr, AKM_REG_CNTL, 1, data))
		return -1;
	delay_ms(1);
	
	/* Set up master mode, master clock, and ES bit. */
	//modified by hetao.su
	//data[0] = 0x40;
	data[0] = 0x4D;
	if (mpu9250_writes(st.reg->i2c_mst, 1, data))
		return -1;
	
	/* Slave 0 reads from AKM data registers. */
	data[0] = BIT_I2C_READ | mpu9250_st.compass_addr;
	if (mpu9250_writes(AK8963_S0_ADDR, 1, data))
		return -1;
	
	/* Compass reads start at this register. */
	data[0] = AKM_REG_ST1;
	if (mpu9250_writes(AK8963_S0_REG, 1, data))
		return -1;
	
	/* Enable slave 0, 8-byte reads. */
	data[0] = BIT_SLAVE_EN | 8;
	if (mpu9250_writes(AK8963_S0_CTRL, 1, data))
		return -1;
	
	/* Slave 1 changes AKM measurement mode. */
	data[0] = mpu9250_st.compass_addr;
	if (mpu9250_writes(AK8963_S1_ADDR, 1, data))
		return -1;
	
	/* AKM measurement mode register. */
	data[0] = AKM_REG_CNTL;
	if (mpu9250_writes(AK8963_S1_REG, 1, data))
		return -1;
	
	/* Enable slave 1, 1-byte writes. */
	data[0] = BIT_SLAVE_EN | 1;
	if (mpu9250_writes(AK8963_S1_CTRL, 1, data))
		return -1;
	
	/* Set slave 1 data. */
	data[0] = AKM_SINGLE_MEASUREMENT;
	if (mpu9250_writes(AK8963_S1_DO, 1, data))
		return -1;
	
	/* Trigger slave 0 and slave 1 actions at each sample. */
	data[0] = 0x03;
	if (mpu9250_writes(AK8963_I2C_DELAY_CTRL, 1, data))
		return -1;
	
	return 0;
}
#endif
