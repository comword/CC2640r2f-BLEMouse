#include "gyro.h"
#include "stm32f4xx_hal.h"
#include "Message.h"
#include "ErrorHelper.h"
#include "timer.h"
#include "LibAlgo.h"

extern SPI_HandleTypeDef hspi2;
//extern UART_HandleTypeDef huart6;

static int32_t cfg_acc_fsr = 4000; /* +/- 4g */
static int32_t cfg_gyr_fsr = 2000; /* +/- 2000dps */

/* Sensitivity configurations */
#define ACC_SENSITIVITY (int32_t) ( ((cfg_acc_fsr/1000) * (1 << 16)) / INT16_MAX)
#define GYR_SENSITIVITY (int32_t) ( (cfg_gyr_fsr * (1 << 16) / INT16_MAX) )

#define DATA_ACCURACY_MASK  ((uint32_t)0x7)

static const uint8_t EXPECTED_WHOAMI[] = { 0x95, 0x98, 0x02, 0x03 };
/* WHOAMI value for ICM207XX or derivative */ //ICM20689 is 0x98,ICM20789 is 0x95 JPL

static inv_icm207xx_t icm_device;

/*
 * Flag set from icm207xx device irq handler
 */
static volatile int irq_from_device;

/*
 * Variable to keep track of the expected period common for all sensors
 */
static uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;

/*
 * Variable to drop the first timestamp(s) after a sensor start catched by the interrupt line.
 * This is needed to be inline with the driver. The first data polled from the FIFO is always discard.
 */
static uint8_t timestamp_to_drop = 0;

/*
 * Variable keeping track of chip information
 */
static uint8_t chip_info[3];

/*
 * Data Structures
 */
static int32_t sRacc_data[3];
static int32_t sRgyro_data[3];
static int16_t sRtemp_data;

static struct {
	int32_t acc_cal_q16[3];
	int32_t acc_bias_q16[3];
	uint8_t accuracy_flag;
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_ACGO_CONFIG_SIZE];
		float   flt; /* ensure correct memory alignment of the buffer */
	} C_buffer;
} sCalAcc;

static struct {
	int32_t gyro_cal_2000dps_q30[3];
	int32_t gyr_cal_q16[3];
	int32_t gyr_uncal_q16[3];
	int32_t gyr_bias_q16[3];
	uint8_t accuracy_flag;
	union {
		uint8_t buf[ALGO_INVN_CALIBRATION_GYR_CAL_FXP_SIZE];
		float   flt; /* ensure correct memory alignment of the buffer */
	} C_buffer;
} sCalGyr;

static struct {
	int32_t grv_quat_q30[4];
	int32_t gravity_q16[3];
	int32_t linearacc_q16[3];
	union {
		uint8_t buf[ALGO_INVN_ORIENTATION_CONFIG_SIZE];
		float flt; /* ensure proper alignement */
	} C_buffer;
} sGRV;

/*
 * Mounting matrix configuration applied for both Accel and Gyro
 * The coefficient values are coded in integer q30
 */
static int32_t cfg_mounting_matrix[9]= { 1.f*(1<<30), 0,           0,
                                         0,           1.f*(1<<30), 0,
                                         0,           0,           1.f*(1<<30) };

/* Forward declaration */
static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static void check_rc(int rc, const char * msg_context);
static void apply_mouting_matrix(const int32_t mounting_matrix[9], const int16_t raw[3], int32_t out[3]);
static void notify_event(uint64_t timestamp);
static void sensor_event(const xSensorEvent * event, void * arg);
static void algorithms_init(void);
static void algorithms_process(void);

/** @brief Read a register through the control interface SPI
* @param[in] spinum, required spi line number
* @param[in] register_addr, register address (location) to access
* @param[in] register_len, length value to read
* @param[in] register_value, pointer on byte value to read
* @retval 0 if correct communication, else wrong communication
*/
static unsigned long spi_master_read_register(SPI_HandleTypeDef * hspi, unsigned char register_addr,
                                          unsigned short register_len, unsigned char *register_value);

/** @brief Write a register through the control interface SPI
* @param[in] spinum, required spi line number
* @param[in] register_addr, register address (location) to access
* @param[in] register_len, length value to write
* @param[in] register_value, pointer on byte value to write
* @retval 0 if correct communication, else wrong communication
*/
static unsigned long spi_master_write_register(SPI_HandleTypeDef * hspi, unsigned char register_addr,
                                           unsigned short register_len, const unsigned char *register_value);

void gyroInit(SPI_HandleTypeDef *spi)
{
	struct inv_icm207xx_serif icm207xx_serif;
	icm207xx_serif.context   = spi; // SPI handle
	icm207xx_serif.read_reg  = idd_io_hal_read_reg;
	icm207xx_serif.write_reg = idd_io_hal_write_reg;
	icm207xx_serif.max_read  = 1024*32; /* maximum number of bytes allowed per serial read */
	icm207xx_serif.max_write = 1024*32; /* maximum number of bytes allowed per serial write */
	icm207xx_serif.is_spi    = 1;
	INV_MSG(INV_MSG_LEVEL_INFO, "Openning serial interface through SPI");

	/*
	 * Reset icm207xx driver states
	 */
	inv_icm207xx_reset_states(&icm_device, &icm207xx_serif);

	/*
	 * Setup the icm207xx device
	 */
	icm207xx_sensor_setup();
    icm207xx_sensor_configuration();
    /*
	 * Initializes the calibration and orientation algorithms
	 */
	algorithms_init();

	/*
	 * Initializes the default sensor ODR in order to properly init the algorithms
	 */
	sensor_configure_odr(period_us);
	timer_configure_callback(ext_interrupt_cb);
//	icm207xx_run_selftest();
//	icm207xx_sensor_setup();
//	icm207xx_sensor_configuration();

}

void doupdategyro()
{
	if(irq_from_device & 1) {
		int rc = 0;
		uint8_t ddry = 0;
		uint8_t int_status;
		int16_t raw_acc[3], raw_gyro[3];

		/*
		 *  Ensure data ready status
		*/
		if((rc = inv_icm207xx_get_int_status(&icm_device, &int_status)) == 0)
			ddry = inv_icm207xx_check_drdy(&icm_device, int_status);

		if(ddry) {
			struct inv_icm207xx_fifo_states fifo_states;

			rc = inv_icm207xx_poll_fifo_data_setup(&icm_device, &fifo_states, int_status);
			check_rc(rc, "Error while polling the icm207xx device");
			if(rc == 1) {
				/*
				 * Overflow detected
				 */
				INV_MSG(INV_MSG_LEVEL_WARNING, "FIFO overflow detected!");
				inv_icm207xx_reset_fifo(&icm_device);
				timer_clear_irq_timestamp();
			}
			else if(fifo_states.packet_count > 0 && fifo_states.packet_size > 0) {
				/*
				 * Read FIFO only when data is expected in FIFO
				 */
				while((rc = inv_icm207xx_poll_fifo_data(&icm_device, &fifo_states, raw_acc, &sRtemp_data, raw_gyro, NULL)) > 0) {

					uint64_t timestamp = timer_get_irq_timestamp();

					/*
					 * Drop the first timestamp(s) caught by the interrupt
					 * because the first data in FIFO is always dropped by
					 * the icm207xx driver. 6-axis fusion needs two
					 * samples to be dropped.
					 */
					while (timestamp_to_drop > 0) {
						timestamp = timer_get_irq_timestamp();
						timestamp_to_drop--;
					}

					/*
					* Apply the mounting matrix configuration to the data polled
					*/
					apply_mouting_matrix(cfg_mounting_matrix, raw_acc, sRacc_data);
					apply_mouting_matrix(cfg_mounting_matrix, raw_gyro, sRgyro_data);

					/*
					 * Compute calibration and orientation algorithms
					 */
					algorithms_process();

					/*
					 * Notify upon new sensor data event
					 */
					notify_event(timestamp);
				}
			}
		}
		irq_from_device &= ~1;
	}
}

int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	SPI_HandleTypeDef *spi = context;
	return spi_master_read_register(spi, reg, rlen, rbuffer);
}

int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	SPI_HandleTypeDef *spi = context;
	return spi_master_write_register(spi, reg, wlen, wbuffer);
}

unsigned long spi_master_read_register(SPI_HandleTypeDef *hspi, unsigned char register_addr,
                                          unsigned short register_len, unsigned char *register_value)
{
	register_addr = READ_BIT_MASK | register_addr;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &register_addr, sizeof(register_addr), SPIx_FLAG_TIMEOUT);
	HAL_SPI_Receive(hspi, register_value, register_len, SPIx_FLAG_TIMEOUT);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return 0;
}

unsigned long spi_master_write_register(SPI_HandleTypeDef *hspi, unsigned char register_addr,
                                           unsigned short register_len, const unsigned char *register_value)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &register_addr, sizeof(register_addr), SPIx_FLAG_TIMEOUT);
	HAL_SPI_Transmit(hspi, register_value, register_len, SPIx_FLAG_TIMEOUT);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return 0;
}

int icm207xx_sensor_setup(void)
{
	int rc;
	uint8_t i, whoami = 0xff;
	/*
	 * Just get the whoami
	 */
	rc = inv_icm207xx_get_whoami(&icm_device, &whoami);
	INV_MSG(INV_MSG_LEVEL_INFO, "ICM20689 WHOAMI=0x%02x", whoami);
	check_rc(rc, "Error reading WHOAMI");
	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value. Got 0x%02x. Expected 0x95, 0x02, 0x03.", whoami);
		check_rc(-1, "");
	}

	rc = inv_icm207xx_get_chip_info(&icm_device, chip_info);
	check_rc(rc, "Could not obtain chip info");

	/*
	 * Configure and initialize the ICM207XX for normal use
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up icm207xx...");

	/* set default power mode */
	if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_GYRO) &&
		!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_ACCEL)) {
		INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting icm207xx in sleep mode...");
		rc = inv_icm207xx_initialize(&icm_device);
		check_rc(rc, "Error %d while setting-up icm207xx device");
	}

	/* set default ODR = 50Hz */
	rc = inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_ACCEL, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm207xx device");

	rc = inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_GYRO, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm207xx device");

	period_us = DEFAULT_ODR_US;

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	return 0;
}

int icm207xx_sensor_configuration(void)
{
	int rc;

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring accelerometer FSR");
	rc = inv_icm207xx_set_accel_fullscale(&icm_device, inv_icm207xx_accel_fsr_2_reg(cfg_acc_fsr));
	check_rc(rc, "Error configuring ACC sensor");

	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring gyroscope FSR");
	rc = inv_icm207xx_set_gyro_fullscale(&icm_device, inv_icm207xx_gyro_fsr_2_reg(cfg_gyr_fsr));
	check_rc(rc, "Error configuring GYR sensor");

	return rc;
}

void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}

int icm207xx_run_selftest(void)
{
	int raw_bias[12];
	int rc = 0;

	if (icm_device.selftest_done == 1) {
		INV_MSG(INV_MSG_LEVEL_INFO, "Self-test has already ran. Skipping.");
	}
	else {
		/*
		 * Perform self-test
		 * For ICM207XX self-test is performed for both RAW_ACC/RAW_GYR
		 */
		INV_MSG(INV_MSG_LEVEL_INFO, "Running self-test...");

		/* Run the self-test */
		rc = inv_icm207xx_run_selftest(&icm_device);
		/* Check transport errors */
		check_rc(rc, "Self-test failure");
		if (rc != 0x3) {
			/*
			 * Check for GYR success (1 << 0) and ACC success (1 << 1),
			 * but don't block as these are 'usage' failures.
			 */
			INV_MSG(INV_MSG_LEVEL_ERROR, "Self-test failure");
			/* 0 would be considered OK, we want KO */
			return INV_ERROR;
		} else
			/* On success, offset will be kept until reset */
			icm_device.selftest_done = 1;

		/* It's advised to re-init the icm207xx device after self-test for normal use */
		rc = icm207xx_sensor_setup();
	}

	/*
	 * Get Low Noise / Low Power bias computed by self-tests scaled by 2^16
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Getting LP/LN bias");
	inv_icm207xx_get_st_bias(&icm_device, raw_bias);
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (FS=250dps) (dps): x=%f, y=%f, z=%f",
			(float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LP bias (FS=250dps) (dps): x=%f, y=%f, z=%f",
			(float)(raw_bias[3] / (float)(1 << 16)), (float)(raw_bias[4] / (float)(1 << 16)), (float)(raw_bias[5] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (FS=2g) (g): x=%f, y=%f, z=%f",
			(float)(raw_bias[0 + 6] / (float)(1 << 16)), (float)(raw_bias[1 + 6] / (float)(1 << 16)), (float)(raw_bias[2 + 6] / (float)(1 << 16)));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LP bias (FS=2g) (g): x=%f, y=%f, z=%f",
			(float)(raw_bias[3 + 6] / (float)(1 << 16)), (float)(raw_bias[4 + 6] / (float)(1 << 16)), (float)(raw_bias[5 + 6] / (float)(1 << 16)));

	return rc;
}

static void apply_mouting_matrix(const int32_t mounting_matrix[9], const int16_t raw[3], int32_t out[3])
{
	unsigned i;

	for(i = 0; i < 3; i++) {
		out[i]  = (int32_t)((int64_t)mounting_matrix[3*i+0]*raw[0] >> 30);
		out[i] += (int32_t)((int64_t)mounting_matrix[3*i+1]*raw[1] >> 30);
		out[i] += (int32_t)((int64_t)mounting_matrix[3*i+2]*raw[2] >> 30);
	}
}

int sensor_control(int enable)
{
	int rc = 0;
	static uint8_t sensors_on = 0;

	/* Keep track of the sensors state */
	if(enable && sensors_on)
		return rc;

	if(enable)
		sensors_on = 1;
	else
		sensors_on = 0;

	/* Handling of Game Rotation Vector (6-axis AG) */
//	if (enable) {
//		/* Handles the orientation algoritm state */
//		Algo_InvnOrientation_BodyToWorldFrameFxp_ResetStates(sGRV.C_buffer.buf);
//		Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Enable(sGRV.C_buffer.buf);
//	} else
//		Algo_InvnOrientation_BodyToWorldFrameFxp_AG_Disable(sGRV.C_buffer.buf);

	/*
	 *  Call driver APIs to start/stop sensors
	 */
	if (enable) {
		/* Clock is more accurate when gyro is enabled, so let's enable it first to prevent side effect at startup */
		if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_GYRO))
			rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_GYRO, 1);
		if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_ACCEL))
			rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_ACCEL, 1);
		if (!inv_icm207xx_is_sensor_enabled(&icm_device, INV_ICM207XX_SENSOR_TEMPERATURE))
			rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_TEMPERATURE, 1);
		/*
		 * There is a situation where two samples need to be dropped: if
		 * accelerometer is enable before gyroscope first interrupt triggers,
		 * both interrupts are raised causing the odr to be wrong if only one
		 * sample is dropped.
		 * We are in this exact situation since both sensors are enabled one after
		 * the other.
		 */
		timestamp_to_drop = 2;
	} else {
		rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_GYRO, 0);
		rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_ACCEL, 0);
		rc += inv_icm207xx_enable_sensor(&icm_device, INV_ICM207XX_SENSOR_TEMPERATURE, 0);
	}

	/* Clear the remaining items in the IRQ timestamp buffer when stopping all sensors */
	if(inv_icm207xx_all_sensors_off(&icm_device))
		rc += timer_clear_irq_timestamp();

	return rc;
}

void notify_event(uint64_t timestamp)
{
	xSensorEvent event;
	memset(&event, 0, sizeof(event));

//	/*
//	 * New raw accel data
//	 */
//	event.sensor	= INV_SENSOR_TYPE_RAW_ACCELEROMETER;
//	event.timestamp = timestamp;
//	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
//	event.data.raw3d.vect[0] = sRacc_data[0];
//	event.data.raw3d.vect[1] = sRacc_data[1];
//	event.data.raw3d.vect[2] = sRacc_data[2];
//
//	sensor_event(&event, NULL);
//
//	/*
//	 * New calibrated accel event
//	 */
//	event.sensor	= INV_SENSOR_TYPE_ACCELEROMETER;
//	event.timestamp = timestamp;
//	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
//	event.data.acc.bias[0] = (float) sCalAcc.acc_bias_q16[0] / (1 << 16);
//	event.data.acc.bias[1] = (float) sCalAcc.acc_bias_q16[1] / (1 << 16);
//	event.data.acc.bias[2] = (float) sCalAcc.acc_bias_q16[2] / (1 << 16);
//	event.data.acc.vect[0] = (float) sCalAcc.acc_cal_q16[0] / (1 << 16);
//	event.data.acc.vect[1] = (float) sCalAcc.acc_cal_q16[1] / (1 << 16);
//	event.data.acc.vect[2] = (float) sCalAcc.acc_cal_q16[2] / (1 << 16);
//	event.data.acc.accuracy_flag = sCalAcc.accuracy_flag & DATA_ACCURACY_MASK;
//
//	sensor_event(&event, NULL);
//
//	/*
//	 * New raw gyro data
//	 */
//	event.sensor	= INV_SENSOR_TYPE_RAW_GYROSCOPE;
//	event.timestamp = timestamp;
//	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
//	event.data.raw3d.vect[0] = sRgyro_data[0];
//	event.data.raw3d.vect[1] = sRgyro_data[1];
//	event.data.raw3d.vect[2] = sRgyro_data[2];
//
//	sensor_event(&event, NULL);
//
//	/*
//	 * New uncalibrated gyro event
//	 */
//	event.sensor	= INV_SENSOR_TYPE_UNCAL_GYROSCOPE;
//	event.timestamp = timestamp;
//	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
//	event.data.gyr.bias[0] = (float) sCalGyr.gyr_bias_q16[0] / (1 << 16);
//	event.data.gyr.bias[1] = (float) sCalGyr.gyr_bias_q16[1] / (1 << 16);
//	event.data.gyr.bias[2] = (float) sCalGyr.gyr_bias_q16[2] / (1 << 16);
//	event.data.gyr.vect[0] = (float) sCalGyr.gyr_uncal_q16[0] / (1 << 16);
//	event.data.gyr.vect[1] = (float) sCalGyr.gyr_uncal_q16[1] / (1 << 16);
//	event.data.gyr.vect[2] = (float) sCalGyr.gyr_uncal_q16[2] / (1 << 16);
//	event.data.gyr.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;
//
//	sensor_event(&event, NULL);
//
//	/*
//	 * New calibrated gyro event
//	 */
//	event.sensor	= INV_SENSOR_TYPE_GYROSCOPE;
//	event.timestamp = timestamp;
//	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
//	event.data.gyr.bias[0] = (float) sCalGyr.gyr_bias_q16[0] / (1 << 16);
//	event.data.gyr.bias[1] = (float) sCalGyr.gyr_bias_q16[1] / (1 << 16);
//	event.data.gyr.bias[2] = (float) sCalGyr.gyr_bias_q16[2] / (1 << 16);
//	event.data.gyr.vect[0] = (float) sCalGyr.gyr_cal_q16[0] / (1 << 16);
//	event.data.gyr.vect[1] = (float) sCalGyr.gyr_cal_q16[1] / (1 << 16);
//	event.data.gyr.vect[2] = (float) sCalGyr.gyr_cal_q16[2] / (1 << 16);
//	event.data.gyr.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;
//
//	sensor_event(&event, NULL);
//
	/*
	 * New GRV event
	 * scheduled on gyroscope data update
	 */
	event.sensor	= INV_SENSOR_TYPE_GAME_ROTATION_VECTOR;
	//event.timestamp = timestamp;
	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
	event.data.quaternion.quat[0] = (float)sGRV.grv_quat_q30[0] / (1 << 30);
	event.data.quaternion.quat[1] = (float)sGRV.grv_quat_q30[1] / (1 << 30);
	event.data.quaternion.quat[2] = (float)sGRV.grv_quat_q30[2] / (1 << 30);
	event.data.quaternion.quat[3] = (float)sGRV.grv_quat_q30[3] / (1 << 30);
	/* Report additionnal accuracy flag, being currently a copy of GYR accuracy flag (ACC flag could also be considered) */
	event.data.quaternion.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;

	sensor_event(&event, NULL);
//
//	/*
//	 * New gravity event
//	 */
//	event.sensor	= INV_SENSOR_TYPE_GRAVITY;
//	event.timestamp = timestamp;
//	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
//	event.data.acc.vect[0] = (float)sGRV.gravity_q16[0] / (1 << 16);
//	event.data.acc.vect[1] = (float)sGRV.gravity_q16[1] / (1 << 16);
//	event.data.acc.vect[2] = (float)sGRV.gravity_q16[2] / (1 << 16);
//	/* Report additionnal accuracy flag, being currently the copy of GRV accuracy flag (copied from GYR accuracy flag) */
//	event.data.acc.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;
//
//	sensor_event(&event, NULL);

	/*
	 * New linear acceleration event
	 */
//	event.sensor	= INV_SENSOR_TYPE_LINEAR_ACCELERATION;
//	event.timestamp = timestamp;
//	event.status	= INV_SENSOR_STATUS_DATA_UPDATED;
//	event.data.acc.vect[0] = (float)sGRV.linearacc_q16[0] / (1 << 16);
//	event.data.acc.vect[1] = (float)sGRV.linearacc_q16[1] / (1 << 16);
//	event.data.acc.vect[2] = (float)sGRV.linearacc_q16[2] / (1 << 16);
//	/* Report additionnal accuracy flag, being currently the copy of GRV accuracy flag (copied from GYR accuracy flag) */
//	event.data.acc.accuracy_flag = sCalGyr.accuracy_flag & DATA_ACCURACY_MASK;
//
//	sensor_event(&event, NULL);
}

void sensor_event(const xSensorEvent * event, void * arg)
{
	/* arg will contained the value provided at init time */
	(void)arg;
//	char out_str[256];
//	unsigned char * ptr = (unsigned char *)out_str;
//	unsigned char idx = 0;
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%u|", event->sensor);
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%lu|", (uint32_t)event->timestamp);
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%d|", event->status);
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%f|", event->data.acc.vect[0]);
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%f|", event->data.acc.vect[1]);
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%f|", event->data.acc.vect[2]);
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
//	HAL_UART_Transmit(&huart6, (uint8_t *)&ptr, idx+1, 1000);

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi2, (uint8_t *)event, sizeof(xSensorEvent), 20);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/*
 * Callback called upon external interrupt line rising
 */
void ext_interrupt_cb(int int_num)
{
	irq_from_device |= int_num;
}

int sensor_configure_odr(int odr_us)
{
	int rc = 0;

	/* All sensors running at the same rate */

	/* Do not reconfigure the rate if it's already applied */
	if(odr_us == period_us)
		return rc;

	/*
	 * Maximum supported rate is 1kHz
	 */
	if(odr_us < MIN_ODR_US)
		odr_us = MIN_ODR_US;

	/*
	 * Minimum rate supported is 50Hz
	 * - To compute a correct orientation with the GRV algorithm
	 * - For accelerometer sensor using gyro-assisted calibration.
	 * The basic accelerometer and gyroscope sensor could report at lower frequency, but we simplify the example with this condition.
	 */
	if(odr_us > MAX_ODR_US)
		odr_us = MAX_ODR_US;
	/*
	 *  Call driver APIs to start/stop sensors
	 */
	rc = inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_ACCEL, odr_us / 1000);
	rc += inv_icm207xx_set_sensor_period(&icm_device, INV_ICM207XX_SENSOR_GYRO, odr_us / 1000);
	/* FIFO has been reset by ODR change */
	if (rc == 0) {
		rc += timer_clear_irq_timestamp();
		/* Clear any remaining interrupts */
		//__disable_irq();
		irq_from_device &= ~1;
		//__enable_irq();
	}

	/* Keep track in static variable of the odr value for further algorihtm use */
	period_us = odr_us;

	/*
	 * Update algorithm parameters for Gyroscope calibration
	 */
	Algo_InvnCalibration_GyroCalibrationFxp_SetSamplingPeriod(sCalGyr.C_buffer.buf, odr_us);

	/*
	 * Update algorithm parameters for Accelerometer calibration
	 */
	Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_SetSamplingPeriod(sCalAcc.C_buffer.buf, odr_us);

	/*
	 * Update algorithm parameters for GRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetGyrSamplingPeriod(sGRV.C_buffer.buf, odr_us, chip_info);
	Algo_InvnOrientation_BodyToWorldFrameFxp_SetAccSamplingPeriod(sGRV.C_buffer.buf, odr_us, chip_info);

	return rc;
}

void algorithms_init(void)
{
	int32_t gyro_offset_2000dps_q30[3] = {
		((sCalGyr.gyr_bias_q16[0] << 3) / 2000) << (30 - 16 - 3),
		((sCalGyr.gyr_bias_q16[1] << 3) / 2000) << (30 - 16 - 3),
		((sCalGyr.gyr_bias_q16[2] << 3) / 2000) << (30 - 16 - 3)
	};

	/* Reset the algorithm and the accuracy to 0 and re-apply known offsets (could be stored in flash) */
	Algo_InvnCalibration_GyroCalibrationFxp_Init(sCalGyr.C_buffer.buf, gyro_offset_2000dps_q30, 0);
	Algo_InvnCalibration_GyroCalibrationFxp_SetUserParam(sCalGyr.C_buffer.buf, 0, HMD_VR_MODE);

	/*
	 * Init the accelerometer calibration
	 */
	Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_Init(sCalAcc.C_buffer.buf, sCalAcc.acc_bias_q16, 0, period_us);

	/*
	 * Init the GRV orientation
	 */
	Algo_InvnOrientation_BodyToWorldFrameFxp_Init(sGRV.C_buffer.buf);
}


void algorithms_process(void)
{
	/* Get the temperature value by applying the sensitivity 326.8 LSB/degC and adding 25degC offset */
	const int32_t temp_degC_q16 = (((int32_t)(sRtemp_data << 16)) / 3268 * 10) + (int32_t)(25 << 16);
	const int32_t temp_100degC = (temp_degC_q16 * 100) >> 16;

	/*
	 * Compute the calibrated accelerometer data
	 */
	{
		const int32_t raw_accel_q25[3] = {
			(sRacc_data[0] * ACC_SENSITIVITY) << (25 - 16),
			(sRacc_data[1] * ACC_SENSITIVITY) << (25 - 16),
			(sRacc_data[2] * ACC_SENSITIVITY) << (25 - 16),
		};
		int32_t accel_bias_q25[3];
		int32_t accel_cal_q25[3];

		Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_UpdateAcc(sCalAcc.C_buffer.buf, raw_accel_q25, temp_100degC, accel_bias_q25);
		accel_cal_q25[0] = raw_accel_q25[0] - accel_bias_q25[0];
		accel_cal_q25[1] = raw_accel_q25[1] - accel_bias_q25[1];
		accel_cal_q25[2] = raw_accel_q25[2] - accel_bias_q25[2];

		sCalAcc.acc_bias_q16[0] = accel_bias_q25[0] >> (25 - 16);
		sCalAcc.acc_bias_q16[1] = accel_bias_q25[1] >> (25 - 16);
		sCalAcc.acc_bias_q16[2] = accel_bias_q25[2] >> (25 - 16);
		sCalAcc.acc_cal_q16[0] = accel_cal_q25[0] >> (25 -16);
		sCalAcc.acc_cal_q16[1] = accel_cal_q25[1] >> (25 -16);
		sCalAcc.acc_cal_q16[2] = accel_cal_q25[2] >> (25 -16);
		sCalAcc.accuracy_flag = Algo_InvnCalibration_AccelCalibrationGyroOptionalFxp_GetAccuracy(sCalAcc.C_buffer.buf);

		/* Update GRV algorithm with acc value and acc accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateAcc(sGRV.C_buffer.buf,
				accel_cal_q25, sCalAcc.accuracy_flag);
	}

	/*
	 * Compute the calibrated gyroscope data
	 */
	{
		const int32_t raw_gyr_2000dps_q15[3] = {
			(sRgyro_data[0] * GYR_SENSITIVITY) / (2*2000),
			(sRgyro_data[1] * GYR_SENSITIVITY) / (2*2000),
			(sRgyro_data[2] * GYR_SENSITIVITY) / (2*2000)
		};
		int32_t gyro_uncal_2000dps_q30[3];
		int32_t gyro_offset_2000dps_q30[3];

		Algo_InvnCalibration_GyroCalibrationFxp_UpdateGyr(sCalGyr.C_buffer.buf, raw_gyr_2000dps_q15, temp_100degC);
		Algo_InvnCalibration_GyroCalibrationFxp_GetUncalibrated(sCalGyr.C_buffer.buf, gyro_uncal_2000dps_q30);
		Algo_InvnCalibration_GyroCalibrationFxp_GetBias(sCalGyr.C_buffer.buf, gyro_offset_2000dps_q30);
		Algo_InvnCalibration_GyroCalibrationFxp_GetCalibrated(sCalGyr.C_buffer.buf, sCalGyr.gyro_cal_2000dps_q30);

		sCalGyr.gyr_uncal_q16[0] = ((gyro_uncal_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_uncal_q16[1] = ((gyro_uncal_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_uncal_q16[2] = ((gyro_uncal_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_bias_q16[0] = ((gyro_offset_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_bias_q16[1] = ((gyro_offset_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_bias_q16[2] = ((gyro_offset_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_cal_q16[0] = ((sCalGyr.gyro_cal_2000dps_q30[0] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_cal_q16[1] = ((sCalGyr.gyro_cal_2000dps_q30[1] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.gyr_cal_q16[2] = ((sCalGyr.gyro_cal_2000dps_q30[2] >> 11) * 2000) >> (30 - 11 - 16);
		sCalGyr.accuracy_flag = Algo_InvnCalibration_GyroCalibrationFxp_GetAccuracy(sCalGyr.C_buffer.buf);

		/* Update GRV with gyr value and gyr accuracy */
		Algo_InvnOrientation_BodyToWorldFrameFxp_UpdateGyr(sGRV.C_buffer.buf,
				sCalGyr.gyro_cal_2000dps_q30, sCalGyr.accuracy_flag);
	}

	/*
	 * Compute the game rotation vector data
	 * Note : the orientation may drift until the GRV accuracy flag reaches 3. Once calibrated, the position is kept as initial reference.
	 */
	{
		Algo_InvnOrientation_BodyToWorldFrameFxp_GetGameRotationVector(sGRV.C_buffer.buf, sGRV.grv_quat_q30);
	}

	/*
	 * Compute the gravity data
	 */
	{
		/* x axis */
		sGRV.gravity_q16[0] = (2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[1] * sGRV.grv_quat_q30[3]) >> 30)
				- 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[0] * sGRV.grv_quat_q30[2]) >> 30)) >> (30 - 16);
		/* y axis */
		sGRV.gravity_q16[1] = (2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[2] * sGRV.grv_quat_q30[3]) >> 30)
				+ 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[0] * sGRV.grv_quat_q30[1]) >> 30)) >> (30 - 16);
		/* z axis */
		sGRV.gravity_q16[2] = ((1 << 30) - 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[1] * sGRV.grv_quat_q30[1]) >> 30)
				- 2 * (int32_t)(((int64_t)sGRV.grv_quat_q30[2] * sGRV.grv_quat_q30[2]) >> 30)) >> (30 - 16);
	}

	/*
	 * Compute the linear acceleration data
	 */
	{
		sGRV.linearacc_q16[0] = sCalAcc.acc_cal_q16[0] - sGRV.gravity_q16[0];
		sGRV.linearacc_q16[1] = sCalAcc.acc_cal_q16[1] - sGRV.gravity_q16[1];
		sGRV.linearacc_q16[2] = sCalAcc.acc_cal_q16[2] - sGRV.gravity_q16[2];
	}
}
