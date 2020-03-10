#include "Invn/Devices/Drivers/Icm207xx/Icm207xx.h"
#include "Invn/Devices/SensorTypes.h"
#include "stm32f4xx_hal.h"

#define READ_BIT_MASK                      0x80
#define SPIx_FLAG_TIMEOUT                 ((uint32_t) 900)
#define SPIx_LONG_TIMEOUT                 ((uint32_t) (300 * SPIx_FLAG_TIMEOUT))

/* sensor ODR limit */
#define MIN_ODR_US        1000  // 1kHz
#define MAX_ODR_US       20000  // 50Hz
#define DEFAULT_ODR_US   20000  // 50Hz

#define ALGO_INVN_CALIBRATION_ACGO_CONFIG_SIZE    512
#define ALGO_INVN_CALIBRATION_CCGO_CONFIG_SIZE    512
#define ALGO_INVN_CALIBRATION_GYR_CAL_FXP_SIZE    256
#define ALGO_INVN_CALIBRATION_GYRO_BT_CONFIG_SIZE 560
#define ALGO_INVN_ORIENTATION_CONFIG_SIZE         624
#define ALGO_INVN_PREDICTIVEQUATERNION_CONFIG_SIZE 52
#define ALGO_INVN_GESTURE_CONFIG_SIZE             360
#define ALGO_INVN_AAR_CONFIG_SIZE                1620

/*
 * Sensor identifier for control function
 */
enum sensor {
	SENSOR_RAW_ACC,
	SENSOR_RAW_GYR,
	SENSOR_ACC,
	SENSOR_GYR,
	SENSOR_UGYR,
	SENSOR_GRV,
	SENSOR_GRA,
	SENSOR_LINACC,
	SENSOR_MAX
};

typedef struct {
	uint8_t sensor;
	uint8_t status;
	union {
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} acc;                             /**< 3d accelerometer data in g */
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} gyr;                             /**< 3d gyroscope data in deg/s */
		struct {
			float        quat[4];          /**< w,x,y,z quaternion data */
			float        accuracy;         /**< heading accuracy in deg */
			uint8_t      accuracy_flag;    /**< accuracy flag specific for GRV*/
		} quaternion;                      /**< quaternion data */
		struct {
			int32_t      vect[3];          /**< x,y,z vector data */
			uint32_t     fsr;              /**< full scale range */
		} raw3d;						   /**< 3d raw acc, mag or gyr*/
	} data;
} xSensorEvent;

void gyroInit(SPI_HandleTypeDef *spi);
void doupdategyro();

int icm207xx_sensor_setup(void);
int icm207xx_sensor_configuration(void);
void inv_icm207xx_sleep(int ms);
void inv_icm207xx_sleep_us(int us);
int icm207xx_run_selftest(void);
int sensor_control(int enable);
int sensor_configure_odr(int odr_us);
void ext_interrupt_cb(int int_num);
