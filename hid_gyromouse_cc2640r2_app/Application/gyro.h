/*
 * gyro.h
 *      Author: henorvell
 */

#ifndef APPLICATION_GYRO_H_
#define APPLICATION_GYRO_H_

#include "Invn/Devices/Drivers/Icm207xx/Icm207xx.h"
#include "Invn/Devices/SensorTypes.h"

#define ALGO_INVN_CALIBRATION_ACGO_CONFIG_SIZE    512
#define ALGO_INVN_CALIBRATION_GYR_CAL_FXP_SIZE    256
#define ALGO_INVN_ORIENTATION_CONFIG_SIZE         624

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

#endif /* APPLICATION_GYRO_H_ */
