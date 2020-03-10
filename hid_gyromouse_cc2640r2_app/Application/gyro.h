/*
 * gyro.h
 *      Author: henorvell
 */

#ifndef APPLICATION_GYRO_H_
#define APPLICATION_GYRO_H_
#include <stdint.h>

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
        } raw3d;                           /**< 3d raw acc, mag or gyr*/
    } data;
} xSensorEvent;

typedef void (*GyroCB_t)(xSensorEvent gyroevent);

void Gyro_init();
void poll_SPI();

#endif /* APPLICATION_GYRO_H_ */
