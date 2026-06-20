#ifndef INC_IMU_INTERFACE_H_
#define INC_IMU_INTERFACE_H_

#include <stdint.h>

/* Generic 3-axis float vector */
typedef struct {
    float x;
    float y;
    float z;
} IMU_Vec3;

/* Unified data packet written into by HMC5883L and BMP180 drivers */
typedef struct {
    /* HMC5883L */
    IMU_Vec3  mag;               /* uT        */
    float     heading;           /* degrees 0-360 */

    /* BMP180 */
    float     pressure;          /* hPa       */
    float     altitude;          /* metres    */
    float     baro_temperature;  /* Celsius   */
} IMU_Data;

#endif /* INC_IMU_INTERFACE_H_ */
