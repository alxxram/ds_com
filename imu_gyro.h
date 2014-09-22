/* 
 * File:   IMU_gyro.h
 * Author: skycatch
 *
 * Created on July 17, 2014, 10:47 AM
 */

#ifndef IMU_GYRO_H
#define	IMU_GYRO_H
#include "imu_sensor.h"
#include "drivers/drv_gyro.h"

class IMU_Gyro: public IMU_Sensor {
    
public:
    typedef void (* gyro_cb_t) (gyro_report & report);
    IMU_Gyro(gyro_cb_t cb);
    int init();
    void read_report();

    virtual ~IMU_Gyro();
private:
    IMU_Gyro(const IMU_Gyro& orig);
    gyro_cb_t m_cb;
};

#endif	/* IMU_GYRO_H */

