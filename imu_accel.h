/* 
 * File:   IMU_accel.h
 * Author: skycatch
 *
 * Created on July 17, 2014, 1:49 PM
 */

#ifndef IMU_ACCEL_H
#define	IMU_ACCEL_H

#include "imu_sensor.h"
#include "drivers/drv_accel.h"

class IMU_Accel: public IMU_Sensor {
public:
    typedef void (* accel_cb_t) (accel_report & report);
    
    IMU_Accel(accel_cb_t);
    int init();
    
    void read_report();
    
    virtual ~IMU_Accel();
    
private:
    IMU_Accel(const IMU_Accel& orig);
    accel_cb_t m_cb;
};

#endif	/* IMU_ACCEL_H */

