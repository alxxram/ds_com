/* 
 * File:   imu_mag.h
 * Author: skycatch
 *
 * Created on July 16, 2014, 2:32 PM
 */

#ifndef IMU_MAG_H
#define	IMU_MAG_H

#include "imu_sensor.h"
#include "drivers/drv_mag.h" 

class IMU_Mag: public IMU_Sensor {
public:
    typedef void (* mag_cb_t) (mag_report &report);
    
    IMU_Mag(mag_cb_t cb);
    int init();
    void read_report(); 
    virtual ~IMU_Mag();
    uint16_t m_z_raw; 
    
private:
    IMU_Mag(const IMU_Mag& orig);
    mag_cb_t m_cb;
};


#endif	/* IMU_MAG_H */

