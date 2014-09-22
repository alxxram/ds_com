/* 
 * File:   IMU_gyro.cpp
 * Author: skycatch
 * 
 * Created on July 17, 2014, 10:47 AM
 */

#include "imu_gyro.h"
#include "drivers/drv_gyro.h"

IMU_Gyro::IMU_Gyro(gyro_cb_t cb) {
    m_cb = cb;
}

int IMU_Gyro::init() {
    return IMU_Sensor::init("/dev/gyro");
}

void IMU_Gyro::read_report() {
    gyro_report report;
    IMU_Sensor::read_report(&report, sizeof (report));

    m_cb(report);
}

IMU_Gyro::IMU_Gyro(const IMU_Gyro& orig) {
    
}

IMU_Gyro::~IMU_Gyro() {
}

