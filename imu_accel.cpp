/* 
 * File:   IMU_accel.cpp
 * Author: skycatch
 * 
 * Created on July 17, 2014, 1:49 PM
 */

#include "imu_accel.h"
#include "imu_sensor.h"

IMU_Accel::IMU_Accel(accel_cb_t cb) {
    m_cb = cb;
}

int IMU_Accel::init() {
    return IMU_Sensor::init("/dev/accel");
}



IMU_Accel::~IMU_Accel() {
}

void IMU_Accel::read_report() {
    accel_report report;
    IMU_Sensor::read_report(&report, sizeof(report));
    m_cb(report);
}
