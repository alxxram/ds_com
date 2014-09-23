
#include "imu_mag.h"
#include <stdio.h>

IMU_Mag::IMU_Mag(mag_cb_t cb) {
    printf("Loading %s\n", __FUNCTION__);
    m_cb = cb;
}

int IMU_Mag::init() {
    return IMU_Sensor::init("/dev/mag");
}

void IMU_Mag::read_report() {
    mag_report report;
    IMU_Sensor::read_report(&report, sizeof (report));

    m_x_raw = report.x_raw;
    m_y_raw = report.y_raw;
    m_z_raw = report.z_raw;

    m_cb(report);
}



IMU_Mag::~IMU_Mag() {

}
