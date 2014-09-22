/* 
 * File:   sensor.cpp
 * Author: skycatch
 * 
 * Created on July 16, 2014, 9:35 AM
 */
#include <unistd.h>
#include <fcntl.h>
#include "imu_sensor.h"
#include "drivers/drv_sensor.h"
#include <stdio.h>
#include <sys/types.h>

IMU_Sensor::IMU_Sensor() {
    
    printf("Loading %s\n", __FUNCTION__);
    m_fd = 0;
}

int IMU_Sensor::init(const char * path) {
    int status = -1;
    m_fd = open(path, O_RDWR);
    
    if(m_fd < 0) {
        printf("Failed to open: %s \n", path);
    } else {
        if (ioctl(m_fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) 
            printf("%s: setting rate failed\n", path);
        else
            status = OK;
    }
    if(status == OK) {
        printf("IMU_Sensor::init (%d) loaded\n", m_fd);
    }
    
    return status;
}

void IMU_Sensor::read_report(void * buf, size_t size) {
    int ret = read(m_fd, buf, size);
    
    if (ret != size) {
       printf("\t Sensor: read fail %d \n", m_fd);
    }
}

IMU_Sensor::IMU_Sensor(const IMU_Sensor& orig) {
}

IMU_Sensor::~IMU_Sensor() {
    if(m_fd > 0) {
        //restore sensor poll rate
        if (ioctl(m_fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) 
            printf("mag setting rate failed\n");
        
        close(m_fd);
    }
}

