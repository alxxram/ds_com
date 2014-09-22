/* 
 * File:   sensor.h
 * Author: skycatch
 *
 * Created on July 16, 2014, 9:35 AM
 */

#ifndef IMU_SENSOR_H
#define	IMU_SENSOR_H


#include <sys/types.h>

 
class IMU_Sensor {
public:
    IMU_Sensor();
    virtual void read_report() = 0;

    virtual ~IMU_Sensor();
   
protected:
    int init(const char * path);
    void read_report(void * buf, size_t size);
    int m_fd;

private:
    IMU_Sensor(const IMU_Sensor& orig);
};



#endif	/* IMU_SENSOR_H */

