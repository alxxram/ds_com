#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_gyro.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>

#include "imu_gyro.h"
#include "imu_mag.h"
#include "imu_sensor.h"
#include "imu_accel.h"

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))
#define DEFAULT_TTYDEV "/dev/ttyS2"  // Telem 2 on pixhawk.

extern "C" __EXPORT int ds_com_main(int argc, char *argv[]);

void print_usage() {
    printf("imu_cap -i <iterations> -d <delay in ms>\n\n");
    printf("imu_cap is a program to read board IMU sensors at a high rate\n");
    printf("Build: %s\n", __TIME__);
}

float max_mag_x = 0;
float min_mag_x = 0;
int16_t max_raw_mag_x = 0;
int16_t min_raw_mag_x = 0;

float max_mag_y = 0;
float min_mag_y = 0;
int16_t max_raw_mag_y = 0;
int16_t min_raw_mag_y = 0;

float max_mag_z = 0;
float min_mag_z = 0;
int16_t max_raw_mag_z = 0;
int16_t min_raw_mag_z = 0;

float max_gyro_x = 0;
float min_gyro_x = 0;
int16_t max_raw_gyro_x = 0;
int16_t min_raw_gyro_x = 0;

float max_gyro_y = 0;
float min_gyro_y = 0;
int16_t max_raw_gyro_y = 0;
int16_t min_raw_gyro_y = 0;

float max_gyro_z = 0;
float min_gyro_z = 0;
int16_t max_raw_gyro_z = 0;
int16_t min_raw_gyro_z = 0;

float max_accel_x = 0;
float min_accel_x = 0;
int16_t max_raw_accel_x = 0;
int16_t min_raw_accel_x = 0;

float max_accel_y = 0;
float min_accel_y = 0;
int16_t max_raw_accel_y = 0;
int16_t min_raw_accel_y = 0;

float max_accel_z = 0;
float min_accel_z = 0;
int16_t max_raw_accel_z = 0;
int16_t min_raw_accel_z = 0;

template<typename T>
bool check_max(T &max_x, T &max_y, T &max_z,
               T   in_x, T   in_y, T   in_z)
{
    max_x = (in_x > max_x) ? in_x : max_x;
    max_y = (in_y > max_y) ? in_y : max_y;
    max_z = (in_z > max_z) ? in_z : max_z;

    if ((in_x >= max_x) || (in_y >= max_y) || (in_z >= max_z)) {
        return true;
    } else {
        return false;
    }
}

template<typename T>
bool check_min(T &min_x, T &min_y, T &min_z,
               T   in_x, T   in_y, T   in_z)
{
    min_x = (in_x < min_x) ? in_x : min_x;
    min_y = (in_y < min_y) ? in_y : min_y;
    min_z = (in_z < min_z) ? in_z : min_z;

    if ((in_x <= min_x) || (in_y <= min_y) || (in_z <= min_z)) {
        return true;
    } else {
        return false;
    }
}

int ds_com_main(int argc, char * argv[]) {

    uint8_t status;
    int iterations = 10;
    int delay_duration = 0;
    int c;

    union data_to_send {
        unsigned char byte[6];
        uint16_t word[3];
    };
    data_to_send data;

    // Start with magnetometer (int16_t) z_raw.  It should vary between 6000 and 400
    auto mag_print = [](mag_report &report) {
        //printf("MagX:%8.4f \t MagY:%8.4f \t MagZ:%8.4f \t", (double) report.x, (double) report.y, (double) report.z);
        //printf("\t MagX %d \t MagY %d \t MagZ %d  \t", report.x_raw, report.y_raw, report.z_raw);
        if (check_max(max_mag_x, max_mag_y, max_mag_z, report.x, report.y, report.z) ||
            check_min(min_mag_x, min_mag_y, min_mag_z, report.x, report.y, report.z)) {
            printf("MagX_max:%8.4f MagX_min:%8.4f  ", (double)max_mag_x, (double)min_mag_x);
            printf("MagY_max:%8.4f MagY_min:%8.4f  ", (double)max_mag_y, (double)min_mag_y);
            printf("MagZ_max:%8.4f MagZ_min:%8.4f\n", (double)max_mag_z, (double)min_mag_z);
        }
        if(check_max(max_raw_mag_x, max_raw_mag_y, max_raw_mag_z, report.x_raw, report.y_raw, report.z_raw) ||
           check_min(min_raw_mag_x, min_raw_mag_y, min_raw_mag_z, report.x_raw, report.y_raw, report.z_raw)) {
            printf("MagX_raw_max:%d MagX_raw_min:%d  ", max_raw_mag_x, min_raw_mag_x);
            printf("MagY_raw_max:%d MagY_raw_min:%d  ", max_raw_mag_y, min_raw_mag_y);
            printf("MagZ_raw_max:%d MagZ_raw_min:%d\n", max_raw_mag_z, min_raw_mag_z);
        }
    };

    auto gyro_print = [](gyro_report &report) {
        //printf("\t GyroX:%8.4f \t GyroY:%8.4f \t GyroZ:%8.4f \t", (double) report.x, (double) report.y, (double) report.z);
        //printf("\t GyroX %d \t GyroY %d \t GyroZ %d  \t", report.x_raw, report.y_raw, report.z_raw);
        if (check_max(max_gyro_x, max_gyro_y, max_gyro_z, report.x, report.y, report.z) ||
            check_min(min_gyro_x, min_gyro_y, min_gyro_z, report.x, report.y, report.z) ||
            check_max(max_raw_gyro_x, max_raw_gyro_y, max_raw_gyro_z, report.x_raw, report.y_raw, report.z_raw) ||
            check_min(min_raw_gyro_x, min_raw_gyro_y, min_raw_gyro_z, report.x_raw, report.y_raw, report.z_raw)) {
            printf("GyroX_max:%8.4f GyroX_min:%8.4f  ", (double)max_gyro_x, (double)min_gyro_x);
            printf("GyroY_max:%8.4f GyroY_min:%8.4f  ", (double)max_gyro_y, (double)min_gyro_y);
            printf("GyroZ_max:%8.4f GyroZ_min:%8.4f\n", (double)max_gyro_z, (double)min_gyro_z);
            printf("GyroX_raw_max:%d GyroX_raw_min:%d  ", max_raw_gyro_x, min_raw_gyro_x);
            printf("GyroY_raw_max:%d GyroY_raw_min:%d  ", max_raw_gyro_y, min_raw_gyro_y);
            printf("GyroZ_raw_max:%d GyroZ_raw_min:%d\n", max_raw_gyro_z, min_raw_gyro_z);
        }
    };

    auto accel_print = [](accel_report & report) {
        //printf("\t AccelX:%8.4f \t AccelY:%8.4f \t AccelZ:%8.4f\n", (double) report.x, (double) report.y, (double) report.z);
        //printf("\t AccelX %d \t AccelY %d \t AccelZ %d \n", report.x_raw, report.y_raw, report.z_raw);
        if (check_max(max_accel_x, max_accel_y, max_accel_z, report.x, report.y, report.z) ||
            check_min(min_accel_x, min_accel_y, min_accel_z, report.x, report.y, report.z) ||
            check_max(max_raw_accel_x, max_raw_accel_y, max_raw_accel_z, report.x_raw, report.y_raw, report.z_raw) ||
            check_min(min_raw_accel_x, min_raw_accel_y, min_raw_accel_z, report.x_raw, report.y_raw, report.z_raw)) {
            printf("AccelX_max:%8.4f AccelX_min:%8.4f  ", (double)max_accel_x, (double)min_accel_x);
            printf("AccelY_max:%8.4f AccelY_min:%8.4f  ", (double)max_accel_y, (double)min_accel_y);
            printf("AccelZ_max:%8.4f AccelZ_min:%8.4f\n", (double)max_accel_z, (double)min_accel_z);
            printf("AccelX_raw_max:%d AccelX_raw_min:%d  ", max_raw_accel_x, min_raw_accel_x);
            printf("AccelY_raw_max:%d AccelY_raw_min:%d  ", max_raw_accel_y, min_raw_accel_y);
            printf("AccelZ_raw_max:%d AccelZ_raw_min:%d\n", max_raw_accel_z, min_raw_accel_z);
        }
    };

    IMU_Mag mag(mag_print);
    status = mag.init();
    if(status != OK) {
        printf("mag status %d\n", status);
        return 1;
    }

    IMU_Gyro gyro(gyro_print);
    status = gyro.init();
    if(status != OK) {
        printf("gyro status: %d\n", status);
        return 1;
    }

    IMU_Accel accel(accel_print);
    status = accel.init();
    if(status != OK) {
        printf("accel status %d\n", status);
        return 1;
    }

    printf ("Hello Skycatch!\n");
    
    int fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf ("Failed to open fd\n");
        return (-1);
    }

    // Configure the serial port
	struct termios uart_config;
	tcgetattr(fd, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	if (cfsetispeed(&uart_config, B57600) < 0 || cfsetospeed(&uart_config, B57600) < 0) {
		printf("ERROR setting baud\n");
		return 1;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		printf("ERROR setting attr\n");
		return 1;
    }

    int res;
    data.byte[0] = 65;
    while (1) {
        //mag.read_report();
        //gyro.read_report();
        accel.read_report();

        /*
        res = write(fd, &data.byte[0], 1);
        if(res < 0) {
            printf("ERROR writing\n");
            return 1;
        } else if (res == 0) {
            continue;
        }
        
        data.byte[0]++;
        if (data.byte[0] == 91) {
            data.byte[0] = 65;
        } 
        */

        usleep(500);
    }
    
    close(fd);
    return 0;
}

