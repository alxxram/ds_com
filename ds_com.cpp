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
          printf("MagX:%8.4f \t MagY:%8.4f \t MagZ:%8.4f \t", (double) report.x, (double) report.y, (double) report.z);
        //printf("\t MagX:%d \t MagY:%d \t MagZ:%d  \t", report.x_raw, report.y_raw, report.z_raw);
    };

    auto gyro_print = [](gyro_report &report) {
        printf("\t GyroX:%8.4f \t GyroY:%8.4f \t GyroZ:%8.4f \t", (double) report.x, (double) report.y, (double) report.z);
        //printf("\t GyroX:%d \t GyroY:%d \t GyroZ:%d  \t", report.x_raw, report.y_raw, report.z_raw);
    };

    auto accel_print = [](accel_report & report) {
        printf("\t AccelX:%8.4f \t AccelY:%8.4f \t AccelZ:%8.4f\n", (double) report.x, (double) report.y, (double) report.z);
        //printf("\t AccelX:%d \t AccelY:%d \t AccelZ:%d \n", report.x_raw, report.y_raw, report.z_raw);
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

    /*
    for (int i = 0; i < 5000; i++) {
        mag.read_report();
        gyro.read_report();
        sleep(1);
    }
    */
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


//    data.byte[0] = 65;
//    data.byte[1] = 65;
    int res;
    while (1) {
        mag.read_report();
        gyro.read_report();
        accel.read_report();
        data.word[0] = mag.m_x_raw;
        data.word[1] = mag.m_y_raw;
        data.word[2] = mag.m_z_raw;

        res = write(fd, &data.byte[0], 6);
        if(res < 0) {
            printf("ERROR writing %x\n", data.byte[0]);
            return 1;
        }
        
        /*
        data.byte[0]++;
        data.byte[1]++;
        if (data.byte[0] == 91) {
            data.byte[0] = 65;
            data.byte[1] = 65;
        } 
        */
        usleep(10);
    }
    
    close(fd);
    return 0;
}

