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

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

extern "C" __EXPORT int ds_com_main(int argc, char *argv[]);

void print_usage() {
    printf("imu_cap -i <iterations> -d <delay in ms>\n\n");
    printf("imu_cap is a program to read board IMU sensors at a high rate\n");
    printf("Build: %s\n", __TIME__);
}

int ds_com_main(int argc, char * argv[]) {
/*
    uint8_t status;
    int iterations = 10;
    int delay_duration = 0;

    int c;

    auto mag_print = [](mag_report & report) {
        printf("\t MAG values: x:%8.4f \t y:%8.4f \t z:%8.4f\n", (double) report.x, (double) report.y, (double) report.z);
        printf("\t MAG values: x_raw:%d \t y_raw:%d \t z_raw:%d\n\n", report.x_raw, report.y_raw, report.z_raw);
    };

    auto gyro_print = [](gyro_report & report) {
        printf("\t Gyro values: x:%8.4f \t y:%8.4f \t z:%8.4f\n", (double) report.x, (double) report.y, (double) report.z);
        printf("\t Gyro values: x_raw:%8d \t y_raw:%8d \t z_raw:%8d\n\n", report.x_raw, report.y_raw, report.z_raw);
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

    //for (int i = 0; i < 20; i++) {
        mag.read_report();
        gyro.read_report();
    //}
    printf ("Hello Skycatch!\n");
    */

    int fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NDELAY);
    //int fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY);

    if (fd < 0) {
        printf ("Failed to open fd\n");
        return (-1);
    }

	/* Try to set baud rate */
	struct termios uart_config;

	/* Fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	if (cfsetispeed(&uart_config, B57600) < 0 || cfsetospeed(&uart_config, B57600) < 0) {
		printf("ERROR setting baud\n");
		return 1;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		printf("ERROR setting attr\n");
		return 1;
    }

    unsigned char data[1];
    data[0] = 65;
    int res, count = 0;
    while (1) {
        res = write(fd, &data[0], 1);
        if(res < 0) {
            printf("ERROR writing %x\n", data[0]);
            return 1;
        }
        
        //if ((data[0] >= 0x6c) && (data[0] <= 0x6e)) {
        //    printf ("Sending 0x%x\n", data[0]);
        //}
        data[0]++;
        if (data[0] == 91) {
            data[0] = 65;
        } 

        /*
        if (count++ == 1000) {
            data[0] = 0;
            count = 0;
        }
        */
        usleep(5000);
    }
    
    close(fd);

    return 0;
}

