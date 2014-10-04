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
    printf("ds_com -e\t\tTo inject errors\n");
    printf("ds_com -s\t\tTo print sensor data to the screen\n");
    printf("ds_com -i\t\tThe number of iterations to loop through\n\n");
    printf("ds_com sends the alphabet forever on port: %s\n", DEFAULT_TTYDEV);
    printf("ds_com can also print magnetometer, accelerometer, and");
    printf(" gyro sensor data to the screen\n");
    printf("Build: %s\n", __TIME__);
}

int ds_com_main(int argc, char * argv[]) {

    uint8_t status;
    int err_inj = 0;
    int log_sensors = 0;
    int c, iterations = 1000;

    while((c = getopt(argc, argv, "esi:")) != -1) {
        switch (c) {
            case 'e':
                err_inj = 1;
            break;
            case 's':
                log_sensors = 1;
            break;
            case 'i':
                iterations = strtoul(optarg, NULL, 10);
            break;
            default:
                print_usage();
                return 1;
        }
    }

    auto mag_print = [](mag_report &report) {
        printf("MagX:%8.4f \t MagY:%8.4f \t MagZ:%8.4f \t", (double) report.x, (double) report.y, (double) report.z);
        printf("\t MagX %d \t MagY %d \t MagZ %d  \t", report.x_raw, report.y_raw, report.z_raw);
    };

    auto gyro_print = [](gyro_report &report) {
        printf("\t GyroX:%8.4f \t GyroY:%8.4f \t GyroZ:%8.4f \t", (double) report.x, (double) report.y, (double) report.z);
        printf("\t GyroX %d \t GyroY %d \t GyroZ %d  \t", report.x_raw, report.y_raw, report.z_raw);
    };

    auto accel_print = [](accel_report & report) {
        printf("\t AccelX:%8.4f \t AccelY:%8.4f \t AccelZ:%8.4f\n", (double) report.x, (double) report.y, (double) report.z);
        printf("\t AccelX %d \t AccelY %d \t AccelZ %d \n", report.x_raw, report.y_raw, report.z_raw);
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
    
    int fd = open(DEFAULT_TTYDEV, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf ("Failed to open fd\n");
        return (1);
    }

    // Configure the serial port
	struct termios uart_config;
	tcgetattr(fd, &uart_config);

	if (cfsetispeed(&uart_config, B57600) < 0 || cfsetospeed(&uart_config, B57600) < 0) {
		printf("ERROR setting baud\n");
		return 1;
	}

	// Clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

    uart_config.c_cflag |= CS8 | CLOCAL;
    uart_config.c_cflag &= ~(CRTSCTS | PARENB | CSTOPB);

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		printf("ERROR setting attr\n");
		return 1;
    }

    unsigned char data = 'A';
    for (int i = 0; i < iterations; i++) {

        if (log_sensors == 1) {
            mag.read_report();
            gyro.read_report();
            accel.read_report();
        }

        int res = write(fd, &data, 1);
        if(res < 0) {
            printf("ERROR writing\n");
            return 1;
        } else if (res == 0) {
            continue;
        }
        
        data++;
        if (data == 'Z' + 1) {
            data = 'A';
        }

        // If error injection is enabled
        if (err_inj == 1) {
            // Corrupt the "G" every time
            if (data == 'G') {
                data = 0;
                printf("Sending zero\n");
            }
        }

        usleep(5000);
    }
    
    close(fd);
    return 0;
}

