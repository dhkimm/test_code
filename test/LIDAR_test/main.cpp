#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>


#include "common/mavlink.h"


int serial_fd = -1;



int main(int argc, char **argv) {
    char *serial_port = "/dev/ttyUSB0";
    int baudrate = 115200;


    serial_fd = erial_port(serial_port, baudrate);
    if (serial_fd < 0) {
        return -1;
    }
}


int serial_port(char *portname, int baudrate) {
    struct termios options;
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1) {
        printf("Error: cannot open serial port %s\n", portname);
        return -1;
    }



    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}


void close_serial_port(int fd) {
    close(fd);
}


void request_lidar_range(void) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;
    mavlink_status_t status;


    mavlink_msg_param_ext_request_read_pack(1, 200, &msg, 0, 0);


    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(serial_fd, buf, len);


    mavlink_canfd_frame_t range;
    int i, count = 0;
    while (1) {
    
        uint8_t byte;
        ssize_t n = read(serial_fd, &byte, 1);
        if (n < 0) {
            continue;
        }
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
 
            if (msg.msgid == MAVLINK_MSG_ID_CAN_FRAME) {
                mavlink_msg_canfd_frame_decode(&msg, &range);
                printf("LIDAR range: %f\n", range);
                break;
            }
        }
      

        if (++count > 100000) {
            printf("Error: no response from LIDAR\n");
            break;
        }
    }
}

