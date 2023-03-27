#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>


#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B57600

int serial_port;

void setup_serial_port() {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    serial_port = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port < 0) {
        fprintf(stderr, "Error: Unable to open serial port.\n");
        exit(1);
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error: Unable to configure serial port.\n");
        exit(1);
    }
}

void send_command(unsigned char* command, unsigned int size) {
    for (unsigned int i = 0; i < size; i++) {
        write(serial_port, &command[i], 1);
        usleep(100);
    }
}


int main() {
    setup_serial_port();

    printf("[%s] test start\n", __func__);

    unsigned char protocol1[] = {0x55, 0xAA, 0xDC, 0x11 , 0x30 , 0xF8 , 0x30 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xE8};
    unsigned char protocol2[] = {0x55, 0xAA, 0xDC, 0x11 , 0x30 , 0x07 , 0xD0 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xF7};
    unsigned char protocol3[] = {0x55, 0xAA, 0xDC, 0x11 , 0x30 , 0x00 , 0x00 , 0xF8 , 0x30 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xF7};
    unsigned char protocol4[] = {0x55, 0xAA, 0xDC, 0x11 , 0x30 , 0x00 , 0x00 , 0x07 , 0xD0 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0xE8};
    unsigned char protocol5[] = {0x55, 0xAA, 0xDC, 0x11 , 0x30 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x20};

    unsigned int protocol_num;
    printf("Select a protocol to send:\n");
    printf("1. Left 1\n");
    printf("2. Right 2\n");
    printf("3. Up 3\n");
    printf("4. Down 4\n");
    printf("5. Stop 5\n");
 
    scanf("%u", &protocol_num);

    unsigned char* selected_protocol;
    unsigned int selected_protocol_size;
    switch (protocol_num) {
        case 1:
            selected_protocol = protocol1;
            selected_protocol_size = sizeof(protocol1);
            break;
        case 2:
            selected_protocol = protocol2;
            selected_protocol_size = sizeof(protocol2);
            break;
        case 3:
            selected_protocol = protocol3;
            selected_protocol_size = sizeof(protocol3);
            break;
        case 4:
            selected_protocol = protocol4;
            selected_protocol_size = sizeof(protocol4);
            break;
        case 5:
            selected_protocol = protocol4;
            selected_protocol_size = sizeof(protocol5);
            break;
 
        default:
            printf("Invalid protocol selection.\n");
            close(serial_port);
            return 0;
    }
    send_command(selected_protocol, selected_protocol_size);

    close(serial_port);

    return 0;
}



