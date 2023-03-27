#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

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

void send_command(const u_int16_t* command, size_t size) {
    size_t size_in_bytes = size * sizeof(u_int16_t);
    if (write(serial_port, command, size_in_bytes) != size_in_bytes) {
        fprintf(stderr, "Error: Failed to write to serial port.\n");
        exit(1);
    }
}



int main() {
    setup_serial_port();

    printf("[%s] test start\n", __func__);
    u_int16_t command[] = {0x55, 0xAA, 0xDC, 0x11 , 0x30 , 0x0B , 0x3F , 0xFC , 0x3F , 0xFC , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x2A};
    send_command(command, sizeof(command));

    close(serial_port);

    return 0;
}

