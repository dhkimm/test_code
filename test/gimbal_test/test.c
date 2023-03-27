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
        printf("Error: Unable to open serial port.\n");
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
        printf("Error: Unable to configure serial port.\n");
        exit(1);
    }
}

void send_mavlink_command(u_int8_t* command, u_int8_t length) {
    u_int16_t checksum = 0xFFFF;
    for (int i = 0; i < length; i++) {
        checksum ^= command[i];
    }

    u_int8_t buf[length + 8];
    buf[0] = 0xFE;
    buf[1] = length;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    memcpy(buf + 5, command, length);
    buf[length + 5] = checksum & 0xFF;
    buf[length + 6] = checksum >> 8;
    buf[length + 7] = 0x00;

    write(serial_port, buf, length + 8);
}


void gbtest(u_int8_t* command, size_t size) {
}


int main() {
  setup_serial_port();

  printf("[%s] test start\n", __func__);
  u_int8_t command[] = {};
  gbtest(command, sizeof(command));

  close(serial_port);
  return 0;
}
