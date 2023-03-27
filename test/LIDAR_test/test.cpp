#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main() {
    int fd;
    struct termios options;
    char buffer[255];

    fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror("Error opening serial port");
        exit(1);
    }

    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    tcsetattr(fd, TCSANOW, &options);

   
    int n = read(fd, buffer, sizeof(buffer));
    if (n < 0) {
        perror("Error reading from serial port");
        exit(1);
    }

    printf("Status value: %s\n", buffer);


    close(fd);

    return 0;
}