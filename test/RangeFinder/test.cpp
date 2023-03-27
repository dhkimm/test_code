#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#include "common/mavlink.h"

// Connect to the ArduPilot autopilot
mavlink_status_t status;
mavlink_message_t message;
int serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);
struct termios options;
tcgetattr(serial_port, &options);
cfsetispeed(&options, B115200);
cfsetospeed(&options, B115200);
tcsetattr(serial_port, TCSANOW, &options);



mavlink_msg_rangefinder_request_distance_pack(1, 200, &message, 0);

// Send the message to the autopilot
unsigned char buf[MAVLINK_MAX_PACKET_LEN];
int len = mavlink_msg_to_send_buffer(buf, &message);
write(serial_port, buf, len);


// Wait for the sensor data to be received
uint8_t msgReceived = false;
while (!msgReceived) {
    uint8_t receivedChar;
    if (read(serial_port, &receivedChar, 1) > 0) {
        if (mavlink_parse_char(MAVLINK_COMM_0, receivedChar, &message, &status)) {
            if (message.msgid == MAVLINK_MSG_ID_RANGEFINDER_DISTANCE) {
                // Print the range sensor distance value
                mavlink_rangefinder_distance_t rangeData;
                mavlink_msg_rangefinder_distance_decode(&message, &rangeData);
                printf("Distance: %.2f meters\n", rangeData.distance);
                msgReceived = true;
            }
        }
    }
}