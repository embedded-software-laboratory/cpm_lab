#include <stdint.h>
#include <linux/joystick.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <linux/input.h>       
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <time.h>


#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.h"
#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.c"


/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;
    bytes = read(fd, event, sizeof(*event));
    return bytes == sizeof(*event);
}



uint8_t joystick_buttons[100];
int16_t joystick_axes[100];


typedef struct sockaddr_in ip_addr;

void die(char* s)
{
    perror(s);
    exit(1);
}


void update_joystick(int js) {
    size_t axis;
    struct js_event event;
    if(read_event(js, &event)) {
        switch (event.type)
        {
            case JS_EVENT_BUTTON:
                //printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
                joystick_buttons[event.number] = event.value;
                break;
            case JS_EVENT_AXIS:
                joystick_axes[event.number] = event.value;
                break;
            default:
                /* Ignore init events. */
                break;
        }
    }
}

uint64_t clock_gettime_nanoseconds() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return (uint64_t)(t.tv_sec) * 1000000000ull + (uint64_t)(t.tv_nsec);
}




int setNonblocking(int fd)
{
    int flags;
    if (-1 == (flags = fcntl(fd, F_GETFL, 0)))
        flags = 0;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}  





int make_socket() {
    int sock;
    if ( (sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        die("socket");
    }

    struct sockaddr_in si_me;
    memset((char *) &si_me, 0, sizeof(si_me));
     
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(6780);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
     
    //bind socket to port
    if( bind(sock, (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        die("bind");
    }
    setNonblocking(sock);
    return sock;
}


int receive(int sock, char* sender_ip_out, char* message_out, size_t message_max_size) {
    ip_addr si_other;
    unsigned int slen = sizeof(ip_addr);
    int recv_len = recvfrom(sock, message_out, message_max_size, 0, (struct sockaddr *) &si_other, &slen);

    if(recv_len > 1) {
        inet_ntop(AF_INET, &(si_other.sin_addr), sender_ip_out, 1000);
    }

    return recv_len;
}



ip_addr make_addr(char* ip, int port) {
    ip_addr target_addr;
    memset((char *) &target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(port);
    if (inet_aton(ip , &target_addr.sin_addr) == 0)  {
        die("inet_aton");
    }
    return target_addr;
}



void send_to(int sock, ip_addr target, void* message, size_t n_bytes) {
    if (sendto(sock, message, n_bytes , 0 , (struct sockaddr *) &target, sizeof(ip_addr))==-1) {
        die("sendto()");
    }
}




int main(int argc, char *argv[])
{

    // Joystick setup
    const char *device;
    int js;
    if (argc > 1)
        device = argv[1];
    else
        device = "/dev/input/js1";

    js = open(device, O_RDONLY | O_NONBLOCK);
    if (js == -1)
        perror("Could not open joystick");



    // Udp setup
    int sock = make_socket();


    uint64_t timestamp_last_print = clock_gettime_nanoseconds();
    uint64_t timestamp_last_command = clock_gettime_nanoseconds();

    char other_ip[100];
    other_ip[0] = 0;
    char receive_buffer[1000];

    spi_miso_data_t telemetry;
    memset(&telemetry, 0, sizeof(spi_miso_data_t));

    crcInit();


    while (1)
    {
        update_joystick(js);


        // look for telemetry from the vehicle to get its IP
        int message_size = 0;
        if((message_size = receive(sock, other_ip, receive_buffer, 1000))>0) {
            //printf("rcvd %i bytes from %s\n", message_size, other_ip);

            if(message_size == sizeof(spi_miso_data_t)) {
                memcpy(&telemetry, receive_buffer, sizeof(spi_miso_data_t));
            }
        }


        // send command to vehicle
        if(strlen(other_ip) > 4) { // got vehicle IP?
            if(timestamp_last_command + 20000000ull < clock_gettime_nanoseconds()) {
                timestamp_last_command = clock_gettime_nanoseconds();

                spi_mosi_data_t commands;
                memset(&commands, 0, sizeof(spi_mosi_data_t));

                double servo_cmd = -joystick_axes[2];
                servo_cmd /= 1<<15; // normalize
                //servo_cmd = (0.3* servo_cmd + 1.0 * (servo_cmd*servo_cmd*servo_cmd))/(0.3 + 1.0); // expo control
                servo_cmd *= 1000;

                // prevent integer overflow
                if(servo_cmd > 30000) servo_cmd = 30000;
                if(servo_cmd < -30000) servo_cmd = -30000;


                commands.servo_command = (int16_t)servo_cmd;

                commands.LED_bits = 0b10100110;

                if(joystick_buttons[0]) {
                    commands.LED_bits |= 0b00000011;
                }
                if(joystick_buttons[1]) {
                    commands.LED_bits |= 0b00110000;
                }
                if(joystick_buttons[2]) {
                    commands.LED_bits |= 0b11000000;
                }

                int32_t throttle = joystick_axes[1]/80;

                if(throttle < 8 && throttle > -8) {
                    commands.motor_pwm = 0;
                    commands.motor_mode = SPI_MOTOR_MODE_BRAKE;
                }
                else if(throttle > 0) {
                    commands.motor_pwm = throttle;
                    commands.motor_mode = SPI_MOTOR_MODE_FORWARD;
                }
                else {
                    commands.motor_pwm = -throttle;
                    commands.motor_mode = SPI_MOTOR_MODE_REVERSE;
                }

                ip_addr target = make_addr(other_ip, 6783);

                commands.debugB = 42;
                commands.CRC = 0;
                commands.CRC = crcFast((uint8_t*)(&commands), sizeof(spi_mosi_data_t));
                //printf("out-CRC %i\n", commands.CRC);


                //printf("sending %i bytes to %s\n", sizeof(spi_mosi_data_t), other_ip);
                send_to(sock, target, &commands, sizeof(spi_mosi_data_t));
            }
        }


        // display joystick values
        if(timestamp_last_print + 1000000000ull < clock_gettime_nanoseconds()) {
            timestamp_last_print = clock_gettime_nanoseconds();


            printf("\n\n\n");
            for (int i = 0; i < 10; ++i)
            {
                printf("btn %i: %i\n", i, joystick_buttons[i]);
            }
            for (int i = 0; i < 10; ++i)
            {
                printf("axs %i: %i\n", i, joystick_axes[i]);
            }


            printf("vehicle IP %s\n", other_ip);
            printf("tick %u\n", telemetry.tick);
            printf("odometer_steps %u\n", telemetry.odometer_steps);
            printf("imu_yaw %u\n", telemetry.imu_yaw);
            printf("imu_acceleration_forward %i\n", telemetry.imu_acceleration_forward);
            printf("imu_acceleration_left %i\n", telemetry.imu_acceleration_left);
            printf("speed %i\n", telemetry.speed);
            printf("battery_voltage %u\n", telemetry.battery_voltage);
            printf("motor_current %u\n", telemetry.motor_current);
            printf("debugC %i\n", telemetry.debugC);
            printf("debugD %i\n", telemetry.debugD);
            printf("CRC %u\n", telemetry.CRC);
            printf("status_flags %u\n", telemetry.status_flags);
        }

    }

    close(js);
    return 0;
}