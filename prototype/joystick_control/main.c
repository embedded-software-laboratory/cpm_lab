#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/joystick.h>
#include <time.h>

#include "../../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/spi_packets.h"


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




int main(int argc, char *argv[])
{
    const char *device;
    int js;

    if (argc > 1)
        device = argv[1];
    else
        device = "/dev/input/js1";

    js = open(device, O_RDONLY | O_NONBLOCK);

    if (js == -1)
        perror("Could not open joystick");


    uint64_t timestamp_last_print = clock_gettime_nanoseconds();


    while (1)
    {
        update_joystick(js);


        if(timestamp_last_print + 100000000ull < clock_gettime_nanoseconds()) {
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
        }

        //printf(".");
        //usleep(1000);
    }

    close(js);
    return 0;
}