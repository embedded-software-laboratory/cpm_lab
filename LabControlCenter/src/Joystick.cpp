#include "Joystick.hpp"
#include <linux/joystick.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


Joystick::Joystick(string device_file) {
    js_fd = open(device_file.c_str(), O_RDONLY | O_NONBLOCK);
    if (js_fd == -1) throw runtime_error("Could not open joystick: " + device_file);
}

void Joystick::update() {
    while(1) {

        struct js_event event;
        ssize_t bytes = read(js_fd, &event, sizeof(event));
        bool read_success = (bytes == sizeof(event));

        if(read_success) {
            switch (event.type)
            {
                case JS_EVENT_BUTTON:
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
        else {
            break;
        }

    }
}

bool Joystick::getButton(uint8_t id) {
    update();
    return joystick_buttons[id];
}

int16_t Joystick::getAxis(uint8_t id) {
    update();
    return joystick_axes[id];
}

Joystick::~Joystick() {

}
