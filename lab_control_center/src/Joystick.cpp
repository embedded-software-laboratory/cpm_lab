// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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

    // hack to get init values of shoulder triggers
    joystick_axes[2] = -int16_t(1<<15);
    joystick_axes[5] = -int16_t(1<<15);
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
    close(js_fd);
}
