#pragma once

#include "defaults.hpp"


class Joystick {

    uint8_t joystick_buttons[256] = {};
    int16_t joystick_axes[256] = {};
    int js_fd = 0;

    void update();

public:
    Joystick(string device_file);
    bool getButton(uint8_t id);
    int16_t getAxis(uint8_t id);
    ~Joystick();
};