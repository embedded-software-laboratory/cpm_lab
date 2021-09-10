#pragma once

#include "defaults.hpp"

/**
 * \class Joystick
 * \brief Class for joystick controls 
 * \ingroup lcc
 */
class Joystick {
    //! TODO
    uint8_t joystick_buttons[256] = {};
    //! TODO
    int16_t joystick_axes[256] = {};
    //! TODO
    int js_fd = 0;

    /**
     * \brief TODO
     */
    void update();

public:
    /**
     * \brief TODO
     * \param device_file TODO
     */
    Joystick(string device_file);
    /**
     * \brief TODO
     * \param id TODO
     */
    bool getButton(uint8_t id);
    /**
     * \brief TODO
     * \param id TODO
     */
    int16_t getAxis(uint8_t id);
    /**
     * \brief TODO
     */
    ~Joystick();
};