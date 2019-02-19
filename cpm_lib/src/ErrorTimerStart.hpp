#pragma once

#include <stdexcept>

class ErrorTimerStart: public std::runtime_error {    
public:
    ErrorTimerStart(const std::string& msg):
        std::runtime_error(msg)
    {
    }
};