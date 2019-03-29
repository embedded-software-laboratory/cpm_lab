#pragma once

#include <stdexcept>

namespace cpm {
    class ErrorTimerStart: public std::runtime_error {    
    public:
        ErrorTimerStart(const std::string& msg);
    };
}