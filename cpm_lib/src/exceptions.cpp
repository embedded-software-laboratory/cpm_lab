#include "cpm/exceptions.hpp"

namespace cpm {
    ErrorTimerStart::ErrorTimerStart(const std::string& msg):
        std::runtime_error(msg)
    {
    }
}