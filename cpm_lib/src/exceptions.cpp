#include "cpm/exceptions.hpp"

/**
 * \file exceptions.cpp
 * \ingroup cpmlib
 */

namespace cpm {
    ErrorTimerStart::ErrorTimerStart(const std::string& msg):
        std::runtime_error(msg)
    {
    }
}