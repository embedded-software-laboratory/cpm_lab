#pragma once
#include <ctime>
#include <stdint.h>

namespace cpm {
    /**
     * \brief Global function to access the current system time in nanoseconds, saves redundant code
     */
    uint64_t get_time_ns();
}