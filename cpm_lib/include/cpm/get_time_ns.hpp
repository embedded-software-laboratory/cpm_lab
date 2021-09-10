#pragma once
#include <ctime>
#include <stdint.h>

namespace cpm {
    /**
     * \brief Global function to access the current system time in nanoseconds, saves redundant code
     * \ingroup cpmlib
     */
    uint64_t get_time_ns();

    /**
     * \brief Same as get_time_ns but allows specifying the clock type
     */
    uint64_t get_time_ns(clockid_t clockid);
}