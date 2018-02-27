#include "cpm_tools/default.hpp"


uint64_t clock_gettime_nanoseconds() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * NANOSEC_PER_SEC + uint64_t(t.tv_nsec);
}