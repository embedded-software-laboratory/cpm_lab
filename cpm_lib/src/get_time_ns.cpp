#include "cpm/get_time_ns.hpp"

uint64_t cpm::get_time_ns() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}