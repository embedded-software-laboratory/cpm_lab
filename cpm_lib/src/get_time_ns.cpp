#include "cpm/get_time_ns.hpp"

/**
 * \file get_time_ns.cpp
 * \ingroup cpmlib
 */

uint64_t cpm::get_time_ns(clockid_t clockid) {
    struct timespec t;
    clock_gettime(clockid, &t);
    return uint64_t(t.tv_sec) * 1000000000ull + uint64_t(t.tv_nsec);
}

uint64_t cpm::get_time_ns() {
    return cpm::get_time_ns(CLOCK_REALTIME);
}