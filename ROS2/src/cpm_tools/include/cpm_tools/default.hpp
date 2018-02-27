#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <thread>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <experimental/optional>

#define NANOSEC_PER_SEC 1000000000ull
#define NANOSEC_PER_MILLISEC 1000000ull

using std::vector;
using std::string;
using std::to_string;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::shared_ptr;
using std::make_shared;
using std::cout;
using std::endl;
using std::thread;
using std::map;
using std::get;
using std::experimental::optional;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

template <typename Base>
struct WithTimestamp : public Base {
    uint64_t timestamp;
    WithTimestamp(){}
    WithTimestamp(const Base &base, uint64_t timestamp):Base(base),timestamp(timestamp){}
    WithTimestamp(Base &&base, uint64_t timestamp):Base(std::move(base)),timestamp(timestamp){}
};


uint64_t clock_gettime_nanoseconds();