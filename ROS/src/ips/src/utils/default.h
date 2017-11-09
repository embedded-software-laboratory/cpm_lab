#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <cmath>
#include <iostream>
#include <ros/time.h>
#include <limits>
#include <map>
#include <experimental/optional>

using std::vector;
using std::string;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::shared_ptr;
using std::make_shared;
using std::cout;
using std::endl;
using std::map;
using std::get;
using std::experimental::optional;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

template <typename Base>
struct WithTimestamp : public Base {
    ros::Time timestamp;
    WithTimestamp(const Base &base, ros::Time timestamp):Base(base),timestamp(timestamp){}
    WithTimestamp(Base &&base, ros::Time timestamp):Base(std::move(base)),timestamp(timestamp){}
};
