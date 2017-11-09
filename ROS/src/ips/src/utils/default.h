#pragma once

#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <iostream>
#include <ros/time.h>
#include <limits>

using std::vector;
using std::string;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::shared_ptr;
using std::make_shared;
using std::cout;
using std::endl;

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();