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
