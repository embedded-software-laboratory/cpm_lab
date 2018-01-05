#pragma once
#include "cpm_tools/default.h"

class Timer {
    const ros::Duration dt;
public:
    Timer(ros::Duration dt);
};