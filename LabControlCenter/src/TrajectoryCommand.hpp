#pragma once
#include <stdint.h>
#include "defaults.hpp"
#include "Point.hpp"

class TrajectoryCommand
{
public:
    TrajectoryCommand();
    void set_path(uint8_t vehicle_id, std::vector<Point> path, bool loop);
    void stop(uint8_t vehicle_id);
    void stop_all();
    
};