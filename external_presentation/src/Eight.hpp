#include <map>
#include <iostream>
#include <vector>
#include "VehicleCommandTrajectory.hpp"

using std::vector;


struct Waypoint
{
    int index;
    int direction;

    Waypoint(int index, int direction);
    bool operator<(const Waypoint other) const;
};


class Eight
{
    public:
    std::multimap<Waypoint, Waypoint> next;
    int size; // number of the trajectory points
    Waypoint current;

    vector<double> trajectory_px;
    vector<double> trajectory_py;
    vector<double> trajectory_vx;
    vector<double> trajectory_vy;
    vector<uint64_t> segment_duration;


//public:
    Eight(int size);
    Waypoint next_waypoint();
};

