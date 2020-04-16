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
    Waypoint current;

    uint64_t current_segment_duration; // needed for get_waypoint because it depends on the chosen way in move_forward
    uint64_t segment_duration_oval; // time which the special oval segments need

    vector<double> trajectory_px;
    vector<double> trajectory_py;
    vector<double> trajectory_vx;
    vector<double> trajectory_vy;
    vector<uint64_t> segment_duration;


//public:
    Eight();
    std::pair<TrajectoryPoint, uint64_t> get_waypoint();
    void move_forward();
};

