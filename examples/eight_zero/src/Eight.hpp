#include <map>
#include <iostream>
#include <vector>
#include "VehicleCommandTrajectory.hpp"

using std::vector;


struct Waypoint
{
    int index;

    // The "normal" eight trajectory is extended by one path connecting the two topmost points
    // and by one connecting the two lowermost points. If one of these paths is used the
    // direction in which the vehicle follows the 8-trajectory changes. Thus, all velocities
    // must change the sign which is done by this variable:
    int direction;

    Waypoint(int index, int direction);
    bool operator<(const Waypoint other) const;
};


class Eight
{
    std::multimap<Waypoint, Waypoint> next;
    Waypoint current;
    Waypoint current2; // it is necessary to plan two points in advice since segment_duration corresponds to the time
                       // in between these two points

    uint64_t current_segment_duration; // needed for get_waypoint because it depends on the chosen way in move_forward
    uint64_t segment_duration_oval; // time which the special oval segments need

    vector<double> trajectory_px;
    vector<double> trajectory_py;
    vector<double> trajectory_vx;
    vector<double> trajectory_vy;
    vector<uint64_t> segment_duration;


public:
    Eight();
    TrajectoryPoint get_trajectoryPoint();
    uint64_t get_segment_duration();
    void move_forward();
};

