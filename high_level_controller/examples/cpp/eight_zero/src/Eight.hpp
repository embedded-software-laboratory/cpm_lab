#include <map>
#include <iostream>
#include <vector>
#include "VehicleCommandTrajectory.hpp"

using std::vector;

/**
 * \struct Waypoint
 * \brief TODO
 * \ingroup eight_zero
 */
struct Waypoint
{
    //! TODO
    int index;

    /**
     * \brief The "normal" eight trajectory is extended by one path connecting the two topmost points
     * and by one connecting the two lowermost points. If one of these paths is used the
     * direction in which the vehicle follows the 8-trajectory changes. Thus, all velocities
     * must change the sign which is done by this variable:
     */
    int direction;

    /**
     * \brief Constructor TODO
     * \param index TODO
     * \param direction TODO
     */
    Waypoint(int index, int direction);

    /**
     * \brief TODO
     * \param other TODO
     */
    bool operator<(const Waypoint other) const;
};

/**
 * \class Eight
 * \brief TODO
 * \ingroup eight_zero
 */
class Eight
{
    //! TODO
    std::multimap<Waypoint, Waypoint> next;
    //! TODO
    Waypoint current;
    //! TODO
    Waypoint current2; // it is necessary to plan two points in advice since segment_duration corresponds to the time
                       // in between these two points

    //! needed for get_waypoint because it depends on the chosen way in move_forward
    uint64_t current_segment_duration;
    //! time which the special oval segments need
    uint64_t segment_duration_oval;

    //! TODO
    vector<double> trajectory_px;
    //! TODO
    vector<double> trajectory_py;
    //! TODO
    vector<double> trajectory_vx;
    //! TODO
    vector<double> trajectory_vy;
    //! TODO
    vector<uint64_t> segment_duration;


public:
    /**
     * \brief Constructor TODO
     */
    Eight();
    
    /**
     * \brief TODO
     */
    TrajectoryPoint get_trajectoryPoint();

    /**
     * \brief TODO
     */
    uint64_t get_segment_duration();

    /**
     * \brief TODO
     */
    void move_forward();
};

