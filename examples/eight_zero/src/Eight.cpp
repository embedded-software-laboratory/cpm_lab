#include "Eight.hpp"
#include <iostream>
#include <iterator>
#include <stdlib.h>
#include <cassert>

#define NUMBER_EIGHT_POINTS 8
// The number of trajectory points in the trajectory-vectors describing the eight only.



typedef std::pair<Waypoint, Waypoint> pWW ;




Waypoint::Waypoint(int index, int direction)
: index {index}, direction {direction}
{
}

// Necessary to override this operator for using a map
bool Waypoint::operator<(const Waypoint other) const{
    if (direction < other.direction) {
        return true;
    }
    else if (direction==other.direction && index < other.index) {
        return true;
    }
    else {
        return false;
    }
}



Eight::Eight()
: next {}, current{8,1}, current2{8,1},
    current_segment_duration{0},
    segment_duration_oval{800000000ull},
    /* // 1x2
    trajectory_px   {          -1,         -0.5,            0,          0.5,            1,         0.5,             0,         -0.5},
    trajectory_py   {           0,          0.5,            0,         -0.5,            0,          0.5,            0,         -0.5},
    trajectory_vx   {           0,            1,          0.3,            1,            0,           -1,         -0.3,           -1},
    trajectory_vy   {           1,            0,         -0.7,            0,            1,            0,         -0.7,            0},
    segment_duration{785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull}
    */
                                                                                                                                    // From here on, the start is described (not the usual eight)
    trajectory_px   {        -0.8,         -0.4,            0,          0.4,          0.8,          0.4,            0,         -0.4,         -0.8,         -0.6},
    trajectory_py   {           0,          0.4,            0,         -0.4,            0,          0.4,            0,         -0.4,         -0.4,         -0.4},
    trajectory_vx   {           0,            1,       0.3939,            1,            0,           -1,      -0.3939,           -1,            0,      0.89442},
    trajectory_vy   {           1,            0,      -0.9191,            0,            1,            0,      -0.9191,            0,            0,            0},
    segment_duration{628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 447210000ull, 185250000ull}
    // IMPORTANT: If the eight trajectory is changed in terms of the number of points, this number
    //            has to be changed in the define at the top as well.
{

    // Insert all "standard" transitions of the forward and the
    // backward eight. Note: The eight-Trajectory is described by the first
    // eight points in the vectors above. Everything else describes the start
    // into the eight.
    for (int i=0; i<NUMBER_EIGHT_POINTS; i++){
        next.insert( pWW( Waypoint(i, 1), Waypoint((i+1)%NUMBER_EIGHT_POINTS, 1) ));
        next.insert( pWW( Waypoint(i,-1), Waypoint((i-1+NUMBER_EIGHT_POINTS) % NUMBER_EIGHT_POINTS, -1) ));
    }

    // Set "special" successors to allow driving an oval.
    next.insert( pWW( Waypoint(1, 1), Waypoint(5, -1) ));
    next.insert( pWW( Waypoint(5, 1), Waypoint(1, -1) ));
    next.insert( pWW( Waypoint(3,-1), Waypoint(7,  1) ));
    next.insert( pWW( Waypoint(7,-1), Waypoint(3,  1) ));

    // Describe the transitions from the start to the eight
    next.insert( pWW( Waypoint( 8, 1), Waypoint( 9,  1) ));
    next.insert( pWW( Waypoint( 9, 1), Waypoint( 7, -1) ));



    assert(segment_duration.size() == trajectory_px.size());
    assert(segment_duration.size() == trajectory_py.size());
    assert(segment_duration.size() == trajectory_vx.size());
    assert(segment_duration.size() == trajectory_vy.size());


    // Assuming a maximum area of 2mx1m, the center of the map must be positioned
    // such that the point (0,0) can be the start point of the vehicle
    const double map_center_x = 0.8;
    const double map_center_y = 0.4;
    for (double &px : trajectory_px)
    {
        px += map_center_x;
    }
    for (double &py : trajectory_py)
    {
        py += map_center_y;
    }
}




/**
 * Returns the current TrajectoryPoint and the segment_duration needed between this point
 * and the one afterwards.
 */
std::pair<TrajectoryPoint, uint64_t> Eight::get_trajectoryPoint(){
    TrajectoryPoint trajectory_point_res;
    trajectory_point_res.px(trajectory_px[current.index]);
    trajectory_point_res.py(trajectory_py[current.index]);
    trajectory_point_res.vx(trajectory_vx[current.index]*current.direction);
    trajectory_point_res.vy(trajectory_vy[current.index]*current.direction);

    return std::pair<TrajectoryPoint, uint64_t>(trajectory_point_res, current_segment_duration);
}


/**
 * Planns the next Waypoint.
 */
void Eight::move_forward(){
    current = current2; // Move one point forward
    Waypoint succ(-1, -1); // only initial value; will be overriden later on

    std::pair<std::multimap<Waypoint, Waypoint>::iterator,
              std::multimap<Waypoint, Waypoint>::iterator> iterators = next.equal_range(current);
    int no_successors = std::distance(iterators.first, iterators.second);


    // choose one of the elements with this key (describing the subsequent trajectory points) randomly
    int choose = rand() % no_successors; // index of element between iterators which is to be chosen
    std::advance(iterators.first, choose);
    succ = iterators.first->second;

    if (((succ.index == (( (current.index+current.direction) % NUMBER_EIGHT_POINTS) 
                                    + NUMBER_EIGHT_POINTS) % NUMBER_EIGHT_POINTS ))
        || (current.index >= NUMBER_EIGHT_POINTS)){ // Transitions from start into eight
        // use normal segment duration
        current_segment_duration = segment_duration[current.index];
    }
    else {
        current_segment_duration = segment_duration_oval;
    }

    current2 = succ;
}