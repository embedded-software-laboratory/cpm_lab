#include "Eight.hpp"
#include <iostream>
#include <iterator>
#include <stdlib.h>
#include <cassert>

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
    // initialize "normal" eight:
    /* // 1x2
    trajectory_px   {          -1,         -0.5,            0,          0.5,            1,         0.5,             0,         -0.5},
    trajectory_py   {           0,          0.5,            0,         -0.5,            0,          0.5,            0,         -0.5},
    trajectory_vx   {           0,            1,          0.3,            1,            0,           -1,         -0.3,           -1},
    trajectory_vy   {           1,            0,         -0.7,            0,            1,            0,         -0.7,            0},
    segment_duration{785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull}
    */
   /* too small:
    trajectory_px   {        -0.8,         -0.4,            0,          0.4,          0.8,          0.4,            0,         -0.4},
    trajectory_py   {           0,          0.3,            0,         -0.3,            0,          0.3,            0,         -0.3},
    trajectory_vx   {           0,            1,       0.3939,            1,            0,           -1,      -0.3939,           -1},
    trajectory_vy   {           1,            0,      -0.9191,            0,            1,            0,      -0.9191,            0},
    segment_duration{549800000ull, 549800000ull, 549800000ull, 549800000ull, 549800000ull, 549800000ull, 549800000ull, 549800000ull}
*/                                                                                                                                  // From here on, the start is described (not the usual eight)
    trajectory_px   {        -0.8,         -0.4,            0,          0.4,          0.8,          0.4,            0,         -0.4,         -0.8,         -0.4},
    trajectory_py   {           0,          0.4,            0,         -0.4,            0,          0.4,            0,         -0.4,         -0.4,         -0.4},
    trajectory_vx   {           0,            1,       0.3939,            1,            0,           -1,      -0.3939,           -1,            0,     0.333333},
    trajectory_vy   {           1,            0,      -0.9191,            0,            1,            0,      -0.9191,            0,            0,            0},
    segment_duration{628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 628300000ull, 979800000ull, 717300000ull}
{
    // Set "special" successors to allow driving an oval.
    // If no successor is given in this map the "normal" one
    // (resulting from in-/decrementing the usual indix) is chosen.
    next.insert( pWW( Waypoint(1, 1), Waypoint(5, -1) ));
    next.insert( pWW( Waypoint(1, 1), Waypoint(2,  1) ));
    next.insert( pWW( Waypoint(5, 1), Waypoint(1, -1) ));
    next.insert( pWW( Waypoint(5, 1), Waypoint(6,  1) ));
    next.insert( pWW( Waypoint(3,-1), Waypoint(2, -1) ));
    next.insert( pWW( Waypoint(3,-1), Waypoint(7,  1) ));
    next.insert( pWW( Waypoint(7,-1), Waypoint(6, -1) ));
    next.insert( pWW( Waypoint(7,-1), Waypoint(3,  1) ));

    // Describe the transitions from the start to the eight
    next.insert( pWW( Waypoint(8, 1), Waypoint(9,  1) ));
    next.insert( pWW( Waypoint(9, 1), Waypoint(3,  1) ));


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

    std::cout << current.index << std::endl;
    return std::pair<TrajectoryPoint, uint64_t>(trajectory_point_res, current_segment_duration);
}


/**
 * Planns the next Waypoint.
 */
void Eight::move_forward(){
    std::cout << "Move" << std::endl;
    current = current2; // Move one point forward

    Waypoint succ(-1, -1); // only initial value; will be overriden later on
    int size = 8; // The size of the arrays in which the eight is described (except the starting points)

    std::pair<std::multimap<Waypoint, Waypoint>::iterator,
              std::multimap<Waypoint, Waypoint>::iterator> iterators = next.equal_range(current);
    int no_successors = std::distance(iterators.first, iterators.second);


    if (no_successors > 0) {
        // there is at least one element in the map with the given key
        // choose one of the elements with this key randomly

        int choose = rand() % no_successors; // index of element between iterators which is to be chosen
        std::advance(iterators.first, choose);
        succ = iterators.first->second;

        if (((succ.index == (( (current.index+current.direction) % size) 
                                        + size) % size ))
            || (current.index == 9)){ // Transition from start into eight
            // use normal segment duration
            current_segment_duration = segment_duration[current.index];
        }
        else {
            current_segment_duration = segment_duration_oval;
        }

    }
    else {
        // follow the "normal" trajectory by going one index ahead
        // corresponding to the current direction
        succ = Waypoint(((current.index + current.direction) % size + size) % size,
                         current.direction);
        current_segment_duration = segment_duration[current.index];
    }

    current2 = succ;
}