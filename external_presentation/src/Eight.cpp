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


bool Waypoint::operator<(const Waypoint other) const{
    //std::cout << "Operator: " << index << " " << other.index
    //    << " " << direction << " " << other.direction <<std::endl;
    return index<other.index || direction<other.direction;
}



Eight::Eight(int size)
: next {}, size {size}, current{0,1},
    segment_duration_oval{1000000000ull},
    // initialize "normal" eight:
    trajectory_px   {          -1,         -0.5,            0,          0.5,            1,         0.5,             0,         -0.5},
    trajectory_py   {           0,          0.5,            0,         -0.5,            0,          0.5,            0,         -0.5},
    trajectory_vx   {           0,            1,          0.3,            1,            0,           -1,         -0.3,           -1},
    trajectory_vy   {           1,            0,         -0.7,            0,            1,            0,         -0.7,            0},
    segment_duration{785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull, 785000000ull}
{
    // set "special" successors to allow driving an oval:
    next.insert( pWW( Waypoint(1, 1), Waypoint(5, -1) ));
    next.insert( pWW( Waypoint(1, 1), Waypoint(2,  1) ));



    assert(segment_duration.size() == trajectory_px.size());
    assert(segment_duration.size() == trajectory_py.size());
    assert(segment_duration.size() == trajectory_vx.size());
    assert(segment_duration.size() == trajectory_vy.size());

    const double map_center_x = 2.25;
    const double map_center_y = 2.0;
    for (double &px : trajectory_px)
    {
        px += map_center_x;
    }
    for (double &py : trajectory_py)
    {
        py += map_center_y;
    }
}





std::pair<TrajectoryPoint, uint64_t> Eight::next_waypoint(){
    TrajectoryPoint trajectory_point_res;
    uint64_t segment_duration_res;
    Waypoint succ(-1, -1); // only initial value; will be overriden later on


    //std::multimap<Waypoint, Waypoint>::iterator it = next.find(Waypoint(1,1));
    std::pair<std::multimap<Waypoint, Waypoint>::iterator,
              std::multimap<Waypoint, Waypoint>::iterator> iterators;
    iterators = next.equal_range(current);
    int no_successors = std::distance(iterators.first, iterators.second);

    if (no_successors > 0) {
        // there is at least one element in the map with the given key
        // choose one of the elements with this key randomly
        std::cout << "Number Succs: " << no_successors << std::endl;

        int choose = rand() % no_successors; // index of element between iterators which is to be chosen
        std::advance(iterators.first, choose);
        succ = iterators.first->second;

        if (succ.index == (current.index+current.direction)
                            % segment_duration.size()){
            // use normal segment duration
            segment_duration_res = segment_duration[current.index];
        }
        else {
            segment_duration_res = segment_duration_oval;
        }

    }
    else {
        // follow the "normal" trajectory by going one index ahead
        // corresponding to the current direction
        succ = Waypoint((current.index + current.direction) % size,
                         current.direction);
        segment_duration_res = segment_duration[current.index];
    }

    trajectory_point_res.px(trajectory_px[current.index]);
    trajectory_point_res.py(trajectory_py[current.index]);
    trajectory_point_res.vx(trajectory_vx[current.index*current.direction]);
    trajectory_point_res.vy(trajectory_vy[current.index*current.direction]);

    current = succ;
    return std::pair<TrajectoryPoint, uint64_t>(trajectory_point_res, segment_duration_res);
}