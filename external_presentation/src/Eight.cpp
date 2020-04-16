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
    //std::cout << "Operator: " << index 
    //    << " " << direction << " " << other.index << " " << other.direction <<std::endl;
    //std::cout << (direction<other.direction || index<other.index) << std::endl;
    if (direction < other.direction) {
        //  std::cout << "True1" << std::endl;
        return true;
    }
    else if (direction==other.direction && index < other.index) {
        //std::cout << "True2" << std::endl;
        return true;
    }
    else {
        //std::cout << "False" << std::endl;
        return false;
    }
    //return (direction<other.direction || index<other.index);
}



Eight::Eight()
: next {}, current{0,1}, current2{0,1},
    current_segment_duration{0},
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
    next.insert( pWW( Waypoint(5, 1), Waypoint(1, -1) ));
    next.insert( pWW( Waypoint(5, 1), Waypoint(6,  1) ));
    next.insert( pWW( Waypoint(3,-1), Waypoint(2, -1) ));
    next.insert( pWW( Waypoint(3,-1), Waypoint(7,  1) ));
    next.insert( pWW( Waypoint(7,-1), Waypoint(6, -1) ));
    next.insert( pWW( Waypoint(7,-1), Waypoint(3,  1) ));



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





std::pair<TrajectoryPoint, uint64_t> Eight::get_waypoint(){
    TrajectoryPoint trajectory_point_res;
    trajectory_point_res.px(trajectory_px[current.index]);
    trajectory_point_res.py(trajectory_py[current.index]);
    trajectory_point_res.vx(trajectory_vx[current.index]*current.direction);
    trajectory_point_res.vy(trajectory_vy[current.index]*current.direction);

    std::cout << "P: (" << trajectory_point_res.px() << ", " << trajectory_point_res.py() << ")" << std::endl;
    std::cout << "V: (" << trajectory_point_res.vx() << ", " << trajectory_point_res.vy() << ")" << std::endl;
    std::cout << "Duration: " << current_segment_duration << std::endl;

    return std::pair<TrajectoryPoint, uint64_t>(trajectory_point_res, current_segment_duration);
}


void Eight::move_forward(){
        //std::cout << "Current: " << current.index << " " << current.direction << std::endl;
    /*std::cout << "Current Map:" <<std::endl;
    std::multimap<Waypoint, Waypoint>::iterator it = next.begin();
    for (; it != next.end(); it++){
        std::cout << "    " << it->second.index << " " << it->second.direction << std::endl;
    }*/
    current = current2; // Move one point forward

    Waypoint succ(-1, -1); // only initial value; will be overriden later on

    int size = (int) segment_duration.size();

    //std::multimap<Waypoint, Waypoint>::iterator it = next.find(Waypoint(1,1));
    std::pair<std::multimap<Waypoint, Waypoint>::iterator,
              std::multimap<Waypoint, Waypoint>::iterator> iterators = next.equal_range(current);
    //iterators = next.equal_range(current);
    int no_successors = std::distance(iterators.first, iterators.second);

/*
    if (current.index == 3 && current.direction == -1){
        std::cout << "Current Map:" <<std::endl;
        std::multimap<Waypoint, Waypoint>::iterator it = next.begin();
        for (; it != next.end(); it++){
            std::cout << "    " << it->first.index << " " << it->first.direction << " " << it->second.index << " " << it->second.direction << std::endl;
        }
    }
    std::cout << "Number Succs: " << no_successors << std::endl;
*/
    if (no_successors > 0) {
        // there is at least one element in the map with the given key
        // choose one of the elements with this key randomly

        int choose = rand() % no_successors; // index of element between iterators which is to be chosen
        std::advance(iterators.first, choose);
        succ = iterators.first->second;

        if (succ.index == (( (current.index+current.direction) % size) 
                                        + size) % size ){
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