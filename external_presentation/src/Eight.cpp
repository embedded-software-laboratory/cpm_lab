#include "Eight.hpp"
#include <iostream>

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
: next {}, size {size}, current{0,1}
{
    next.insert( pWW( Waypoint(1, 1), Waypoint(5, -1) ));
    next.insert( pWW( Waypoint(1, 1), Waypoint(2,  1) ));
}


Waypoint Eight::next_waypoint(){
    bool b = Waypoint(1,1) < Waypoint(1,2);
    std::cout << "Comparison: " << b << std::endl;
    std::cout << "Map size: "<< next.size() << std::endl;

    std::multimap<Waypoint, Waypoint>::iterator it = next.find(Waypoint(1,1));
    if (it != next.end()) {
        // there are exactly two elements in the map
        // choose one of them randomly ...
        return it->second;
    }
    else {
        // follow the "normal" trajectory by going one index ahead
        // corresponding to the current direction
        current = Waypoint((current.index + current.direction) % size,
                            current.direction);
        return current;
    }

    return Waypoint(1,2);
    //return next[Waypoint(1,1)];
}