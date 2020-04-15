#include "Eight.hpp"

typedef std::pair<Waypoint, Waypoint> pWW ;


bool Waypoint::operator<(const Waypoint other){
    return index<other.index && direction<other.direction;
}


Eight::Eight(){
    next.insert( pWW( Waypoint(1, 1), Waypoint(5, -1) ));
    next.insert( pWW( Waypoint(1, 1), Waypoint(2,  1) ));
}


Waypoint Eight::get_next_waypoint(){
    return next.at(next.find(Waypoint(1,1)));
}