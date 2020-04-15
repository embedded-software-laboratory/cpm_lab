#include "Eight.hpp"
#include <iostream>

typedef std::pair<Waypoint, Waypoint> pWW ;

Waypoint::Waypoint(int index, int direction)
: index {index}, direction {direction}
{

}



Eight::Eight()
: next {}
{
    next.insert( pWW( Waypoint(1, 1), Waypoint(5, -1) ));
    next.insert( pWW( Waypoint(2, 1), Waypoint(2,  1) ));
}


Waypoint Eight::get_next_waypoint(){
    bool b = Waypoint(1,1) < Waypoint(1,2);
    std::cout << "Comparison: " << b << std::endl;
    std::cout << "Map size: "<< next.size() << std::endl;

    std::map<Waypoint, Waypoint>::iterator it = next.find(Waypoint(1,1));
    if (it != next.end())
        return it->second;

    return Waypoint(1,2);
    //return next[Waypoint(1,1)];
}