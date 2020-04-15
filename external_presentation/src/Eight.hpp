#include <map>
#include <iostream>

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
    int size; // number of the trajectory points
    Waypoint current;


//public:
    Eight(int size);
    Waypoint next_waypoint();
};

