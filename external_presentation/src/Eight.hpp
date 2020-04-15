#include <map>


struct Waypoint
{
    int index;
    int direction;

    Waypoint(int index, int direction);
    bool operator<(const Waypoint other);
};


class Eight
{
    public:
    std::map<Waypoint, Waypoint> next;


//public:
    Eight();
    Waypoint get_next_waypoint();
};
