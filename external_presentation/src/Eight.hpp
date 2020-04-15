#include <map>
#include <iostream>

struct Waypoint
{
    int index;
    int direction;

    Waypoint(int index, int direction);
    //bool operator<(const Waypoint &other);
    bool operator<(const Waypoint other) const{
        std::cout << "Operator: " << index << " " << other.index
            << " " << direction << " " << other.direction <<std::endl;
        return index<other.index || direction<other.direction;
    }
};


class Eight
{
    public:
    std::map<Waypoint, Waypoint> next;


//public:
    Eight();
    Waypoint get_next_waypoint();
};

/*
bool operator<(const Waypoint lhs, const Waypoint rhs){
    return lhs.index<rhs.index && lhs.direction<rhs.direction;
}*/