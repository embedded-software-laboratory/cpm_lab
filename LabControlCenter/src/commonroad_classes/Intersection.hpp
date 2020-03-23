#pragma once

#include <map>

/**
 * \struct Incoming
 * \brief Specifies a part of the intersection, as in commonroad
 */
struct Incoming
{
    int incoming_lanelet; //Lanelet ref
    int successors_right; //Lanelet ref
    int successors_straight; //Lanelet ref
    int successors_left; //Lanelet ref
    int is_left_of; //Incoming ref
};

/**
 * \class Intersection
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent an intersection specified in an XML file
 */
class Intersection
{
private:
    std::map<int, Incoming> incoming_map;

public:
    //TODO: Constructor, getter
};