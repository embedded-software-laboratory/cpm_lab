#pragma once

#include <vector>

/**
 * \class Crossing
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a crossing specified in an XML file
 * Is this specific class ever used outside the XML specs?
 */
class Crossing
{
private:
    std::vector<int> crossing_lanelets; //Lanelet ref

public:
    //TODO: Constructor, getter
};