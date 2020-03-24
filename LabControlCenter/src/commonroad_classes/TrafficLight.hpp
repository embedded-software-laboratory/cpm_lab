#pragma once

#include <string>
#include <vector>

#include "commonroad_classes/geometry/Position.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"

/**
 * \enum class TrafficLightColor
 * \brief Specifies a traffic light color, as in commonroad, NotInSpec for types that should not exist
 */
enum class TrafficLightColor {Red, RedYellow, Yellow, Green, NotInSpec};

/**
 * \enum class Direction
 * \brief Specifies a direction, as in commonroad, NotInSpec for types that should not exist
 */
enum class Direction {Right, Straight, Left, LeftStraight, StraightRight, LeftRight, All, NotInSpec};

/**
 * \struct TrafficCycleElement
 * \brief Specifies a single light cycle, as in commonroad
 */
struct TrafficCycleElement
{
    TrafficLightColor color;
    unsigned int duration;
};

/**
 * \struct TrafficLightCycle
 * \brief Specifies a full light cycle, as in commonroad
 */
struct TrafficLightCycle
{
    std::vector<TrafficCycleElement> cycle_elements;
    unsigned int time_offset;
};

/**
 * \class TrafficLight
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a traffic light specified in an XML file
 */
class TrafficLight : public InterfaceTransform
{
private:
    //Commonroad types
    TrafficLightCycle cycle;
    Position position; //TODO: Position is specified as being always exact
    Direction direction;
    bool is_active; //Probably defaults to true, as it must not occur

public:
    //TODO: Constructor, getter

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}
};