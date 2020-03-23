#pragma once

#include "commonroad_classes/geometry/Position.hpp"
#include "commonroad_classes/datatypes/IntervalOrExact.hpp"

/**
 * \class GoalState
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a GoalState specified in an XML file
 */
class GoalState
{
private:
    //Commonroad data
    IntervalOrExact time; //Time values should probably be within the range of double, we did not want to define an extra type for this - gets transformed in getter to nanoseconds view
    Position position;
    double orientation;
    double velocity;

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    GoalState(
        IntervalOrExact _time,
        Position _position,
        double _orientation,
        double _velocity
    )
    :
    time(_time),
    position(_position),
    orientation(_orientation),
    velocity(_velocity)
    {}

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg(); 

    //TODO: Getter
};
