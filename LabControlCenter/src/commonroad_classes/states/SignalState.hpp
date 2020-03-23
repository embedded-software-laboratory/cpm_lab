#pragma once

#include "commonroad_classes/datatypes/IntervalOrExact.hpp"

/**
 * \class SignalState
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a SignalState specified in an XML file
 */
class SignalState
{
private:
    //Commonroad data
    IntervalOrExact time; //Time values should probably be within the range of double, we did not want to define an extra type for this - gets transformed in getter to nanoseconds view
    bool horn;
    bool indicator_left;
    bool indicator_right;
    bool braking_lights;
    bool hazard_warning_lights;
    bool flashing_blue_lights;

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    SignalState(
        IntervalOrExact _time,
        bool _horn,
        bool _indicator_left,
        bool _indicator_right,
        bool _braking_lights,
        bool _hazard_warning_lights,
        bool _flashing_blue_lights
    )
    :
    time(_time),
    horn(_horn),
    indicator_left(_indicator_left),
    indicator_right(_indicator_right),
    braking_lights(_braking_lights),
    hazard_warning_lights(_hazard_warning_lights),
    flashing_blue_lights(_flashing_blue_lights)
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
