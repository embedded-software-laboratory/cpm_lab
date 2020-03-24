#pragma once

#include <cstdint>

#include "commonroad_classes/geometry/Position.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"

/**
 * \class StateExact
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a StateExact specified in an XML file
 */
class StateExact : public InterfaceTransform
{
private:
    //Commonroad data
    Position position; //Exact position!
    double orientation;
    uint64_t time;
    double velocity;
    double acceleration;
    double yaw_rate;
    double slip_angle;

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    StateExact(
        Position _position,
        double _orientation,
        uint64_t _time,
        double _velocity,
        double _acceleration,
        double _yaw_rate,
        double _slip_angle
    )
    :
    position(_position),
    orientation(_orientation),
    time(_time),
    velocity(_velocity),
    acceleration(_acceleration),
    yaw_rate(_yaw_rate),
    slip_angle(_slip_angle)
    {}

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg() {}

    //TODO: Getter
};
