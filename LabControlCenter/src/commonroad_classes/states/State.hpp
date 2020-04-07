#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Position.hpp"
#include "commonroad_classes/datatypes/IntervalOrExact.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"

/**
 * \class State
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a state specified in an XML file
 */
class State : public InterfaceTransform
{
private:
    //Commonroad data
    std::optional<Position> position;
    std::optional<IntervalOrExact> orientation;
    std::optional<IntervalOrExact> time; //Time values should probably be within the range of double - gets transformed in getter to nanoseconds view; also: State and InitialState differ in spec, as in InitialState time must be exact - we did not make an extra type for this here, as we expect conform xml types
    std::optional<IntervalOrExact> velocity;
    std::optional<IntervalOrExact> acceleration;
    std::optional<IntervalOrExact> yaw_rate;
    std::optional<IntervalOrExact> slip_angle;
    std::optional<IntervalOrExact> steering_angle;
    std::optional<IntervalOrExact> roll_angle;
    std::optional<IntervalOrExact> roll_rate;
    std::optional<IntervalOrExact> pitch_angle;
    std::optional<IntervalOrExact> pitch_rate;
    std::optional<IntervalOrExact> velocity_y;
    std::optional<IntervalOrExact> position_z;
    std::optional<IntervalOrExact> velocity_z;
    std::optional<IntervalOrExact> roll_angle_front;
    std::optional<IntervalOrExact> roll_rate_front;
    std::optional<IntervalOrExact> velocity_y_front;
    std::optional<IntervalOrExact> position_z_front;
    std::optional<IntervalOrExact> velocity_z_front;
    std::optional<IntervalOrExact> roll_angle_rear;
    std::optional<IntervalOrExact> roll_rate_rear;
    std::optional<IntervalOrExact> velocity_y_rear;
    std::optional<IntervalOrExact> position_z_rear;
    std::optional<IntervalOrExact> velocity_z_rear;
    std::optional<IntervalOrExact> left_front_wheel_angular_speed;
    std::optional<IntervalOrExact> right_front_wheel_angular_speed;
    std::optional<IntervalOrExact> left_rear_wheel_angular_speed;
    std::optional<IntervalOrExact> right_rear_wheel_angular_speed;
    std::optional<IntervalOrExact> delta_y_front;
    std::optional<IntervalOrExact> delta_y_rear;
    std::optional<IntervalOrExact> curvature;
    std::optional<IntervalOrExact> curvature_change;
    std::optional<IntervalOrExact> jerk;
    std::optional<IntervalOrExact> jounce;

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    State(const xmlpp::Node* node){}

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
