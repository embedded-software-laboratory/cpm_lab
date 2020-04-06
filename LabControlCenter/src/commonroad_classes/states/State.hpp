#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <memory>

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
    std::unique_ptr<Position> position;
    std::unique_ptr<IntervalOrExact> orientation;
    std::unique_ptr<IntervalOrExact> time; //Time values should probably be within the range of double - gets transformed in getter to nanoseconds view; also: State and InitialState differ in spec, as in InitialState time must be exact - we did not make an extra type for this here, as we expect conform xml types
    std::unique_ptr<IntervalOrExact> velocity;
    std::unique_ptr<IntervalOrExact> acceleration;
    std::unique_ptr<IntervalOrExact> yaw_rate;
    std::unique_ptr<IntervalOrExact> slip_angle;
    std::unique_ptr<IntervalOrExact> steering_angle;
    std::unique_ptr<IntervalOrExact> roll_angle;
    std::unique_ptr<IntervalOrExact> roll_rate;
    std::unique_ptr<IntervalOrExact> pitch_angle;
    std::unique_ptr<IntervalOrExact> pitch_rate;
    std::unique_ptr<IntervalOrExact> velocity_y;
    std::unique_ptr<IntervalOrExact> position_z;
    std::unique_ptr<IntervalOrExact> velocity_z;
    std::unique_ptr<IntervalOrExact> roll_angle_front;
    std::unique_ptr<IntervalOrExact> roll_rate_front;
    std::unique_ptr<IntervalOrExact> velocity_y_front;
    std::unique_ptr<IntervalOrExact> position_z_front;
    std::unique_ptr<IntervalOrExact> velocity_z_front;
    std::unique_ptr<IntervalOrExact> roll_angle_rear;
    std::unique_ptr<IntervalOrExact> roll_rate_rear;
    std::unique_ptr<IntervalOrExact> velocity_y_rear;
    std::unique_ptr<IntervalOrExact> position_z_rear;
    std::unique_ptr<IntervalOrExact> velocity_z_rear;
    std::unique_ptr<IntervalOrExact> left_front_wheel_angular_speed;
    std::unique_ptr<IntervalOrExact> right_front_wheel_angular_speed;
    std::unique_ptr<IntervalOrExact> left_rear_wheel_angular_speed;
    std::unique_ptr<IntervalOrExact> right_rear_wheel_angular_speed;
    std::unique_ptr<IntervalOrExact> delta_y_front;
    std::unique_ptr<IntervalOrExact> delta_y_rear;
    std::unique_ptr<IntervalOrExact> curvature;
    std::unique_ptr<IntervalOrExact> curvature_change;
    std::unique_ptr<IntervalOrExact> jerk;
    std::unique_ptr<IntervalOrExact> jounce;

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
