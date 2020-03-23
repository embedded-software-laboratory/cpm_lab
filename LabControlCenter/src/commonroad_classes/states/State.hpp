#pragma once

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
    Position position;
    IntervalOrExact orientation;
    IntervalOrExact time; //Time values should probably be within the range of double, we did not want to define an extra type for this - gets transformed in getter to nanoseconds view
    IntervalOrExact velocity;
    IntervalOrExact acceleration;
    IntervalOrExact yaw_rate;
    IntervalOrExact slip_angle;
    IntervalOrExact steering_angle;
    IntervalOrExact roll_angle;
    IntervalOrExact roll_rate;
    IntervalOrExact pitch_angle;
    IntervalOrExact pitch_rate;
    IntervalOrExact velocity_y;
    IntervalOrExact position_z;
    IntervalOrExact velocity_z;
    IntervalOrExact roll_angle_front;
    IntervalOrExact roll_rate_front;
    IntervalOrExact velocity_y_front;
    IntervalOrExact position_z_front;
    IntervalOrExact velocity_z_front;
    IntervalOrExact roll_angle_rear;
    IntervalOrExact roll_rate_rear;
    IntervalOrExact velocity_y_rear;
    IntervalOrExact position_z_rear;
    IntervalOrExact velocity_z_rear;
    IntervalOrExact left_front_wheel_angular_speed;
    IntervalOrExact right_front_wheel_angular_speed;
    IntervalOrExact left_rear_wheel_angular_speed;
    IntervalOrExact right_rear_wheel_angular_speed;
    IntervalOrExact delta_y_front;
    IntervalOrExact delta_y_rear;
    IntervalOrExact curvature;
    IntervalOrExact curvature_change;
    IntervalOrExact jerk;
    IntervalOrExact jounce;

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     * Thus, this rather ugly (bc. large) constructor is used instead
     */
    State(
        Position _position,
        IntervalOrExact _orientation,
        IntervalOrExact _time,
        IntervalOrExact _velocity,
        IntervalOrExact _acceleration,
        IntervalOrExact _yaw_rate,
        IntervalOrExact _slip_angle,
        IntervalOrExact _steering_angle,
        IntervalOrExact _roll_angle,
        IntervalOrExact _roll_rate,
        IntervalOrExact _pitch_angle,
        IntervalOrExact _pitch_rate,
        IntervalOrExact _velocity_y,
        IntervalOrExact _position_z,
        IntervalOrExact _velocity_z,
        IntervalOrExact _roll_angle_front,
        IntervalOrExact _roll_rate_front,
        IntervalOrExact _velocity_y_front,
        IntervalOrExact _position_z_front,
        IntervalOrExact _velocity_z_front,
        IntervalOrExact _roll_angle_rear,
        IntervalOrExact _roll_rate_rear,
        IntervalOrExact _velocity_y_rear,
        IntervalOrExact _position_z_rear,
        IntervalOrExact _velocity_z_rear,
        IntervalOrExact _left_front_wheel_angular_speed,
        IntervalOrExact _right_front_wheel_angular_speed,
        IntervalOrExact _left_rear_wheel_angular_speed,
        IntervalOrExact _right_rear_wheel_angular_speed,
        IntervalOrExact _delta_y_front,
        IntervalOrExact _delta_y_rear,
        IntervalOrExact _curvature,
        IntervalOrExact _curvature_change,
        IntervalOrExact _jerk,
        IntervalOrExact _jounce
    )
    :
    position(_position),
    orientation(_orientation),
    time(_time),
    velocity(_velocity),
    acceleration(_acceleration),
    yaw_rate(_yaw_rate),
    slip_angle(_slip_angle),
    steering_angle(_steering_angle),
    roll_angle(_roll_angle),
    roll_rate(_roll_rate),
    pitch_angle(_pitch_angle),
    pitch_rate(_pitch_rate),
    velocity_y(_velocity_y),
    position_z(_position_z),
    velocity_z(_velocity_z),
    roll_angle_front(_roll_angle_front),
    roll_rate_front(_roll_rate_front),
    velocity_y_front(_velocity_y_front),
    position_z_front(_position_z_front),
    velocity_z_front(_velocity_z_front),
    roll_angle_rear(_roll_angle_rear),
    roll_rate_rear(_roll_rate_rear),
    velocity_y_rear(_velocity_y_rear),
    position_z_rear(_position_z_rear),
    velocity_z_rear(_velocity_z_rear),
    left_front_wheel_angular_speed(_left_front_wheel_angular_speed),
    right_front_wheel_angular_speed(_right_front_wheel_angular_speed),
    left_rear_wheel_angular_speed(_left_rear_wheel_angular_speed),
    right_rear_wheel_angular_speed(_right_rear_wheel_angular_speed),
    delta_y_front(_delta_y_front),
    delta_y_rear(_delta_y_rear),
    curvature(_curvature),
    curvature_change(_curvature_change),
    jerk(_jerk),
    jounce(_jounce)
    {}

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    virtual void transform_coordinate_system(double scale) override;

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg(); 

    //TODO: Getter
};
