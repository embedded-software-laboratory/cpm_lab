#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Position.hpp"
#include "commonroad_classes/geometry/Shape.hpp"
#include "commonroad_classes/datatypes/IntervalOrExact.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include "LCCErrorLogger.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \class State
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a state specified in an XML file
 */
class State : public InterfaceTransform, public InterfaceDraw
{
private:
    //Commonroad data
    std::optional<Position> position;
    std::optional<IntervalOrExact> orientation;
    std::optional<IntervalOrExact> time; //Time values should probably be within the range of double
    std::optional<IntervalOrExact> velocity;
    std::optional<IntervalOrExact> acceleration;
    std::optional<IntervalOrExact> yaw_rate;
    std::optional<IntervalOrExact> slip_angle;

    //2020 specs only
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

    //Transformation scale of transform_coordinate_system is remembered to draw circles / arrows correctly scaled
    double transform_scale = 1.0;

    //Remember line in commonroad file for logging
    int commonroad_line = 0;

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    State(const xmlpp::Node* node);

    /**
     * \brief Reduce code redundancy by putting the few lines necessary for each translation into one function
     * \param node Current node (parent of child)
     * \param child_name Name of the child node
     * \param warn Warn if the child does not exist
     */
    std::optional<IntervalOrExact> get_interval(const xmlpp::Node* node, std::string child_name, bool warn);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale, double translate_x, double translate_y) override;

    /**
     * \brief This function is used to draw the data structure that imports this interface
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * To change local translation, just transform the coordinate system beforehand
     * As this does not always work with local orientation (where sometimes the translation in the object must be called before the rotation if performed, to rotate within the object's coordinate system),
     * local_orientation was added as a parameter
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     * \param global_orientation - optional: Rotation that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_x - optional: Translation in x-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_y - optional: Translation in y-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param local_orientation - optional: Rotation that needs to be applied within the object's coordinate system
     */
    void draw(const DrawingContext& ctx, double scale = 1.0, double global_orientation = 0.0, double global_translate_x = 0.0, double global_translate_y = 0.0, double local_orientation = 0.0) override;

    /**
     * \brief Setter for drawing lanelet references (Can also be constructed without this)
     * \param _draw_lanelet_refs Function that, given an lanelet reference and the typical drawing arguments, draws a lanelet reference
     */
    void set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs);

    /**
     * \brief This function is used to transform (rotate, translate) a context, e.g. because position/orientation and shape information are given in different objects, but need to be combined for drawing
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     */
    void transform_context(const DrawingContext& ctx, double scale = 1.0);

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg() {} 

    //Getter
    Position get_position(); //Must exist, throw error if it does not
    IntervalOrExact get_time(); //Must exist, throw error if it does not
    std::optional<double> get_orientation_mean();
    std::optional<IntervalOrExact> get_velocity();
    
    const std::optional<IntervalOrExact> get_orientation() const;
    const std::optional<IntervalOrExact> get_acceleration() const;
    const std::optional<IntervalOrExact> get_yaw_rate() const;
    const std::optional<IntervalOrExact> get_slip_angle() const;
    const std::optional<IntervalOrExact> get_steering_angle() const;
    const std::optional<IntervalOrExact> get_roll_angle() const;
    const std::optional<IntervalOrExact> get_roll_rate() const;
    const std::optional<IntervalOrExact> get_pitch_angle() const;
    const std::optional<IntervalOrExact> get_pitch_rate() const;
    const std::optional<IntervalOrExact> get_velocity_y() const;
    const std::optional<IntervalOrExact> get_position_z() const;
    const std::optional<IntervalOrExact> get_velocity_z() const;
    const std::optional<IntervalOrExact> get_roll_angle_front() const;
    const std::optional<IntervalOrExact> get_roll_rate_front() const;
    const std::optional<IntervalOrExact> get_velocity_y_front() const;
    const std::optional<IntervalOrExact> get_position_z_front() const;
    const std::optional<IntervalOrExact> get_velocity_z_front() const;
    const std::optional<IntervalOrExact> get_roll_angle_rear() const;
    const std::optional<IntervalOrExact> get_roll_rate_rear() const;
    const std::optional<IntervalOrExact> get_velocity_y_rear() const;
    const std::optional<IntervalOrExact> get_position_z_rear() const;
    const std::optional<IntervalOrExact> get_velocity_z_rear() const;
    const std::optional<IntervalOrExact> get_left_front_wheel_angular_speed() const;
    const std::optional<IntervalOrExact> get_right_front_wheel_angular_speed() const;
    const std::optional<IntervalOrExact> get_left_rear_wheel_angular_speed() const;
    const std::optional<IntervalOrExact> get_right_rear_wheel_angular_speed() const;
    const std::optional<IntervalOrExact> get_delta_y_front() const;
    const std::optional<IntervalOrExact> get_delta_y_rear() const;
    const std::optional<IntervalOrExact> get_curvature() const;
    const std::optional<IntervalOrExact> get_curvature_change() const;
    const std::optional<IntervalOrExact> get_jerk() const;
    const std::optional<IntervalOrExact> get_jounce() const;
};
