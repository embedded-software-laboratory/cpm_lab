// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

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
#include "commonroad_classes/InterfaceTransformTime.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include "LCCErrorLogger.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \class State
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a state specified in an XML file
 * \ingroup lcc_commonroad
 */
class State : public InterfaceTransform, public InterfaceDraw, public InterfaceTransformTime
{
private:
    //Commonroad data
    //! Position in this state, must be set, 2018/2020 specs
    std::optional<Position> position = std::nullopt;
    //! Optional orientation in this state, 2018/2020 specs
    std::optional<IntervalOrExact> orientation = std::nullopt;
    //! Time (commonroad representation, not nanoseconds) in this state, must be set, 2018/2020 specs
    std::optional<IntervalOrExact> time = std::nullopt; 
    //! Optional velocity in this state, 2018/2020 specs
    std::optional<IntervalOrExact> velocity = std::nullopt;
    //! Optional acceleration in this state, 2018/2020 specs
    std::optional<IntervalOrExact> acceleration = std::nullopt;
    //! Optional yaw_rate in this state, 2018/2020 specs
    std::optional<IntervalOrExact> yaw_rate = std::nullopt;
    //! Optional slip_angle in this state, 2018/2020 specs
    std::optional<IntervalOrExact> slip_angle = std::nullopt;

    //2020 specs only
    //! Optional steering angle in this state, 2020 specs only
    std::optional<IntervalOrExact> steering_angle = std::nullopt;
    //! Optional roll angle in this state, 2020 specs only
    std::optional<IntervalOrExact> roll_angle = std::nullopt;
    //! Optional roll rate in this state, 2020 specs only
    std::optional<IntervalOrExact> roll_rate = std::nullopt;
    //! Optional pitch angle in this state, 2020 specs only
    std::optional<IntervalOrExact> pitch_angle = std::nullopt;
    //! Optional pitch angle in this state, 2020 specs only
    std::optional<IntervalOrExact> pitch_rate = std::nullopt;
    //! Optional velocity in y direction in this state, 2020 specs only
    std::optional<IntervalOrExact> velocity_y = std::nullopt;
    //! Optional position on the z axis in this state, 2020 specs only
    std::optional<IntervalOrExact> position_z = std::nullopt;
    //! Optional velocity on the z axis in this state, 2020 specs only
    std::optional<IntervalOrExact> velocity_z = std::nullopt;
    //! Optional roll angle in the front in this state, 2020 specs only
    std::optional<IntervalOrExact> roll_angle_front = std::nullopt;
    //! Optional roll rate in the front in this state, 2020 specs only
    std::optional<IntervalOrExact> roll_rate_front = std::nullopt;
    //! Optional velocity in y direction in the front in this state, 2020 specs only
    std::optional<IntervalOrExact> velocity_y_front = std::nullopt;
    //! Optional position on the z axis in the front in this state, 2020 specs only
    std::optional<IntervalOrExact> position_z_front = std::nullopt;
    //! Optional velocity on the z axis in the front in this state, 2020 specs only
    std::optional<IntervalOrExact> velocity_z_front = std::nullopt;
    //! Optional roll angle in the rear in this state, 2020 specs only
    std::optional<IntervalOrExact> roll_angle_rear = std::nullopt;
    //! Optional roll rate in the rear in this state, 2020 specs only
    std::optional<IntervalOrExact> roll_rate_rear = std::nullopt;
    //! Optional velocity in y direction in the rear in this state, 2020 specs only
    std::optional<IntervalOrExact> velocity_y_rear = std::nullopt;
    //! Optional position on the z axis in the rear in this state, 2020 specs only
    std::optional<IntervalOrExact> position_z_rear = std::nullopt;
    //! Optional velocity on the z axis in the rear in this state, 2020 specs only
    std::optional<IntervalOrExact> velocity_z_rear = std::nullopt;
    //! Optional angular speed of the left front wheel in this state, 2020 specs only
    std::optional<IntervalOrExact> left_front_wheel_angular_speed = std::nullopt;
    //! Optional angular speed of the right front wheel in this state, 2020 specs only
    std::optional<IntervalOrExact> right_front_wheel_angular_speed = std::nullopt;
    //! Optional angular speed of the left rear wheel in this state, 2020 specs only
    std::optional<IntervalOrExact> left_rear_wheel_angular_speed = std::nullopt;
    //! Optional angular speed of the right rear wheel in this state, 2020 specs only
    std::optional<IntervalOrExact> right_rear_wheel_angular_speed = std::nullopt;
    //! Optional delta y in the front in this state, 2020 specs only
    std::optional<IntervalOrExact> delta_y_front = std::nullopt;
    //! Optional delta y in the rear in this state, 2020 specs only
    std::optional<IntervalOrExact> delta_y_rear = std::nullopt;
    //! Optional curvature in this state, 2020 specs only
    std::optional<IntervalOrExact> curvature = std::nullopt;
    //! Optional curvature change in this state, 2020 specs only
    std::optional<IntervalOrExact> curvature_change = std::nullopt;
    //! Optional jerk in this state, 2020 specs only
    std::optional<IntervalOrExact> jerk = std::nullopt;
    //! Optional jounce in this state, 2020 specs only
    std::optional<IntervalOrExact> jounce = std::nullopt;

    //! Transformation scale of transform_coordinate_system is remembered to draw circles / arrows correctly scaled
    double transform_scale = 1.0;

    //! Remember line in commonroad file for logging
    int commonroad_line = 0;

public:
    /**
     * \brief Constructor that creates a state object from a commonroad xml state node
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
     * \param scale The factor by which to transform all number values related to position, or the min lane width (for commonroadscenario) - 0 means: No transformation desired
     * \param angle Rotation of the coordinate system, around the origin, w.r.t. right-handed coordinate system (according to commonroad specs), in radians
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    void transform_coordinate_system(double scale, double angle, double translate_x, double translate_y) override;

    /**
     * \brief This function is used to change timing-related values, like velocity, where needed
     * \param time_scale The factor with which time step size was changed (e.g. 0.5 to 1.0 results in a factor of 2.0)
     */
    void transform_timing(double time_scale) override;

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
    /**
     * \brief Get the set position, which must exist - throws an error if it does not
     */
    Position get_position(); 
    /**
     * \brief Get the set time, which must exist - throws an error if it does not
     */
    IntervalOrExact get_time();
    /**
     * \brief Get the orientation mean or nullopt if orientation was not set
     */
    std::optional<double> get_orientation_mean();
    /**
     * \brief Get the velocity or nullopt if it was not set
     */
    std::optional<IntervalOrExact>& get_velocity();
    
    /**
     * \brief Get the orientation or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_orientation() const;
    /**
     * \brief Get the acceleration or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_acceleration() const;
    /**
     * \brief Get the yaw rate or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_yaw_rate() const;
    /**
     * \brief Get the slip angle or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_slip_angle() const;
    /**
     * \brief Get the steering angle or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_steering_angle() const;
    /**
     * \brief Get the roll angle or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_roll_angle() const;
    /**
     * \brief Get the roll rate or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_roll_rate() const;
    /**
     * \brief Get the pitch angle or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_pitch_angle() const;
    /**
     * \brief Get the pitch rate or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_pitch_rate() const;
    /**
     * \brief Get the velocity in y direction or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_velocity_y() const;
    /**
     * \brief Get the position on the z axis or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_position_z() const;
    /**
     * \brief Get the velocity on the z axis or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_velocity_z() const;
    /**
     * \brief Get the roll angle in the front or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_roll_angle_front() const;
    /**
     * \brief Get the roll rate in the front or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_roll_rate_front() const;
    /**
     * \brief Get the velocity in y direction in the front or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_velocity_y_front() const;
    /**
     * \brief Get the position on the z axis in the front or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_position_z_front() const;
    /**
     * \brief Get the velocity on the z axis in the front or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_velocity_z_front() const;
    /**
     * \brief Get the roll angle in the rear or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_roll_angle_rear() const;
    /**
     * \brief Get the roll rate in the rear or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_roll_rate_rear() const;
    /**
     * \brief Get the velocity in y direction in the rear or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_velocity_y_rear() const;
    /**
     * \brief Get the position on the z axis in the rear or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_position_z_rear() const;
    /**
     * \brief Get the velocity on the z axis in the rear or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_velocity_z_rear() const;
    /**
     * \brief Get the angular speed of the left front wheel or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_left_front_wheel_angular_speed() const;
    /**
     * \brief Get the angular speed of the right front wheel or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_right_front_wheel_angular_speed() const;
    /**
     * \brief Get the angular speed of the left rear wheel or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_left_rear_wheel_angular_speed() const;
    /**
     * \brief Get the angular speed of the right rear wheel or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_right_rear_wheel_angular_speed() const;
    /**
     * \brief Get delta y front or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_delta_y_front() const;
    /**
     * \brief Get delta y rear or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_delta_y_rear() const;
    /**
     * \brief Get the curvature or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_curvature() const;
    /**
     * \brief Get the curvature change or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_curvature_change() const;
    /**
     * \brief Get the jerk or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_jerk() const;
    /**
     * \brief Get the jounce or nullopt if it was not set
     */
    const std::optional<IntervalOrExact>& get_jounce() const;
};
