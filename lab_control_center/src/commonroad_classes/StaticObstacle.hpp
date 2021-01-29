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

#include "commonroad_classes/geometry/Shape.hpp"

#include "commonroad_classes/states/Occupancy.hpp"
#include "commonroad_classes/states/State.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/InterfaceTransformTime.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include "LCCErrorLogger.hpp"

#include "ObstacleSimulationData.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \enum ObstacleTypeStatic
 * \brief Specifies static obstacle types, as in commonroad, NotInSpec for types that should not exist
 * 2018 and 2020 differ because in 2020, we have static and dynamic obstacles, whereas in 2018, we only have obstacles of type static or dynamic - Throw an error in case of wrong (dynamic) types if role was set to static
 * We do not need to store "role", as we already have two different classes for that
 * \ingroup lcc_commonroad
 */
enum class ObstacleTypeStatic {Unknown, ParkedVehicle, ConstructionZone, RoadBoundary};

/**
 * \class StaticObstacle
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a StaticObstacle specified in an XML file
 * \ingroup lcc_commonroad
 */
class StaticObstacle : public InterfaceTransform, public InterfaceTransformTime
{
private:
    //! The obstacle type, e.g. a parked vehicle
    ObstacleTypeStatic type;
    //! The obstacle type as string
    std::string obstacle_type_text;
    //! Shape of the object, must exist
    std::optional<Shape> shape = std::nullopt;
    //! Initial state of the object, must exist
    std::optional<State> initial_state = std::nullopt;

    //Other 2018 obstacle-values should only be set for dynamic obstacles - nonetheless, we check for their existence in the constructor (and show warnings, if necessary)

    //! Transformation scale of transform_coordinate_system is remembered to draw text correctly scaled
    double transform_scale = 1.0;

    //! Remember line in commonroad file for logging
    int commonroad_line = 0;

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs
     * \param node A (static) obstacle node
     * \param _draw_lanelet_refs Function that, given an lanelet reference and the typical drawing arguments, draws a lanelet reference
     */
    StaticObstacle(
        const xmlpp::Node* node,
        std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs
    );

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

    //No draw function, obstacles are handled by LCC directly via simulation

    //Getter
    /**
     * \brief Returns a single trajectory point constructed from initial state, and further static obstacle information
     * Throws errors if expected types are missing
     */
    ObstacleSimulationData get_obstacle_simulation_data();

    /**
     * \brief Get the obstacle type
     */
    ObstacleTypeStatic get_type();
    /**
     * \brief Get the obstacle type as string
     */
    std::string get_obstacle_type_text();
    /**
     * \brief Get the obstacle shape, which must exist (optional due to no default constructor & potential translation issues)
     */
    const std::optional<Shape>& get_shape() const;
    /**
     * \brief Get the obstacle initial state, which must exist (optional due to no default constructor & potential translation issues)
     */
    const std::optional<State>& get_initial_state() const;
};