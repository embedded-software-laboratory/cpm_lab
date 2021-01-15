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

#include <functional>
#include <memory>
#include <vector>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Circle.hpp"
#include "commonroad_classes/geometry/Polygon.hpp"
#include "commonroad_classes/geometry/Rectangle.hpp"
#include "commonroad_classes/geometry/Shape.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceGeometry.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include "LCCErrorLogger.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

#include "CommonroadDDSGoalState.hpp"

/**
 * \class Position
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 * \ingroup lcc_commonroad
 */
class Position : public InterfaceTransform, public InterfaceDraw, public InterfaceGeometry
{
private:
    //! Transformation scale of transform_coordinate_system is remembered to draw circles / arrows correctly scaled
    double transform_scale = 1.0;
    
    //! Exact position (positionExact), position can also be set inexact in form of a shape (circles, polygons, rectangles, lanelet_refs)
    std::optional<Point> point = std::nullopt;

    //Inexact position (positionInterval)
    //! Circles, part of the inexact position shape
    std::vector<Circle> circles;
    //! Lanelet references, part of the inexact position shape
    std::vector<int> lanelet_refs;
    //! Polygons, part of the inexact position shape
    std::vector<Polygon> polygons;
    //! Rectangles, part of the inexact position shape
    std::vector<Rectangle> rectangles;

    /**
     * \brief Function to draw a shape given lanelet references, given by the CommonRoadScenario class
     */
    std::function<void (int, const DrawingContext&, double, double, double, double)> draw_lanelet_refs;

    /**
     * \brief Function to get center of lanelet_ref
     */
    std::function<std::pair<double, double> (int)> get_lanelet_center;

    //! Remember line in commonroad file for logging
    int commonroad_line = 0;

public:
    /**
     * \brief Constructor, set up a position object
     */
    Position(const xmlpp::Node* node);

    /**
     * \brief Second constructor, value is irrelevant - value given s.t. !!no default constructor exists!!
     * Call this if you need to use the specified default value from the specs because the value was not set explicitly
     */
    Position(int irrelevant_int);

    /**
     * \brief Setter for drawing lanelet references (Position can also be constructed without this)
     * \param _draw_lanelet_refs Function that, given a lanelet reference and the typical drawing arguments, draws a lanelet reference
     */
    void set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs);

    /**
     * \brief Setter for getting lanelet center
     * \param _get_lanelet_center Function that returns a lanelet center
     */
    void set_lanelet_get_center_function(std::function<std::pair<double, double> (int)> _get_lanelet_center);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale, double angle, double translate_x, double translate_y) override;

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
     * \brief Get center (positional value) of the shape, if one exists
     * \return Center of the shape
     */
    std::pair<double, double> get_center() override;

    //Further getters
    /**
     * \brief Get lanelet ref for position; currently only a single stored reference is allowed, else throws an error
     */
    std::optional<int> get_lanelet_ref();
    /**
     * \brief Tells if the position is exact or given in form of a shape
     */
    bool is_exact();
    /**
     * \brief Tells if the position is given in form of a lanelet reference
     */
    bool position_is_lanelet_ref();

    /**
     * \brief This function is used to transform (rotate, translate) a context, e.g. because position/orientation and shape information are given in different objects, but need to be combined for drawing
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     */
    void transform_context(const DrawingContext& ctx, double scale = 1.0);
    
    /**
     * \brief Empty / not yet supported, to translate a position to DDS
     */
    void to_dds_msg() {} 
    
    /**
     * \brief Translate the set position to a position interval
     */
    CommonroadDDSPositionInterval to_dds_position_interval();

    /**
     * \brief Translate the set position to an exact position
     */
    CommonroadDDSPoint to_dds_point();

    //Getters for basic types
    /**
     * \brief Get the position in form of a point, if it exists
     */
    std::optional<Point> get_point();
    /**
     * \brief Access the circles, which are part of the optional inexact position shape
     */
    const std::vector<Circle>& get_circles() const;
    /**
     * \brief Access the lanelet references, which are part of the optional inexact position shape
     */
    const std::vector<int>& get_lanelet_refs() const;
    /**
     * \brief Access the polygons, which are part of the optional inexact position shape
     */
    const std::vector<Polygon>& get_polygons() const;
    /**
     * \brief Access the rectangles, which are part of the optional inexact position shape
     */
    const std::vector<Rectangle>& get_rectangles() const;
};