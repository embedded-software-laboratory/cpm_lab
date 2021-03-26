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

#include <string>
#include <vector>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Position.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include "LCCErrorLogger.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \enum TrafficLightColor
 * \brief Specifies a traffic light color, as in commonroad
 * \ingroup lcc_commonroad
 */
enum class TrafficLightColor {Red, RedYellow, Yellow, Green, Inactive};

/**
 * \enum Direction
 * \brief Specifies a direction, as in commonroad
 * \ingroup lcc_commonroad
 */
enum class Direction {Right, Straight, Left, LeftStraight, StraightRight, LeftRight, All};

/**
 * \struct TrafficCycleElement
 * \brief Specifies a single light cycle, as in commonroad
 * \ingroup lcc_commonroad
 */
struct TrafficCycleElement
{
    //! Traffig light colors / order of colors for each duration
    std::vector<TrafficLightColor> colors;
    //! Duration with which each traffic light color is shown, in the given order
    std::vector<unsigned int> durations;
};

/**
 * \struct TrafficLightCycle
 * \brief Specifies a full light cycle, as in commonroad
 * \ingroup lcc_commonroad
 */
struct TrafficLightCycle
{
    //! Definition of different traffic light cycles, giving order and duration of the colors
    std::vector<TrafficCycleElement> cycle_elements;
    //! Time offset before the cycle start, optional
    std::optional<unsigned int> time_offset = std::nullopt;
};

/**
 * \struct TrafficLightElement
 * \brief Specifies a single traffic light; TrafficLight might contain more than one light according to specs
 * \ingroup lcc_commonroad
 */
struct TrafficLightElement
{
    //Commonroad types
    //! Traffic light cycle, defining when which color is shown
    TrafficLightCycle cycle;
    //! Specified as being always exact, gives the position of the light, might be undefined (in that case: Position is given within some lanelet in form of a reference)
    std::optional<Position> position = std::nullopt;
    //! Direction shown by the light, default is all
    Direction direction = Direction::All;
    //! Indicated if the traffic light is currently active. Is assumed to default to true
    bool is_active = true; 
};

/**
 * \class TrafficLight
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a traffic light specified in an XML file
 * \ingroup lcc_commonroad
 */
class TrafficLight : public InterfaceTransform, public InterfaceDraw
{
private:
    //! Contains all seperate traffic lights defined for this ID
    std::vector<TrafficLightElement> traffic_light_elements;

    //! ID of the traffic light(s)
    int id;

    //! Helper function from commonroadscenario to get position defined by lanelet if no position was defined for the traffic sign
    std::function<std::optional<std::pair<double, double>>(int)> get_position_from_lanelet;

    /**
     * \brief Helper function that draws a tiny traffic light symbol
     * \param ctx The drawing context for the LCC's map view
     * \param scale To change the size of the drawn symbol
     */
    void draw_traffic_light_symbol(const DrawingContext& ctx, double scale);

    //Helper functions for better readability
    /**
     * \brief Helper function to translate an xml position node
     * \param position_node The Commonroad XML position node 
     */
    Position translate_position(const xmlpp::Node* position_node);
    /**
     * \brief Helper function to translate an xml direction node
     * \param direction_node The Commonroad XML direction node 
     */
    Direction translate_direction(const xmlpp::Node* direction_node);
    /**
     * \brief Helper function to translate an xml active node
     * \param active_node The Commonroad XML active node 
     */
    bool translate_active(const xmlpp::Node* active_node);
    /**
     * \brief Helper function to translate an xml cycle node
     * \param cycle_node The Commonroad XML cycle node 
     */
    TrafficLightCycle translate_cycle(const xmlpp::Node* cycle_node);

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs
     * \param node A trafficLight node
     * \param _get_position_from_lanelet A function that allows to obtain a position value defined for the sign by a lanelet reference, if it exists
     */
    TrafficLight(
        const xmlpp::Node* node,
        std::function<std::optional<std::pair<double, double>>(int)> _get_position_from_lanelet
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
};