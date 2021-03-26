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

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "commonroad_classes/geometry/Point.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceGeometry.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include "commonroad_classes/CommonroadDrawConfiguration.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include "LCCErrorLogger.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \enum LaneletType
 * \brief Stores lanelet type, as in spec
 * \ingroup lcc_commonroad
 */
enum class LaneletType {
    Unspecified, Urban, Interstate, Country, Highway, Sidewalk, Crosswalk, BusLane, BicycleLane, ExitRamp, MainCarriageWay, AccessRamp, DriveWay, BusStop, Unknown
};

/**
 * \enum VehicleType
 * \brief Stores lanelet type, as in spec
 * \ingroup lcc_commonroad
 */
enum class VehicleType {
    Vehicle, Car, Truck, Bus, Motorcycle, Bicycle, Pedestrian, PriorityVehicle, Train
};

/**
 * \enum DrivingDirection
 * \brief Stores driving direction, used by Adjacent
 * \ingroup lcc_commonroad
 */
enum class DrivingDirection {
    Same, Opposite
};

/**
 * \enum LineMarking
 * \brief Holds all line marking types defined by the specification, used by Bound
 * \ingroup lcc_commonroad
 */
enum class LineMarking {
    Dashed, Solid, BroadDashed, BroadSolid, Unknown, NoMarking
};

/**
 * \struct Adjacent
 * \brief Holds information on adjacent road tiles and their driving direction
 * Initial values in case it does not exist
 * \ingroup lcc_commonroad
 */
struct Adjacent
{
    //! ID of adjacent lanelets
    int ref_id = -1;
    //! Driving direction
    DrivingDirection direction;
};

/**
 * \struct Bound
 * \brief Bound of a lanelet, defined by points and marking, e.g. dashed if adjacent to another lanelet
 * \ingroup lcc_commonroad
 */
struct Bound
{
    //! Lanelet bound points, that define one side of the lanelet shape, min. 2
    std::vector<Point> points;
    //! Line marking of a lanelet, e.g. dashed if adjacent to another lanelet
    std::optional<LineMarking> line_marking = std::nullopt;
};

/**
 * \struct StopLine
 * \brief Defines position of a stop line on the lanelet (w. possible reference to traffic signs etc)
 * Initial value in case it does not exist
 * \ingroup lcc_commonroad
 */
struct StopLine
{
    //! Position of the stop line. Must consist of up to two points.
    std::vector<Point> points;
    //! Line marking of the stop line, e.g. dashed
    LineMarking line_marking;
    //! References to traffic signs that might be next to the stop line
    std::vector<int> traffic_sign_refs;
    //! References to traffic lights that might be next to the stop line; only one possible, but a vector easier to handle if nonexistent
    std::vector<int> traffic_light_ref;
};

/**
 * \class Lanelet
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store Lanelets specified in the XML file (segments of a lane, with references to following and adjacent lanes)
 * \ingroup lcc_commonroad
 */
class Lanelet : public InterfaceTransform, public InterfaceDraw, public InterfaceGeometry
{
private:
    //! Left boundary of the lanelet, shape is formed combined with the right boundary
    Bound left_bound;
    //! Right boundary of the lanelet, shape is formed combined with the left boundary
    Bound right_bound;
    //! Lanelet references to predecessors of the lanelet, multiple possible e.g. in case of a fork
    std::vector<int> predecessors;
    //! Lanelet references to successors of the lanelet, multiple possible e.g. in case of a fork
    std::vector<int> successors;  
    //! Adjacent lanelets on the left, optional
    std::optional<Adjacent> adjacent_left = std::nullopt; 
    //! Adjacent lanelets on the right, optional
    std::optional<Adjacent> adjacent_right = std::nullopt;
    //! Stop line on the lanelet, optional
    std::optional<StopLine> stop_line = std::nullopt;
    //! Type of the lanelet
    LaneletType lanelet_type; 
    //! For which vehicles the lanelet is one-way
    std::vector<VehicleType> user_one_way; 
    //! For which vehicles the lanelet is bidirectional
    std::vector<VehicleType> user_bidirectional;
    //! References to traffic sign IDs for traffic signs on the lanelet
    std::vector<int> traffic_sign_refs;
    //! References to traffic light IDs for traffic lights on the lanelet
    std::vector<int> traffic_light_refs;
    //! 2018 specs, defines the speed limit, optional (2020: Defined by optional traffic sign)
    std::optional<double> speed_limit = std::nullopt;

    //! Remember line in commonroad file for logging
    int commonroad_line = 0;

    //! Look up in draw if some parts should be drawn or not
    std::shared_ptr<CommonroadDrawConfiguration> draw_configuration;

    /**
     * \brief This function translates a bound node to Bound
     * \param node A bound node
     * \param name The name of the node
     */
    Bound translate_bound(const xmlpp::Node* node, std::string name);

    /**
     * \brief This function translates a reference node to a vector of integer references
     * \param node A reference node
     * \param name The name of the node
     */
    std::vector<int> translate_refs(const xmlpp::Node* node, std::string name);

    /**
     * \brief This function translates a laneletAdjacentRef node to Adjacent
     * \param node A laneletAdjacentRef node
     * \param name The name of the node
     */
    std::optional<Adjacent> translate_adjacent(const xmlpp::Node* node, std::string name);

    /**
     * \brief This function translates a stopLine node to StopLine (2020 only)
     * NOTE: Lanelet bounds need to be translated before calling this function! 
     * (If no stop line points are defined, bound end points define the stop line)
     * \param node A stopLine node
     * \param name The name of the node
     */
    std::optional<StopLine> translate_stopline(const xmlpp::Node* node, std::string name);

    /**
     * \brief This function translates a laneletType node to LaneletType (2020 only)
     * \param node A laneletType node
     * \param name The name of the node
     */
    LaneletType translate_lanelet_type(const xmlpp::Node* node, std::string name);

    /**
     * \brief This function translates a vehicleType node to VehicleType (2020 only)
     * \param node A vehicleType node
     * \param name The name of the node
     */
    std::vector<VehicleType> translate_users(const xmlpp::Node* node, std::string name);

    /**
     * \brief Translates a line marking string given from a node to a line marking Enum
     * \param line_node A line marking node
     */
    LineMarking translate_line_marking(const xmlpp::Node* line_node);

    //Helper functions
    /**
     * \brief Set drawing style of a drawn boundary
     * \param ctx Drawing context of the LCC's map view
     * \param line_marking Line marking style, e.g. dashed
     * \param dash_length Length of a dash
     */
    void set_boundary_style(const DrawingContext& ctx, std::optional<LineMarking> line_marking, double dash_length);
    /**
     * \brief Translate enum LaneletType to a string
     * \param lanelet_type The enum to translate to text
     */
    std::string to_text(LaneletType lanelet_type);
    /**
     * \brief Translate enum VehicleType to a string
     * \param vehicle_type The enum to translate to text
     */
    std::string to_text(VehicleType vehicle_type);

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs
     * \param node A lanelet node
     * \param traffic_sign_positions A map in which, during lanelet translation, lanelet ID and if position comes from a stop line are being stored
     * \param traffic_light_positions A map in which, during lanelet translation, lanelet ID and if position comes from a stop line are being stored
     * \param _draw_configuration A shared pointer pointing to the configuration for the scenario that sets which optional parts should be drawn
     */
    Lanelet(
        const xmlpp::Node* node, 
        std::map<int, std::pair<int, bool>>& traffic_sign_positions, 
        std::map<int, std::pair<int, bool>>& traffic_light_positions,
        std::shared_ptr<CommonroadDrawConfiguration> _draw_configuration
    );

    /**
     * \brief Iterate through the bounds, which should form pairs for each point (left and right)
     * Calculate distances
     * \return Min distance of opposing lane points
     */
    double get_min_width();

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

    /**
     * \brief This functions is supposed to be used to draw a lanelet which was given as a reference e.g. for position
     * This just draws a filled rectangle at the lanelet's position
     * Color, alpha value etc must be set beforehand
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     * \param global_orientation - optional: Rotation that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_x - optional: Translation in x-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_y - optional: Translation in y-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     */
    void draw_ref(const DrawingContext& ctx, double scale = 1.0, double global_orientation = 0.0, double global_translate_x = 0.0, double global_translate_y = 0.0);
    
    /**
     * \brief Not defined, but possible later on, to translate a lanelet to a DDS message
     */
    void to_dds_msg() {}

    //Getter (no setter, bc we do not want to manipulate data except for transformation)

    /**
     * \brief Get center (positional value) of the shape, if one exists
     * \return Center of the shape
     */
    std::pair<double, double> get_center() override;

    /**
     * \brief Get center of all points of the lanelet
     * \return Center of the shape of all points (get_center just takes a look at the two boundary points in the middle of the lanelet)
     */
    std::pair<double, double> get_center_of_all_points();

    /**
     * \brief Get center (positional value) of the stopline, if one exists
     * \return Center of the stopline
     */
    std::optional<std::pair<double, double>> get_stopline_center();

    /**
     * \brief Get the center of the end of the lanelet, e.g. useful for traffic lights
     * \return Center of the end points of the bounds
     */
    std::pair<double, double> get_end_center()

    /**
     * \brief Get min. and max. x and y value of all points of the lanelet, if such points exist
     * \return [[min_x, max_x], [min_y, max_y]]
     */
    std::optional<std::array<std::array<double, 2>, 2>> get_range_x_y();

    /**
     * \brief Get the lanelet shape as a polygon
     * \return Polygon of lanelet shape
     */
    std::vector<Point> get_shape();
};
