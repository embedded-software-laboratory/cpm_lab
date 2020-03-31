#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <string>
#include <vector>

#include "commonroad_classes/geometry/Point.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \enum class LaneletType
 * \brief Stores lanelet type, as in spec; NotInSpec for types that should not exist
 */
enum class LaneletType {
    Unspecified, Urban, Country, Highway, Sidewalk, Crosswalk, BusLane, BicycleLane, ExitRamp, MainCarriageWay, AccessRamp, DriveWay, BusStop, NotInSpec
};

/**
 * \enum class VehicleType
 * \brief Stores lanelet type, as in spec; NotInSpec for types that should not exist
 */
enum class VehicleType {
    Vehicle, Car, Truck, Bus, Motorcycle, Bicycle, Pedestrian, PriorityVehicle, Train, NotInSpec
};

/**
 * \enum class DrivingDirection
 * \brief Stores driving direction, used by Adjacent; NotInSpec for types that should not exist
 */
enum class DrivingDirection {
    Same, Opposite, NotInSpec
};

/**
 * \enum class LineMarking
 * \brief Holds all line marking types defined by the specification, used by Bound; NotInSpec for types that should not exist
 */
enum class LineMarking {
    Unspecified, Dashed, Solid, BroadDashed, BroadSolid, NotInSpec
};

/**
 * \struct Adjacent
 * \brief Holds information on adjacent road tiles and their driving direction
 * Initial values in case it does not exist
 */
struct Adjacent
{
    int ref_id = -1;
    DrivingDirection direction;

    //In case it does not exist, not part of specs
    bool exists = false;
};

/**
 * \struct Bound
 * \brief Bound of a lanelet, defined by points and marking, e.g. dashed if adjacent to another lanelet
 */
struct Bound
{
    std::vector<Point> points; //min. 2
    LineMarking line_marking;
};

/**
 * \struct StopLine
 * \brief Defines position of a stop line on the lanelet (w. possible reference to traffic signs etc)
 * Initial value in case it does not exist
 */
struct StopLine
{
    std::vector<Point> points; //Must consist of exactly two points
    LineMarking line_marking;
    std::vector<int> traffic_sign_refs; //trafficsignref
    std::vector<int> traffic_light_ref; //only one possible, but easier to handle if nonexistent, trafficlightref

    //In case it does not exist, not part of specs
    bool exists = false;
};

/**
 * \class Lanelet
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store Lanelets specified in the XML file (segments of a lane, with references to following and adjacent lanes)
 */
class Lanelet : public InterfaceTransform
{
private:
    Bound left_bound;
    Bound right_bound;
    std::vector<int> predecessors; //Multiple possible e.g. in case of a fork; laneletref
    std::vector<int> successors;   //Multiple possible e.g. in case of a fork; laneletref
    Adjacent adjacent_left;  //-1 if empty? TODO!
    Adjacent adjacent_right; //-1 if empty? TODO!
    StopLine stop_line;
    LaneletType lanelet_type; //enum class possible
    std::vector<VehicleType> user_one_way; //enum class possible
    std::vector<VehicleType> user_bidirectional; //enum class possible
    std::vector<int> traffic_sign_refs; //trafficsignref
    std::vector<int> traffic_light_refs; //trafficlightref
    double speed_limit = -1.0; //From 2018 specs, must not be set (-1 if empty?)

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs (TODO: Custom error type for this case)
     * \param node A lanelet node
     */
    Lanelet(const xmlpp::Node* node);

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
    Adjacent translate_adjacent(const xmlpp::Node* node, std::string name);

    /**
     * \brief This function translates a stopLine node to StopLine (2020 only)
     * \param node A stopLine node
     * \param name The name of the node
     */
    StopLine translate_stopline(const xmlpp::Node* node, std::string name);

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

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}
    
    void to_dds_msg() {}

    //TODO: Getter (no setter, bc we do not want to manipulate data except for transformation)
};
