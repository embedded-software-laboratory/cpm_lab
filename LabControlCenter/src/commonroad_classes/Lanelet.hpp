#pragma once

#include <string>
#include <vector>

#include "commonroad_classes/geometry/Point.hpp"

/**
 * \enum class LaneletType
 * \brief Stores lanelet type, as in spec; NotInSpec for types that should not exist
 */
enum class LaneletType {
    Urban, Country, Highway, Sidewalk, Crosswalk, BusLane, BicycleLane, ExitRamp, MainCarriageWay, AccessRamp, DriveWay, BusStop, NotInSpec
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
 */
struct Adjacent
{
    int ref_id;
    DrivingDirection direction;

    //In case it does not exist, not in spec
    bool exists;
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
 */
struct StopLine
{
    Point point_1;
    Point point_2;
    LineMarking line_marking;
    std::vector<int> traffic_sign_refs; //trafficsignref
    std::vector<int> traffic_light_ref; //only one possible, but easier to handle if nonexistent, trafficlightref
};

/**
 * \class Lanelet
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store Lanelets specified in the XML file (segments of a lane, with references to following and adjacent lanes)
 */
class Lanelet
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

public:
    //TODO: Constructor

    //TODO: From interface
    void transform_to_lane_width(unsigned int width) {}
    void to_dds_msg() {}

    //TODO: Getter (no setter, bc we do not want to manipulate data except for transformation)
};
