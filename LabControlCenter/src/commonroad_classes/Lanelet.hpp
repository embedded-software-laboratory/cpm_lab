#pragma once

#include <string>
#include <vector>

#include "commonroad_classes/geometry/Point.hpp"

/**
 * \enum DrivingDirection
 * \brief Stores driving direction, used by Adjacent
 */
enum DrivingDirection {
    Same, Opposite
};

/**
 * \enum LineMarking
 * \brief Holds all line marking types defined by the specification, used by Bound
 */
enum LineMarking {
    Unspecified, Dashed, Solid, BroadDashed, BroadSolid
};

/**
 * \struct Adjacent
 * \brief Holds information on adjacent road tiles and their driving direction
 */
struct Adjacent
{
    int ref_id;
    DrivingDirection direction;
};

/**
 * \struct Bound
 * \brief Bound of a lanelet, defined by points and marking, e.g. dashed if adjacent to another lanelet
 */
struct Bound
{
    std::vector<Point> points;
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
    std::vector<int> traffic_sign_refs;
    int traffic_light_ref; //-1 if empty?
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
    std::vector<int> predecessors; //Multiple possible e.g. in case of a fork
    std::vector<int> successors;   //Multiple possible e.g. in case of a fork
    Adjacent adjacent_left;  //-1 if empty? TODO!
    Adjacent adjacent_right; //-1 if empty? TODO!
    StopLine stop_line;
    std::string lanelet_type; //Enum possible
    std::vector<std::string> user_one_way; //Enum possible
    std::vector<std::string> user_bidirectional; //Enum possible
    std::vector<int> traffic_sign_refs;
    std::vector<int> traffic_light_refs;

public:
    //TODO: Constructor

    //TODO: From interface
    void transform_to_lane_width(unsigned int width);
    void to_dds_msg(); 

    //TODO: Getter (no setter, bc we do not want to manipulate data except for transformation)
};
