#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <vector>

#include "commonroad_classes/Lanelet.hpp"
#include "commonroad_classes/TrafficSign.hpp"
#include "commonroad_classes/TrafficLight.hpp"
#include "commonroad_classes/Intersection.hpp"
#include "commonroad_classes/StaticObstacle.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"
#include "commonroad_classes/PlanningProblem.hpp"

#include "commonroad_classes/XMLTranslation.hpp"

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//TODO: Put Enums etc inside class definition??
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * \enum ObstacleType
 * \brief Obstacle types according to spec, for 2018 specs Obstacle
 */
enum class ObstacleRole {Static, Dynamic, NotInSpec};

/**
 * \enum class ScenarioTag
 * \brief This enum class is also defined in commonroad and 'categorizes' the scenario type, NotInSpec for types that should not exist
 * From 2020 specs
 */
enum class ScenarioTag {
    Interstate, Highway, Urban, Comfort, Critical, Evasive, CutIn, IllegalCutIn, Intersection, LaneChange, LaneFollowing, MergingLanes,
    MultiLane, NoOncomingTraffic, OnComingTraffic, ParallelLanes, RaceTrack, Roundabout, Rural, Simulated, SingeLane, SlipRoad,
    SpeedLimit, TrafficJam, TurnLeft, TurnRight, TwoLane, NotInSpec
};

/**
 * \enum Attributes
 * \brief Expected attributes of this class according to specs; WARNING: Due to conformance to 2018-specs (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2018b.xsd) some things are redundant
 */
enum class Attribute {CommonRoadVersion, BenchmarkID, Date, Author, Affiliation, Source, TimeStepSize, Tags, NotInSpec};

/**
 * \enum Elements
 * \brief Expected elements of this class according to specs
 * Obstacle: From 2018 specs
 */
enum class Element {Location, ScenarioTags, Lanelet, TrafficSign, TrafficLight, Intersection, StaticObstacle, DynamicObstacle, Obstacle, PlanningProblem, NotInSpec};

/**
 * \struct Location
 * \brief Holds location information for class CommonRoadScenario
 * Mostly relevant for UI, probably irrelevant for HLCs
 */
struct Location 
{
    std::string country;
    std::string federal_state;
    int gps_latitude;
    int gps_longitude;
    std::string zipcode;
    std::string name;
    //Geo transformation is left out, the location information itself is already probably only relevant for some part of the UI, not for the simulation itself
    //For 2018 versions, this means that country / location information are missing too
};

/**
 * \class CommonRoadScenario
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a commonroad scenario specified in an XML file, which is loaded once and then replaced by more efficient C++ access methods (instead of parsing the XML over and over again)
 */
class CommonRoadScenario
{
private:
    //Meta information (only relevant maybe for UI except for time_step_size)
    std::string author;
    std::string affiliation;
    std::string benchmark_id;
    std::string common_road_version;
    std::string date;
    std::string source;
    uint64_t time_step_size;
    std::vector<std::string> tags; //From 2018 specs

    //Commonroad data
    std::vector<ScenarioTag> scenario_tags; //From 2020 specs
    Location location;
    //We store the IDs in the map and the object (in the object: for dds communication, if required)
    std::map<int, Lanelet> lanelets;
    std::map<int, TrafficSign> traffic_signs;
    std::map<int, TrafficLight> traffic_lights; //0..1 in specification is a mistake, see https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
    std::map<int, Intersection> intersections;
    std::map<int, StaticObstacle> static_obstacles;
    std::map<int, DynamicObstacle> dynamic_obstacles;
    std::map<int, PlanningProblem> planning_problems;

    //TODO: Both of these following functions as part of another interface?
    /**
     * \brief This function provides a translation of the node attributes in XML (as string) to one the expected node attributes of the root node (warning if non-existant)
     * \param node root_node
     */
    void translate_attributes(const xmlpp::Node* root_node);

    /**
     * \brief Parse the given xml node and store its contents in the according object
     * We only take a look at the scenario's children - these are then translated by using the appropriate function of their corresponding classes
     * \param node The root node (on first call), then, in recursions within the object's constructors, deeper nodes within the XML structure
     */
    void translate_element(const xmlpp::Node* node);

    /**
     * \brief Parse the given xml location node and store its contents 
     * \param node The location node 
     */
    void translate_location(const xmlpp::Node* node);

    /**
     * \brief Parse the given xml scenario tag node and store its contents 
     * \param node The scenario tag node 
     */
    void translate_scenario_tags(const xmlpp::Node* node);

    /**
     * \brief Get the obstacle role of an obstacle node
     * \param node Obstacle node
     */
    ObstacleRole get_obstacle_role(const xmlpp::Node* node);

public:
    /**
     * \brief The constructor gets an XML file and parses it once, translating it to the C++ data structure
     * From there on, the CommonRoadScenario Object can be used to access the scenario, send it to HLCs, fit it to the map etc
     * An error is thrown in case the XML file is invalid / does not match the expected CommonRoad specs (TODO: Custom error type for this case)
     * \param xml_filepath The path of the XML file that specificies the commonroad scenario
     */
    CommonRoadScenario(std::string xml_filepath);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * \param width The min. width of all lanes in the scenario
     */
    void transform_to_lane_width(double width) {}

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg() {}

    //TODO: Getter, by type and by ID, and constructor
};
