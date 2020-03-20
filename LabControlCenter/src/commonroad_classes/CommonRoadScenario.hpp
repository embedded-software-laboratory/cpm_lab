#pragma once

#include <map>
#include <string>
#include <vector>

#include "commonroad_classes/Lanelet.hpp"
#include "commonroad_classes/TrafficSign.hpp"
#include "commonroad_classes/TrafficLight.hpp"
#include "commonroad_classes/Intersection.hpp"
#include "commonroad_classes/StaticObstacle.hpp"
#include "commonroad_classes/DynamicObstacle.hpp"
#include "commonroad_classes/PlanningProblem.hpp"

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
    int gpd_longitude;
    std::string zipcode;
    std::string name;
    //Geo transformation is left out, the location information itself is already probably only relevant for some part of the UI, not for the simulation itself
};

/**
 * \enum CommonRoadObject
 * \brief Holds all object types that are associated with an ID 
 * Used for an ID map to speed up search when an ID is looked up
 * Might be unecessary, as the search is type-dependent anyway
 */
enum CommonRoadObject
{
   Lanelet, TrafficSign, TrafficLight, Intersection, StaticObstacle, DynamicObstacle, PlanningProblem 
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

    //Commonroad data
    std::vector<std::string> tags; //Enum possible, but irrelevant for this implementation
    Location location;
    //We store the IDs in the map and the object (in the object: for dds communication, if required)
    std::map<int, Lanelet> lanelets;
    std::map<int, TrafficSign> traffic_signs;
    std::map<int, TrafficLight> traffic_lights; //0..1 in specification is a mistake, see https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
    std::map<int, Intersection> intersections;
    std::map<int, StaticObstacle> static_obstacles;
    std::map<int, DynamicObstacle> dynamic_obstacles;
    std::map<int, PlanningProblem> planning_problems;

    //'Help' data - map ID to commonroad type s.t. matching object can be found faster
    //CommonRoadObject defined in enum
    //Could make search faster, alternatively just look in all maps - @Max what would you prefer?
    std::map<int, CommonRoadObject> id_to_object;

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
     * TODO: Interface for transform instead of function in each object? Makes requirement stronger
     */
    void transform_to_lane_width(unsigned int width);

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * TODO: Change return type to whatever the name of the IDL type is
     * TODO: Interface for transform instead of function in each object? Makes requirement stronger
     */
    void to_dds_msg(); 

    //TODO: Getter, by type and by ID
};
