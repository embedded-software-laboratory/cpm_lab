#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <iostream>
#include <map>
#include <mutex>
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

#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/InterfaceDraw.hpp"
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
 * Initial values in case it does not exist
 */
struct Location 
{
    std::string country;
    std::string federal_state;
    int gps_latitude = -1;
    int gps_longitude = -1;
    std::string zipcode;
    std::string name;
    //Geo transformation is left out, the location information itself is already probably only relevant for some part of the UI, not for the simulation itself
    //For 2018 versions, this means that country / location information are missing too

    //In case it does not exist, not part of specs
    bool exists = false;
};

/**
 * \class CommonRoadScenario
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a commonroad scenario specified in an XML file, which is loaded once and then replaced by more efficient C++ access methods (instead of parsing the XML over and over again)
 */
class CommonRoadScenario : public InterfaceTransform, public InterfaceDraw
{
private:
    //Meta information (only relevant maybe for UI except for time_step_size)
    std::string author;
    std::string affiliation;
    std::string benchmark_id;
    std::string common_road_version;
    std::string date;
    std::string source;
    double time_step_size = -1.0;
    std::vector<std::string> tags; //From 2018 specs

    //Commonroad data
    std::vector<ScenarioTag> scenario_tags; //From 2020 specs
    Location location; //TODO: Optional?
    //We store the IDs in the map and the object (in the object: for dds communication, if required)
    std::map<int, Lanelet> lanelets;
    std::map<int, TrafficSign> traffic_signs;
    std::map<int, TrafficLight> traffic_lights; //0..1 in specification is a mistake, see https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
    std::map<int, Intersection> intersections;
    std::map<int, StaticObstacle> static_obstacles;
    std::map<int, DynamicObstacle> dynamic_obstacles;
    std::map<int, PlanningProblem> planning_problems;

    //Not commonroad
    //Mutex to lock the object while it is being translated from an XML file
    std::mutex xml_translation_mutex;

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
     * \brief A load function to load another file
     * While the file is being loaded, other public functions are "skipped" (using try_lock mutex) when called
     * \param xml_filepath The path of the XML file that specificies the commonroad scenario
     */
    void load_file(std::string xml_filepath);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The min lane width
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    void transform_coordinate_system(double lane_width, double translate_x, double translate_y) override;

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

    void draw_lanelet_ref(int lanelet_ref, const DrawingContext& ctx, double scale = 1.0, double global_orientation = 0.0, double global_translate_x = 0.0, double global_translate_y = 0.0, double local_orientation = 0.0);

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg() {}

    //TODO: Getter, by type and by ID, and constructor
};
