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

#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <sstream>
#include <vector>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

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

#include "commonroad_classes/CommonRoadTransformation.hpp"

#include "commonroad_classes/CommonroadDrawConfiguration.hpp"

#include "LCCErrorLogger.hpp"

#include "cpm/Writer.hpp"
#include "CommonroadDDSPlanningProblems.hpp"

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//TODO: Put Enums etc inside class definition??
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/**
 * \enum ObstacleType
 * \brief Obstacle types according to spec, for 2018 specs Obstacle
 */
enum class ObstacleRole {Static, Dynamic};

/**
 * \enum class ScenarioTag
 * \brief This enum class is also defined in commonroad and 'categorizes' the scenario type
 * From 2020 specs
 */
enum class ScenarioTag {
    Interstate, Highway, Urban, Comfort, Critical, Evasive, CutIn, IllegalCutIn, Intersection, LaneChange, LaneFollowing, MergingLanes,
    MultiLane, NoOncomingTraffic, OnComingTraffic, ParallelLanes, RaceTrack, Roundabout, Rural, Simulated, SingeLane, SlipRoad,
    SpeedLimit, TrafficJam, TurnLeft, TurnRight, TwoLane, EmergencyBraking
};

/**
 * \enum Attributes
 * \brief Expected attributes of this class according to specs; WARNING: Due to conformance to 2018-specs (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2018b.xsd) some things are redundant
 */
enum class Attribute {CommonRoadVersion, BenchmarkID, Date, Author, Affiliation, Source, TimeStepSize, Tags};

/**
 * \enum Elements
 * \brief Expected elements of this class according to specs
 * Obstacle: From 2018 specs
 */
enum class Element {Location, ScenarioTags, Lanelet, TrafficSign, TrafficLight, Intersection, StaticObstacle, DynamicObstacle, Obstacle, PlanningProblem};

/**
 * \struct Geo Transformation (2020 only)
 * \brief Holds additional location information for class CommonRoadScenario, most important: Scaling + translating the problem
 */
struct GeoTransformation 
{
    //Geo reference was left out, we only deal with additional transformation here
    //Data from additionalTransformation
    double x_translation = 0.0;
    double y_translation = 0.0;
    double z_rotation = 0.0;
    double scaling = 1.0; //Must be > 0
};

/**
 * \struct Location
 * \brief Holds location information for class CommonRoadScenario
 * Mostly relevant for UI, probably irrelevant for HLCs
 * Initial values in case it does not exist
 */
struct Location 
{
    std::optional<std::string> country; //outdated 2020 (removed from spec after some time)
    std::optional<std::string> federal_state; //outdated 2020
    int gps_latitude = -1;
    int gps_longitude = -1;
    std::optional<int> geo_name_id = -1;  //new 2020 (added to spec after some time)
    std::optional<std::string> zipcode = std::nullopt;
    std::optional<std::string> name = std::nullopt;
    std::optional<GeoTransformation> geo_transformation = std::nullopt;
    //For 2018 versions, this means that country / location information are missing too
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
    std::optional<Location> location = std::nullopt;
    //We store the IDs in the map and the object (in the object: for dds communication, if required)
    std::map<int, Lanelet> lanelets;
    std::map<int, TrafficSign> traffic_signs;
    std::map<int, TrafficLight> traffic_lights; //Was fixed in new 2020 specs, 0..1 in outdated specification was a mistake, see https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
    std::map<int, Intersection> intersections;
    std::map<int, StaticObstacle> static_obstacles;
    std::map<int, DynamicObstacle> dynamic_obstacles;
    std::map<int, PlanningProblem> planning_problems;

    //Lanelets may contain traffic sign / light IDs, whereas traffic signs / lights might not contain a position value, but can have that value
    //Thus, to draw these traffic "symbols", we need to combine the information that we can obtain from lanelet and the symbols
    //-> We remember the (last) lanelet-ID for each symbol-ID; if the symbol does not have its own location, it can look one up here
    //As transformations may take place, we cannot store the position, but just a lanelet reference to obtain the current position from there
    std::map<int, std::pair<int, bool>> lanelet_traffic_sign_positions; //<Sign ID, <Lanelet ID, is_stopline>>
    std::map<int, std::pair<int, bool>> lanelet_traffic_light_positions; //<Light ID, <Lanelet ID, is_stopline>>

    /**
     * Function used by a traffic sign to find out if a position was set for it by a lanelet definition
     * \param id ID of the traffic sign
     */
    std::optional<std::pair<double, double>> get_lanelet_sign_position(int id);

    /**
     * Function used by a traffic light to find out if a position was set for it by a lanelet definition
     * \param id ID of the traffic light
     */
    std::optional<std::pair<double, double>> get_lanelet_light_position(int id);

    //Not commonroad
    //Mutex to lock the object while it is being translated from an XML file
    std::mutex xml_translation_mutex;

    //Load / store translation in YAML
    CommonRoadTransformation yaml_transformation_storage;

    //Obstacle simulation callback functions (when new scenario is loaded)
    std::function<void()> setup_obstacle_sim_manager; 
    std::function<void()> reset_obstacle_sim_manager;

    //Obstacle aggregator callback function (when new scenario is loaded)
    std::function<void()> reset_obstacle_aggregator;

    //Configuration class for what optional parts to draw (gets default-constructed by using {})
    std::shared_ptr<CommonroadDrawConfiguration> draw_configuration{std::make_shared<CommonroadDrawConfiguration>()};

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

    /**
     * \brief Give an output (cout, string) of the translated scenario
     */
    void test_output();

    /**
     * \brief Clear all stored data -> May only be called when xml_translation_mutex is locked!
     */
    void clear_data();

    /**
     * Calculate the center (mean position) of the planning problem based on lanelets and obstacles
     * Only gets re-calculated whenever the problem is transformed or another problem is loaded
     */
    void calculate_center();
    std::pair<double, double> center;

    //Lanelet ref functions
    void draw_lanelet_ref(int lanelet_ref, const DrawingContext& ctx, double scale = 1.0, double global_orientation = 0.0, double global_translate_x = 0.0, double global_translate_y = 0.0);
    std::pair<double, double> get_lanelet_center(int id);

    /**
     * \brief This function is e.g. to center the imported XML scenario; it does NOT call the obstacle sim functions because they are to be called afterwards in main
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     * \param angle The angle with which to rotate around the origin, counter-clockwise, as specified by commonroad
     * \param scale Scales the whole coordinate system up / down
     */
    void transform_coordinate_system_helper(double translate_x, double translate_y, double angle = 0.0, double scale = 1.0);

    /**
     * \brief Calculates the scale for a given min. lane width
     * Should only be called within a locked-mutex section
     * \param min_lane_width Desired min. lane width
     * \return Scale factor to get the desired lane width, or a value <= 0.0 in case of an error
     */
    double get_scale(double min_lane_width);

public:
    /**
     * \brief The constructor itself just creates the data-storing object. It is filled with data using the load_file function
     */
    CommonRoadScenario();

    /**
     * \brief The scenario and the obstacle simulation are tightly connected: If a new scenario gets loaded, the obstacle simulation must be reset and set up again as well
     * \param _setup Set up the obstacle simulation manager with the newly translated scenario
     * \param _reset Reset the obstacle simulation manager (data structures, running threads etc.)
     */
    void register_obstacle_sim(std::function<void()> _setup, std::function<void()> _reset);

    /**
     * \brief The scenario and the obstacle aggregator are tightly connected: If a new scenario gets loaded, the obstacle aggregator must be reset and set up again as well
     * \param _reset Reset the obstacle aggregator's data structures
     */
    void register_obstacle_aggregator(std::function<void()> _reset);

    /**
     * \brief A load function to load another file
     * While the file is being loaded, other public functions are "skipped" (using try_lock mutex) when called
     * It gets an XML file and parses it once, translating it to the C++ data structure
     * From there on, the CommonRoadScenario Object can be used to access the scenario, send it to HLCs, fit it to the map etc
     * An error is thrown in case the XML file is invalid / does not match the expected CommonRoad specs
     * \param xml_filepath The path of the XML file that specificies the commonroad scenario
     * \param center_coordinates Center the coordinates of the scenario automatically
     */
    void load_file(std::string xml_filepath, bool center_coordinates = true);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * ORDER: As specified by commonroad: Translate, rotate, scale
     * \param lane_width The min lane width
     * \param angle The angle with which to rotate around the origin, counter-clockwise, as specified by commonroad
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    void transform_coordinate_system(double lane_width, double angle, double translate_x, double translate_y) override;

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
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg() {}

    /**
     * \brief Access to internal YAML transformation profile; apply changes stored in profile (again) for the current file
     */
    void apply_stored_transformation();
    /**
     * \brief Access to internal YAML transformation profile; Store current changes in profile for the current file
     */
    void store_applied_transformation();
    /**
     * \brief Access to internal YAML transformation profile; Reset changes stored in profile for the current file
     */
    void reset_stored_transformation();

    /**
     * \brief This getter returns the draw configuration class
     * By setting values in this class, you can toggle which parts of the selected scenario are supposed to be drawn
     */
    std::shared_ptr<CommonroadDrawConfiguration> get_draw_configuration();

    //TODO: Getter, by type and by ID, and constructor
    const std::string& get_author();
    const std::string& get_affiliation();
    const std::string& get_benchmark_id();
    const std::string& get_common_road_version();
    const std::string& get_date();
    const std::string& get_source();

    //We need to be able to get and set time_step_size, to change the speed of the simulation
    double get_time_step_size();
    void set_time_step_size(double new_time_step_size); 
    // const std::vector<const std::string>& get_scenario_tags_2018();
    // const std::vector<const ScenarioTag>& get_scenario_tags_2020();
    const std::optional<Location> get_location();

    std::vector<int> get_dynamic_obstacle_ids();
    std::optional<DynamicObstacle> get_dynamic_obstacle(int id);

    std::vector<int> get_static_obstacle_ids();
    std::optional<StaticObstacle> get_static_obstacle(int id);

    std::vector<int> get_planning_problem_ids();
    std::optional<PlanningProblem> get_planning_problem(int id);

    std::optional<Lanelet> get_lanelet(int id);

    /*****************************************************************************************/
    /******************                 DDS Functions               **************************/
    /*****************************************************************************************/
    /**
     * \brief Send all currently stored planning problems into the network / to the HLCs
     * \param writer_planning_problems DDS writer to send planning problems
     */
    void send_planning_problems(std::shared_ptr<cpm::Writer<CommonroadDDSPlanningProblemElement>> writer_planning_problems);
};
