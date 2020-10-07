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

#include "commonroad_classes/CommonRoadScenario.hpp"

CommonRoadScenario::CommonRoadScenario()
{
    //TODO: Warn in case of unknown attributes set? E.g. if attribute list is greater than 8?

    //TODO: translate_element -> replace by behaviour like in translate_attributes, where we explicitly look up values?

    //TODO: Translate time step size to uint64_t - nanoseconds representation?
}

void CommonRoadScenario::test_output()
{
    //Test-print translation (unfinished, also print not yet finished parts later for a test if everything worked as expected)
    std::cout << "***************************************************************" << std::endl;
    std::cout << "TEST PRINT: Saved translation is: " << std::endl;
    std::cout << "***************************************************************" << std::endl;

    //Meta information (only relevant maybe for UI except for time_step_size)
    std::cout << "Author: " << author << std::endl;
    std::cout << "Affiliation: " << affiliation << std::endl;
    std::cout << "Benchmark ID: " << benchmark_id << std::endl;
    std::cout << "Version: " << common_road_version << std::endl;
    std::cout << "Date: " << date << std::endl;
    std::cout << "Source: " << source << std::endl;
    std::cout << "Time step size: " << time_step_size << std::endl;
    std::cout << "Tag (2018 specs): ";
    for (const auto tag : tags)
    {
        std::cout << "| " << tag;
    }
    std::cout << std::endl;

    //Commonroad data
    std::cout << "Tag size (2020 specs): " << scenario_tags.size() << std::endl;
    std::cout << "Location data: " << std::endl;
    if (location.has_value())
    {
        std::cout << "\tCountry: " << location.value().country.value_or("empty") << std::endl;
        std::cout << "\tFederal state :" << location.value().federal_state.value_or("empty") << std::endl;
        std::cout << "\tLatitude: " << location.value().gps_latitude << std::endl;
        std::cout << "\tLongitude: " << location.value().gps_longitude << std::endl;
        std::cout << "\tName: " << location.value().name.value_or("empty") << std::endl;
        std::cout << "\tZipcode: " << location.value().zipcode.value_or("empty") << std::endl;
    }
    else
    {
        std::cout << "\tNo location set" << std::endl;
    }
}

void CommonRoadScenario::clear_data()
{
    author.clear();
    affiliation.clear();
    benchmark_id.clear();
    common_road_version.clear();
    date.clear();
    source.clear();
    time_step_size = -1.0;
    tags.clear();
    scenario_tags.clear();
    location = std::optional<Location>();
    lanelets.clear();
    traffic_signs.clear();
    traffic_lights.clear();
    intersections.clear();
    static_obstacles.clear();
    dynamic_obstacles.clear();
    planning_problems.clear();

    if (reset_obstacle_sim_manager)
    {
        reset_obstacle_sim_manager();
    }
    if (reset_obstacle_aggregator)
    {
        reset_obstacle_aggregator();
    }
}

void CommonRoadScenario::register_obstacle_sim(std::function<void()> _setup, std::function<void()> _reset)
{
    setup_obstacle_sim_manager = _setup;
    reset_obstacle_sim_manager = _reset;
}

void CommonRoadScenario::register_obstacle_aggregator(std::function<void()> _reset)
{
    reset_obstacle_aggregator = _reset;
}

void CommonRoadScenario::load_file(std::string xml_filepath, bool center_coordinates)
{
    std::unique_lock<std::mutex> lock(xml_translation_mutex);

    //Delete all old data
    clear_data();

    //Translate new data
    xmlpp::DomParser parser;
    try
    {
        //Parse XML file (DOM parser)
        //xmlpp::KeepBlanks::KeepBlanks(false);
        //Ignore whitespaces (see http://xmlsoft.org/html/libxml-parser.html#xmlParserOption)
        parser.set_parser_options(256);
        parser.parse_file(xml_filepath);
        if(!parser) std::cerr << "Cannot parse file!" << std::endl;

        //Get parent node
        const auto pNode = parser.get_document()->get_root_node(); //deleted by DomParser.

        //Get desired parent node parts
        // const auto nodename = pNode->get_name();
        // const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(pNode);
        //Throw an error if the parent node does not meet the expectation (-> is not conform to commonroad specs)
        // if(nodename.empty() || !(nodeElement))
        // {
        //      TODO: Add custom error if desired, should lead to a translation error elsewhere though anyway
        // }
        // if (nodename.compare("commonRoad") != 0)
        // {
        // }

        //Store scenario attributes
        translate_attributes(pNode);

        //We want to go through the first layer of the CommonRoadScenario only - the objects that we want to store take the parsing from here 
        //Thus, we go through the children of the scenario and parse each of them using according constructors

        //TODO: Maybe use XMLTranslation units here as well instead!
        for(const auto& child : pNode->get_children())
        {
            translate_element(child);
        }

    }
    catch(const SpecificationError& e)
    {
        clear_data();
        throw SpecificationError(std::string("Could not translate CommonRoadScenario, file incompatible to specifications:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        clear_data();
        throw;
    }
    

    //test_output();

    //Now check if the file is spec-conformant (very basic version, just require existance of a field that is required by specs and part of the "commonRoad" element, which should thus exist as well)
    if (common_road_version.size() == 0 && author.size() == 0 && affiliation.size() == 0)
    {
        //-> One of the fields version, author, affiliation should be set (they are all required)
        clear_data();
        throw SpecificationError("Translation failed / Invalid XML file chosen. None of commonRoadVersion / author / affiliation information could be found in your XML file. Translation will not be used.");
    }
    else if (time_step_size == -1.0)
    {
        clear_data();
        throw SpecificationError("Translation failed / Invalid XML file chosen. Time step size must be set. Translation will not be used.");
    }
    else if (lanelets.size() == 0 && traffic_signs.size() == 0 && traffic_lights.size() == 0 && intersections.size() == 0 && static_obstacles.size() == 0 && dynamic_obstacles.size() == 0 && planning_problems.size() == 0)
    {
        //Check if all relevant fields are empty - reset the object in that case as well
        std::cerr << "WARNING: All relevant data fields are empty (except for version / author / affiliation)." << std::endl;
    }

    lock.unlock();

    //Calculate the center of the planning problem
    calculate_center();

    //Automatically center the planning problem
    if (center_coordinates)
    {
        transform_coordinate_system(- center.first, - center.second);
    }

    //Load new obstacle simulations
    if (setup_obstacle_sim_manager)
    {
        setup_obstacle_sim_manager();
    }
}

void CommonRoadScenario::translate_attributes(const xmlpp::Node* root_node)
{
    //If no value: Error is thrown anyway (set to true) - so in this case, we can directly use .value()
    common_road_version = xml_translation::get_attribute_text(root_node, "commonRoadVersion", true).value(); 
    benchmark_id = xml_translation::get_attribute_text(root_node, "benchmarkID", true).value();
    date = xml_translation::get_attribute_text(root_node, "date", true).value();
    author = xml_translation::get_attribute_text(root_node, "author", true).value();
    affiliation = xml_translation::get_attribute_text(root_node, "affiliation", true).value();
    source = xml_translation::get_attribute_text(root_node, "source", true).value();
    time_step_size = xml_translation::get_attribute_double(root_node, "timeStepSize", true).value();
    
    auto tags_list = xml_translation::get_attribute_text(root_node, "tags", false);
    if (tags_list.has_value())
    {
        //Only create tag list if it exists
        std::stringstream tag_stream(tags_list.value());
        std::string tag;
        while (std::getline(tag_stream, tag, ' '))
        {
            tags.push_back(tag);
        }
    }
}

using namespace std::placeholders;
void CommonRoadScenario::translate_element(const xmlpp::Node* node)
{
    //Find out which object we are dealing with, pass on translation to these objects (if possible)
    const auto node_name = node->get_name();

    if(node_name.empty())
    {
        //TODO: Throw error or keep it this way?
        std::stringstream error_stream;
        error_stream << "Node element empty in scenario parse, from line " << node->get_line();
        LCCErrorLogger::Instance().log_error(error_stream.str());
        return;
    }

    //nodename is the name of the node, e.g. "lanelet"
    //-> Switch based on node name!
    //get_attribute_int(node, "id", true).value() -> Throws error (set to true) if no ID exists, thus .value() can be used safely if no error was thrown
    if (node_name.compare("location") == 0)
    {
        translate_location(node);
    }
    else if (node_name.compare("scenarioTags") == 0)
    {
        translate_scenario_tags(node);
    }
    else if (node_name.compare("lanelet") == 0)
    {
        lanelets.insert({xml_translation::get_attribute_int(node, "id", true).value(), Lanelet(node)});
    }
    else if (node_name.compare("trafficSign") == 0)
    {
        traffic_signs.insert({xml_translation::get_attribute_int(node, "id", true).value(), TrafficSign(node)});
    }
    else if (node_name.compare("trafficLight") == 0)
    {
        traffic_lights.insert({xml_translation::get_attribute_int(node, "id", true).value(), TrafficLight(node)});
    }
    else if (node_name.compare("intersection") == 0)
    {
        intersections.insert({xml_translation::get_attribute_int(node, "id", true).value(), Intersection(node)});
    }
    else if (node_name.compare("staticObstacle") == 0)
    {
        static_obstacles.insert({
            xml_translation::get_attribute_int(node, "id", true).value(), 
            StaticObstacle(
                node,
                std::bind(&CommonRoadScenario::draw_lanelet_ref, this, _1, _2, _3, _4, _5, _6),
                std::bind(&CommonRoadScenario::get_lanelet_center, this, _1)
            )}
        );
    }
    else if (node_name.compare("dynamicObstacle") == 0)
    {
        dynamic_obstacles.insert({
            xml_translation::get_attribute_int(node, "id", true).value(), 
            DynamicObstacle(
                node,
                std::bind(&CommonRoadScenario::draw_lanelet_ref, this, _1, _2, _3, _4, _5, _6),
                std::bind(&CommonRoadScenario::get_lanelet_center, this, _1)
            )}
        );
    }
    else if (node_name.compare("obstacle") == 0)
    {
        ObstacleRole obstacle_role = get_obstacle_role(node);
        if (obstacle_role == ObstacleRole::Static)
        {
            static_obstacles.insert({
                xml_translation::get_attribute_int(node, "id", true).value(), 
                StaticObstacle(
                    node,
                    std::bind(&CommonRoadScenario::draw_lanelet_ref, this, _1, _2, _3, _4, _5, _6),
                    std::bind(&CommonRoadScenario::get_lanelet_center, this, _1)
                )}
            );
        }
        else if (obstacle_role == ObstacleRole::Dynamic)
        {
            dynamic_obstacles.insert({
                xml_translation::get_attribute_int(node, "id", true).value(), 
                DynamicObstacle(
                    node,
                    std::bind(&CommonRoadScenario::draw_lanelet_ref, this, _1, _2, _3, _4, _5, _6),
                    std::bind(&CommonRoadScenario::get_lanelet_center, this, _1)
                )}
            );
        }
        
    }
    else if (node_name.compare("planningProblem") == 0)
    {
        planning_problems.insert({
            xml_translation::get_attribute_int(node, "id", true).value(), 
            PlanningProblem(
                node,
                std::bind(&CommonRoadScenario::draw_lanelet_ref, this, _1, _2, _3, _4, _5, _6),
                std::bind(&CommonRoadScenario::get_lanelet_center, this, _1)
            )}
        );
    }
    else if (node_name.compare("comment") == 0)
    {
        //Ignore
    }
    else
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node element not conformant to specs (commonroad), node should not exist - name is: " << node_name << ", line is: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

void CommonRoadScenario::translate_location(const xmlpp::Node* node) 
{
    Location translated_location;
    translated_location.country = xml_translation::get_child_child_text(node, "country", false); 
    translated_location.federal_state = xml_translation::get_child_child_text(node, "federalState", false); 
    translated_location.gps_latitude = xml_translation::get_child_child_double(node, "gpsLatitude", true).value(); //If no value: Error is thrown anyway (set to true)
    translated_location.gps_longitude = xml_translation::get_child_child_double(node, "gpsLongitude", true).value(); //If no value: Error is thrown anyway (set to true)
    translated_location.zipcode = xml_translation::get_child_child_text(node, "zipcode", false);
    translated_location.name = xml_translation::get_child_child_text(node, "name", false);

    location = std::optional<Location>(translated_location);

    //TODO: Warning in case of unexpected nodes? (Except for geotransformation)
}

void CommonRoadScenario::translate_scenario_tags(const xmlpp::Node* node) 
{
    for(const auto& child : node->get_children())
    {
        const auto node_name = child->get_name();

        if (node_name.compare("interstate") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Interstate);
        }
        else if (node_name.compare("highway") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Highway);
        }
        else if (node_name.compare("urban") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Urban);
        }   
        else if (node_name.compare("comfort") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Comfort);
        }
        else if (node_name.compare("critical") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Critical);
        }
        else if (node_name.compare("evasive") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Evasive);
        }
        else if (node_name.compare("cut_in") == 0)
        {
            scenario_tags.push_back(ScenarioTag::CutIn);
        }
        else if (node_name.compare("illegal_cutin") == 0)
        {
            scenario_tags.push_back(ScenarioTag::IllegalCutIn);
        }
        else if (node_name.compare("intersection") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Intersection);
        }
        else if (node_name.compare("lane_change") == 0)
        {
            scenario_tags.push_back(ScenarioTag::LaneChange);
        }
        else if (node_name.compare("lane_following") == 0)
        {
            scenario_tags.push_back(ScenarioTag::LaneFollowing);
        }
        else if (node_name.compare("merging_lanes") == 0)
        {
            scenario_tags.push_back(ScenarioTag::MergingLanes);
        }
        else if (node_name.compare("multi_lane") == 0)
        {
            scenario_tags.push_back(ScenarioTag::MultiLane);
        }
        else if (node_name.compare("no_oncoming_traffic") == 0)
        {
            scenario_tags.push_back(ScenarioTag::NoOncomingTraffic);
        }
        else if (node_name.compare("on_coming_traffic") == 0)
        {
            scenario_tags.push_back(ScenarioTag::OnComingTraffic);
        }
        else if (node_name.compare("parallel_lanes") == 0)
        {
            scenario_tags.push_back(ScenarioTag::ParallelLanes);
        }
        else if (node_name.compare("race_track") == 0)
        {
            scenario_tags.push_back(ScenarioTag::RaceTrack);
        }
        else if (node_name.compare("roundabout") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Roundabout);
        }
        else if (node_name.compare("rural") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Rural);
        }
        else if (node_name.compare("simulated") == 0)
        {
            scenario_tags.push_back(ScenarioTag::Simulated);
        }
        else if (node_name.compare("single_lane") == 0)
        {
            scenario_tags.push_back(ScenarioTag::SingeLane);
        }
        else if (node_name.compare("slip_road") == 0)
        {
            scenario_tags.push_back(ScenarioTag::SlipRoad);
        }
        else if (node_name.compare("speed_limit") == 0)
        {
            scenario_tags.push_back(ScenarioTag::SpeedLimit);
        }
        else if (node_name.compare("traffic_jam") == 0)
        {
            scenario_tags.push_back(ScenarioTag::TrafficJam);
        }
        else if (node_name.compare("turn_left") == 0)
        {
            scenario_tags.push_back(ScenarioTag::TurnLeft);
        }
        else if (node_name.compare("turn_right") == 0)
        {
            scenario_tags.push_back(ScenarioTag::TurnRight);
        }
        else if (node_name.compare("two_lane") == 0)
        {
            scenario_tags.push_back(ScenarioTag::TwoLane);
        }
        else
        {
            std::stringstream error_stream;
            error_stream << "Unspecified scenario tag in scenario parse ignored, from line " << node->get_line();
            LCCErrorLogger::Instance().log_error(error_stream.str());
        }
    }
}

ObstacleRole CommonRoadScenario::get_obstacle_role(const xmlpp::Node* node)
{
    //Role is itself an element node, with its content in another child node
    std::string role_text = xml_translation::get_child_child_text(node, "role", true).value(); //Must exist, else error is thrown
    
    if (role_text.compare("static") == 0)
    {
        return ObstacleRole::Static;
    }
    else if (role_text.compare("dynamic") == 0)
    {
        return ObstacleRole::Dynamic;
    }
    else
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Could not translate obstacle role, incompatible to specifications (static, dynamic), in obstacle, line: " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

void CommonRoadScenario::transform_coordinate_system(double translate_x, double translate_y) 
{
    //Do not block the UI if locked, needs to be done again then
    if (xml_translation_mutex.try_lock())
    {
        double scale = 1.0;
        if (translate_x != 0.0 || translate_y != 0.0)
        {
            for (auto &lanelet_entry : lanelets)
            {
                lanelet_entry.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &static_obstacle : static_obstacles)
            {
                static_obstacle.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &dynamic_obstacle : dynamic_obstacles)
            {
                dynamic_obstacle.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &planning_problem : planning_problems)
            {
                planning_problem.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &traffic_sign : traffic_signs)
            {
                traffic_sign.second.transform_coordinate_system(scale, translate_x, translate_y);
            } 

            for (auto &traffic_light : traffic_lights)
            {
                traffic_light.second.transform_coordinate_system(scale, translate_x, translate_y);
            } 

            //Update center
            std::cout << "New center after transformation:" << std::endl;
            calculate_center();
        }

        xml_translation_mutex.unlock();
    }
}

/******************************Interface functions***********************************/

void CommonRoadScenario::transform_coordinate_system(double lane_width, double translate_x, double translate_y) 
{
    //Do not block the UI if locked, needs to be done again then
    if (xml_translation_mutex.try_lock())
    {
        //Get current min. lane width of lanelets (calculated from point distances)
        double min_width = -1.0;
        for (auto lanelet : lanelets)
        {
            double new_min_width = lanelet.second.get_min_width();
            if (min_width < 0.0 || new_min_width < min_width)
            {
                min_width = new_min_width;
            }
        }

        //Scale can be smaller than 0 - in this case, it is simply not applied (functions are still called for translate_x and translate_y)
        double scale = lane_width / min_width;
        if (scale > 0 || translate_x != 0.0 || translate_y != 0.0)
        {
            for (auto &lanelet_entry : lanelets)
            {
                lanelet_entry.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &static_obstacle : static_obstacles)
            {
                static_obstacle.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &dynamic_obstacle : dynamic_obstacles)
            {
                dynamic_obstacle.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &planning_problem : planning_problems)
            {
                planning_problem.second.transform_coordinate_system(scale, translate_x, translate_y);
            }

            for (auto &traffic_sign : traffic_signs)
            {
                traffic_sign.second.transform_coordinate_system(scale, translate_x, translate_y);
            } 

            for (auto &traffic_light : traffic_lights)
            {
                traffic_light.second.transform_coordinate_system(scale, translate_x, translate_y);
            } 

            //Update center
            calculate_center();
        }
        else if (min_width < 0)
        {
            std::stringstream error_stream;
            error_stream << "Could not transform scenario coordinate system to min lane width - no lanelets defined";
            LCCErrorLogger::Instance().log_error(error_stream.str());
        }

        xml_translation_mutex.unlock();

        //Need to reset the simulation and aggregator as well (as the coordinate system was changed)
        if (reset_obstacle_sim_manager)
        {
            reset_obstacle_sim_manager();
        }
        if (setup_obstacle_sim_manager)
        {
            setup_obstacle_sim_manager();
        }
    }
}

void CommonRoadScenario::set_time_step_size(double new_time_step_size)
{
    //Only accept physically meaningful & useful values
    if (new_time_step_size <= 0) return;

    std::unique_lock<std::mutex> lock(xml_translation_mutex);
    double time_scale = time_step_size / new_time_step_size;
    time_step_size = new_time_step_size;

    //Change velocity, acceleration
    for (auto &planning_problem : planning_problems)
    {
        planning_problem.second.transform_timing(time_scale);
    }

    lock.unlock();

    //Need to reset the simulation and aggregator as well (as the timing was changed)
    if (reset_obstacle_sim_manager)
    {
        reset_obstacle_sim_manager();
    }
    if (setup_obstacle_sim_manager)
    {
        setup_obstacle_sim_manager();
    }
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void CommonRoadScenario::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    if (xml_translation_mutex.try_lock())
    {
        //Draw lanelets
        ctx->save();

        //Perform required translation + rotation
        //Local orientation is irrelevant here (Use global orientation if you want to change the orientation of the whole scenario. Local orientation is only used for e.g. shapes, where this actually makes sense.)
        ctx->translate(global_translate_x, global_translate_y);
        ctx->rotate(global_orientation);

        ctx->set_source_rgb(0.5,0.5,0.5);
        for (auto &lanelet_entry : lanelets)
        {
            lanelet_entry.second.draw(ctx, scale);
        }

        for (auto &static_obstacle : static_obstacles)
        {
            static_obstacle.second.draw(ctx, scale);
        }

        for (auto &dynamic_obstacle : dynamic_obstacles)
        {
            dynamic_obstacle.second.draw(ctx, scale);
        }

        for (auto &planning_problem : planning_problems)
        {
            planning_problem.second.draw(ctx, scale);
        }

        for (auto &traffic_sign : traffic_signs)
        {
            traffic_sign.second.draw(ctx, scale);
        } 

        for (auto &traffic_light : traffic_lights)
        {
            traffic_light.second.draw(ctx, scale);
        } 

        //TODO: Intersections - do these need to be drawn specifically? They are already visible, because they are based on references only; but: Would allow to draw arrows on e.g. crossings to successor roads

        ctx->restore();

        xml_translation_mutex.unlock();
    }
}
#pragma GCC diagnostic pop

void CommonRoadScenario::draw_lanelet_ref(int lanelet_ref, const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y)
{
    //Mutex locking not necessary / possible here (called within draw from other objects)

    auto lanelet_it = lanelets.find(lanelet_ref);
    if (lanelet_it != lanelets.end())
    {
        ctx->save();
        //ctx->set_source_rgba(0, 0.7, 0.7, 0.7); -> Better set the color yourself before calling draw_...
        lanelet_it->second.draw_ref(ctx, scale, global_orientation, global_translate_x, global_translate_y);
        ctx->restore();
    }
    else
    {
        std::stringstream error_stream;
        error_stream << "Lanelet reference not found in draw_lanelet_ref! Did you set the right ref in the scenario?";
        LCCErrorLogger::Instance().log_error(error_stream.str());
    }
}


/******************************Getter***********************************/

//This one is private
void CommonRoadScenario::calculate_center()
{
    //As for most functions in this class, this function should also only be called after locking the translation mutex, as translation might be performed by another thread

    //Init center
    center = std::pair<double, double>(0.0, 0.0);

    //Working with numeric limits at start lead to unforseeable behaviour with min and max, thus we now use this approach instead
    bool uninitialized = true;
    double x_min, x_max, y_min, y_max;
    for (auto lanelet : lanelets)
    {
        auto x_y_range = lanelet.second.get_range_x_y();

        if (x_y_range.has_value())
        {
            if (uninitialized)
            {
                x_min = x_y_range.value()[0][0];
                x_max = x_y_range.value()[0][1];
                y_min = x_y_range.value()[1][0];
                y_max = x_y_range.value()[1][1];

                uninitialized = false;
            }
            else
            {
                x_min = std::min(x_min, x_y_range.value()[0][0]);
                x_max = std::max(x_max, x_y_range.value()[0][1]);
                y_min = std::min(y_min, x_y_range.value()[1][0]);
                y_max = std::max(y_max, x_y_range.value()[1][1]);
            }
        }
    }

    std::cout << "New x_min: " << x_min << std::endl;
    std::cout << "New x_max: " << x_max << std::endl;
    std::cout << "New y_min: " << y_min << std::endl;
    std::cout << "New y_max: " << y_max << std::endl;

    //Set values to zero if no values could be found in any of the lanelets
    if (lanelets.size() == 0 || uninitialized)
    {
        x_min = 0.0;
        x_max = 0.0;
        y_min = 0.0;
        y_max = 0.0;
    }

    center.first = (0.5 * x_min) + (0.5 * x_max);
    center.second = (0.5 * y_min) + (0.5 * y_max);

    //Center should be w.r.t. LCC IPS coordinates, which has its center at (2.25, 2), not at (0, 0)
    center.first -= 2.25;
    center.second -= 2.0;

    std::cout << "New center: " << center.first << ", " << center.second << std::endl;
}

const std::string& CommonRoadScenario::get_author()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return author;
}

const std::string& CommonRoadScenario::get_affiliation()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return affiliation;
}

const std::string& CommonRoadScenario::get_benchmark_id()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return benchmark_id;
}

const std::string& CommonRoadScenario::get_common_road_version()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return common_road_version;
}

const std::string& CommonRoadScenario::get_date()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return date;
}

const std::string& CommonRoadScenario::get_source()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return source;
}

double CommonRoadScenario::get_time_step_size()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return time_step_size;
}

// const std::vector<const std::string>& CommonRoadScenario::get_scenario_tags_2018()
// {
//     return tags;
// }

// const std::vector<const ScenarioTag>& CommonRoadScenario::get_scenario_tags_2020()
// {
//     return scenario_tags;
// }

const std::optional<Location> CommonRoadScenario::get_location()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    return location;
}

std::vector<int> CommonRoadScenario::get_dynamic_obstacle_ids()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    std::vector<int> ids;
    for (const auto& entry : dynamic_obstacles)
    {
        ids.push_back(entry.first);
    }

    return ids;
}

std::optional<DynamicObstacle> CommonRoadScenario::get_dynamic_obstacle(int id)
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    //TODO: Alternative: Return DynamicObstacle& for performance reasons and throw error if id does not exist in map
    if (dynamic_obstacles.find(id) != dynamic_obstacles.end())
    {
        return std::optional<DynamicObstacle>(dynamic_obstacles.at(id));
    }
    return std::nullopt;
}

std::vector<int> CommonRoadScenario::get_static_obstacle_ids()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    std::vector<int> ids;
    for (const auto& entry : static_obstacles)
    {
        ids.push_back(entry.first);
    }

    return ids;
}

std::optional<StaticObstacle> CommonRoadScenario::get_static_obstacle(int id)
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    if (static_obstacles.find(id) != static_obstacles.end())
    {
        return std::optional<StaticObstacle>(static_obstacles.at(id));
    }
    return std::nullopt;
}


std::vector<int> CommonRoadScenario::get_planning_problem_ids()
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    std::vector<int> ids;
    for (const auto& entry : planning_problems)
    {
        ids.push_back(entry.first);
    }

    return ids;
}

std::optional<PlanningProblem> CommonRoadScenario::get_planning_problem(int id)
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    if (planning_problems.find(id) != planning_problems.end())
    {
        return std::optional<PlanningProblem>(planning_problems.at(id));
    }
    return std::nullopt;
}


std::optional<Lanelet> CommonRoadScenario::get_lanelet(int id)
{
    std::lock_guard<std::mutex> lock(xml_translation_mutex);
    //TODO: Alternative: Return Lanelet& for performance reasons and throw error if id does not exist in map
    if (lanelets.find(id) != lanelets.end())
    {
        return std::optional<Lanelet>(lanelets.at(id));
    }
    return std::nullopt;
}

std::pair<double, double> CommonRoadScenario::get_lanelet_center(int id)
{
    //Mutex locking not necessary / possible here (called within draw from other objects)

    auto lanelet_it = lanelets.find(id);
    if (lanelet_it != lanelets.end())
    {
        return lanelet_it->second.get_center();
    }
    else
    {
        std::stringstream error_stream;
        error_stream << "Lanelet reference not found in draw_lanelet_ref! Did you set the right ref in the scenario?";
        LCCErrorLogger::Instance().log_error(error_stream.str());
        return {0, 0};
    }
}