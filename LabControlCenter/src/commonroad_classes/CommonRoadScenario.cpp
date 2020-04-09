#include "commonroad_classes/CommonRoadScenario.hpp"

CommonRoadScenario::CommonRoadScenario(std::string xml_filepath)
{
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
        const auto nodename = pNode->get_name();
        const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(pNode);

        //Throw an error if the parent node does not meet the expectation (-> is not conform to commonroad specs)
        if(nodename.empty() || !(nodeElement))
        {
            //TODO: Throw error
        }
        if (nodename.compare("commonRoad") != 0)
        {
            //TODO: Throw error
        }

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
    catch(const std::exception& ex)
    {
        std::cerr << "Exception caught: " << ex.what() << std::endl;
    }

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
    std::cout << "\tCountry: " << location.country << std::endl;
    std::cout << "\tFederal state :" << location.federal_state << std::endl;
    std::cout << "\tLatitude: " << location.gps_latitude << std::endl;
    std::cout << "\tLongitude: " << location.gps_longitude << std::endl;
    std::cout << "\tName: " << location.name << std::endl;
    std::cout << "\tZipcode: " << location.zipcode << std::endl;

    //TODO: Warn in case of unknown attributes set? E.g. if attribute list is greater than 8?

    //TODO: translate_element -> replace by behaviour like in translate_attributes, where we explicitly look up values?

    //TODO: Translate time step size to uint64_t - nanoseconds representation?
}

void CommonRoadScenario::translate_attributes(const xmlpp::Node* root_node)
{
    common_road_version = xml_translation::get_attribute_text(root_node, "commonRoadVersion", true);
    benchmark_id = xml_translation::get_attribute_text(root_node, "benchmarkID", true);
    date = xml_translation::get_attribute_text(root_node, "date", true);
    author = xml_translation::get_attribute_text(root_node, "author", true);
    affiliation = xml_translation::get_attribute_text(root_node, "affiliation", true);
    source = xml_translation::get_attribute_text(root_node, "source", true);
    time_step_size = static_cast<uint64_t>(xml_translation::get_attribute_double(root_node, "timeStepSize", true));
    
    std::string tags_list = xml_translation::get_attribute_text(root_node, "tags", false);
    if (tags_list != "empty")
    {
        //Only create tag list if it exists
        std::stringstream tag_stream(tags_list);
        std::string tag;
        while (std::getline(tag_stream, tag, ' '))
        {
            tags.push_back(tag);
        }
    }
}

void CommonRoadScenario::translate_element(const xmlpp::Node* node)
{
    //Find out which object we are dealing with, pass on translation to these objects (if possible)
    const auto node_name = node->get_name();

    if(node_name.empty())
    {
        //TODO: Throw error
        std::cerr << "TODO: Better warning // Node element empty in scenario parse" << std::endl;
        return;
    }

    //nodename is the name of the node, e.g. "lanelet"
    //-> Switch based on node name!
    if (node_name.compare("location") == 0)
    {
        location.exists = true;
        translate_location(node);
    }
    else if (node_name.compare("scenarioTags") == 0)
    {
        translate_scenario_tags(node);
    }
    else if (node_name.compare("lanelet") == 0)
    {
        lanelets.insert({xml_translation::get_attribute_int(node, "id"), Lanelet(node)});
    }
    else if (node_name.compare("trafficSign") == 0)
    {
        traffic_signs.insert({xml_translation::get_attribute_int(node, "id"), TrafficSign(node)});
    }
    else if (node_name.compare("trafficLight") == 0)
    {
        traffic_lights.insert({xml_translation::get_attribute_int(node, "id"), TrafficLight(node)});
    }
    else if (node_name.compare("intersection") == 0)
    {
        //intersections[xml_translation::get_attribute_int(node, "id")] = Intersection(node);
    }
    else if (node_name.compare("staticObstacle") == 0)
    {
        //static_obstacles[xml_translation::get_attribute_int(node, "id")] = StaticObstacle(node);
    }
    else if (node_name.compare("dynamicObstacle") == 0)
    {
        //dynamic_obstacles[xml_translation::get_attribute_int(node, "id")] = DynamicObstacle(node);
    }
    else if (node_name.compare("obstacle") == 0)
    {
        ObstacleRole obstacle_role = get_obstacle_role(node);
        if (obstacle_role == ObstacleRole::Static)
        {
            //static_obstacles[xml_translation::get_attribute_int(node, "id")] = StaticObstacle(node);
        }
        else if (obstacle_role == ObstacleRole::Dynamic)
        {
            //dynamic_obstacles[xml_translation::get_attribute_int(node, "id")] = DynamicObstacle(node);
        }
        
    }
    else if (node_name.compare("planningProblem") == 0)
    {
        planning_problems.insert({xml_translation::get_attribute_int(node, "id"), PlanningProblem(node)});
    }
    else if (node_name.compare("comment") == 0)
    {
        //Ignore
    }
    else
    {
        std::cerr << "TODO: Better warning // Node element not conformant to specs (commonroad) - name is: " << node_name << std::endl;
    }
}

void CommonRoadScenario::translate_location(const xmlpp::Node* node) 
{
    location.country = xml_translation::get_child_child_text(node, "country", true);
    location.federal_state = xml_translation::get_child_child_text(node, "federalState", true);
    location.gps_latitude = xml_translation::get_child_child_double(node, "gpsLatitude", true);
    location.gps_longitude = xml_translation::get_child_child_double(node, "gpsLongitude", true);
    location.zipcode = xml_translation::get_child_child_text(node, "zipcode", false);
    location.name = xml_translation::get_child_child_text(node, "name", false);

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
            std::cerr << "TODO: Better warning // Unspecified scenario tag, ignored" << std::endl;
        }
    }
}

ObstacleRole CommonRoadScenario::get_obstacle_role(const xmlpp::Node* node)
{
    //Role is itself an element node, with its content in another child node
    //Possible TODO: Catch error? role_node might not have any child
    const xmlpp::Node* role_node = node->get_first_child("role");
    std::string role_text = xml_translation::get_first_child_text(role_node);
    
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
        std::cerr << "TODO: Better warning // Node element not conformant to specs (obstacle)" << std::endl;
        return ObstacleRole::NotInSpec;
    }
}

/******************************Interface functions***********************************/

void CommonRoadScenario::draw(const DrawingContext& ctx, double scale)
{
    //Draw lanelets
    ctx->save();
    ctx->set_source_rgb(0,0,1.0);
    for (auto lanelet_entry : lanelets)
    {
        lanelet_entry.second.draw(ctx, scale);
    }
    ctx->restore();
}