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
        for (const auto& attribute : nodeElement->get_attributes())
        {
            translate_attribute(attribute->get_name(), attribute->get_value());
        }

        //We want to go through the first layer of the CommonRoadScenario only - the objects that we want to store take the parsing from here 
        //Thus, we go through the children of the scenario and parse each of them using according constructors
        for(const auto& child : pNode->get_children())
        {
            translate_element(child);
        }

    }
    catch(const std::exception& ex)
    {
        std::cerr << "Exception caught: " << ex.what() << std::endl;
    }
}

void CommonRoadScenario::translate_attribute(std::string node_name, std::string node_value)
{
    if (node_name.compare("commonRoadVersion") == 0)
    {
        common_road_version = node_value;
    }
    else if (node_name.compare("benchmarkID") == 0)
    {
        benchmark_id = node_value;
    }
    else if (node_name.compare("date") == 0)
    {
        date = node_value;
    }
    else if (node_name.compare("author") == 0)
    {
        author = node_value;
    }
    else if (node_name.compare("affiliation") == 0)
    {
        affiliation = node_value;
    }
    else if (node_name.compare("source") == 0)
    {
        source = node_value;
    }
    else if (node_name.compare("timeStepSize") == 0)
    {
        try {
            time_step_size = std::stoull(node_value);
        }
        catch (...) {
            //TODO: Better error
            std::cerr << "TODO: Better warning // Invalid value for time step size" << std::endl;
            time_step_size = 0;
        }
    }
    else if (node_name.compare("tags") == 0)
    {
        std::stringstream tag_stream(node_value);
        std::string tag;
        while (std::getline(tag_stream, tag, ' '))
        {
            tags.push_back(tag);
        }
    }
    else
    {
        std::cerr << "TODO: Better warning // Attribute of Commonroad scenario unknown" << std::endl;
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
        translate_location(node);
    }
    else if (node_name.compare("scenarioTags") == 0)
    {
        translate_scenario_tags(node);
    }
    else if (node_name.compare("lanelet") == 0)
    {
        //lanelets[get_id(node_element)] = Lanelet(node);
    }
    else if (node_name.compare("trafficSign") == 0)
    {
        //traffic_signs[get_id(node_element)] = TrafficSign(node);
    }
    else if (node_name.compare("trafficLight") == 0)
    {
        //traffic_lights[get_id(node_element)] = TrafficLight(node);
    }
    else if (node_name.compare("intersection") == 0)
    {
        //intersections[get_id(node_element)] = Intersection(node);
    }
    else if (node_name.compare("staticObstacle") == 0)
    {
        //static_obstacles[get_id(node_element)] = StaticObstacle(node);
    }
    else if (node_name.compare("dynamicObstacle") == 0)
    {
        //dynamic_obstacles[get_id(node_element)] = DynamicObstacle(node);
    }
    else if (node_name.compare("obstacle") == 0)
    {
        ObstacleRole obstacle_role = get_obstacle_role(node);
        if (obstacle_role == ObstacleRole::Static)
        {
            //static_obstacles[get_id(node_element)] = StaticObstacle(node);
        }
        else if (obstacle_role == ObstacleRole::Dynamic)
        {
            //dynamic_obstacles[get_id(node_element)] = DynamicObstacle(node);
        }
        
    }
    else if (node_name.compare("planningProblem") == 0)
    {
        //planning_problems[get_id(node_element)] = PlanningProblem(node);
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
    //Require existence of these nodes (according to specs)
    const auto child_country = node->get_first_child("country");
    if (!child_country)
        std::cerr << "TODO: Better warning // Node element country missing in location" << std::endl;

    const auto child_state = node->get_first_child("federalState");
    if (!child_state)
        std::cerr << "TODO: Better warning // Node element country missing in location" << std::endl;

    const auto child_lat = node->get_first_child("gpsLatitude");
    if (!child_lat)
        std::cerr << "TODO: Better warning // Node element country missing in location" << std::endl;

    const auto child_long = node->get_first_child("gpsLongitude");
    if (!child_long)
        std::cerr << "TODO: Better warning // Node element country missing in location" << std::endl;

    //Still go through all childs, because we should also notice nodes that should not be there
    //And because we do that anyway, we can also translate everything there instead
    for(const auto& child : node->get_children())
    {
        //Find out which object we are dealing with, pass on translation to these objects (if possible)
        const auto child_name = child->get_name();
        const xmlpp::TextNode* child_text = dynamic_cast<const xmlpp::TextNode*>(child);
        const xmlpp::Element* child_element = dynamic_cast<const xmlpp::Element*>(child);
        const xmlpp::ContentNode* child_content = dynamic_cast<const xmlpp::ContentNode*>(child);

        if (child_element)
        {
            std::cout << "Is element, not text: " << child_name << std::endl;
        }
        if (child_content)
        {
            std::cout << "Is content, not text: " << child_name << std::endl;
        }
        if (child_text)
        {
            std::cout << "Is text: " << child_name << std::endl;
        }

        std::cout << child_name << std::endl;

        if((child_name.empty() || !(child_text))) //Country is handled differently, I do not know why
        {
            //TODO: Throw error
            std::cerr << "TODO: Better warning // Node element empty in location parse " << child_name << std::endl;
            continue;
        }

        if (child_name.compare("country") == 0)
        {
            location.country = child_text->get_content();
        }
        else if (child_name.compare("federalState") == 0)
        {
            location.federal_state = child_text->get_content();
        }
        else if (child_name.compare("gpsLatitude") == 0)
        {
            try
            {
                location.gps_latitude = std::stod(child_text->get_content());
            }
            catch(const std::exception& e)
            {
                location.gps_latitude = -1;
                std::cerr << "TODO: Better warning // Wrong number format for gps latitude" << std::endl;
            }
        }
        else if (child_name.compare("gpsLongitude") == 0)
        {
            try
            {
                location.gpd_longitude = std::stod(child_text->get_content());
            }
            catch(const std::exception& e)
            {
                location.gpd_longitude = -1;
                std::cerr << "TODO: Better warning // Wrong number format for gps longitude" << std::endl;
            }
        }
        else if (child_name.compare("zipcode") == 0)
        {
            location.zipcode = child_text->get_content();
        }
        else if (child_name.compare("name") == 0)
        {
            location.name = child_text->get_content();
        }
        else if (child_name.compare("geoTransformation") == 0)
        {
            //Gets ignored
        }
        else
        {
            std::cerr << "TODO: Better warning // Node element not conformant to specs (location)" << std::endl;
        }
    }
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

int CommonRoadScenario::get_id(const xmlpp::Element* element_node) 
{
    const auto attribute_id = element_node->get_attribute("id");

    if (attribute_id)
    {
        try
        {
            std::cout << "ID is: " << std::stoi(attribute_id->get_value()) << std::endl;
            return std::stoi(attribute_id->get_value());
        }
        catch(...)
        {
            std::cerr << "TODO: Better warning // Could not translate node ID" << std::endl;
            return -1;
        }
        
    }
    else
    {
        std::cerr << "TODO: Better warning // Could not find node ID" << std::endl;
        return -1;   
    }
}

ObstacleRole CommonRoadScenario::get_obstacle_role(const xmlpp::Node* node)
{
    //Role is itself an element node, with its content in another child node
    //Possible TODO: Catch error? role_node might not have any child
    const xmlpp::Node* role_node = node->get_first_child("role");
    const xmlpp::TextNode* child_text = dynamic_cast<const xmlpp::TextNode*>(role_node->get_first_child());

    if(!(child_text))
    {
        //TODO: Throw error
        std::cerr << "TODO: Better warning // Wrong node value in obstacleRole" << std::endl;
        return ObstacleRole::NotInSpec;
    }
    
    if (child_text->get_content().compare("static") == 0)
    {
        return ObstacleRole::Static;
    }
    else if (child_text->get_content().compare("dynamic") == 0)
    {
        return ObstacleRole::Dynamic;
    }
    else
    {
        std::cerr << "TODO: Better warning // Node element not conformant to specs (obstacle)" << std::endl;
        return ObstacleRole::NotInSpec;
    }
}


//Old parse structure (relevant parts only)
// const auto nodeText = dynamic_cast<const xmlpp::TextNode*>(node);

//   if(nodeText && nodeText->is_white_space()) //Let's ignore the indenting - you don't always want to do this.
//     return;

//   const auto nodename = node->get_name();

//   if(!nodeText && !nodename.empty()) //Let's not say "name: text".
//   {
//     //nodename is the name of the node, e.g. "lanelet"
//   }

//   //Treat the various node types differently:
//   if(nodeText)
//   {
//     //Value of the above node, CatchConvertError(nodeText->get_content())
//      //This is only relevant for values in between brackets, e.g. <x> 1 </x>
//   }
//   else if(const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(node))
//   {
//     //A normal Element node:

//     //Print attributes:
//     for (const auto& attribute : nodeElement->get_attributes())
//     {
//       //Type name like "id", "ref", ...: attribute->get_name()
//       //Value: attribute->get_value()
//     }
//    }

//     //Recurse through child nodes:
//     for(const auto& child : node->get_children())
//     {
//         parse_xml(child); //recursive
//     }