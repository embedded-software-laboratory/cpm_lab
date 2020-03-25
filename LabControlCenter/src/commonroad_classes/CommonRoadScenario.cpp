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
        if (nodename != "commonRoad")
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
    if (node_name == "commonRoadVersion")
    {
        common_road_version = node_value;
    }
    else if (node_name == "benchmarkID")
    {
        benchmark_id = node_value;
    }
    else if (node_name == "date")
    {
        date = node_value;
    }
    else if (node_name == "author")
    {
        author = node_value;
    }
    else if (node_name == "affiliation")
    {
        affiliation = node_value;
    }
    else if (node_name == "source")
    {
        source = node_value;
    }
    else if (node_name == "timeStepSize")
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
    else if (node_name == "tags")
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
    if (node_name == "location")
    {
        translate_location(node);
    }
    else if (node_name == "scenarioTags")
    {
        translate_scenario_tags(node);
    }
    else if (node_name == "lanelet")
    {
        //lanelets[get_id(node_element->get_attributes())] = Lanelet(node);
    }
    else if (node_name == "trafficSign")
    {
        //traffic_signs[get_id(node_element->get_attributes())] = TrafficSign(node);
    }
    else if (node_name == "trafficLight")
    {
        //traffic_lights[get_id(node_element->get_attributes())] = TrafficLight(node);
    }
    else if (node_name == "intersection")
    {
        //intersections[get_id(node_element->get_attributes())] = Intersection(node);
    }
    else if (node_name == "staticObstacle")
    {
        //static_obstacles[get_id(node_element->get_attributes())] = StaticObstacle(node);
    }
    else if (node_name == "dynamicObstacle")
    {
        //dynamic_obstacles[get_id(node_element->get_attributes())] = DynamicObstacle(node);
    }
    else if (node_name == "obstacle")
    {
        ObstacleRole obstacle_role = get_obstacle_role(node);
        if (obstacle_role == ObstacleRole::Static)
        {
            //static_obstacles[get_id(node_element->get_attributes())] = StaticObstacle(node);
        }
        else if (obstacle_role == ObstacleRole::Dynamic)
        {
            //dynamic_obstacles[get_id(node_element->get_attributes())] = DynamicObstacle(node);
        }
        
    }
    else if (node_name == "planningProblem")
    {
        //planning_problems[get_id(node_element->get_attributes())] = PlanningProblem(node);
    }
    else
    {
        std::cerr << "TODO: Better warning // Node element not conformant to specs (commonroad) - name is: " << node_name << std::endl;
    }
}

void CommonRoadScenario::translate_location(const xmlpp::Node* node) 
{
    for(const auto& child : node->get_children())
    {
        //Find out which object we are dealing with, pass on translation to these objects (if possible)
        const auto child_name = child->get_name();
        const xmlpp::TextNode* child_text = dynamic_cast<const xmlpp::TextNode*>(child);

        if(child_name.empty() || !(child_text))
        {
            //TODO: Throw error
            std::cerr << "TODO: Better warning // Node element empty in location parse" << std::endl;
            return;
        }

        if (child_name == "country")
        {
            location.country = child_text->get_content();
        }
        else if (child_name == "federalState")
        {
            location.federal_state = child_text->get_content();
        }
        else if (child_name == "gpsLatitude")
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
        else if (child_name == "gpsLongitude")
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
        else if (child_name == "zipcode")
        {
            location.zipcode = child_text->get_content();
        }
        else if (child_name == "name")
        {
            location.name = child_text->get_content();
        }
        else if (child_name == "geoTransformation")
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

        if (node_name == "interstate")
        {
            scenario_tags.push_back(ScenarioTag::Interstate);
        }
        else if (node_name == "highway")
        {
            scenario_tags.push_back(ScenarioTag::Highway);
        }
        else if (node_name == "urban")
        {
            scenario_tags.push_back(ScenarioTag::Urban);
        }   
        else if (node_name == "comfort")
        {
            scenario_tags.push_back(ScenarioTag::Comfort);
        }
        else if (node_name == "critical")
        {
            scenario_tags.push_back(ScenarioTag::Critical);
        }
        else if (node_name == "evasive")
        {
            scenario_tags.push_back(ScenarioTag::Evasive);
        }
        else if (node_name == "cut_in")
        {
            scenario_tags.push_back(ScenarioTag::CutIn);
        }
        else if (node_name == "illegal_cutin")
        {
            scenario_tags.push_back(ScenarioTag::IllegalCutIn);
        }
        else if (node_name == "intersection")
        {
            scenario_tags.push_back(ScenarioTag::Intersection);
        }
        else if (node_name == "lane_change")
        {
            scenario_tags.push_back(ScenarioTag::LaneChange);
        }
        else if (node_name == "lane_following")
        {
            scenario_tags.push_back(ScenarioTag::LaneFollowing);
        }
        else if (node_name == "merging_lanes")
        {
            scenario_tags.push_back(ScenarioTag::MergingLanes);
        }
        else if (node_name == "multi_lane")
        {
            scenario_tags.push_back(ScenarioTag::MultiLane);
        }
        else if (node_name == "no_oncoming_traffic")
        {
            scenario_tags.push_back(ScenarioTag::NoOncomingTraffic);
        }
        else if (node_name == "on_coming_traffic")
        {
            scenario_tags.push_back(ScenarioTag::OnComingTraffic);
        }
        else if (node_name == "parallel_lanes")
        {
            scenario_tags.push_back(ScenarioTag::ParallelLanes);
        }
        else if (node_name == "race_track")
        {
            scenario_tags.push_back(ScenarioTag::RaceTrack);
        }
        else if (node_name == "roundabout")
        {
            scenario_tags.push_back(ScenarioTag::Roundabout);
        }
        else if (node_name == "rural")
        {
            scenario_tags.push_back(ScenarioTag::Rural);
        }
        else if (node_name == "simulated")
        {
            scenario_tags.push_back(ScenarioTag::Simulated);
        }
        else if (node_name == "single_lane")
        {
            scenario_tags.push_back(ScenarioTag::SingeLane);
        }
        else if (node_name == "slip_road")
        {
            scenario_tags.push_back(ScenarioTag::SlipRoad);
        }
        else if (node_name == "speed_limit")
        {
            scenario_tags.push_back(ScenarioTag::SpeedLimit);
        }
        else if (node_name == "traffic_jam")
        {
            scenario_tags.push_back(ScenarioTag::TrafficJam);
        }
        else if (node_name == "turn_left")
        {
            scenario_tags.push_back(ScenarioTag::TurnLeft);
        }
        else if (node_name == "turn_right")
        {
            scenario_tags.push_back(ScenarioTag::TurnRight);
        }
        else if (node_name == "two_lane")
        {
            scenario_tags.push_back(ScenarioTag::TwoLane);
        }
        else
        {
            std::cerr << "TODO: Better warning // Unspecified scenario tag, ignored" << std::endl;
        }
    }
}

int CommonRoadScenario::get_id(const xmlpp::Element::AttributeList attributes) 
{
    for (const auto& attribute : attributes)
    {
        if (attribute->get_name() == "id")
        {
            try
            {
                return std::stoi(attribute->get_value());
            }
            catch(...)
            {
                std::cerr << "TODO: Better warning // Could not translate node ID" << std::endl;
                return -1;
            }
            
        }
    }

    std::cerr << "TODO: Better warning // Could not find node ID" << std::endl;
    return -1;
}

ObstacleRole CommonRoadScenario::get_obstacle_role(const xmlpp::Node* node)
{
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //TODO: USE GET_ATTRIBUTE / ELEMENT ETC, ALSO FOR GET_IT!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    for(const auto& child : node->get_children())
    {
        //Find out which object we are dealing with, pass on translation to these objects (if possible)
        const auto child_name = child->get_name();

        if (child_name == "obstacleRole")
        {
            const xmlpp::TextNode* child_text = dynamic_cast<const xmlpp::TextNode*>(child);

            if(!(child_text))
            {
                //TODO: Throw error
                std::cerr << "TODO: Better warning // Wrong node value in obstacleRole" << std::endl;
                return ObstacleRole::NotInSpec;
            }
            
            if (child_text->get_content() == "static")
            {
                return ObstacleRole::Static;
            }
            else if (child_text->get_content() == "dynamic")
            {
                return ObstacleRole::Dynamic;
            }
            else
            {
                std::cerr << "TODO: Better warning // Node element not conformant to specs (commonroad)" << std::endl;
                return ObstacleRole::NotInSpec;
            }
        }
    }

    return ObstacleRole::NotInSpec;
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