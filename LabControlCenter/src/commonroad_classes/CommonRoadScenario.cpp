#include "commonroad_classes/CommonRoadScenario.hpp"

CommonRoadScenario::CommonRoadScenario(std::string xml_filepath)
{
    xmlpp::DomParser parser;

    try
    {
        //Parse XML file (DOM parser)
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
    const xmlpp::Element* node_element = dynamic_cast<const xmlpp::Element*>(node);

    if(node_name.empty() || !(node_element))
    {
        //TODO: Throw error
        std::cerr << "TODO: Better warning // Node element empty in scenario parse" << std::endl;
        return;
    }

    //nodename is the name of the node, e.g. "lanelet"
    //-> Switch based on node name!
    if (node_name == "location")
    {
        tranlsate_location(node);
    }
    else if (node_name == "scenarioTags")
    {
        tranlsate_scenario_tags(node);
    }
    else if (node_name == "lanelet")
    {
        lanelets[get_id(node_element->get_attributes())] = Lanelet(node);
    }
    else if (node_name == "trafficSign")
    {
        traffic_signs[get_id(node_element->get_attributes())] = TrafficSign(node);
    }
    else if (node_name == "trafficLight")
    {
        traffic_lights[get_id(node_element->get_attributes())] = TrafficLight(node);
    }
    else if (node_name == "intersection")
    {
        intersections[get_id(node_element->get_attributes())] = Intersection(node);
    }
    else if (node_name == "staticObstacle")
    {
        static_obstacles[get_id(node_element->get_attributes())] = StaticObstacle(node);
    }
    else if (node_name == "dynamicObstacle")
    {
        dynamic_obstacles[get_id(node_element->get_attributes())] = DynamicObstacle(node);
    }
    else if (node_name == "obstacle")
    {
        ObstacleType obstacle_type = get_obstacle_type(node->get_children());
        if (obstacle_type == ObstacleType::Static)
        {
            static_obstacles[get_id(node_element->get_attributes())] = StaticObstacle(node);
        }
        else if (obstacle_type == ObstacleType::Dynamic)
        {
            dynamic_obstacles[get_id(node_element->get_attributes())] = DynamicObstacle(node);
        }
        
    }
    else if (node_name == "planningProblem")
    {
        planning_problems[get_id(node_element->get_attributes())] = PlanningProblem(node);
    }
    else
    {
        std::cerr << "TODO: Better warning // Node element not conformant to specs" << std::endl;
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