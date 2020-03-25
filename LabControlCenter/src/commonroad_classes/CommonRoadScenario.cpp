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
            //Different behaviour based on type
            //Type name like "id", "ref", ...: attribute->get_name()
            //Value: attribute->get_value()

            //This enum-switch behaviour leads to cleaner code
            //Also allows to check for conformance to expected nodes in each object translation function (thus also do this for other objects)
            translate_attribute(attribute->get_name(), attribute->get_value());

        //We want to go through the first layer of the CommonRoadScenario only - the objects that we want to store take the parsing from here 
        //Thus, we go through the children of the scenario and parse each of them using according constructors
        for(const auto& child : pNode->get_children())
        {
            parse_xml(child); //recursive
        }

    }
    catch(const std::exception& ex)
    {
        std::cerr << "Exception caught: " << ex.what() << std::endl;
    }
}

void CommonRoadScenario::parse_xml(const xmlpp::Node* node)
{
    //Find out which object we are dealing with, pass on translation to these objects (if possible)
    const auto nodename = node->get_name();
    const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(node);

    if(nodename.empty() || !(nodeElement))
    {
        //TODO: Throw error
        return;
    }

    //nodename is the name of the node, e.g. "lanelet"
    //-> Switch based on node name!

    //Print attributes:
    for (const auto& attribute : nodeElement->get_attributes())
    {
        //Type name like "id", "ref", ...: attribute->get_name()
        //Value: attribute->get_value()
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
        benchmark_id = node_value;
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

Element CommonRoadScenario::string_to_element(std::string node_name)
{
    if (node_name == "location")
    {
        return Element::Location;
    }
    else if (node_name == "scenarioTags")
    {
        return Element::ScenarioTags;
    }
    else if (node_name == "lanelet")
    {
        return Element::Lanelet;
    }
    else if (node_name == "trafficSign")
    {
        return Element::TrafficSign;
    }
    else if (node_name == "trafficLight")
    {
        return Element::TrafficLight;
    }
    else if (node_name == "intersection")
    {
        return Element::Intersection;
    }
    else if (node_name == "staticObstacle")
    {
        return Element::StaticObstacle;
    }
    else if (node_name == "dynamicObstacle")
    {
        return Element::DynamicObstacle;
    }
    else if (node_name == "obstacle")
    {
        return Element::Obstacle;
    }
    else if (node_name == "planningProblem")
    {
        return Element::PlanningProblem;
    }
    else
    {
        return Element::NotInSpec;
    }
}