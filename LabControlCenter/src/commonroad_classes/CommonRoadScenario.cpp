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

            //TODO: Transfrom string to enum with external function, then use switch on enum to set values
            //Leads to cleaner code, also do this in parse_xml
            //Also allows to check for conformance to expected nodes in each object translation function (thus also do this for other objects)
        }

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