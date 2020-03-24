#include "commonroad_classes/CommonRoadScenario.hpp"

CommonRoadScenario::CommonRoadScenario(std::string xml_filepath)
{
    xmlpp::DomParser parser;

    try
    {
        parser.parse_file(xml_filepath);

        if(!parser) std::cerr << "Cannot parse file!" << std::endl;
        const auto pNode = parser.get_document()->get_root_node(); //deleted by DomParser.
        parse_xml(pNode);

    }
    catch(const std::exception& ex)
    {
        std::cerr << "Exception caught: " << ex.what() << std::endl;
    }
}

void CommonRoadScenario::parse_xml(const xmlpp::Node* node)
{
  const auto nodeText = dynamic_cast<const xmlpp::TextNode*>(node);

  if(nodeText && nodeText->is_white_space()) //Let's ignore the indenting - you don't always want to do this.
    return;

  const auto nodename = node->get_name();

  if(!nodeText && !nodename.empty()) //Let's not say "name: text".
  {
    //nodename is the name of the node, e.g. "lanelet"
  }

  //Treat the various node types differently:
  if(nodeText)
  {
    //Value of the above node, CatchConvertError(nodeText->get_content())
  }
  else if(const xmlpp::Element* nodeElement = dynamic_cast<const xmlpp::Element*>(node))
  {
    //A normal Element node:

    //Print attributes:
    for (const auto& attribute : nodeElement->get_attributes())
    {
      //Type name like "id", "ref", ...: attribute->get_name()
      //Value: attribute->get_value()
    }
   }

    //Recurse through child nodes:
    for(const auto& child : node->get_children())
    {
        parse_xml(child); //recursive
    }
}