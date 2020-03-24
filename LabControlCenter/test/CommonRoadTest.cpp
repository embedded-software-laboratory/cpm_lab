#include <libxml++-2.6/libxml++/libxml++.h>

#include <iostream>
#include <vector>

void print_node(const xmlpp::Node* node)
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
    //Value of the above node
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
        print_node(child); //recursive
    }
}

int main()
{
    std::string filepath = "/home/cpm-lab/dev/software/LabControlCenter/ui/map_view/C-USA_US101-30_1_T-1.xml";

    xmlpp::DomParser parser;
    std::vector<double> lanelet_x;
    std::vector<double> lanelet_y;

    try
    {
        parser.parse_file(filepath);

        if(!parser) std::cerr << "Cannot parse file!" << std::endl;
        const auto pNode = parser.get_document()->get_root_node(); //deleted by DomParser.
        print_node(pNode);

    }
    catch(const std::exception& ex)
    {
        std::cerr << "Exception caught: " << ex.what() << std::endl;
    }

    return 0;
}