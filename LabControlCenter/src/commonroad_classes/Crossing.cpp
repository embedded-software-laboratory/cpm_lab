#include "commonroad_classes/Crossing.hpp"

Crossing::Crossing(const xmlpp::Node* node)
{
    //TODO: Assert node type to be crossing
    
    //Get refs
    xml_translation::iterate_elements_with_attribute(
        node,
        [&] (std::string text) {
            crossing_lanelets.push_back(xml_translation::string_to_int(text));
        },
        "crossingLanelet",
        "ref"
    );

    if (crossing_lanelets.size() == 0)
    {
        std::cerr << "TODO: Better warning // Crossing should contain at least one lanelet reference - line " << node->get_line() << std::endl;
    }

    //Test output
    std::cout << "Crossing: " << std::endl;
    std::cout << "\tLanelet references: ";
    for (const auto ref : crossing_lanelets)
    {
        std::cout << " | " << ref;
    }
    std::cout << std::endl;
}