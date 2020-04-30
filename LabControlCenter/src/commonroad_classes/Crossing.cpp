#include "commonroad_classes/Crossing.hpp"

Crossing::Crossing(const xmlpp::Node* node)
{
    //TODO: Assert node type to be crossing
    
    try
    {
        //Get refs
        xml_translation::iterate_elements_with_attribute(
            node,
            [&] (std::string text) {
                crossing_lanelets.push_back(xml_translation::string_to_int(text));
            },
            "crossingLanelet",
            "ref"
        );
    }
    catch(const std::exception& e)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        //TODO: If desired, add "addInfo" function to error class to provide additional information
        throw;
    }
    

    if (crossing_lanelets.size() == 0)
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Crossing should contain at least one lanelet reference - line " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
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