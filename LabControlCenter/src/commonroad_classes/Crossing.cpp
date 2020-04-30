#include "commonroad_classes/Crossing.hpp"

Crossing::Crossing(const xmlpp::Node* node)
{
    //TODO: Assert node type to be crossing - can't do that, bc crossing is unused and thus no expectable names for the crossign types are given in the specs
    
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
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Crossing:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
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