#include "commonroad_classes/Intersection.hpp"

Intersection::Intersection(const xmlpp::Node* node)
{
    //TODO: Check if node is of type intersection
    //2020 only

    xml_translation::iterate_children(
        node, 
        [&] (const xmlpp::Node* child) 
        {
            //Translate incoming node
            Incoming incoming;

            //Mandatory argument
            incoming.incoming_lanelet = get_child_attribute_ref(child, "incomingLanelet", true);

            //Non-mandatory arguments
            incoming.successors_right = get_child_attribute_ref(child, "successorsRight", false);
            incoming.successors_straight = get_child_attribute_ref(child, "successorsStraight", false);
            incoming.successors_left = get_child_attribute_ref(child, "successorsLeft", false);
            incoming.is_left_of = get_child_attribute_ref(child, "isLeftOf", false);

            incoming_map.insert({xml_translation::get_attribute_int(child, "id"), incoming});
        }, 
        "incoming"
    );

    if (incoming_map.size() == 0)
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Intersection should contain at least one incoming reference - line " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    std::cout << "Lanelet: " << std::endl;
    std::cout << "\tIncoming references (only incomingLanelet shown): ";
    for (const auto entry : incoming_map)
    {
        std::cout << " | " << entry.second.incoming_lanelet.value_or(-1);
    }
    std::cout << std::endl;
}

std::optional<int> Intersection::get_child_attribute_ref(const xmlpp::Node* node, std::string child_name, bool warn)
{
    const auto child_node = xml_translation::get_child_if_exists(node, child_name, warn);
    if (child_node)
    {
        return std::optional<int>(xml_translation::get_attribute_int(child_node, "ref", true));
    }

    return std::optional<int>();
}