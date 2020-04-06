#include "commonroad_classes/TrafficSign.hpp"

TrafficSign::TrafficSign(const xmlpp::Node* node)
{
    //We can have multiple sign elements for one sign, consisting of multiple posts
    //Each element has its own position and can either be virtual or not virtual
    xml_translation::get_children(
        node,
        [&] (xmlpp::Node* child)
        {
            TrafficSignElement element;

            //TrafficSignElement groups several trafficSignPosts
            //We may have none or multiple additional values per ID
            //Thus, we take a look at the line number as well s.t. we know how these values are connected
            std::vector<std::string> traffic_sign_ids;
            std::vector<int> traffic_sign_id_lines;
            std::vector<std::string> additional_values;
            std::vector<int> additional_values_lines;

            xml_translation::get_children(
                child,
                [&] (xmlpp::Node* id_child)
                {
                    traffic_sign_ids.push_back(xml_translation::get_first_child_text(id_child));
                    traffic_sign_id_lines.push_back(id_child->get_line());
                },
                "trafficSignID"
            );
            xml_translation::get_children(
                child,
                [&] (xmlpp::Node* value_child)
                {
                    additional_values.push_back(xml_translation::get_first_child_text(value_child));
                    additional_values_lines.push_back(value_child->get_line());
                },
                "additionalValue"
            );

            //TODO: Create TrafficSignPost elements depending on the line values (use std bound functions for search etc!)

            //TODO: Entscheidung zwischen Default-Konstruktoren, unique_ptr, shared_ptr und optional treffen, um optionale und später initialisierte Werte zu behandeln
            //Damit lassen sich die aktuellen Fehlermeldungen beheben und die Implementierung kann fortgesetzt werden

            //Get position value, which must not be specified (is handled by default constructor of position if missing) -> TODO
            const auto position_node = xml_translation::get_child_if_exists(child, "position", false);
            if (position_node)
            {
                element.position = std::unique_ptr<Position>(new Position(position_node));
            }

            //The nodes can be set to be virtual as well in another array
            xml_translation::get_children(
                child, 
                [&] (xmlpp::Node* virtual_child)
                {
                    std::string virtual_string = xml_translation::get_first_child_text(virtual_child);
                    if (virtual_string.compare("true") == 0)
                    {
                        element.is_virtual.push_back(true);
                    }
                    else if (virtual_string.compare("false") == 0)
                    {
                        element.is_virtual.push_back(false);
                    } 
                    else 
                    {
                        std::cerr << "TODO: Better warning // Value of node element 'virtual' not conformant to specs (commonroad) - at: " << virtual_child->get_line() << std::endl;
                        element.is_virtual.push_back(false);
                    }
                },
                "virtual"
            );

            traffic_sign_elements.push_back(element);
        },
        "trafficSignElement"
    );
}
