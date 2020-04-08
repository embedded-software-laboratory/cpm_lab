#include "commonroad_classes/TrafficLight.hpp"

TrafficLight::TrafficLight(const xmlpp::Node* node)
{
    TrafficLightElement element;

    /********************************************/
    //TODO: Element is not part of the specification
    //But: TrafficLight can contain a sequence of cycle,position,direction,active
    //This must probably be handled by using line numbers or, alternatively, by iterating all children with the according names
    //Then, several elements can be constructed from that
    //At the moment, only the first element is translated, this must be changed

    //Get position value, which must not be specified
    const auto position_node = xml_translation::get_child_if_exists(node, "position", false);
    if (position_node)
    {
        element.position = std::optional<Position>{std::in_place, position_node};
    }
    else
    {
        //Use default-value constructor (parameter is irrelevant)
        element.position = std::optional<Position>{std::in_place, 0};
    }

    //Get direction value, which must not exist
    const auto direction_node = xml_translation::get_child_if_exists(node, "direction", false);
    if (direction_node)
    {
        std::string direction_string = xml_translation::get_first_child_text(direction_node);
        if (direction_string.compare("right") == 0)
        {
            element.direction = Direction::Right;
        }
        else if (direction_string.compare("straight") == 0)
        {
            element.direction = Direction::Straight;
        }
        else if (direction_string.compare("left") == 0)
        {
            element.direction = Direction::Left;
        }
        else if (direction_string.compare("leftStraight") == 0)
        {
            element.direction = Direction::LeftStraight;
        }
        else if (direction_string.compare("straightRight") == 0)
        {
            element.direction = Direction::StraightRight;
        }
        else if (direction_string.compare("leftRight") == 0)
        {
            element.direction = Direction::LeftRight;
        }
        else if (direction_string.compare("all") == 0)
        {
            element.direction = Direction::All;
        }
        else 
        {
            std::cerr << "TODO: Better warning // Node element not conformant to specs (direction) in: " << direction_node->get_line() << std::endl;
            element.direction = Direction::NotInSpec;
        }
    }

    //Get active, which must not exist
    const auto active_node = xml_translation::get_child_if_exists(node, "active", false);
    if (active_node)
    {
        std::string active_string = xml_translation::get_first_child_text(active_node);
        if (active_string.compare("true") == 0)
        {
            element.is_active = true;
        }
        else if (active_string.compare("false") == 0)
        {
            element.is_active = false;
        } 
        else 
        {
            std::cerr << "TODO: Better warning // Value of node element 'virtual' not conformant to specs (commonroad) - at: " << active_node->get_line() << std::endl;
            element.is_active = false;
        }
    }
    else
    {
        //TODO: Find out active default value
    }
    
    //Get cycle node and translate traffic light cycle
    const auto cycle_node = xml_translation::get_child_if_exists(node, "cycle", false);
    if (cycle_node)
    {
        //TODO: Translate this part
    }
    else
    {
        std::cerr << "TODO: Better warning // Cycle node should exist (trafficLight) - at: " << node->get_line() << std::endl;
    }
    
    //TODO: Maybe use functions for better readability
    
    //Test output
    std::cout << "TrafficLight: " << std::endl;
    std::cout << "\tIs active: " << element.is_active << std::endl;
    std::cout << "\tHas position: " << element.position.has_value() << std::endl;
}