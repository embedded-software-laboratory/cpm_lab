#include "commonroad_classes/TrafficLight.hpp"

TrafficLight::TrafficLight(const xmlpp::Node* node)
{
    //Get position value, which must not be specified
    const auto position_node = xml_translation::get_child_if_exists(node, "position", false);
    if (position_node)
    {
        position = std::optional<Position>{std::in_place, position_node};
    }
    else
    {
        //Use default-value constructor (parameter is irrelevant)
        position = std::optional<Position>{std::in_place, 0};
    }

    //Get direction value, which must not exist
    const auto direction_node = xml_translation::get_child_if_exists(node, "direction", false);
    if (direction_node)
    {
        std::string direction_string = xml_translation::get_first_child_text(direction_node);
        if (direction_string.compare("right") == 0)
        {
            direction = Direction::Right;
        }
        else if (direction_string.compare("straight") == 0)
        {
            direction = Direction::Straight;
        }
        else if (direction_string.compare("left") == 0)
        {
            direction = Direction::Left;
        }
        else if (direction_string.compare("leftStraight") == 0)
        {
            direction = Direction::LeftStraight;
        }
        else if (direction_string.compare("straightRight") == 0)
        {
            direction = Direction::StraightRight;
        }
        else if (direction_string.compare("leftRight") == 0)
        {
            direction = Direction::LeftRight;
        }
        else if (direction_string.compare("all") == 0)
        {
            direction = Direction::All;
        }
        else 
        {
            std::cerr << "TODO: Better warning // Node element not conformant to specs (direction) in: " << direction_node->get_line() << std::endl;
            direction = Direction::NotInSpec;
        }
    }

    //Get active, which must not exist
    const auto active_node = xml_translation::get_child_if_exists(node, "active", false);
    if (active_node)
    {
        std::string active_string = xml_translation::get_first_child_text(active_node);
        if (active_string.compare("true") == 0)
        {
            is_active = true;
        }
        else if (active_string.compare("false") == 0)
        {
            is_active = false;
        } 
        else 
        {
            std::cerr << "TODO: Better warning // Value of node element 'virtual' not conformant to specs (commonroad) - at: " << virtual_child->get_line() << std::endl;
            is_active = false;
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

    }
    else
    {
        std::cerr << "TODO: Better warning // Cycle node should exist (trafficLight) - at: " << node->get_line() << std::endl;
    }
    
    //TODO: Maybe use functions for better readability
    //TODO: trafficLIght uses a sequence, just like trafficSign, so we need to use the same "wrapping" function and change the structure in the .hpp slightly
}