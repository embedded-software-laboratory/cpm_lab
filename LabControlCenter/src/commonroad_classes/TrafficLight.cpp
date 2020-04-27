#include "commonroad_classes/TrafficLight.hpp"

TrafficLight::TrafficLight(const xmlpp::Node* node)
{
    //TODO: Assert node type

    /********************************************/
    //TrafficLightElement is not part of the specification
    //But: TrafficLight can contain a sequence of cycle,position,direction,active
    //This must probably be handled by using line numbers or, alternatively, by iterating all children with the according names
    //Then, several elements can be constructed from that

    //We use the XMLTranslation iteration functions here, as it is easier to operate on the vectors if we can use indices and .at()
    xml_translation::iterate_children(
        node,
        [&] (xmlpp::Node* child)
        {
            positions.push_back(translate_position(child));
            position_lines.push_back(child->get_line());
        },
        "position"
    );
    xml_translation::iterate_children(
        node,
        [&] (xmlpp::Node* child)
        {
            directions.push_back(translate_direction(child));
            direction_lines.push_back(child->get_line());
        },
        "direction"
    );
    xml_translation::iterate_children(
        node,
        [&] (xmlpp::Node* child)
        {
            actives.push_back(translate_active(child));
            active_lines.push_back(child->get_line());
        },
        "active"
    );
    xml_translation::iterate_children(
        node,
        [&] (xmlpp::Node* child)
        {
            cycles.push_back(translate_cycle(child));
            cycle_lines.push_back(child->get_line());
        },
        "cycle"
    );

    //TODO: I have absolutely no idea how to put these together
    //Line numbers are not enough without a reference
    //Even if "cycle" always appears, it might appear at any position in between the three other optional arguments
    //Thus, a position node specified before a cycle node could just as much be part of that cycle node as a position node located after a cycle node
    //example: cycle | position | cycle | cycle -> I cannot tell whether "position" belongs to the first or the second cycle 
    
    //Test output
    std::cout << "TrafficLight: " << std::endl;
    std::cout << "\tActive size: " << actives.size() << std::endl;
    std::cout << "\tPosition size: " << positions.size() << std::endl;
    std::cout << "\tDirection size: " << directions.size() << std::endl;
    std::cout << "\tCycle size: " << cycles.size() << std::endl;
}

Position TrafficLight::translate_position(const xmlpp::Node* position_node)
{
    //Get position value, which must not be specified
    return Position(position_node);
}

Direction TrafficLight::translate_direction(const xmlpp::Node* direction_node)
{
    //Get direction value, which must not exist
    std::string direction_string = xml_translation::get_first_child_text(direction_node);
    if (direction_string.compare("right") == 0)
    {
        return Direction::Right;
    }
    else if (direction_string.compare("straight") == 0)
    {
        return Direction::Straight;
    }
    else if (direction_string.compare("left") == 0)
    {
        return Direction::Left;
    }
    else if (direction_string.compare("leftStraight") == 0)
    {
        return Direction::LeftStraight;
    }
    else if (direction_string.compare("straightRight") == 0)
    {
        return Direction::StraightRight;
    }
    else if (direction_string.compare("leftRight") == 0)
    {
        return Direction::LeftRight;
    }
    else if (direction_string.compare("all") == 0)
    {
        return Direction::All;
    }
    else 
    {
        std::cerr << "TODO: Better warning // Node element not conformant to specs (direction) in: " << direction_node->get_line() << std::endl;
        return Direction::NotInSpec;
    }    
}

bool TrafficLight::translate_active(const xmlpp::Node* active_node)
{
    std::string active_string = xml_translation::get_first_child_text(active_node);
    if (active_string.compare("true") == 0)
    {
        return true;
    }
    else if (active_string.compare("false") == 0)
    {
        return false;
    } 
    else 
    {
        std::cerr << "TODO: Better warning // Value of node element 'virtual' not conformant to specs (commonroad) - at: " << active_node->get_line() << std::endl;
        return false;
    }
}

TrafficLightCycle TrafficLight::translate_cycle(const xmlpp::Node* cycle_node)
{
    //Get cycle node and translate traffic light cycle
    TrafficLightCycle cycle;

    xml_translation::iterate_children(
        cycle_node,
        [&] (xmlpp::Node* element_node)
        {
            TrafficCycleElement element;

            xml_translation::iterate_children(
                element_node,
                [&] (xmlpp::Node* duration_node)
                {
                    element.durations.push_back(xml_translation::get_child_child_uint(duration_node, "duration", true));
                },
                "duration"
            );

            xml_translation::iterate_children(
                element_node,
                [&] (xmlpp::Node* color_node)
                {
                    std::string color = xml_translation::get_child_child_text(color_node, "color", true);
                    if (color.compare("red") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::Red);
                    }
                    else if (color.compare("redYellow") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::RedYellow);
                    }
                    else if (color.compare("green") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::Green);
                    }
                    else if (color.compare("yellow") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::Yellow);
                    }
                    else
                    {
                        std::cerr << "TODO: Better warning // Value of node element 'color' not conformant to specs (commonroad) - at: " << color_node->get_line() << std::endl;
                        element.colors.push_back(TrafficLightColor::NotInSpec);
                    }
                    
                },
                "color"
            );

            cycle.cycle_elements.push_back(element);
        },
        "trafficCycleElement"
    );
    
    //Must be a positive value, -1 if nonexistant in xml_translation -> check for that
    int offset = xml_translation::get_child_child_int(cycle_node, "timeOffset", false);
    if (offset >= 0)
    {
        cycle.time_offset = static_cast<unsigned int>(offset);
    }

    return cycle;
}

void TrafficLight::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    //TODO: Check if that's all
    
    for (auto& position : positions)
    {
        position.transform_coordinate_system(scale, translate_x, translate_y);
    }
}

void TrafficLight::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation) 
{
    std::cerr << "TODO: Better warning // Drawing TrafficLights is currently unsupported" << std::endl;

    for (auto position : positions)
    {
        position.transform_context(ctx, scale);

        //Draw lights next to each other
        //TODO: Consider additional values
        //ctx->show_text("Light"); This is not sufficient, need draw matrix etc -> not worth it atm, as we currently only use 2018 files
    }
}