// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "commonroad_classes/TrafficLight.hpp"

TrafficLight::TrafficLight(const xmlpp::Node* node)
{
    //Check if node is of type trafficLight
    assert(node->get_name() == "trafficLight");

    /********************************************/
    //TrafficLightElement is not part of the specification
    //But: TrafficLight can contain a sequence of cycle,position,direction,active
    //This must probably be handled by using line numbers or, alternatively, by iterating all children with the according names
    //Then, several elements can be constructed from that

    try
    {
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
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate TrafficLight:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    

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
        std::stringstream error_msg_stream;
        error_msg_stream << "Node element not conformant to specs (direction), line: " << direction_node->get_line();
        throw SpecificationError(error_msg_stream.str());
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
        std::stringstream error_msg_stream;
        error_msg_stream << "Value of node element 'virtual' not conformant to specs, line: " << active_node->get_line();
        throw SpecificationError(error_msg_stream.str());
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
                    element.durations.push_back(xml_translation::get_child_child_uint(duration_node, "duration", true).value()); //Error thrown if it doesn't exist, so we can use .value()
                },
                "duration"
            );

            xml_translation::iterate_children(
                element_node,
                [&] (xmlpp::Node* color_node)
                {
                    std::string color = xml_translation::get_child_child_text(color_node, "color", true).value(); //Error thrown if it doesn't exist, so we can use .value()
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
                        std::stringstream error_msg_stream;
                        error_msg_stream << "Value of node element 'color' not conformant to specs, line: " << color_node->get_line();
                        throw SpecificationError(error_msg_stream.str());
                    }
                    
                },
                "color"
            );

            cycle.cycle_elements.push_back(element);
        },
        "trafficCycleElement"
    );
    
    cycle.time_offset = xml_translation::get_child_child_uint(cycle_node, "timeOffset", false);

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

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void TrafficLight::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation) 
{
    std::cerr << "TODO: Better warning // Drawing TrafficLights is currently unsupported" << std::endl;

    // for (auto position : positions)
    // {
    //     position.transform_context(ctx, scale);

    //     //Draw lights next to each other
    //     //TODO: Consider additional values
    //     ctx->show_text("Light"); This is not sufficient, need draw matrix etc -> not worth it atm, as we currently only use 2018 files
    // }
}
#pragma GCC diagnostic pop