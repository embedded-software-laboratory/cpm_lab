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

#include "commonroad_classes/TrafficSign.hpp"

TrafficSign::TrafficSign(
    const xmlpp::Node* node,
    std::function<std::optional<std::pair<double, double>>(int)> _get_position_from_lanelet
) :
    get_position_from_lanelet(_get_position_from_lanelet)
{
    //Check if node is of type trafficSign
    assert(node->get_name() == "trafficSign");

    try
    {
        //Translate ID again, to be used in draw() 
        id = xml_translation::get_attribute_int(node, "id", true).value();

        //We can have multiple sign elements for one sign, consisting of multiple posts
        //Each element has its own position and can either be virtual or not virtual
        xml_translation::iterate_children(
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

                xml_translation::iterate_children(
                    child,
                    [&] (xmlpp::Node* id_child)
                    {
                        traffic_sign_ids.push_back(xml_translation::get_first_child_text(id_child));
                        traffic_sign_id_lines.push_back(id_child->get_line());
                    },
                    "trafficSignID"
                );
                xml_translation::iterate_children(
                    child,
                    [&] (xmlpp::Node* value_child)
                    {
                        additional_values.push_back(xml_translation::get_first_child_text(value_child));
                        additional_values_lines.push_back(value_child->get_line());
                    },
                    "additionalValue"
                );

                //Create TrafficSignPost elements depending on the line values (use std bound functions for search etc!)
                size_t values_index = 0; //Store up to which index additional_values have already been stored in previous elements
                for (size_t sign_index = 0; sign_index < traffic_sign_ids.size(); ++sign_index)
                {
                    //Set up traffic post consisting of one ID and possibly several additional values
                    TrafficSignPost traffic_post;
                    traffic_post.traffic_sign_id = traffic_sign_ids.at(sign_index);

                    if (sign_index == traffic_sign_ids.size() - 1)
                    {
                        //Last element, take all remaining additional values
                        if (additional_values.size() > values_index)
                        {
                            std::copy(additional_values.begin() + values_index, additional_values.end(), std::back_inserter(traffic_post.additional_values));
                        }
                    }
                    else
                    {
                        //Take all additional values up to upper bound (using the starting line of the next traffic post)
                        int next_post_line = traffic_sign_id_lines.at(sign_index + 1);
                        auto next_additional_it = std::upper_bound(additional_values_lines.begin(), additional_values_lines.end(), next_post_line);

                        if (next_additional_it != additional_values_lines.end())
                        {
                            //There is an upper bound, copy up to next additional values of next element
                            auto next_additional_index = std::distance(additional_values_lines.begin(), next_additional_it);
                            std::copy(additional_values.begin() + values_index, additional_values.begin() + next_additional_index, std::back_inserter(traffic_post.additional_values));

                            values_index = next_additional_index;
                        }
                        else if (additional_values.size() > values_index)
                        {
                            //There is no upper bound, but there are still elements left - take all remaining values
                            std::copy(additional_values.begin() + values_index, additional_values.end(), std::back_inserter(traffic_post.additional_values));

                            values_index = additional_values.size();
                        }
                    }
                    
                    element.traffic_sign_posts.push_back(traffic_post);
                }

                //Get position value, which must not be specified
                const auto position_node = xml_translation::get_child_if_exists(child, "position", false);
                if (position_node)
                {
                    element.position = std::optional<Position>{std::in_place, position_node};

                    //Assert that position is exact (due to specs)
                    if (! element.position->is_exact())
                    {
                        std::stringstream error_msg_stream;
                        error_msg_stream << "Position of TrafficSign must be exact, not conformant to specs, line: " << position_node->get_line();
                        throw SpecificationError(error_msg_stream.str());
                    }
                }
                else
                {
                    //Do NOT use default position here, we have to assume that the position in this case is given by the lanelet reference
                    element.position = std::nullopt;
                }
                

                //The nodes can be set to be virtual as well in another array
                xml_translation::iterate_children(
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
                            std::stringstream error_msg_stream;
                            error_msg_stream << "Value of node element 'virtual' not conformant to specs, line: " << virtual_child->get_line();
                            throw SpecificationError(error_msg_stream.str());
                        }
                    },
                    "virtual"
                );

                traffic_sign_elements.push_back(element);
            },
            "trafficSignElement"
        );

        //Make sure that the translation is not empty
        if(traffic_sign_elements.size() == 0)
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Is empty: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate TrafficSign:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    

    //Test output
    // std::cout << "Traffic sign translated: " << std::endl;
    // std::cout << "\tNumber of elements: " << traffic_sign_elements.size() << std::endl;
    // std::cout << "\tNow iterating..." << std::endl;
    // for (const auto traffic_sign_element : traffic_sign_elements)
    // {
    //     std::cout << "\tTraffic sign element:" << std::endl;
    //     std::cout << "\t\t Is virtual - size: " << traffic_sign_element.is_virtual.size() << std::endl;
    //     std::cout << "\t\t Position has value: " << traffic_sign_element.position.has_value() << std::endl;
    //     std::cout << "\t\t Posts:" << std::endl;
    //     for (const auto traffic_sign_post : traffic_sign_element.traffic_sign_posts)
    //     {
    //         std::cout << "\t\t\t Post ID: " << traffic_sign_post.traffic_sign_id << std::endl;
    //         std::cout << "\t\t\t Additional values: ";
    //         for (const auto additional_value : traffic_sign_post.additional_values)
    //         {
    //             std::cout << " | " << additional_value;
    //         }
    //         std::cout << std::endl;
    //     }
    // }
}

void TrafficSign::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{    
    for (auto& element : traffic_sign_elements)
    {
        if (element.position)
        {
            element.position->transform_coordinate_system(scale, angle, translate_x, translate_y);
        }
    }
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void TrafficSign::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation) 
{    
    ctx->save();

    //Perform required translation + rotation
    //Local rotation does not really make sense here and is thus ignored (rotating a circle in its own coordinate system is pointless)
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    for (auto element : traffic_sign_elements)
    {
        double x = 0.0;
        double y = 0.0;
        bool position_exists = false;

        if (element.position.has_value())
        {
            //Position must be exact, this was made sure during translation
            x = element.position->get_point().value().get_x() * scale;
            y = element.position->get_point().value().get_y() * scale;
            position_exists = true;

            std::cout << "using position" << std::endl;
        }
        else if (get_position_from_lanelet)
        {
            auto position = get_position_from_lanelet(id);
            if (position.has_value())
            {
                x = position->first * scale;
                y = position->second * scale;
                position_exists = true;
            }
        }
        
        if (position_exists)
        {
            //Draw test-circle
            double radius = 0.05;
            ctx->set_source_rgb(0.6, 0.9, 0.2);
            ctx->set_line_width(0.005);
            ctx->move_to(x, y);
            ctx->begin_new_path();
            ctx->arc(x, y, radius * scale, 0.0, 2 * M_PI);
            ctx->close_path();
            ctx->fill_preserve();
            ctx->stroke();

            std::stringstream descr_stream;
            for(auto post : element.traffic_sign_posts)
            {
                descr_stream << post.traffic_sign_id << " ";
                if (post.additional_values.size() > 0)
                {
                    descr_stream << "( ";
                    for (auto add_val : post.additional_values)
                    {
                        descr_stream << add_val << " ";
                    }
                    descr_stream << ")";
                }
            }

            //Draw IDs
            ctx->translate(x, y);
            draw_text_centered(ctx, 0, 0, 0, 8, descr_stream.str());
        }
        else
        {
            LCCErrorLogger::Instance().log_error("Could not draw traffic sign: Could not obtain any valid position from defintion (also no lanelet reference exists)");
        }
    }

    ctx->restore();
}
#pragma GCC diagnostic pop

const std::vector<TrafficSignElement>& TrafficSign::get_traffic_sign_elements() const
{
    return traffic_sign_elements;
}