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

TrafficSign::TrafficSign(const xmlpp::Node* node)
{
    //Check if node is of type trafficSign
    assert(node->get_name() == "trafficSign");

    try
    {
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
                }
                else
                {
                    //Use default-value constructor (parameter is irrelevant)
                    element.position = std::optional<Position>{std::in_place, 0};
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
    std::cout << "Traffic sign translated: " << std::endl;
    std::cout << "\tNumber of elements: " << traffic_sign_elements.size() << std::endl;
    std::cout << "\tNow iterating..." << std::endl;
    for (const auto traffic_sign_element : traffic_sign_elements)
    {
        std::cout << "\tTraffic sign element:" << std::endl;
        std::cout << "\t\t Is virtual - size: " << traffic_sign_element.is_virtual.size() << std::endl;
        std::cout << "\t\t Position has value: " << traffic_sign_element.position.has_value() << std::endl;
        std::cout << "\t\t Posts:" << std::endl;
        for (const auto traffic_sign_post : traffic_sign_element.traffic_sign_posts)
        {
            std::cout << "\t\t\t Post ID: " << traffic_sign_post.traffic_sign_id << std::endl;
            std::cout << "\t\t\t Additional values: ";
            for (const auto additional_value : traffic_sign_post.additional_values)
            {
                std::cout << " | " << additional_value;
            }
            std::cout << std::endl;
        }
    }
}

void TrafficSign::transform_coordinate_system(double scale, double translate_x, double translate_y)
{    
    for (auto& element : traffic_sign_elements)
    {
        if (element.position)
        {
            element.position->transform_coordinate_system(scale, translate_x, translate_y);
        }
    }
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void TrafficSign::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation) 
{
    LCCErrorLogger::Instance().log_error("Drawing TrafficSigns is currently unsupported");
    
    for (auto element : traffic_sign_elements)
    {
        if (element.position.has_value())
        {
            element.position->transform_context(ctx, scale);
        }

        for(auto post : element.traffic_sign_posts)
        {
            //Draw posts next to each other
            //TODO: Consider additional values
            //ctx->show_text("Post"); This is not sufficient, need draw matrix etc -> not worth it atm, as we currently only use 2018 files
        }
    }
}
#pragma GCC diagnostic pop

const std::vector<TrafficSignElement>& TrafficSign::get_traffic_sign_elements() const
{
    return traffic_sign_elements;
}