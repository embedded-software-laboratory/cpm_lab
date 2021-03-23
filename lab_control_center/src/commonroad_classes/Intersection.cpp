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

#include "commonroad_classes/Intersection.hpp"

/**
 * \file Intersection.cpp
 * \ingroup lcc_commonroad
 */

Intersection::Intersection(const xmlpp::Node* node)
{
    //Check if node is of type intersection
    assert(node->get_name() == "intersection");
    //2020 only

    try
    {
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

                incoming_map.insert({xml_translation::get_attribute_int(child, "id", true).value(), incoming}); //As mentioned in other classes: Value must exist, else error is thrown, so .value() can be used safely here
            }, 
            "incoming"
        );
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Intersection:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    

    if (incoming_map.size() == 0)
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Intersection should contain at least one incoming reference - line " << node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    //Test output
    // std::cout << "Lanelet: " << std::endl;
    // std::cout << "\tIncoming references (only incomingLanelet shown): ";
    // for (const auto entry : incoming_map)
    // {
    //     std::cout << " | " << entry.second.incoming_lanelet.value_or(-1);
    // }
    // std::cout << std::endl;
}

std::vector<int> Intersection::get_child_attribute_ref(const xmlpp::Node* node, std::string child_name, bool warn)
{
    //Refs are optional, so this element might not exist in the XML file
    std::vector<int> refs;
    
    //Get refs
    xml_translation::iterate_elements_with_attribute(
        node,
        [&] (std::string text) {
            auto optional_integer = xml_translation::string_to_int(text);
            if (optional_integer.has_value())
            {
                refs.push_back(optional_integer.value());
            }
            else
            {
                std::stringstream error_msg_stream;
                error_msg_stream << "At least one Intersection value is not an integer - line " << node->get_line();
                throw SpecificationError(error_msg_stream.str());
            }
        },
        child_name,
        "ref"
    );

    //Warn if at least on reference should be there
    if (warn && refs.size() == 0)
    {
        std::stringstream error_stream;
        error_stream << "Missing value in Intersection, line " << node->get_line() << " for entry " << child_name << "!";
        LCCErrorLogger::Instance().log_error(error_stream.str());
    }

    return refs;
}

const std::map<int, Incoming>& Intersection::get_incoming_map() const
{
    return incoming_map;
}