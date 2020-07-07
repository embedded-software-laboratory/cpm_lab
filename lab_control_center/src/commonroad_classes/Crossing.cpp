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
                auto optional_integer = xml_translation::string_to_int(text);
                if (optional_integer.has_value())
                {
                    crossing_lanelets.push_back(optional_integer.value());
                }
                else
                {
                    std::stringstream error_msg_stream;
                    error_msg_stream << "At least one crossing lanelet reference is not an integer - line " << node->get_line();
                    throw SpecificationError(error_msg_stream.str());
                }
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