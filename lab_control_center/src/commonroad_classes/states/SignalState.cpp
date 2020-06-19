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

#include "commonroad_classes/states/SignalState.hpp"

SignalState::SignalState(const xmlpp::Node* node)
{
    //Check if node is of type signalState
    assert(node->get_name() == "signalState");

    try
    {
        //Time probably must be set
        const auto time_node = xml_translation::get_child_if_exists(node, "time", true);
        if (time_node)
        {
            time = std::optional<IntervalOrExact>(std::in_place, time_node);
        }
        else
        {
            //Time is the only actually required value
            std::stringstream error_msg_stream;
            error_msg_stream << "No time node in SignalState (required by specification) - line " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }

        horn = get_child_bool(node, "horn");
        indicator_left = get_child_bool(node, "indicatorLeft");
        indicator_right = get_child_bool(node, "indicatorRight");
        braking_lights = get_child_bool(node, "brakingLights");
        hazard_warning_lights = get_child_bool(node, "hazardWarningLights");
        flashing_blue_lights = get_child_bool(node, "flashingBlueLights");
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate SignalState:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    
}

bool SignalState::get_child_bool(const xmlpp::Node* node, std::string child_name)
{
    const auto child_node = xml_translation::get_child_if_exists(node, child_name, false);

    if (child_node)
    {
        std::string bool_string = xml_translation::get_first_child_text(child_node);

        if (bool_string.compare("true") == 0)
        {
            return true;
        }
        else if (bool_string.compare("false") == 0)
        {
            return false;
        } 
        else 
        {
            std::cerr << "TODO: Better warning // Value of node element " << child_name << " not conformant to specs (commonroad) - at: " << node->get_line() << std::endl;
            return false;
        }
    }

    //TODO: Check default value
    return false;
}