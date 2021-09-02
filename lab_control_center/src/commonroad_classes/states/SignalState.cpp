#include "commonroad_classes/states/SignalState.hpp"

/**
 * \file SignalState.cpp
 * \ingroup lcc_commonroad
 */

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

std::optional<bool> SignalState::get_child_bool(const xmlpp::Node* node, std::string child_name)
{
    const auto child_node = xml_translation::get_child_if_exists(node, child_name, false);

    if (child_node)
    {
        std::string bool_string = xml_translation::get_first_child_text(child_node);

        if (bool_string.compare("true") == 0)
        {
            return std::optional<bool>(true);
        }
        else if (bool_string.compare("false") == 0)
        {
            return std::optional<bool>(false);
        } 
        else 
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Value of node element " << child_name << " not conformant to specs in SignalState - at: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }

    return std::optional<bool>();
}

const std::optional<IntervalOrExact>& SignalState::get_time() const
{
    return time;
}

const std::optional<bool>& SignalState::get_horn() const
{
    return horn;
}

const std::optional<bool>& SignalState::get_indicator_left() const
{
    return indicator_left;
}

const std::optional<bool>& SignalState::get_indicator_right() const
{
    return indicator_right;
}

const std::optional<bool>& SignalState::get_braking_lights() const
{
    return braking_lights;
}

const std::optional<bool>& SignalState::get_hazard_warning_lights() const
{
    return hazard_warning_lights;
}

const std::optional<bool>& SignalState::get_flashing_blue_lights() const
{
    return flashing_blue_lights;
}
