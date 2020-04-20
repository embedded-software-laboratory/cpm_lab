#include "commonroad_classes/states/SignalState.hpp"

SignalState::SignalState(const xmlpp::Node* node)
{
    //TODO: Make sure that node is of type signalState

    //Time probably must be set
    const auto time_node = xml_translation::get_child_if_exists(node, "time", true);
    if (time_node)
    {
        time = std::optional<IntervalOrExact>(std::in_place, time_node);
    }
    else
    {
        //Time is the only actually required value
        std::cerr << "TODO: Better warning // No time node in SignalState - Line: " << node->get_line() << std::endl;
    }

    horn = get_child_bool(node, "horn");
    indicator_left = get_child_bool(node, "indicatorLeft");
    indicator_right = get_child_bool(node, "indicatorRight");
    braking_lights = get_child_bool(node, "brakingLights");
    hazard_warning_lights = get_child_bool(node, "hazardWarningLights");
    flashing_blue_lights = get_child_bool(node, "flashingBlueLights");
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