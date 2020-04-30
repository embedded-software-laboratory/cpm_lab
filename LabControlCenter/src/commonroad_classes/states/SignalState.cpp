#include "commonroad_classes/states/SignalState.hpp"

SignalState::SignalState(const xmlpp::Node* node)
{
    //TODO: Make sure that node is of type signalState

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
    catch(const std::exception& e)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        //TODO: If desired, add "addInfo" function to error class to provide additional information
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