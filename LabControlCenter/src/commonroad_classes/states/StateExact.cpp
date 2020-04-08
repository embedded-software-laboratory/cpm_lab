#include "commonroad_classes/states/StateExact.hpp"

StateExact::StateExact(const xmlpp::Node* node)
{
    //2018 and 2020 specs are the same
    //TODO: Assert node "type"

    const auto position_node = xml_translation::get_child_if_exists(node, "position", true);
    if (position_node)
    {
        position = std::optional<Position>{std::in_place, position_node};
    }
    else
    {
        std::cerr << "TODO: Better warning // No position node in StateExact - is this an error or do you want to use the default value? Line: " << node->get_line() << std::endl;

        //Use default-value constructor (parameter is irrelevant)
        position = std::optional<Position>{std::in_place, 0};
    }

    velocity = xml_translation::get_child_child_double_exact(node, "velocity", true);
    orientation = xml_translation::get_child_child_double_exact(node, "orientation", true);
    yaw_rate = xml_translation::get_child_child_double_exact(node, "yawRate", true);
    slip_angle = xml_translation::get_child_child_double_exact(node, "slipAngle", true);

    //Acceleration must not exist
    const auto acc_node = xml_translation::get_child_if_exists(node, "acceleration", false);
    if (acc_node)
    {
        acceleration = std::optional<double>(xml_translation::get_child_child_double_exact(node, "acceleration", true));
    }

    //Warn if time is not specified; must always be zero according to specs, so we ignore the actual value
    //TODO: Is that okay?
    const auto time_node = xml_translation::get_child_if_exists(node, "time", true);
    time = 0;

    //Test output
    std::cout << "StateExact: " << std::endl;
    std::cout << "\tPosition exists: " << position.has_value() << std::endl;
    std::cout << "\tVelocity: " << velocity << std::endl;
    std::cout << "\tAcceleration defined: " << acceleration.has_value() << std::endl;
    std::cout << "\tOrientation: " << orientation << std::endl;
    std::cout << "\tYaw rate: " << yaw_rate << std::endl;
    std::cout << "\tSlip angle: " << slip_angle << std::endl;
    std::cout << "\tTime: " << time << std::endl;
}