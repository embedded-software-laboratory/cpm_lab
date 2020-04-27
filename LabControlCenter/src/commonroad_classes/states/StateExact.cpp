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

void StateExact::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    //TODO: Check if that's all
    
    if (position.has_value())
    {
        position->transform_coordinate_system(scale, translate_x, translate_y);
    }

    if (scale > 0)
    {
        transform_scale *= scale;   
    }
}

void StateExact::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    //Simple function that only draws the position (and orientation), but not the object itself
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    position->draw(ctx, scale);

    //Draw arrow - TODO: Maybe make this a utility function
    position->transform_context(ctx, scale);
    ctx->rotate(orientation + local_orientation);
    double arrow_scale = 0.6 * scale; //To quickly change the scale to your liking
    ctx->set_line_width(0.015 * arrow_scale * transform_scale);
    ctx->move_to(0.0, 0.0);
    ctx->line_to(1.0 * arrow_scale, 0.0);
    ctx->line_to(0.9 * arrow_scale, 0.1 * arrow_scale);
    ctx->line_to(0.9 * arrow_scale, -0.1 * arrow_scale);
    ctx->line_to(1.0 * arrow_scale, 0.0);
    ctx->fill_preserve();
    ctx->stroke();
    ctx->restore();
}

void StateExact::set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double, double)> _draw_lanelet_refs)
{
    if(position.has_value())
    {
        position->set_lanelet_ref_draw_function(_draw_lanelet_refs);
    }
}

void StateExact::transform_context(const DrawingContext& ctx, double scale)
{
    //Draw at the stored position (if possible), else somehow within the possible position
    position->transform_context(ctx, scale);
    
    //Rotate, if necessary
    ctx->rotate(orientation);
}