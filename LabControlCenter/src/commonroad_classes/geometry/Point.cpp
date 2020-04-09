#include "commonroad_classes/geometry/Point.hpp"

Point::Point(const xmlpp::Node* node) 
{
    //TODO: Assert node name to be point

    //2018 and 2020
    x = xml_translation::get_child_child_double(node, "x", true);
    y = xml_translation::get_child_child_double(node, "y", true);

    //2020
    z = xml_translation::get_child_child_double(node, "z", false);
    //TODO: Remember somehow if z was set at all? Is optional and 2020 only, but -1.0 is a valid value -> use std::optional

    //Test output
    std::cout << "New point created: " << "(" << x << ", " << y << ", " << z << ")" << std::endl;
}

Point::Point(int irrelevant_int)
{
    //TODO: Find out default value
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

void Point::draw(const DrawingContext& ctx, double scale)
{
    //Current state: Leave out z-value
    //Idea for z-value: Add shade, or change saturation based on current cairo value, or put small number for height into point
    ctx->save();
    ctx->set_line_width(0.03);
    ctx->set_line_cap(Cairo::LINE_CAP_ROUND);
    ctx->begin_new_path();
    ctx->move_to(x * scale, y * scale);
    ctx->line_to(x * scale, y * scale);
    //ctx->arc(x * scale, y * scale, 0.01, 0.0, 2*M_PI); //Draw a small circle where the point should be
    //ctx->fill_preserve();
    ctx->stroke();
    ctx->restore();
}