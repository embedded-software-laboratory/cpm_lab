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

Point::Point(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

void Point::draw(const DrawingContext& ctx, double scale, double orientation, double translate_x, double translate_y)
{
    //Current state: Leave out z-value
    //Idea for z-value: Add shade, or change saturation based on current cairo value, or put small number for height into point
    ctx->save();
    ctx->set_line_width(0.03);
    ctx->set_line_cap(Cairo::LINE_CAP_ROUND);
    ctx->move_to(x * scale, y * scale);
    ctx->line_to(x * scale, y * scale);
    ctx->stroke();
    ctx->restore();
}

double Point::get_x()
{
    return x;
}

double Point::get_y()
{
    return y;
}

double Point::get_z()
{
    return z;
}