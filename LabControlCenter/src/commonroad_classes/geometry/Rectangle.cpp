#include "commonroad_classes/geometry/Rectangle.hpp"

Rectangle::Rectangle(const xmlpp::Node* node)
{
    //TODO: Check if node is of type rectangle

    length = xml_translation::get_child_child_double(node, "length", true); //mandatory
    width = xml_translation::get_child_child_double(node, "width", true); //mandatory

    //Get point value, which must not be specified
    const auto point_node = xml_translation::get_child_if_exists(node, "center", false);
    if (point_node)
    {
        center = std::optional<Point>{std::in_place, point_node};
    }
    else
    {
        //Use default-value constructor (parameter is irrelevant)
        center = std::optional<Point>{std::in_place, 0};
    }

    if (xml_translation::get_child_if_exists(node, "orientation", false))
    {
        orientation = xml_translation::get_child_child_double(node, "orientation", true);
    }
    else
    {
        //TODO: Find out default value
    }
    

    //Test output
    std::cout << "Rectangle:" << std::endl;
    std::cout << "\tLenght, width: " << length << ", " << width << std::endl;
    std::cout << "\tOrientation set: " << orientation.has_value() << std::endl;
    std::cout << "\tCenter set: " << center.has_value() << std::endl;
}

void Rectangle::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    ctx->set_line_width(0.005);

    //Move to corner from center
    ctx->translate((center->get_x() - (length/2)) * scale, (center->get_y() - (width/2)) * scale);

    //Rotate, if necessary
    if (orientation.has_value())
    {
        ctx->rotate(orientation.value());
    }
    //Also perform desired local orientation change
    ctx->rotate(local_orientation);

    //Draw lines
    ctx->line_to((- (length/2)) * scale, (  (width/2)) * scale);
    ctx->line_to((  (length/2)) * scale, (  (width/2)) * scale);
    ctx->line_to((  (length/2)) * scale, (- (width/2)) * scale);
    ctx->line_to((- (length/2)) * scale, (- (width/2)) * scale);
    ctx->fill_preserve();
    ctx->stroke();

    ctx->restore();
}

const std::optional<Point>& Rectangle::get_center() const
{
    return center;
}