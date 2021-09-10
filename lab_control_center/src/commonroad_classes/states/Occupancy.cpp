#include "commonroad_classes/states/Occupancy.hpp"

/**
 * \file Occupancy.cpp
 * \ingroup lcc_commonroad
 */

Occupancy::Occupancy(const xmlpp::Node* node)
{
    //Check if node is of type occupancy
    assert(node->get_name() == "occupancy");

    commonroad_line = node->get_line();

    try
    {
        const auto shape_node = xml_translation::get_child_if_exists(node, "shape", true);
        if (shape_node)
        {
            shape = std::optional<Shape>(std::in_place, shape_node);
        }
        
        const auto time_node = xml_translation::get_child_if_exists(node, "time", true);
        if (time_node)
        {
            time = std::optional<IntervalOrExact>(std::in_place, time_node);
            //Time must have a value; make sure that, as specified, the value is greater than zero
            if (! time.value().is_greater_zero())
            {
                std::stringstream error_stream;
                error_stream << "Time must be greater than zero, in line: ";
                error_stream << time_node->get_line();
                throw SpecificationError(error_stream.str());
            }
        }
        else
        {
            //Time is the only actually required value
            std::stringstream error_msg_stream;
            error_msg_stream << "No time node in Occupancy (required by specification) - line " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Occupancy:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    
}

void Occupancy::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    if (shape.has_value())
    {
        shape->transform_coordinate_system(scale, angle, translate_x, translate_y);
    }
}

void Occupancy::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    //Draw shape
    if (shape.has_value())
    {
        shape->draw(ctx, scale, 0, 0, 0, local_orientation);
    }
    else
    {
        std::stringstream error_stream;
        error_stream << "Cannot draw occupancy, shape is missing, from line " << commonroad_line;
        LCCErrorLogger::Instance().log_error(error_stream.str());
    }

    ctx->restore();
}

void Occupancy::transform_context(const DrawingContext& ctx, double scale)
{
    //Transform to position of shape center, where the object can be drawn if desired
    if (shape.has_value())
    {
        shape->transform_context(ctx, scale);
    }
    else
    {
        std::stringstream error_stream;
        error_stream << "Cannot transform based on occupancy data, shape is missing, from line " << commonroad_line;
        LCCErrorLogger::Instance().log_error(error_stream.str());
    }
}

//********************************************************************************************************************************************
//Getter
//********************************************************************************************************************************************

std::pair<double, double> Occupancy::get_center()
{
    if (!shape.has_value())
    {
        throw SpecificationError("Occupancy should have shape value, but does not");
    }
    return shape->get_center();
}

IntervalOrExact Occupancy::get_time()
{
    if (!time.has_value())
    {
        throw SpecificationError("Occupancy should have time value, but does not");
    }
    return time.value();
}

const std::optional<Shape>& Occupancy::get_shape() const
{
    return shape;
}

const std::optional<IntervalOrExact>& Occupancy::get_time() const
{
    return time;
}