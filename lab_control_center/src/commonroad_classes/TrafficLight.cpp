#include "commonroad_classes/TrafficLight.hpp"

/**
 * \file TrafficLight.cpp
 * \ingroup lcc_commonroad
 */

TrafficLight::TrafficLight(
    const xmlpp::Node* node,
    std::function<std::optional<std::pair<double, double>>(int)> _get_position_from_lanelet
) :
    get_position_from_lanelet(_get_position_from_lanelet)
{
    //Check if node is of type trafficLight
    assert(node->get_name() == "trafficLight");

    /********************************************/
    //TrafficLightElement is not part of the specification
    //But: TrafficLight can contain a sequence of cycle,position,direction,active
    //This must probably be handled by using line numbers or, alternatively, by iterating all children with the according names
    //Then, several elements can be constructed from that

    try
    {
        //Translate ID again, to be used in draw() 
        id = xml_translation::get_attribute_int(node, "id", true).value();

        //We use the XMLTranslation iteration functions here, as it is easier to operate on the vectors if we can use indices and .at()
        //TrafficLights are defined in a certain order:
        //- Cycle (must exist)
        //- Position (optional)
        //- Direction (optional)
        //- Active (optional)
        //-> Start a new definition with every new cycle definition. 
        size_t pos = -1; //To know which element to fill with new info
        std::vector<std::string> allowed_next_values = { "cycle" }; //Indicates which next values are allowed, to check for spec consistency

        xml_translation::iterate_children(
            node,
            [&] (xmlpp::Node* child)
            {
                auto child_name = std::string(child->get_name());

                //Check if the allowed order is kept
                if (std::find(allowed_next_values.begin(), allowed_next_values.end(), child_name) == allowed_next_values.end())
                {
                    std::stringstream error_stream;
                    error_stream << "TrafficLight in line " << child->get_line() << " does not fulfill specs, order of elements not kept (cycle, position etc.)."
                        << "If the order is not kept, the specification is no longer consistent and the TrafficLight cannot be translated properly!";
                    throw SpecificationError(error_stream.str());
                }

                if (child_name == "cycle")
                {
                    //Create next entry
                    ++pos;
                    
                    cycle = translate_cycle(child);
                    allowed_next_values = { "position", "direction", "active" };
                }
                else if (child_name == "position")
                {
                    position = translate_position(child);
                    allowed_next_values = { "direction", "active" };
                }
                else if (child_name == "direction")
                {
                    direction = translate_direction(child);
                    allowed_next_values = { "active" };
                }
                else if (child_name == "active")
                {
                    is_active = translate_active(child);
                    allowed_next_values = { };
                }
                else
                {
                    std::stringstream error_stream;
                    error_stream << "TrafficLight in line " << child->get_line() << " does not fulfill specs, unknown element: " << child_name;
                    throw SpecificationError(error_stream.str());
                }
            },
            "" //Iterate all children
        );
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate TrafficLight:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }

    LCCErrorLogger::Instance().log_error("Warning: The specification for traffic lights does not allow for unique definitions (due to <xs:sequence> & no required order of the elements), so we do not show more than if the light exists at all");
}

Position TrafficLight::translate_position(const xmlpp::Node* position_node)
{
    //Get position value, which must not be specified
    auto pos = Position(position_node);

    //According to specs, the position must always be exact
    if (!pos.is_exact())
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Position in TrafficLight not conformant to specs (must be exact), line: " << position_node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }

    return pos;
}

Direction TrafficLight::translate_direction(const xmlpp::Node* direction_node)
{
    //Get direction value, which must not exist
    std::string direction_string = xml_translation::get_first_child_text(direction_node);
    if (direction_string.compare("right") == 0)
    {
        return Direction::Right;
    }
    else if (direction_string.compare("straight") == 0)
    {
        return Direction::Straight;
    }
    else if (direction_string.compare("left") == 0)
    {
        return Direction::Left;
    }
    else if (direction_string.compare("leftStraight") == 0)
    {
        return Direction::LeftStraight;
    }
    else if (direction_string.compare("straightRight") == 0)
    {
        return Direction::StraightRight;
    }
    else if (direction_string.compare("leftRight") == 0)
    {
        return Direction::LeftRight;
    }
    else if (direction_string.compare("all") == 0)
    {
        return Direction::All;
    }
    else 
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Node element not conformant to specs (direction), line: " << direction_node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }    
}

bool TrafficLight::translate_active(const xmlpp::Node* active_node)
{
    std::string active_string = xml_translation::get_first_child_text(active_node);
    if (active_string.compare("true") == 0)
    {
        return true;
    }
    else if (active_string.compare("false") == 0)
    {
        return false;
    } 
    else 
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Value of node element 'active' not conformant to specs, line: " << active_node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

TrafficLightCycle TrafficLight::translate_cycle(const xmlpp::Node* cycle_node)
{
    //Get cycle node and translate traffic light cycle
    TrafficLightCycle cycle;

    xml_translation::iterate_children(
        cycle_node,
        [&] (xmlpp::Node* element_node)
        {
            TrafficCycleElement element;

            xml_translation::iterate_children(
                element_node,
                [&] (xmlpp::Node* duration_node)
                {
                    element.durations.push_back(xml_translation::get_child_child_uint(duration_node, "duration", true).value()); //Error thrown if it doesn't exist, so we can use .value()
                },
                "duration"
            );

            xml_translation::iterate_children(
                element_node,
                [&] (xmlpp::Node* color_node)
                {
                    std::string color = xml_translation::get_child_child_text(color_node, "color", true).value(); //Error thrown if it doesn't exist, so we can use .value()
                    if (color.compare("red") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::Red);
                    }
                    else if (color.compare("redYellow") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::RedYellow);
                    }
                    else if (color.compare("green") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::Green);
                    }
                    else if (color.compare("yellow") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::Yellow);
                    }
                    else if (color.compare("inactive") == 0)
                    {
                        element.colors.push_back(TrafficLightColor::Inactive);
                    }
                    else
                    {
                        std::stringstream error_msg_stream;
                        error_msg_stream << "Value of node element 'color' not conformant to specs, line: " << color_node->get_line();
                        throw SpecificationError(error_msg_stream.str());
                    }
                    
                },
                "color"
            );

            cycle.cycle_elements.push_back(element);
        },
        "trafficCycleElement"
    );
    
    cycle.time_offset = xml_translation::get_child_child_uint(cycle_node, "timeOffset", false);

    return cycle;
}

void TrafficLight::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    if (position.has_value())
    {
        position->transform_coordinate_system(scale, angle, translate_x, translate_y);
    }
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void TrafficLight::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation) 
{
    ctx->save();

    //Perform required translation + rotation, local rotation is ignored (because the traffic light's orientation does not really have a meaning)
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    //If a value for the position is given, use that instead of the lanelet reference position
    if (position.has_value())
    {
        ctx->save();
        position->transform_context(ctx, scale);

        draw_traffic_light_symbol(ctx, scale);

        ctx->restore();
    }
    else
    {
        //Draw at position given by lanelet, where no position was given
        if (get_position_from_lanelet)
        {
            auto position = get_position_from_lanelet(id);
            auto x = 0.0;
            auto y = 0.0;
            if (position.has_value())
            {
                x = position->first * scale;
                y = position->second * scale;

                ctx->save();
                ctx->translate(x, y);
                draw_traffic_light_symbol(ctx, scale);
                ctx->restore();
            }
            else
            {
                //This log causes severe flickering - why? TODO
                LCCErrorLogger::Instance().log_error("Could not draw traffic sign: Could not obtain any valid position from definition (also no lanelet reference exists)");
            }
        }
        else
        {
            LCCErrorLogger::Instance().log_error("Could not draw traffic sign: Could not obtain any valid position from definition (also no lanelet reference exists)");
        }
    } 

    ctx->restore();
}
#pragma GCC diagnostic pop

void TrafficLight::draw_traffic_light_symbol(const DrawingContext& ctx, double scale)
{
    auto length = .04;
    auto width = .1;
    auto radius = .01;

    //Draw a simple traffic light icon - rectangle
    {
        ctx->save();

        ctx->set_source_rgb(.1, .1, .1);
        ctx->set_line_width(0.005);

        //Move to first corner from center
        ctx->move_to((- (length/2)) * scale, (- (width/2)) * scale);

        //Draw lines
        ctx->line_to((- (length/2)) * scale, (  (width/2)) * scale);
        ctx->line_to((  (length/2)) * scale, (  (width/2)) * scale);
        ctx->line_to((  (length/2)) * scale, (- (width/2)) * scale);
        ctx->line_to((- (length/2)) * scale, (- (width/2)) * scale);
        ctx->fill_preserve();
        ctx->stroke();

        ctx->restore();
    }

    //Draw a simple traffic light icon - circles
    {
        ctx->save();
        ctx->set_source_rgb(1.0, .1, .1);
        ctx->set_line_width(0.005);

        //Move to center
        ctx->move_to(0, width/3 * scale);

        //Draw circle
        ctx->arc(0, width/3 * scale, radius * scale, 0.0, 2 * M_PI);
        ctx->fill_preserve();
        ctx->stroke();

        ctx->restore();
        ctx->save();
        ctx->set_source_rgb(.8, .7, 0);
        ctx->set_line_width(0.005);

        //Move to center
        ctx->move_to(0, 0);

        //Draw circle
        ctx->arc(0, 0, radius * scale, 0.0, 2 * M_PI);
        ctx->fill_preserve();
        ctx->stroke();

        ctx->restore();
        ctx->save();
        ctx->set_source_rgb(.3, .5, .15);
        ctx->set_line_width(0.005);

        //Move to center
        ctx->move_to(0, -width/3 * scale);

        //Draw circle
        ctx->arc(0, -width/3 * scale, radius * scale, 0.0, 2 * M_PI);
        ctx->fill_preserve();
        ctx->stroke();

        ctx->restore();
    }
}