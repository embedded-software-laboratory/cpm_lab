#include "commonroad_classes/Lanelet.hpp"

Lanelet::Lanelet(const xmlpp::Node* node)
{
    //Check if node is of type lanelet
    assert(node->get_name() == "lanelet");

    try
    {
        //2018 and 2020
        left_bound = translate_bound(node, "leftBound");
        right_bound = translate_bound(node, "rightBound");

        //Make sure that left and right bound are of equal size (points should be opposing pairs)
        if (left_bound.points.size() != right_bound.points.size())
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Left and right bounds of lanelet not of equal size (# of points), line: " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }

        predecessors = translate_refs(node, "predecessor");
        successors = translate_refs(node, "successor");
        adjacent_left = translate_adjacent(node, "adjacentLeft");
        adjacent_right = translate_adjacent(node, "adjacentRight");

        //2018
        speed_limit = xml_translation::get_child_child_double(node, "speedLimit", false); //false bc. only in 2018 specs and only optional there

        //2020
        stop_line = translate_stopline(node, "stopLine");
        lanelet_type = translate_lanelet_type(node, "laneletType");
        user_one_way = translate_users(node, "userOneWay");
        user_bidirectional = translate_users(node, "userBidirectional");
        traffic_sign_refs = translate_refs(node, "trafficSignRef");
        traffic_light_refs = translate_refs(node, "trafficLightRef");
    }
    catch(const SpecificationError& e)
    {
        throw SpecificationError(std::string("Could not translate Lanelet:\n") + e.what());
    }
    catch(...)
    {
        //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
        throw;
    }
    

    //Test output
    std::cout << "Lanelet ------------------------------" << std::endl;
    std::cout << "Left bound marking: TODO" << std::endl;
    std::cout << "Right bound marking: TODO" << std::endl;

    std::cout << "Predecessors refs: ";
    for (int ref : predecessors)
    {
        std:: cout << "| " << ref;
    }
    std::cout << std::endl;

    std::cout << "Successor refs: ";
    for (int ref : successors)
    {
        std:: cout << "| " << ref;
    }
    std::cout << std::endl;

    std::cout << "Adjacent left: " << adjacent_left.exists << "(exists), " << adjacent_left.ref_id << " (ID), (TODO: print direction)" << std::endl;
    std::cout << "Adjacent right: " << adjacent_right.exists << "(exists), " << adjacent_right.ref_id << " (ID), (TODO: print direction)" << std::endl;

    std::cout << "Speed limit (2018): " << speed_limit.value_or(-1.0) << std::endl;

    std::cout << "Lanelet end --------------------------" << std::endl << std::endl;
}

Bound Lanelet::translate_bound(const xmlpp::Node* node, std::string name)
{
    const auto bound_node = xml_translation::get_child_if_exists(node, name, true); //True because this is a required part of both specs (2018 and 2020)

    //Same for both specs
    Bound bound;

    xml_translation::iterate_children(
        bound_node, 
        [&] (const xmlpp::Node* child) 
        {
            bound.points.push_back(Point(child));
        }, 
        "point"
    );

    //Warning if bound does not meet specification
    if (bound.points.size() < 2)
    {
        throw SpecificationError("Bound does not contain min amount of children");
    }

    const auto line_node = xml_translation::get_child_if_exists(bound_node, "lineMarking", false);
    if (line_node)
    {
        bound.line_marking = std::optional<LineMarking>(translate_line_marking(line_node));        
    }
    
    return bound;
}

std::vector<int> Lanelet::translate_refs(const xmlpp::Node* node, std::string name)
{
    //Refs are optional, so this element might not exist in the XML file
    std::vector<int> refs;
    
    //Get refs
    xml_translation::iterate_elements_with_attribute(
        node,
        [&] (std::string text) {
            auto optional_integer = xml_translation::string_to_int(text);
            if (optional_integer.has_value())
            {
                refs.push_back(optional_integer.value());
            }
            else
            {
                std::stringstream error_msg_stream;
                error_msg_stream << "At least one lanelet reference is not an integer - line " << node->get_line();
                throw SpecificationError(error_msg_stream.str());
            }
        },
        name,
        "ref"
    );

    return refs;
}

Adjacent Lanelet::translate_adjacent(const xmlpp::Node* node, std::string name)
{
    //Adjacents are optional, so this element might not exist
    const auto adjacent_node = xml_translation::get_child_if_exists(node, name, false);
    Adjacent adjacent;

    if (adjacent_node)
    {
        adjacent.exists = true;
        adjacent.ref_id = xml_translation::get_attribute_int(adjacent_node, "ref", true).value(); //As mentioned in other classes: Value must exist, else error is thrown, so .value() can be used safely here
    
        std::string direction_string = xml_translation::get_attribute_text(adjacent_node, "drivingDir", true).value(); //See comment above
        if(direction_string.compare("same") == 0)
        {
            adjacent.direction = DrivingDirection::Same;
        }
        else if(direction_string.compare("opposite") == 0)
        {
            adjacent.direction = DrivingDirection::Opposite;
        }
        else {
            std::stringstream error_msg_stream;
            error_msg_stream << "Specified driving direction not part of specs, in line " << adjacent_node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }   
    }

    return adjacent;
}

StopLine Lanelet::translate_stopline(const xmlpp::Node* node, std::string name)
{
    //Stop lines are optional, so this element might not exist
    const auto line_node = xml_translation::get_child_if_exists(node, name, false);
    StopLine line;

    if (line_node)
    {
        line.exists = true;

        //Translate line points
        xml_translation::iterate_children(
            line_node, 
            [&] (const xmlpp::Node* child) 
            {
                stop_line.points.push_back(Point(child));
            }, 
            "point"
        );

        if (stop_line.points.size() != 2)
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Specified stop line has too many points, not part of specs, in line " << line_node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }

        //Translate line marking
        const auto line_marking = xml_translation::get_child_if_exists(line_node, "lineMarking", true);
        if (line_marking)
        {
            line.line_marking = translate_line_marking(line_marking);
        }

        //Translate refs to traffic signs and traffic lights
        line.traffic_sign_refs = translate_refs(line_node, "trafficSignRef");
        line.traffic_light_ref = translate_refs(line_node, "trafficLightRef");
    }

    return line;
}

LaneletType Lanelet::translate_lanelet_type(const xmlpp::Node* node, std::string name)
{
    //2020 specs only (TODO: Warn if missing in 2020 XML)
    //get_child_child_text is not used here to be able to show the line where the error occured if the value matches none of the enumeration values
    const auto lanelet_type_node = xml_translation::get_child_if_exists(node, name, false);
    if (lanelet_type_node)
    {
        std::string lanelet_type_string = xml_translation::get_first_child_text(lanelet_type_node);

        if (lanelet_type_string.compare("urban") == 0)
        {
            return LaneletType::Urban;
        }
        else if (lanelet_type_string.compare("country") == 0)
        {
            return LaneletType::Country;
        }
        else if (lanelet_type_string.compare("highway") == 0)
        {
            return LaneletType::Highway;
        }
        else if (lanelet_type_string.compare("sidewalk") == 0)
        {
            return LaneletType::Sidewalk;
        }
        else if (lanelet_type_string.compare("crosswalk") == 0)
        {
            return LaneletType::Crosswalk;
        }
        else if (lanelet_type_string.compare("busLane") == 0)
        {
            return LaneletType::BusLane;
        }
        else if (lanelet_type_string.compare("bicycleLane") == 0)
        {
            return LaneletType::BicycleLane;
        }
        else if (lanelet_type_string.compare("exitRamp") == 0)
        {
            return LaneletType::ExitRamp;
        }
        else if (lanelet_type_string.compare("mainCarriageWay") == 0)
        {
            return LaneletType::MainCarriageWay;
        }
        else if (lanelet_type_string.compare("accessRamp") == 0)
        {
            return LaneletType::AccessRamp;
        }
        else if (lanelet_type_string.compare("driveWay") == 0)
        {
            return LaneletType::DriveWay;
        }
        else if (lanelet_type_string.compare("busStop") == 0)
        {
            return LaneletType::BusStop;
        }
        else 
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Specified lanelet type not part of specs, in line " << lanelet_type_node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }
    }

    return LaneletType::Unspecified;
}

std::vector<VehicleType> Lanelet::translate_users(const xmlpp::Node* node, std::string name)
{
    //Users are optional, so this element might not exist (also: 2020 specs only) 
    std::vector<VehicleType> vehicle_vector;

    //Fill vector with existing values
    xml_translation::iterate_children(
        node, 
        [&] (const xmlpp::Node* child) 
        {
            std::string user_string = xml_translation::get_first_child_text(child);
            VehicleType type_out;

            if (user_string.compare("vehicle") == 0)
            {
                type_out = VehicleType::Vehicle;
            }
            else if (user_string.compare("car") == 0)
            {
                type_out = VehicleType::Car;
            }
            else if (user_string.compare("truck") == 0)
            {
                type_out = VehicleType::Truck;
            }
            else if (user_string.compare("bus") == 0)
            {
                type_out = VehicleType::Bus;
            }
            else if (user_string.compare("motorcycle") == 0)
            {
                type_out = VehicleType::Motorcycle;
            }
            else if (user_string.compare("bicycle") == 0)
            {
                type_out = VehicleType::Bicycle;
            }
            else if (user_string.compare("pedestrian") == 0)
            {
                type_out = VehicleType::Pedestrian;
            }
            else if (user_string.compare("priorityVehicle") == 0)
            {
                type_out = VehicleType::PriorityVehicle;
            }
            else if (user_string.compare("train") == 0)
            {
                type_out = VehicleType::Train;
            }
            else 
            {
                std::stringstream error_msg_stream;
                error_msg_stream << "Specified vehicle type not part of specs, in line " << child->get_line();
                throw SpecificationError(error_msg_stream.str());
            }

            vehicle_vector.push_back(type_out);
        }, 
        name
    );

    return vehicle_vector;
}

LineMarking Lanelet::translate_line_marking(const xmlpp::Node* line_node)
{
    std::string line_marking_text = xml_translation::get_first_child_text(line_node);
    if (line_marking_text.compare("dashed") == 0)
    {
        //2018 and 2020
        return LineMarking::Dashed;
    }
    else if (line_marking_text.compare("solid") == 0)
    {
        //2018 and 2020
        return LineMarking::Solid;
    }
    else if (line_marking_text.compare("broad_dashed") == 0)
    {
        //2020 only
        return LineMarking::BroadDashed;
    }
    else if (line_marking_text.compare("broad_solid") == 0)
    {
        //2020 only
        return LineMarking::BroadSolid;
    }
    else
    {
        std::stringstream error_msg_stream;
        error_msg_stream << "Specified line marking not part of specs, in " << line_node->get_line();
        throw SpecificationError(error_msg_stream.str());
    }
}

double Lanelet::get_min_width()
{
    if (left_bound.points.size() != right_bound.points.size())
    {
        //TODO: Replace by assertion, should have failed by exception before
        std::cerr << "TODO: Better warning // Lanelet bounds (left, right) not of equal size" << std::endl;
    }

    size_t min_vec_size = std::max(left_bound.points.size(), right_bound.points.size());

    double min_width = -1.0;
    for (size_t i = 0; i < min_vec_size; ++i)
    {
        //Width = sqrt(diff_x^2 + diff_y^2) -> Ignore third dimension for this (as we do not need it)
        double new_width = std::sqrt(std::pow((left_bound.points.at(i).get_x() - right_bound.points.at(i).get_x()), 2) + std::pow((left_bound.points.at(i).get_y() - right_bound.points.at(i).get_y()), 2));

        if (min_width < 0 || new_width < min_width)
        {
            min_width = new_width;
        }
    }

    return min_width;
}

/******************************Interface functions***********************************/

void Lanelet::transform_coordinate_system(double scale, double translate_x, double translate_y)
{
    for (auto& point : stop_line.points)
    {
        point.transform_coordinate_system(scale, translate_x, translate_y);
    }

    for (auto& point : right_bound.points)
    {
        point.transform_coordinate_system(scale, translate_x, translate_y);
    }

    for (auto& point : left_bound.points)
    {
        point.transform_coordinate_system(scale, translate_x, translate_y);
    }
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void Lanelet::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    //Current state: Only draw boundaries
    //Local orientation does not really make sense here, so it is ignored
    ctx->save();

    //Perform required translation + rotation
    //Local orientation is irrelevant here (we do not want to rotate the lanelet around its center)
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    ctx->set_line_width(0.005);

    //Draw points - I do not know why save() and restore() exist, but we cannot pause drawing a line and do something else between even though they are used in Point
    //Thus, points must be drawn before the line is drawn
    for (auto point : left_bound.points)
    {
        point.draw(ctx, scale);
    }
    for (auto point : right_bound.points)
    {
        point.draw(ctx, scale);
    }

    //Draw lines between points
    if (left_bound.points.size() > 0 && right_bound.points.size() > 0)
    {
        ctx->begin_new_path();
        ctx->move_to(left_bound.points.at(0).get_x() * scale, left_bound.points.at(0).get_y() * scale);
        for (auto point : left_bound.points)
        {
            //Draw lines between points
            ctx->line_to(point.get_x() * scale, point.get_y() * scale);
        }
        ctx->stroke();

        ctx->begin_new_path();
        ctx->move_to(right_bound.points.at(0).get_x() * scale, right_bound.points.at(0).get_y() * scale);
        for (auto point : right_bound.points)
        {
            //Draw lines between points
            ctx->line_to(point.get_x() * scale, point.get_y() * scale);
        }
        ctx->stroke();
    }

    //Draw arrows for lanelet orientation
    //These things must be true, or else the program should already have thrown an error before / the calculation above is wrong
    // assert(left_bound.points.size() == right_bound.points.size());
    // size_t arrow_start_pos = 0;
    // size_t arrow_end_pos = 1;
    // ctx->set_source_rgba(0.0, 0.0, 0.0, 0.05);
    // while (arrow_end_pos < left_bound.points.size())
    // {
    //     double x_1 = (left_bound.points.at(arrow_start_pos).get_x() + right_bound.points.at(arrow_start_pos).get_x()) / 2.0;
    //     double y_1 = (left_bound.points.at(arrow_start_pos).get_y() + right_bound.points.at(arrow_start_pos).get_y()) / 2.0;
    //     double x_2 = (left_bound.points.at(arrow_end_pos).get_x() + right_bound.points.at(arrow_end_pos).get_x()) / 2.0;
    //     double y_2 = (left_bound.points.at(arrow_end_pos).get_y() + right_bound.points.at(arrow_end_pos).get_y()) / 2.0;

    //     draw_arrow(ctx, x_1, y_1, x_2, y_2, scale);

    //     ++arrow_start_pos;
    //     ++arrow_end_pos;
    // }


    //TODO: Line markings, stop lines etc

    ctx->restore();
}
#pragma GCC diagnostic pop

void Lanelet::draw_ref(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y)
{
    //Local orientation does not really make sense here, so it is ignored
    ctx->save();

    //Perform required translation + rotation
    //Local orientation is irrelevant here
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    ctx->set_line_width(0.005);

    //Draw lines between points
    if (left_bound.points.size() > 0 && right_bound.points.size() > 0)
    {
        ctx->begin_new_path();
        ctx->move_to(left_bound.points.at(0).get_x() * scale, left_bound.points.at(0).get_y() * scale);

        //Draw lines on left side, then switch to right side & draw in backwards order (-> draw rectangle)
        for (auto point : left_bound.points)
        {
            ctx->line_to(point.get_x() * scale, point.get_y() * scale);
        }

        for(auto reverse_it_point = right_bound.points.rbegin(); reverse_it_point != right_bound.points.rend(); ++reverse_it_point)
        {
            ctx->line_to(reverse_it_point->get_x() * scale, reverse_it_point->get_y() * scale);
        }

        //Close rectangle
        ctx->line_to(left_bound.points.at(0).get_x() * scale, left_bound.points.at(0).get_y() * scale);

        ctx->fill_preserve();
        ctx->stroke();
    }

    ctx->restore();
}

std::pair<double, double> Lanelet::get_center()
{
    //The calculation of the center follows a simple assumption: Center = Middle of middle segment (middle value of all points might not be within the lanelet boundaries)
    assert(left_bound.points.size() == right_bound.points.size());

    size_t vec_size = left_bound.points.size();
    size_t middle_index = static_cast<size_t>(static_cast<double>(vec_size) / 2.0);

    double x = (left_bound.points.at(middle_index).get_x() - right_bound.points.at(middle_index).get_x()) / 2.0;
    double y = (left_bound.points.at(middle_index).get_y() - right_bound.points.at(middle_index).get_y()) / 2.0;

    return std::pair<double, double>(x, y);
}