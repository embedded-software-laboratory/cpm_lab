// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "commonroad_classes/Lanelet.hpp"

/**
 * \file Lanelet.cpp
 * \ingroup lcc_commonroad
 */

Lanelet::Lanelet(
    const xmlpp::Node* node,
    std::map<int, std::pair<int, bool>>& traffic_sign_positions, 
    std::map<int, std::pair<int, bool>>& traffic_light_positions,
    std::shared_ptr<CommonroadDrawConfiguration> _draw_configuration
)
    :
    draw_configuration(_draw_configuration)
{
    //Check if node is of type lanelet
    assert(node->get_name() == "lanelet");

    commonroad_line = node->get_line();

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

        //Add positional values for each traffic sign / light ref
        auto lanelet_id = xml_translation::get_attribute_int(node, "id", true).value();
        if (stop_line.has_value())
        {
            for (const auto ref : stop_line->traffic_sign_refs)
            {
                traffic_sign_positions[ref] = {lanelet_id, true};
            }
            for (const auto ref : stop_line->traffic_light_ref)
            {
                traffic_light_positions[ref] = {lanelet_id, true};
            }
        }
        for (const auto ref : traffic_sign_refs)
        {
            traffic_sign_positions[ref] = {lanelet_id, false};
        }
        for (const auto ref : traffic_light_refs)
        {
            traffic_light_positions[ref] = {lanelet_id, false};
        }
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
    // std::cout << "Lanelet ------------------------------" << std::endl;
    // std::cout << "Left bound marking: TODO" << std::endl;
    // std::cout << "Right bound marking: TODO" << std::endl;

    // std::cout << "Predecessors refs: ";
    // for (int ref : predecessors)
    // {
    //     std:: cout << "| " << ref;
    // }
    // std::cout << std::endl;

    // std::cout << "Successor refs: ";
    // for (int ref : successors)
    // {
    //     std:: cout << "| " << ref;
    // }
    // std::cout << std::endl;

    // std::cout << "Adjacent left: " << adjacent_left.has_value() << "(exists)" << std::endl;
    // std::cout << "Adjacent right: " << adjacent_right.has_value() << "(exists)" << std::endl;

    // std::cout << "Speed limit (2018): " << speed_limit.value_or(-1.0) << std::endl;

    // std::cout << "Lanelet end --------------------------" << std::endl << std::endl;
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

std::optional<Adjacent> Lanelet::translate_adjacent(const xmlpp::Node* node, std::string name)
{
    //Adjacents are optional, so this element might not exist
    const auto adjacent_node = xml_translation::get_child_if_exists(node, name, false);

    if (adjacent_node)
    {
        Adjacent adjacent_obj;
        auto adjacent = std::optional<Adjacent>(adjacent_obj);
        
        adjacent->ref_id = xml_translation::get_attribute_int(adjacent_node, "ref", true).value(); //As mentioned in other classes: Value must exist, else error is thrown, so .value() can be used safely here
    
        std::string direction_string = xml_translation::get_attribute_text(adjacent_node, "drivingDir", true).value(); //See comment above
        if(direction_string.compare("same") == 0)
        {
            adjacent->direction = DrivingDirection::Same;
        }
        else if(direction_string.compare("opposite") == 0)
        {
            adjacent->direction = DrivingDirection::Opposite;
        }
        else {
            std::stringstream error_msg_stream;
            error_msg_stream << "Specified driving direction not part of specs, in line " << adjacent_node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }   

        return adjacent;
    }

    return std::nullopt;
}

std::optional<StopLine> Lanelet::translate_stopline(const xmlpp::Node* node, std::string name)
{
    //Stop lines are optional, so this element might not exist
    const auto line_node = xml_translation::get_child_if_exists(node, name, false);

    if (line_node)
    {
        StopLine line_obj;
        auto line = std::optional<StopLine>(line_obj);

        //Translate line points
        xml_translation::iterate_children(
            line_node, 
            [&] (const xmlpp::Node* child) 
            {
                line->points.push_back(Point(child));
            }, 
            "point"
        );

        if (line->points.size() != 2)
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Specified stop line has too many points, not part of specs, in line " << line_node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }

        //Translate line marking
        const auto line_marking = xml_translation::get_child_if_exists(line_node, "lineMarking", true);
        if (line_marking)
        {
            line->line_marking = translate_line_marking(line_marking);
        }

        //Translate refs to traffic signs and traffic lights
        line->traffic_sign_refs = translate_refs(line_node, "trafficSignRef");
        line->traffic_light_ref = translate_refs(line_node, "trafficLightRef");

        return line;
    }

    return std::nullopt;
}

LaneletType Lanelet::translate_lanelet_type(const xmlpp::Node* node, std::string name)
{
    //get_child_child_text is not used here to be able to show the line where the error occured if the value matches none of the enumeration values
    const auto lanelet_type_node = xml_translation::get_child_if_exists(node, name, false);
    if (lanelet_type_node)
    {
        std::string lanelet_type_string = xml_translation::get_first_child_text(lanelet_type_node);

        if (lanelet_type_string.compare("urban") == 0)
        {
            return LaneletType::Urban;
        }
        else if (lanelet_type_string.compare("interstate") == 0)
        {
            return LaneletType::Interstate;
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
        else if (lanelet_type_string.compare("unknown") == 0)
        {
            return LaneletType::Unknown;
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
    assert(left_bound.points.size() == right_bound.points.size());

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

void Lanelet::set_boundary_style(const DrawingContext& ctx, std::optional<LineMarking> line_marking, double dash_length)
{
    if (line_marking.has_value())
    {
        //Set or disable dashes
        if (line_marking.value() == LineMarking::BroadDashed || line_marking.value() == LineMarking::Dashed)
        {
            std::vector<double> dashes {dash_length};
            ctx->set_dash(dashes, 0.0);
        }
        else
        {
            //Disable dashes
            std::vector<double> dashes {};
            ctx->set_dash(dashes, 0.0);
        }
        
        //Set line width
        if (line_marking.value() == LineMarking::BroadSolid || line_marking.value() == LineMarking::BroadDashed)
        {
            ctx->set_line_width(0.03);
        }
        else
        {
            ctx->set_line_width(0.005);
        }
    }
    else
    {
        //Disable dashes
        std::vector<double> dashes {};
        ctx->set_dash(dashes, 0.0);

        ctx->set_line_width(0.005);
    }
    
}

std::string Lanelet::to_text(LaneletType lanelet_type)
{
    switch (lanelet_type)
    {
        case LaneletType::AccessRamp:
            return "Acc";
            break;
        case LaneletType::BicycleLane:
            return "Bic";
            break;
        case LaneletType::BusLane:
            return "BusL";
            break;
        case LaneletType::BusStop:
            return "BusS";
            break;
        case LaneletType::Country:
            return "Cou";
            break;
        case LaneletType::Crosswalk:
            return "Cro";
            break;
        case LaneletType::DriveWay:
            return "Dri";
            break;
        case LaneletType::ExitRamp:
            return "Ex";
            break;
        case LaneletType::Highway:
            return "Hi";
            break;
        case LaneletType::Interstate:
            return "Int";
            break;
        case LaneletType::MainCarriageWay:
            return "Mai";
            break;
        case LaneletType::Sidewalk:
            return "Sid";
            break;
        case LaneletType::Unknown:
            return "Unk";
            break;
        case LaneletType::Unspecified:
            return "Uns";
            break;
        case LaneletType::Urban:
            return "Ur";
            break;
    }

    return "Error";
}

std::string Lanelet::to_text(VehicleType vehicle_type)
{
    switch ((vehicle_type))
    {
    case VehicleType::Bicycle:
        return "Bic";
        break;
    case VehicleType::Bus:
        return "Bus";
        break;
    case VehicleType::Car:
        return "Car";
        break;
    case VehicleType::Motorcycle:
        return "Mot";
        break;
    case VehicleType::Pedestrian:
        return "Ped";
        break;
    case VehicleType::PriorityVehicle:
        return "Pri";
        break;
    case VehicleType::Train:
        return "Tra";
        break;
    case VehicleType::Truck:
        return "Tru";
        break;
    case VehicleType::Vehicle:
        return "Veh";
        break;
    }

    return "Error";
}


/******************************Interface functions***********************************/

void Lanelet::transform_coordinate_system(double scale, double angle, double translate_x, double translate_y)
{
    if (stop_line.has_value())
    {
        for (auto& point : stop_line->points)
        {
            point.transform_coordinate_system(scale, angle, translate_x, translate_y);
        }
    }

    if (speed_limit.has_value())
    {
        auto new_speed_limit = speed_limit.value() * scale;
        speed_limit = std::optional<double>(new_speed_limit);
    }

    for (auto& point : right_bound.points)
    {
        point.transform_coordinate_system(scale, angle, translate_x, translate_y);
    }

    for (auto& point : left_bound.points)
    {
        point.transform_coordinate_system(scale, angle, translate_x, translate_y);
    }
}

//Suppress warning for unused parameter (s)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
void Lanelet::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    assert(draw_configuration);

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
    // for (auto point : left_bound.points)
    // {
    //     point.draw(ctx, scale);
    // }
    // for (auto point : right_bound.points)
    // {
    //     point.draw(ctx, scale);
    // }

    //Draw lines between points
    if (left_bound.points.size() > 0 && right_bound.points.size() > 0)
    {
        ctx->begin_new_path();
        set_boundary_style(ctx, left_bound.line_marking, 0.03);
        ctx->move_to(left_bound.points.at(0).get_x() * scale, left_bound.points.at(0).get_y() * scale);
        for (auto point : left_bound.points)
        {
            //Draw lines between points
            ctx->line_to(point.get_x() * scale, point.get_y() * scale);
        }
        ctx->stroke();

        ctx->begin_new_path();
        set_boundary_style(ctx, right_bound.line_marking, 0.03);
        ctx->move_to(right_bound.points.at(0).get_x() * scale, right_bound.points.at(0).get_y() * scale);
        for (auto point : right_bound.points)
        {
            //Draw lines between points
            ctx->line_to(point.get_x() * scale, point.get_y() * scale);
        }
        ctx->stroke();
    }


   if (draw_configuration->draw_lanelet_orientation.load()) {
        //Draw arrows for lanelet orientation
        //These things must be true, or else the program should already have thrown an error before / the calculation above is wrong
        assert(left_bound.points.size() == right_bound.points.size());
        size_t arrow_start_pos = 0;
        size_t arrow_end_pos = 1;
        ctx->set_source_rgba(0.0, 0.0, 0.0, 0.05);
        while (arrow_end_pos < left_bound.points.size())
        {
            double x_1 = (0.5 * left_bound.points.at(arrow_start_pos).get_x() + 0.5 * right_bound.points.at(arrow_start_pos).get_x());
            double y_1 = (0.5 * left_bound.points.at(arrow_start_pos).get_y() + 0.5 * right_bound.points.at(arrow_start_pos).get_y());
            double x_2 = (0.5 * left_bound.points.at(arrow_end_pos).get_x() + 0.5 * right_bound.points.at(arrow_end_pos).get_x());
            double y_2 = (0.5 * left_bound.points.at(arrow_end_pos).get_y() + 0.5 * right_bound.points.at(arrow_end_pos).get_y());

            draw_arrow(ctx, x_1, y_1, x_2, y_2, scale);

            ++arrow_start_pos;
            ++arrow_end_pos;
        }
    }

    //Draw stop lines - we already made sure that it consists of two points in translation
    if (stop_line.has_value())
    {
        ctx->begin_new_path();
        ctx->set_source_rgb(.9,.3,.3);
        set_boundary_style(ctx, std::optional<LineMarking>{stop_line->line_marking}, 0.03);
        ctx->move_to(stop_line->points.at(0).get_x() * scale, stop_line->points.at(0).get_y() * scale);
        ctx->line_to(stop_line->points.at(1).get_x() * scale, stop_line->points.at(1).get_y() * scale);
        ctx->stroke();
    }

    //Draw time, velocity description
    if (draw_configuration->draw_lanelet_types.load())
    {
        ctx->save();

        std::stringstream descr_stream;
        descr_stream << "T: " << to_text(lanelet_type);

        if (speed_limit.has_value())
        {
            descr_stream << " | Speed: " << speed_limit.value();
        }
        
        //Move to lanelet center for text, draw centered around it, rotate by angle of lanelet
        auto center = get_center();
        ctx->translate(center.first, center.second);
        //Calculate lanelet angle (trivial solution used here will not work properly for arcs etc)
        auto alpha = atan((left_bound.points.rbegin()->get_y() - left_bound.points.at(0).get_y()) / (left_bound.points.rbegin()->get_x() - left_bound.points.at(0).get_x())); //alpha = arctan(dy / dx)
        
        draw_text_centered(ctx, 0, 0, alpha, 4, descr_stream.str());

        ctx->restore();
    }

    //TODO: Line markings, stop lines etc
    if (user_one_way.size() > 0 || user_bidirectional.size() > 0)
    {
        std::stringstream error_stream;
        error_stream << "Speed limit and user restrictions are currently not drawn for lanelets, from line " << commonroad_line;
        LCCErrorLogger::Instance().log_error(error_stream.str());
    }

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

    double x = (0.5 * left_bound.points.at(middle_index).get_x() + 0.5 * right_bound.points.at(middle_index).get_x());
    double y = (0.5 * left_bound.points.at(middle_index).get_y() + 0.5 * right_bound.points.at(middle_index).get_y());

    return std::pair<double, double>(x, y);
}

std::pair<double, double> Lanelet::get_center_of_all_points()
{
    //The calculation of the center follows a simple assumption: Center = Middle of middle segment (middle value of all points might not be within the lanelet boundaries)
    assert(left_bound.points.size() == right_bound.points.size());

    size_t vec_size = left_bound.points.size();
    
    double x = 0.0;
    double y = 0.0;
    for (size_t index = 0; index < vec_size; ++index)
    {
        //Calculate avg iteratively to avoid overflow
        x += ((left_bound.points.at(index).get_x() + right_bound.points.at(index).get_x()) / 2.0 - x) / (index + 1);
        y += ((left_bound.points.at(index).get_y() + right_bound.points.at(index).get_y()) / 2.0 - y) / (index + 1);
    }

    return std::pair<double, double>(x, y);
}

std::optional<std::pair<double, double>> Lanelet::get_stopline_center()
{
    if (stop_line.has_value())
    {
        //We already made sure during translation that Point actually consists of two points
        return std::optional<std::pair<double, double>>({
            stop_line->points.at(0).get_x() * 0.5 + stop_line->points.at(1).get_x() * 0.5,
            stop_line->points.at(0).get_y() * 0.5 + stop_line->points.at(1).get_y() * 0.5
        });
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<std::array<std::array<double, 2>, 2>> Lanelet::get_range_x_y()
{
    //The calculation of the center follows a simple assumption: Center = Middle of middle segment (middle value of all points might not be within the lanelet boundaries)
    assert(left_bound.points.size() == right_bound.points.size());

    size_t vec_size = left_bound.points.size();

    //Return empty optional if points are missing
    if (vec_size == 0)
    {
        return std::nullopt;
    }
    
    //Working with numeric limits at start lead to unforseeable behaviour with min and max, thus we now use this approach instead
    bool uninitialized = true;
    double x_min, x_max, y_min, y_max;
    for (size_t index = 0; index < vec_size; ++index)
    {
        if (uninitialized)
        {
            x_min = left_bound.points.at(index).get_x();
            y_min = left_bound.points.at(index).get_y();
            x_max = left_bound.points.at(index).get_x();
            y_max = left_bound.points.at(index).get_y();
            uninitialized = false;
        }

        x_min = std::min(left_bound.points.at(index).get_x(), x_min);
        y_min = std::min(left_bound.points.at(index).get_y(), y_min);
        x_max = std::max(left_bound.points.at(index).get_x(), x_max);
        y_max = std::max(left_bound.points.at(index).get_y(), y_max);

        x_min = std::min(right_bound.points.at(index).get_x(), x_min);
        y_min = std::min(right_bound.points.at(index).get_y(), y_min);
        x_max = std::max(right_bound.points.at(index).get_x(), x_max);
        y_max = std::max(right_bound.points.at(index).get_y(), y_max);
    }

    std::array<std::array<double, 2>, 2> result;
    result[0][0] = x_min;
    result[0][1] = x_max;
    result[1][0] = y_min;
    result[1][1] = y_max;

    return result;
}


std::vector<Point> Lanelet::get_shape()
{
    std::vector<Point> shape; 

    for (auto point : left_bound.points)
    {
        shape.push_back(point);
    }

    for(auto reverse_it_point = right_bound.points.rbegin(); reverse_it_point != right_bound.points.rend(); ++reverse_it_point)
    {
        shape.push_back(*reverse_it_point);
    }

    return shape;
}