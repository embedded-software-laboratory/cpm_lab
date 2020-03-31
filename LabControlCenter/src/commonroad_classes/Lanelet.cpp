#include "commonroad_classes/Lanelet.hpp"

Lanelet::Lanelet(const xmlpp::Node* node)
{
    //2018 and 2020
    left_bound = translate_bound(node, "leftBound");
    right_bound = translate_bound(node, "rightBound");
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

Bound Lanelet::translate_bound(const xmlpp::Node* node, std::string name)
{
    bound_node = xml_translation::get_child_if_exists(node, name, true); //True because this is a required part of both specs (2018 and 2020)

    //Same for both specs
    Bound bound;

    bound.points = xml_translation::get_children(
        bound_node, 
        [] (const xmlpp::Node* child) 
        {
            return Point(child);
        }, 
        "point"
    );

    //Warning if bound does not meet specification
    if (bound.points.size() < 2)
    {
        std::cerr << "TODO: Better warning // Bound does not contain min amount of children" << std::endl;
    }

    const auto line_node = xml_translation::get_child_if_exists(bound_node, "lineMarking", false);
    if (line_node)
    {
        bound.line_marking = translate_line_marking(line_node);        
    }
    else
    {
        bound.line_marking = LineMarking::Unspecified;
    }
    
    return bound;
}

std::vector<int> Lanelet::translate_refs(const xmlpp::Node* node, std::string name)
{
    //Refs are optional, so this element might not exist in the XML file
    return xml_translation::get_elements_with_attribute(
        node,
        [] (std::string text) {
            return xml_translation::string_to_int(text);
        },
        name,
        "ref"
    );
}

Adjacent Lanelet::translate_adjacent(const xmlpp::Node* node, std::string name)
{
    //Adjacents are optional, so this element might not exist
    return xml_translation::get_children(
        node,
        [] (const xmlpp::Node* child) 
        {
            Adjacent adjacent;

            adjacent.ref_id = xml_translation::get_attribute_int(child, "ref", true);
            
            std::string direction_string = xml_translation::get_attribute_text(child, "drivingDir", true);
            if(direction_string.compare("same") == 0)
            {
                adjacent.direction = DrivingDirection::Same;
            }
            else if(direction_string.compare("same") == 0)
            {
                adjacent.direction = DrivingDirection::Opposite;
            }
            else {
                std::cerr << "TODO: Better warning // Specified driving direction not part of specs, saved as NotInSpec, in line " << child->get_line() << std::endl;
                adjacent.direction = DrivingDirection::NotInSpec;
            }

            return adjacent;
        },
        name
    );
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
        std::vector<Point> line_points = xml_translation::get_children(
            line_node, 
            [] (const xmlpp::Node* child) 
            {
                return Point(child);
            }, 
            "point"
        );

        if (line_points.size() != 2)
        {
            std::cerr << "TODO: Better warning // Specified stop line has too many points, not part of specs, in line " << line_node->get_line() << std::endl;
        }

        line.point_1 = line_points.at(0);
        line.point_2 = line_points.at(1);

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
    
}

VehicleType Lanelet::translate_users(const xmlpp::Node* node, std::string name)
{
    //Users are optional, so this element might not exist
}

LineMarking Lanelet::translate_line_marking(const xmlpp::Node* line node)
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
        std::cerr << "TODO: Better warning // Specified line marking not part of specs, saved as NotInSpec in " << line_node->get_line() << std::endl;
        return LineMarking::NotInSpec;
    }
}