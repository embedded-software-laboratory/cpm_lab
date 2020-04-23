#include "commonroad_classes/StaticObstacle.hpp"

StaticObstacle::StaticObstacle(const xmlpp::Node* node)
{
    //TODO: Warn in case node is not static obstacle / does not have static obstacle role
    
    std::string obstacle_type_text = xml_translation::get_child_child_text(node, "type", true); //Must exist
    if (obstacle_type_text.compare("unknown") == 0)
    {
        type = ObstacleTypeStatic::Unknown;
    }
    else if (obstacle_type_text.compare("parkedVehicle") == 0)
    {
        type = ObstacleTypeStatic::ParkedVehicle;
    }
    else if (obstacle_type_text.compare("constructionZone") == 0)
    {
        type = ObstacleTypeStatic::ConstructionZone;
    }
    else if (obstacle_type_text.compare("roadBoundary") == 0)
    {
        type = ObstacleTypeStatic::RoadBoundary;
    }
    else if (obstacle_type_text.compare("car") == 0 || 
        obstacle_type_text.compare("truck") == 0 || 
        obstacle_type_text.compare("bus") == 0 || 
        obstacle_type_text.compare("motorcycle") == 0 || 
        obstacle_type_text.compare("bicycle") == 0 || 
        obstacle_type_text.compare("pedestrian") == 0 || 
        obstacle_type_text.compare("priorityVehicle") == 0 || 
        obstacle_type_text.compare("train") == 0)
    {
        //Behavior for dynamic types, which should not be used here
        std::cerr << "TODO: Better warning // Node element not conformant to specs - usage of dynamic type for static object (obstacleType)" << std::endl;
        type = ObstacleTypeStatic::WrongDynamicType;
    }
    else
    {
        std::cerr << "TODO: Better warning // Node element not conformant to specs (obstacleType) in" << std::endl;
        type = ObstacleTypeStatic::NotInSpec;
    }
    
    const auto shape_node = xml_translation::get_child_if_exists(node, "shape", true); //Must exist
    if (shape_node)
    {
        shape = std::optional<Shape>{std::in_place, shape_node};
    }

    const auto state_node = xml_translation::get_child_if_exists(node, "initialState", true); //Must exist
    if (state_node)
    {
        initial_state = std::optional<State>{std::in_place, state_node};
    }

    const auto trajectory_node = xml_translation::get_child_if_exists(node, "trajectory", false); //Must not exist
    const auto occupancy_node = xml_translation::get_child_if_exists(node, "occupancySet", false); //Must not exist
    if (trajectory_node || occupancy_node)
    {
        std::cerr << "TODO: Better warning // Trajectory / occupancy defined for static object (not allowed)" << std::endl;
    }
}

/******************************Interface functions***********************************/

void StaticObstacle::draw(const DrawingContext& ctx, double scale, double global_orientation, double global_translate_x, double global_translate_y, double local_orientation)
{
    ctx->save();

    //Perform required translation + rotation
    ctx->translate(global_translate_x, global_translate_y);
    ctx->rotate(global_orientation);

    //TODO: Different color / sticker / ... based on type
    switch(type)
    {
        case ObstacleTypeStatic::Unknown:
            ctx->set_source_rgb(0.1,0.1,0.1);
            break;
        case ObstacleTypeStatic::ParkedVehicle:
            ctx->set_source_rgb(0,0.1,0.5);
            break;
        case ObstacleTypeStatic::ConstructionZone:
            ctx->set_source_rgb(0.8,0,0.2);
            break;
        case ObstacleTypeStatic::RoadBoundary:
            ctx->set_source_rgb(0.5,0.5,0.5);
            break;
    }

    if (initial_state.has_value())
    {
        initial_state->transform_context(ctx, scale);

        if (shape.has_value())
        {
            shape->draw(ctx, scale, 0, 0, 0, local_orientation);
        }
        else
        {
            std::cerr << "TODO: Better warning // Cannot draw shape at position, no value set for shape" << std::endl;
        }
    }
    else
    {
        std::cerr << "TODO: Better warning // Cannot draw StaticObstacle, initial state value is missing" << std::endl;
    }

    ctx->restore();
}

void StaticObstacle::set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double, double)> _draw_lanelet_refs)
{
    if(initial_state.has_value())
    {
        initial_state->set_lanelet_ref_draw_function(_draw_lanelet_refs);
    }
}