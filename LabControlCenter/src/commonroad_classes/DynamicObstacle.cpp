#include "commonroad_classes/DynamicObstacle.hpp"

DynamicObstacle::DynamicObstacle(const xmlpp::Node* node)
{
    //TODO: Warn in case node is not static obstacle / does not have static obstacle role
    std::string obstacle_type_text = xml_translation::get_child_child_text(node, "type", true); //Must exist
    if (obstacle_type_text.compare("unknown") == 0)
    {
        type = ObstacleTypeDynamic::Unknown;
    }
    else if (obstacle_type_text.compare("car") == 0)
    {
        type = ObstacleTypeDynamic::Car;
    }
    else if (obstacle_type_text.compare("truck") == 0)
    {
        type = ObstacleTypeDynamic::Truck;
    }
    else if (obstacle_type_text.compare("bus") == 0)
    {
        type = ObstacleTypeDynamic::Bus;
    }
    else if (obstacle_type_text.compare("motorcycle") == 0)
    {
        type = ObstacleTypeDynamic::Motorcycle;
    }
    else if (obstacle_type_text.compare("bicycle") == 0)
    {
        type = ObstacleTypeDynamic::Bicycle;
    }
    else if (obstacle_type_text.compare("pedestrian") == 0)
    {
        type = ObstacleTypeDynamic::Pedestrian;
    }
    else if (obstacle_type_text.compare("priorityVehicle") == 0)
    {
        type = ObstacleTypeDynamic::PriorityVehicle;
    }
    else if (obstacle_type_text.compare("train") == 0)
    {
        type = ObstacleTypeDynamic::Train;
    }
    else if (obstacle_type_text.compare("parkedVehicle") == 0 || 
        obstacle_type_text.compare("constructionZone") == 0 || 
        obstacle_type_text.compare("roadBoundary") == 0)
    {
        //Behavior for dynamic types, which should not be used here
        std::cerr << "TODO: Better warning // Node element not conformant to specs - usage of dynamic type for static object (obstacleType)" << std::endl;
        type = ObstacleTypeDynamic::WrongStaticType;
    }
    else
    {
        std::cerr << "TODO: Better warning // Node element not conformant to specs (obstacleType) in" << std::endl;
        type = ObstacleTypeDynamic::NotInSpec;
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

    //Only one of the two must exist
    const auto trajectory_node = xml_translation::get_child_if_exists(node, "trajectory", false);
    const auto occupancy_node = xml_translation::get_child_if_exists(node, "occupancySet", false);
    if (! (trajectory_node || occupancy_node))
    {
        std::cerr << "TODO: Better warning // Trajectory / occupancy not defined for dynamic object (one must be defined) - line " << trajectory_node->get_line() << std::endl;
    }
    if (trajectory_node && occupancy_node)
    {
        std::cerr << "TODO: Better warning // Trajectory / occupancy both defined for dynamic object (only one must be defined) - line " << trajectory_node->get_line() << std::endl;
    }

    //TODO: Translate trajectory, occupancy, initial signal state
}

/******************************Interface functions***********************************/

void DynamicObstacle::draw(const DrawingContext& ctx, double scale) 
{
    //TODO: Different color / sticker / ... based on type
    ctx->set_source_rgb(1.0,0.5,0.0);

    if (initial_state.has_value())
    {
        ctx->save();

        initial_state->transform_context(ctx, scale);

        if (shape.has_value())
        {
            shape->draw(ctx, scale);
        }
        else
        {
            std::cerr << "TODO: Better warning // Cannot draw shape at position, no value set for shape" << std::endl;
        }

        ctx->restore();
    }
    else
    {
        std::cerr << "TODO: Better warning // Cannot draw StaticObstacle, initial state value is missing" << std::endl;
    }
}