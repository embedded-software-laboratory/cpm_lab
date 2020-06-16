#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include <vector>

#include "commonroad_classes/geometry/Shape.hpp"

#include "commonroad_classes/states/Occupancy.hpp"
#include "commonroad_classes/states/SignalState.hpp"
#include "commonroad_classes/states/State.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include "CommonroadDDSShape.hpp"

#include "LCCErrorLogger.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \enum class ObstacleTypeDynamic
 * \brief Specifies dynamic obstacle types, as in commonroad, NotInSpec for types that should not exist
 * 2018 and 2020 differ because in 2020, we have static and dynamic obstacles, whereas in 2018, we only have obstacles of type static or dynamic
 * Nonetheless, we should only use the dynamic types here, also according to 2018 specs (We throw an error if a wrong (static) type is used)
 * We do not need to store "role", as we already have two different classes for that
 */
enum class ObstacleTypeDynamic {Unknown, Car, Truck, Bus, Motorcycle, Bicycle, Pedestrian, PriorityVehicle, Train};

/**
 * \struct CommonTrajectoryPoint
 * \brief This class is used as a part of the return type for the trajectory getter
 * It allows to conveniently store all relevant information of each CommonTrajectoryPoint
 */
struct CommonTrajectoryPoint
{
    std::optional<std::pair<double, double>> position; //x, y
    std::optional<int> lanelet_ref; //Must be set if position is not set
    std::optional<double> orientation; //yaw
    std::optional<IntervalOrExact> time; //Must exist, but is not default-constructable -> use optional
    std::optional<IntervalOrExact> velocity;

    CommonroadDDSShape shape; //Occupancy set or lanelet ref might encode current position in form of a shape

    bool is_exact;
    //is_moving is set in the simulation part, because it just depends on the overall trajectory size
};

/**
 * \struct CommonroadTrajectory
 * \brief This class is used as a return type for the trajectory getter
 * It allows to conveniently store all relevant information of the dynamic obstacle
 */
struct CommonroadTrajectory
{
    std::vector<CommonTrajectoryPoint> trajectory;
    std::optional<ObstacleTypeDynamic> obstacle_type;
};

/**
 * \class DynamicObstacle
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a DynamicObstacle specified in an XML file
 */
class DynamicObstacle : public InterfaceTransform, public InterfaceDraw
{
private:
    //Commonroad type
    std::optional<ObstacleTypeDynamic> type;
    std::string obstacle_type_text;
    std::optional<Shape> shape;
    std::optional<State> initial_state;
    std::optional<SignalState> initial_signal_state;

    //Choice in specification - thus, only one of these two value will be valid for each object
    std::vector<State> trajectory;
    std::vector<Occupancy> occupancy_set;

    std::vector<SignalState> signal_series;

    //Transformation scale of transform_coordinate_system is remembered to draw text correctly scaled
    double transform_scale = 1.0;

    //Remember line in commonroad file for logging
    int commonroad_line = 0;

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs
     * \param node A (dynamic) obstacle node
     */
    DynamicObstacle(const xmlpp::Node* node);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale, double translate_x, double translate_y) override;

    /**
     * \brief This function is used to draw the data structure that imports this interface
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * To change local translation, just transform the coordinate system beforehand
     * As this does not always work with local orientation (where sometimes the translation in the object must be called before the rotation if performed, to rotate within the object's coordinate system),
     * local_orientation was added as a parameter
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     * \param global_orientation - optional: Rotation that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_x - optional: Translation in x-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_y - optional: Translation in y-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param local_orientation - optional: Rotation that needs to be applied within the object's coordinate system
     */
    void draw(const DrawingContext& ctx, double scale = 1.0, double global_orientation = 0.0, double global_translate_x = 0.0, double global_translate_y = 0.0, double local_orientation = 0.0) override;

    //Helper function for draw, because this is done multiple times
    void draw_shape_with_text(const DrawingContext& ctx, double scale = 1.0, double local_orientation = 0.0);
    void draw_text(const DrawingContext& ctx, double scale, double local_orientation, std::pair<double, double> center);

    /**
     * \brief Setter for drawing lanelet references (Can also be constructed without this)
     * \param _draw_lanelet_refs Function that, given an lanelet reference and the typical drawing arguments, draws a lanelet reference
     */
    void set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs);

    //Getter
    /**
     * \brief Returns a trajectory constructed from occupancy or trajectory data, and further dynamic obstacle information
     * Throws errors if expected types are missing
     */
    CommonroadTrajectory get_obstacle_dynamics();

    std::string get_obstacle_type_text();
    const std::optional<ObstacleTypeDynamic>& get_type() const;
    const std::optional<Shape>& get_shape() const;
    const std::optional<State>& get_initial_state() const;
    const std::optional<SignalState>& get_initial_signal_state() const;
    const std::vector<State>& get_trajectory() const;
    const std::vector<Occupancy>& get_occupancy_set() const;
    const std::vector<SignalState>& get_signal_series() const;
};