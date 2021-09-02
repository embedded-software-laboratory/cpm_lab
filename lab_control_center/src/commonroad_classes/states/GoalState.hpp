#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Position.hpp"
#include "commonroad_classes/datatypes/Interval.hpp"
#include "commonroad_classes/datatypes/IntervalOrExact.hpp"
#include "commonroad_classes/InterfaceTransformTime.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include "commonroad_classes/CommonroadDrawConfiguration.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

#include "CommonroadDDSGoalState.hpp"
#include "LCCErrorLogger.hpp"

/**
 * \class GoalState
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a GoalState specified in an XML file
 * \ingroup lcc_commonroad
 */
class GoalState : public InterfaceTransform, public InterfaceDraw, public InterfaceTransformTime
{
private:
    //Commonroad data
    //! Time values should probably be within the range of unsigned doubles; Can only be defined as interval according to spec. Is defined w.r.t. global time step size
    std::optional<IntervalOrExact> time = std::nullopt;
    //! Position of the goal state. Must not be defined
    std::optional<Position> position = std::nullopt; 
    //! Allowed orientation range within the goal state. Must not be defined
    std::optional<Interval> orientation = std::nullopt;
    //! Allowed velocity range within the goal state. Must not be defined
    std::optional<Interval> velocity = std::nullopt;
    //! ID of the planning problem + unique ID for each goal state within, can be used when drawing the goal state
    std::string unique_id = "ERR";
    
    //! Transformation scale of transform_coordinate_system is remembered to draw circles / arrows correctly scaled
    double transform_scale = 1.0;

    //! Look up in draw configuration if some parts should be drawn or not
    std::shared_ptr<CommonroadDrawConfiguration> draw_configuration;

public:
    /**
     * \brief Constructor, set up a goalstate object from a commonroad xml goalstate node
     * \param node Goal state node to translate
     * \param _draw_lanelet_refs Function that, given an lanelet reference and the typical drawing arguments, draws a lanelet reference
     * \param _get_lanelet_center Function to get the center (x, y) of a lanelet, given its ID
     * \param _draw_configuration A shared pointer pointing to the configuration for the scenario that sets which optional parts should be drawn
     */
    GoalState(
        const xmlpp::Node* node,
        std::function<void (int, const DrawingContext&, double, double, double, double)> _draw_lanelet_refs,
        std::function<std::pair<double, double> (int)> _get_lanelet_center,
        std::shared_ptr<CommonroadDrawConfiguration> _draw_configuration
    );

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position, or the min lane width (for commonroadscenario) - 0 means: No transformation desired
     * \param angle Rotation of the coordinate system, around the origin, w.r.t. right-handed coordinate system (according to commonroad specs), in radians
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    void transform_coordinate_system(double scale, double angle, double translate_x, double translate_y) override;

    /**
     * \brief This function is used to change timing-related values, like velocity, where needed
     * \param time_scale The factor with which time step size was changed (e.g. 1.0->2.0 results in a factor of 0.5)
     */
    void transform_timing(double time_scale) override;

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

    /**
     * \brief Convert to DDS representation
     * \param time_step_size Relevant to translate time information to actual time
     */
    CommonroadDDSGoalState to_dds_msg(double time_step_size);

    //Setter
    /**
     * \brief Set a unique ID for this goal state for drawing, to 
     * be able to connect planning problem entries in the view with
     * planning problems shown on the map.
     * \param _unique_id Unique goal state ID, should be: Planning problem ID + planning problem pos. in vector + goal state pos. in vector
     */
    void set_unique_id(std::string _unique_id);

    //Getter
    /**
     * \brief Get the point or interval in time within which the goal state must be reached, or nullopt if not set
     */
    const std::optional<IntervalOrExact>& get_time() const;
    /**
     * \brief Get the position of the goal state, or nullopt if not set
     */
    const std::optional<Position>& get_position() const;
    /**
     * \brief Get the allowed orientation within the goal state, or nullopt if not set
     */
    const std::optional<Interval>& get_orientation() const;
    /**
     * \brief Get the allowed velocity within the goal state, or nullopt if not set
     */
    const std::optional<Interval>& get_velocity() const;
    /**
     * \brief Get the unique ID of the goal state (planning problem ID + unique goal state ID)
     */
    const std::string& get_unique_id() const;
};
