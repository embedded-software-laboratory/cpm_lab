#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <cassert>
#include <vector>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/states/StateExact.hpp"
#include "commonroad_classes/states/GoalState.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/InterfaceTransformTime.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include "commonroad_classes/CommonroadDrawConfiguration.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

#include "CommonroadDDSGoalState.hpp"


/**
 * \class PlanningProblem
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a PlanningProblem specified in an XML file
 * \ingroup lcc_commonroad
 */
class PlanningProblem : public InterfaceTransform, public InterfaceDraw, public InterfaceTransformTime
{
private:
    //! Initial state of the planning problem
    std::optional<StateExact> initial_state = std::nullopt;

    //! List of possible / allowed goal states for the planning problem, when starting at the given initial state
    std::vector<GoalState> goal_states;

    //! Required for drawing
    int planning_problem_id;

    //! Draw configuration to know if the init state description should be drawn
    std::shared_ptr<CommonroadDrawConfiguration> draw_configuration;

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs
     * \param node A planning problem node
     * \param _draw_lanelet_refs Function that, given an lanelet reference and the typical drawing arguments, draws a lanelet reference
     * \param _get_lanelet_center Function that returns a lanelet center
     * \param _draw_configuration A shared pointer pointing to the configuration for the scenario that sets which optional parts should be drawn
     */
    PlanningProblem(
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

    //Getter
    /**
     * \brief Get the initial state
     */
    const std::optional<StateExact>& get_initial_state() const;

    /**
     * \brief Get the list of goal states with the same planning problem ID
     */
    const std::vector<GoalState>& get_goal_states() const;

    /**
     * \brief Translate the planning problem to a DDS msg
     * \param time_step_size Relevant to translate time information to actual time
     */
    std::vector<CommonroadDDSGoalState> get_dds_goal_states(double time_step_size);
};