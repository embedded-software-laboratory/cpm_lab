#pragma once

#include "commonroad_classes/states/StateExact.hpp"
#include "commonroad_classes/states/GoalState.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"


/**
 * \class PlanningProblem
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a PlanningProblem specified in an XML file
 */
class PlanningProblem : public InterfaceTransform
{
private:
    StateExact initial_state;
    GoalState goal_state;

public:
    //TODO: Constructor, getter

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}
};