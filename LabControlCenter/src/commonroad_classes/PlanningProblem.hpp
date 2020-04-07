#pragma once

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

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
    std::optional<StateExact> initial_state;
    std::optional<GoalState> goal_state;

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