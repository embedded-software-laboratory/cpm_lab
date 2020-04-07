#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Position.hpp"
#include "commonroad_classes/datatypes/IntervalOrExact.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class GoalState
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a GoalState specified in an XML file
 */
class GoalState : public InterfaceTransform
{
private:
    //Commonroad data
    std::optional<IntervalOrExact> time; //Time values should probably be within the range of double, we did not want to define an extra type for this - gets transformed in getter to nanoseconds view
    std::optional<Position> position;
    double orientation;
    double velocity;

public:
    /**
     * \brief Constructor, set up a goalstate object
     */
    GoalState(const xmlpp::Node* node) {};

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}

    void draw() {} //Give Cairo context?
    void to_dds_msg() {} 

    //TODO: Getter
};
