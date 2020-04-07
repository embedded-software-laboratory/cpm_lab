#pragma once

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Shape.hpp"

#include "commonroad_classes/states/State.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"

/**
 * \enum class ObstacleTypeStatic
 * \brief Specifies static obstacle types, as in commonroad, NotInSpec for types that should not exist
 */
enum class ObstacleTypeStatic {Unknown, ParkedVehicle, ConstructionZone, RoadBoundary, NotInSpec};

/**
 * \class StaticObstacle
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a StaticObstacle specified in an XML file
 */
class StaticObstacle : public InterfaceTransform
{
private:
    ObstacleTypeStatic type;
    std::optional<Shape> shape;
    std::optional<State> initial_state;

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