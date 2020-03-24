#pragma once

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
    Shape shape;
    State initial_state;

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