#pragma once

#include "src/commonroad_classes/geometry/Point.hpp"

/**
 * \class InterfaceGeometry
 * \brief This interface requires the deriving classes to implement a transfrom function
 * It is mainly used for clarity, to define common behaviour
 */
class InterfaceGeometry
{
public:
    /**
     * \brief Get center (positional value) of a certain (composed) shape
     * \return Center of the shape
     */
    virtual std::pair<double, double> get_center() = 0;

    //Good practice
    virtual ~InterfaceGeometry() {};
};