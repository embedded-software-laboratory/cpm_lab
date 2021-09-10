#pragma once

#include "src/commonroad_classes/geometry/Point.hpp"

/**
 * \class InterfaceGeometry
 * \brief This interface requires the deriving classes to implement a transfrom function
 * It is mainly used for clarity, to define common behaviour
 * \ingroup lcc_commonroad
 */
class InterfaceGeometry
{
public:
    /**
     * \brief Get center (positional value) of a certain (composed) shape
     * \return Center of the shape
     */
    virtual std::pair<double, double> get_center() = 0;

    //! Destructor. Good practice
    virtual ~InterfaceGeometry() {};
};