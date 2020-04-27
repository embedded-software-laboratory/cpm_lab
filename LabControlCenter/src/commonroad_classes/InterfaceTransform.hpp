#pragma once

/**
 * \class InterfaceTransform
 * \brief This interface requires the deriving classes to implement a transfrom function
 * It is mainly used for clarity, to define common behaviour
 */
class InterfaceTransform
{
public:
    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position, or the min lane width (for commonroadscenario) - 0 means: No transformation desired
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    virtual void transform_coordinate_system(double scale, double translate_x, double translate_y) = 0;

    //Good practice
    virtual ~InterfaceTransform() {};
};