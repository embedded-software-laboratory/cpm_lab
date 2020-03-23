#pragma once

#include "commonroad_classes/InterfaceTransform.hpp"

/**
 * \class Interval
 * \brief This class is created as commonroad uses similar class types (easier to handle in translation and as return type)
 * It should also make other classes that use this type more readable.
 */
class Interval : public InterfaceTransform
{
private:
    double interval_start;
    double interval_end;
public:
    /**
     * \brief Simple constructor to directly set the interval values
     */
    Interval(double start, double end)
    {
        interval_start = start;
        interval_end = end;
    }

    //Getter (no setter, as we only want to set Interval at translation or change it using transform_...)
    double get_interval_start()
    {
        return interval_start;
    }

    double get_interval_end()
    {
        return interval_end;
    }

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    virtual void transform_coordinate_system(double scale) override
    {
        interval_start *= scale;
        interval_end *= scale;
    }
};