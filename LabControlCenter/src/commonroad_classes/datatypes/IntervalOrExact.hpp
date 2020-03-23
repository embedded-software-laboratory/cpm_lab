#pragma once

#include "commonroad_classes/InterfaceTransform.hpp"

/**
 * \class IntervalOrExact
 * \brief This class is created as commonroad uses similar class types (easier to handle in translation and as return type)
 * It should also make other classes that use this type more readable.
 */
class IntervalOrExact : public InterfaceTransform
{
private:
    double interval_start;
    double interval_end;
    double exact_value;
    bool is_exact;
public:
    /**
     * \brief Simple constructor to directly set the exact value
     * Also tells the class that it is not an IntervalOrExact
     */
    IntervalOrExact(double exact)
    {
        interval_start = 0;
        interval_end = 0;
        exact_value = exact;
        is_exact = true;
    }

    /**
     * \brief Simple constructor to directly set the IntervalOrExact
     * Also tells the class that it is not an exact value
     */
    IntervalOrExact(double start, double end)
    {
        interval_start = start;
        interval_end = end;
        exact_value = 0;
        is_exact = false;
    }

    //Getter (no setter, as we only want to set IntervalOrExact at translation or change it using transform_...)
    bool is_exact()
    {
        return is_exact;
    }

    double get_exact_value()
    {
        return exact_value;
    }
    
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
        exact_value *= scale;
    }
};