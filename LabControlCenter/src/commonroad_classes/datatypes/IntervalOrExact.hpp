#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include "commonroad_classes/XMLTranslation.hpp"
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
    bool exact; //is_exact
public:
    /**
     * \brief Constructor
     */
    IntervalOrExact(const xmlpp::Node* node)
    {
        //TODO: Make sure that this is an interval node type
        //Need to look for several interval types, as we here just use one interval type to cover all possible ones

        //TODO: Sadly, sequences are allowed here as well, so we can have more than one interval
    }

    //Getter (no setter, as we only want to set IntervalOrExact at translation or change it using transform_...)
    bool is_exact()
    {
        return exact;
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
    void transform_coordinate_system(double scale) override
    {
        interval_start *= scale;
        interval_end *= scale;
        exact_value *= scale;
    }
};