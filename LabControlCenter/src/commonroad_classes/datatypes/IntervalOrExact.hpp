#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/datatypes/Interval.hpp"

#include "commonroad_classes/XMLTranslation.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"

#include "commonroad_classes/SpecificationError.hpp"

/**
 * \class IntervalOrExact
 * \brief This class is created as commonroad uses similar class types (easier to handle in translation and as return type)
 * It should also make other classes that use this type more readable.
 */
class IntervalOrExact : public InterfaceTransform
{
private:
    std::optional<Interval> interval;
    std::optional<double> exact;
public:
    /**
     * \brief Constructor
     */
    IntervalOrExact(const xmlpp::Node* node)
    {
        //We do not assert the interval type here, as the name of the node in this case mostly refers to a specific datatype like 'time' or 'orientation', of which there are many throughout the documentation

        try
        {
            //We must either have an exact or an interval node
            const auto exact_node = xml_translation::get_child_if_exists(node, "exact", false);
            if (exact_node)
            {
                exact = xml_translation::get_child_child_double(node, "exact", true);
            }
            else
            {
                //Show warning if no interval exists as well
                const auto interval_start_node = xml_translation::get_child_if_exists(node, "intervalStart", true);

                if(interval_start_node)
                {
                    interval = std::optional<Interval>(std::in_place, node);
                }
            }
        }
        catch(const SpecificationError& e)
        {
            throw SpecificationError(std::string("Could not translate IntervalOrExact:\n") + e.what());
        }
        catch(...)
        {
            //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
            throw;
        }
        
    }

    //Getter (no setter, as we only want to set IntervalOrExact at translation or change it using transform_...)
    bool is_exact()
    {
        return exact.has_value();
    }

    const std::optional<double> get_exact_value()
    {
        return exact;
    }

    bool is_interval()
    {
        return interval.has_value();
    }
    
    const std::optional<Interval> get_interval()
    {
        return interval;
    }

    double get_mean()
    {
        if (exact.has_value())
        {
            return exact.value();
        }
        else
        {
            double sum;
            double amnt;
            for (auto value : interval.value().get_interval_avg())
            {
                sum += value;
                amnt += 1;
            }

            if (amnt > 0)
            {
                return sum / amnt;
            }
            else
            {
                return 0.0;
            }
        }
    }

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     * \param translate_x Currently ignored, must be changed if this is used for position values
     * \param translate_y Currently ignored, must be changed if this is used for position values
     */
    void transform_coordinate_system(double scale, double translate_x, double translate_y) override
    {
        if (exact.has_value())
        {
            if (scale > 0)
            {
                double transformed_exact = exact.value() * scale;
                exact = std::optional<double>(transformed_exact);
            }
        }

        if (interval.has_value())
        {
            interval->transform_coordinate_system(scale, translate_x, translate_y);
        }
    }
};