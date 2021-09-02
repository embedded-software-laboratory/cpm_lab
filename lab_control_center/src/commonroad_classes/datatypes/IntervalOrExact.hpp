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
 * \ingroup lcc_commonroad
 */
class IntervalOrExact : public InterfaceTransform
{
private:
    //! Commonroad Interval; if this is set, there is no exact value set
    std::optional<Interval> interval = std::nullopt;
    //! Exact value; if this is set, no interval is set
    std::optional<double> exact = std::nullopt;
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

        //Make sure that the object is not empty
        if (!interval && !exact)
        {
            std::stringstream err_stream;
            err_stream << "Neither interval nor exact value provided in line " << node->get_line() << "!";
            throw SpecificationError(err_stream.str());
        }
    }

    //Getter (no setter, as we only want to set IntervalOrExact at translation or change it using transform_...)

    /**
     * \brief Check if the data is exact or an interval
     */
    bool is_exact()
    {
        return exact.has_value();
    }

    /**
     * \brief Get the exact value, if it exists (else: nullopt)
     */
    const std::optional<double> get_exact_value()
    {
        return exact;
    }

    /**
     * \brief Check if the data is an interval or exact
     */
    bool is_interval()
    {
        return interval.has_value();
    }

    /**
     * \brief Check if all the contained data (also for intervals) is greater than or equal to zero (equality only allowed for intervals)
     */
    bool is_greater_zero()
    {
        bool greater_zero = true;
        if (is_exact())
        {
            greater_zero &= (exact.value() > 0);
        }
        if(is_interval())
        {
            greater_zero &= interval->is_greater_zero();
        }

        return greater_zero;
    }
    
    /**
     * \brief Get interval data or nullopt if it does not exist
     */
    const std::optional<Interval> get_interval()
    {
        return interval;
    }

    /**
     * \brief Get exact value or mean of the stored interval(s)
     */
    double get_mean()
    {
        if (exact.has_value())
        {
            return exact.value();
        }
        else
        {
            return interval.value().get_interval_avg();
        }
    }

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position, or the min lane width (for commonroadscenario) - 0 means: No transformation desired
     * \param angle Rotation of the coordinate system, around the origin, w.r.t. right-handed coordinate system (according to commonroad specs), in radians
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    void transform_coordinate_system(double scale, double angle, double translate_x, double translate_y) override
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
            interval->transform_coordinate_system(scale, angle, translate_x, translate_y);
        }
    }

    /**
     * \brief Rotates the exact value or interval by angle around z (gets interpreted as orientation)
     * Takes mod. 2 * PI, but also makes sure afterwards that the interval end is always greater than the start
     * (to compute the mean properly)
     * \param angle Angle to rotate by, in radians
     */
    void rotate_orientation(double angle)
    {
        if (exact.has_value())
        {
            auto old_value = exact.value_or(0.0);
            exact = std::optional<double>(rotate_orientation_around_z(old_value, angle));
        }

        if (interval.has_value())
        {
            interval->rotate_orientation(angle);
        }
    }

    /**
     * \brief Translate to DDS interval, if not exact
     * \param ratio Relevant to translate e.g. time information to actual time
     */
    CommonroadDDSInterval to_dds_interval(double ratio = 1.0)
    {
        //Throw error if conversion is invalid because of interval type
        if (!interval.has_value())
        {
            throw std::runtime_error("IntervalOrExact cannot be translated to DDS Interval, is exact");
        }

        return interval->to_dds_msg(ratio);
    }
};