// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <utility>
#include <vector>

#include "commonroad_classes/XMLTranslation.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include "CommonroadDDSGoalState.hpp"

/**
 * \class Interval
 * \brief This class is created as commonroad uses similar class types (easier to handle in translation and as return type)
 * It should also make other classes that use this type more readable.
 * \ingroup lcc_commonroad
 */
class Interval : public InterfaceTransform
{
private:
    //! Internal commonroad interval representation
    std::pair<double, double> interval;

public:
    /**
     * \brief Simple constructor to directly set the interval values
     */
    Interval(const xmlpp::Node* node)
    {
        //We do not assert the interval type here, as the name of the node in this case mostly refers to a specific datatype like 'time' or 'orientation', of which there are many throughout the documentation

        try
        {
            //Both values must exist
            interval.first = xml_translation::get_child_child_double(node, "intervalStart", true).value();
            interval.second = xml_translation::get_child_child_double(node, "intervalEnd", true).value();
        }
        catch(const SpecificationError& e)
        {
            throw SpecificationError(std::string("Could not translate Interval:\n") + e.what());
        }
        catch(...)
        {
            //Propagate error, if any subclass of CommonRoadScenario fails, then the whole translation should fail
            throw;
        }

        //Test output
        // std::cout << "Interval(s): " << std::endl;
        // for (const auto interval : intervals)
        // {
        //     std::cout << "\t" << interval.first << " - " << interval.second << std::endl;
        // }
    }

    //Getter (no setter, as we only want to set Interval at translation or change it using transform_...)

    /**
     * \brief Get interval start
     */
    double get_start() const
    {
        return interval.first;
    }

    /**
     * \brief Get interval end
     */
    double get_end() const
    {
        return interval.second;
    }

    /**
     * \brief Use these functions to get the list of values in the middle of each interval
     */
    double get_interval_avg() const
    {
        return 0.5 * interval.first + 0.5 * interval.second;
    }

    /**
     * \brief Check if the interval is greater than or equal to zero with all values
     */
    bool is_greater_zero()
    {
        bool greater_zero = true;
        greater_zero &= (interval.first >= 0); //Due to sample files, this seems to be allowed to be zero
        greater_zero &= (interval.second > 0);

        return greater_zero;
    }


    //Suppress warning for unused parameter (s)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"

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
        if (scale > 0)
        {
            interval.first *= scale;
            interval.second *= scale;
        }
    }

    /**
     * \brief Rotates the interval by angle around z (gets interpreted as orientation)
     * Takes mod. 2 * PI, but also makes sure afterwards that the interval end is always greater than the start
     * (to compute the mean properly)
     * \param angle Angle to rotate by, in radians
     */
    void rotate_orientation(double angle)
    {
        auto start = std::fmod((interval.first - angle), 2.0 * M_PI);
        auto end = std::fmod((interval.second - angle), 2.0 * M_PI);

        //Make sure that start <= end (makes mean computation easier / no special cases)
        if (end < start) end += 2.0 * M_PI;

        interval.first = start;
        interval.second = end;
    }

    /**
     * \brief Translate to DDS
     * \param ratio Relevant to translate e.g. time information to actual time
     */
    CommonroadDDSInterval to_dds_msg(double ratio = 1.0)
    {
        CommonroadDDSInterval dds_interval;
        dds_interval.interval_start(interval.first * ratio);
        dds_interval.interval_end(interval.second * ratio);

        return dds_interval;
    }

    #pragma GCC diagnostic pop
};