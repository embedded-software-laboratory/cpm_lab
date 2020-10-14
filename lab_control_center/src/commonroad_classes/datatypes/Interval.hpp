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

/**
 * \class Interval
 * \brief This class is created as commonroad uses similar class types (easier to handle in translation and as return type)
 * It should also make other classes that use this type more readable.
 */
class Interval : public InterfaceTransform
{
private:
    std::vector<std::pair<double, double>> intervals;

public:
    /**
     * \brief Simple constructor to directly set the interval values
     */
    Interval(const xmlpp::Node* node)
    {
        //We do not assert the interval type here, as the name of the node in this case mostly refers to a specific datatype like 'time' or 'orientation', of which there are many throughout the documentation

        //Sadly, sequences are allowed here as well, so we can have more than one interval
        std::vector<double> interval_start;
        std::vector<double> interval_end;

        try
        {
            xml_translation::iterate_children(
                node, 
                [&] (const xmlpp::Node* child) 
                {
                    interval_start.push_back(xml_translation::get_first_child_double(child));
                }, 
                "intervalStart"
            );
            
            xml_translation::iterate_children(
                node, 
                [&] (const xmlpp::Node* child) 
                {
                    interval_end.push_back(xml_translation::get_first_child_double(child));
                }, 
                "intervalEnd"
            );
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
        

        if (interval_start.size() != interval_end.size())
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Different amount of start and end nodes in Interval, line " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }

        if (interval_start.size() == 0)
        {
            std::stringstream error_msg_stream;
            error_msg_stream << "Unexpected empty Interval, line " << node->get_line();
            throw SpecificationError(error_msg_stream.str());
        }

        for(size_t i = 0; i < interval_start.size(); ++i)
        {
            intervals.push_back(std::make_pair(interval_start.at(i), interval_end.at(i)));
        }

        //Test output
        std::cout << "Interval(s): " << std::endl;
        for (const auto interval : intervals)
        {
            std::cout << "\t" << interval.first << " - " << interval.second << std::endl;
        }
    }

    //Getter (no setter, as we only want to set Interval at translation or change it using transform_...)

    /**
     * \brief Determine how many intervals are stored in this data structure
     */
    size_t get_intervals_size()
    {
        return intervals.size();
    }

    /**
     * \brief For constant for loop - interval type: std::pair<double, double>
     */
    std::vector<std::pair<double, double>>::const_iterator cbegin() const
    {
        return intervals.cbegin();
    }

    /**
     * \brief For constant for loop - interval type: std::pair<double, double>
     */
    std::vector<std::pair<double, double>>::const_iterator cend() const
    {
        return intervals.cend();
    }

    /**
     * \brief Use these functions to get the list of values in the middle of each interval
     */
    std::vector<double> get_interval_avg() const
    {
        std::vector<double> avgs;
        for (const auto interval : intervals)
        {
            avgs.push_back((interval.first + interval.second) / 2.0);
        }

        return avgs;
    }

    bool is_greater_zero()
    {
        bool greater_zero = true;
        for (auto interval : intervals)
        {
            greater_zero &= (interval.first > 0);
            greater_zero &= (interval.second > 0);

            if (!greater_zero) break;
        }

        return greater_zero;
    }


    //Suppress warning for unused parameter (s)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     * \param translate_x Currently ignored, must be changed if this is used for position values
     * \param translate_y Currently ignored, must be changed if this is used for position values
     */
    void transform_coordinate_system(double scale, double angle, double translate_x, double translate_y) override
    {
        for (auto &interval : intervals)
        {
            if (scale > 0)
            {
                interval.first *= scale;
                interval.second *= scale;
            }
        }
    }

    #pragma GCC diagnostic pop
};