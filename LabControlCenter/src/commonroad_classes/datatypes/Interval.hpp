#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <utility>
#include <vector>

#include "commonroad_classes/XMLTranslation.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"

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
        //TODO: Make sure that this is an interval node type
        //Need to look for several interval types, as we here just use one interval type to cover all possible ones

        //Sadly, sequences are allowed here as well, so we can have more than one interval
        std::vector<double> interval_start;
        xml_translation::iterate_children(
            node, 
            [&] (const xmlpp::Node* child) 
            {
                interval_start.push_back(xml_translation::get_first_child_double(child));
            }, 
            "intervalStart"
        );
        
        std::vector<double> interval_end;
        xml_translation::iterate_children(
            node, 
            [&] (const xmlpp::Node* child) 
            {
                interval_end.push_back(xml_translation::get_first_child_double(child));
            }, 
            "intervalEnd"
        );

        if (interval_start.size() != interval_end.size())
        {
            std::cerr << "TODO: Better warning // Different amount of start and end nodes in Interval, line " << node->get_line() << std::endl;
        }

        if (interval_start.size() == 0)
        {
            std::cerr << "TODO: Better warning // Unexpected empty interval, line " << node->get_line() << std::endl;
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

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override
    {
        for (auto &interval : intervals)
        {
            interval.first *= scale;
            interval.second *= scale;
        }
    }
};