#pragma once

#include <string>
#include <vector>

#include "commonroad_classes/geometry/Position.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"

/**
 * \struct TrafficSignElement
 * \brief Specifies a traffic sign element
 * The commonroad XML file specifies specific string values, this restriction is not applied here (for simplicity)
 */
struct TrafficSignElement
{
    std::string traffic_sign_id;
    std::vector<std::string> additional_values;
};

/**
 * \struct TrafficSignPost
 * \brief Specifies a traffic sign post
 * Not directly specified in commonroad, but multiple elements can have the same position and 'virtual' tags
 */
struct TrafficSignPost
{
    std::vector<TrafficSignElement> traffic_sign_elements;
    Position position; //Must be exact according to spec!
    std::vector<bool> is_virtual;
};

/**
 * \class TrafficSign
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a traffic sign specified in an XML file
 */
class TrafficSign : public InterfaceTransform
{
private:
    std::vector<TrafficSignPost> traffic_sign_posts;

public:
    //TODO: Constructor, getter

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}
};