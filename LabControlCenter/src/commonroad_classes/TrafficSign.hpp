#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Position.hpp"

#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \struct TrafficSignPost
 * \brief Specifies a traffic sign element
 * The commonroad XML file specifies specific string values, this restriction is not applied here (for simplicity)
 */
struct TrafficSignPost
{
    std::string traffic_sign_id;
    std::vector<std::string> additional_values;
};

/**
 * \struct TrafficSignElement
 * \brief Specifies a traffic sign post
 * Not directly specified in commonroad, but multiple elements can have the same position and 'virtual' tags
 * TODO: Look up default values, position must not be specified!
 */
struct TrafficSignElement
{
    std::vector<TrafficSignPost> traffic_sign_posts;
    std::optional<Position> position; //Must be exact according to spec! Ptr because we do not have a default constructor
    std::vector<bool> is_virtual;
};

/**
 * \class TrafficSign
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a traffic sign specified in an XML file
 * 2020 only! (Not specified in 2018 specs)
 */
class TrafficSign : public InterfaceTransform
{
private:
    std::vector<TrafficSignElement> traffic_sign_elements;

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs (TODO: Custom error type for this case)
     * \param node A trafficSign node
     */
    TrafficSign(const xmlpp::Node* node);

    //TODO: Constructor, getter

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}
};