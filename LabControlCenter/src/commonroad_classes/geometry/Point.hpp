#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class Point
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Point : public InterfaceTransform
{
private:
    double x;
    double y;
    double z; //must not be set (then ??)
public:
    /**
     * \brief Constructor, set up a point object
     */
    Point(const xmlpp::Node* node);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}

    void to_dds_msg() {}

    //TODO: Getter
};