#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <vector>

#include "commonroad_classes/geometry/Circle.hpp"
#include "commonroad_classes/geometry/Polygon.hpp"
#include "commonroad_classes/geometry/Rectangle.hpp"

#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class Shape
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Shape : public InterfaceTransform
{
private:
    std::vector<Circle> circles;
    std::vector<Polygon> polygons;
    std::vector<Rectangle> rectangles;

public:
    Shape(const xmlpp::Node* node);

    //TODO: Draw function, maybe given a cairo context
    void draw() {}

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