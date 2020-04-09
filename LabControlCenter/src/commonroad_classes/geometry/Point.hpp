#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class Point
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Point : public InterfaceTransform, public InterfaceDraw
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
     * \brief Second constructor, value is irrelevant - value given s.t. no default constructor exists
     * Call this if you need to use the specified default value from the specs because the value was not set explicitly
     */
    Point(int irrelevant_int);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}

    /**
     * \brief This function is used to draw the data structure that imports this interface
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     */
    void draw(const DrawingContext& ctx, double scale = 1.0) override;

    void to_dds_msg() {}

    //Getter
    double get_x();
    double get_y();
    double get_z();
};