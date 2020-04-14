#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <memory>
#include <vector>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Circle.hpp"
#include "commonroad_classes/geometry/Polygon.hpp"
#include "commonroad_classes/geometry/Rectangle.hpp"
#include "commonroad_classes/geometry/Shape.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class Position
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Position : public InterfaceTransform, public InterfaceDraw
{
private:
    //TODO: Solve below w. inheritance? Or keep it this way? -> Probably easier to keep it this way

    //According to the specification, a position might not be given exactly (as a point)
    bool is_exact;
    
    //Exact position (positionExact)
    std::optional<Point> point;

    //Inexact position (positionInterval)
    std::vector<Circle> circles;
    std::vector<int> lanelet_refs;
    std::vector<Polygon> polygons;
    std::vector<Rectangle> rectangles;

public:
    /**
     * \brief Constructor, set up a position object
     */
    Position(const xmlpp::Node* node);

    /**
     * \brief Second constructor, value is irrelevant - value given s.t. no default constructor exists
     * Call this if you need to use the specified default value from the specs because the value was not set explicitly
     */
    Position(int irrelevant_int);

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

    /**
     * \brief This function is used to draw a shape using the orientation information of this data structure (TODO / WIP, might change)
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     */
    void draw_shape(const DrawingContext& ctx, std::optional<Shape> shape, double scale = 1.0);
    
    void to_dds_msg() {} 

    //TODO: Getter
};