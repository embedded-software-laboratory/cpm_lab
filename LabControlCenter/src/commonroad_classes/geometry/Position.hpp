#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <memory>
#include <vector>

#include "commonroad_classes/geometry/Circle.hpp"
#include "commonroad_classes/geometry/Polygon.hpp"
#include "commonroad_classes/geometry/Rectangle.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class Position
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Position : public InterfaceTransform
{
private:
    //TODO: Solve below w. inheritance? Or keep it this way? -> Probably easier to keep it this way

    //According to the specification, a position might not be given exactly (as a point)
    bool is_exact;
    
    //Exact position (positionExact)
    std::unique_ptr<Point> point;

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
     * Set up move and copy semantics (rule of five) because we use a unique_ptr
     */
    Position(const Position&) = delete;               // Copy constructor
    Position& operator=(const Position&) = delete;  // Copy assignment
    Position(Position&& other) : point(std::move(other.point)) {}   
    Position& operator=(Position&& other) 
    {
        point = std::move(other.point);
        return *this;
    }

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override {}

    void draw() {} //Give Cairo context?
    void to_dds_msg() {} 

    //TODO: Getter
};