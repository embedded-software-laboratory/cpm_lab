#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <vector>

#include "commonroad_classes/geometry/Point.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/InterfaceGeometry.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include "CommonroadDDSShape.hpp"

#include "LCCErrorLogger.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \class Polygon
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Polygon : public InterfaceTransform, public InterfaceDraw, public InterfaceGeometry
{
private:
    std::vector<Point> points; //min. 3

    //Remember line in commonroad file for logging
    int commonroad_line = 0;

public:
    Polygon(const xmlpp::Node* node);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale, double translate_x, double translate_y) override;

    /**
     * \brief This function is used to draw the data structure that imports this interface
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * To change local translation, just transform the coordinate system beforehand
     * As this does not always work with local orientation (where sometimes the translation in the object must be called before the rotation if performed, to rotate within the object's coordinate system),
     * local_orientation was added as a parameter
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     * \param global_orientation - optional: Rotation that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_x - optional: Translation in x-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param global_translate_y - optional: Translation in y-direction that needs to be applied before drawing - set as global transformation to the whole coordinate system
     * \param local_orientation - optional: Rotation that needs to be applied within the object's coordinate system
     */
    void draw(const DrawingContext& ctx, double scale = 1.0, double global_orientation = 0.0, double global_translate_x = 0.0, double global_translate_y = 0.0, double local_orientation = 0.0) override;

    /**
     * \brief Get center (positional value) of the circle
     * \return Center of the circle
     */
    std::pair<double, double> get_center() override;

    /**
     * \brief Const getter for polygon points
     */
    const std::vector<Point>& get_points() const;
    
    /**
     * \brief Translates all relevant parts of the data structure to a DDS object, which is returned
     * No interface was created for this function because the return type depends on the class
     */
    CommonroadDDSPolygon to_dds_msg();
};