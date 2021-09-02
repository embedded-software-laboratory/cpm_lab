#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Point.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/InterfaceGeometry.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include "CommonroadDDSShape.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \class Rectangle
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 * \ingroup lcc_commonroad
 */
class Rectangle : public InterfaceTransform, public InterfaceDraw, public InterfaceGeometry
{
private:
    //! Length of the rectangle, must be unsigned
    double length;
    //! Width of the rectangle, must be unsigned
    double width; 
    //! Center of the rectangle, must not be set (then: interpreted as origin)
    std::optional<Point> center = std::nullopt; 
    //! Orientation of the rectangle, must not be set (then: interpreted as 0)
    std::optional<double> orientation = std::nullopt; 

public:
    /**
     * \brief Constructor, creates a rectangle object from a commonroad xml rectangle node
     */
    Rectangle(const xmlpp::Node* node);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position, or the min lane width (for commonroadscenario) - 0 means: No transformation desired
     * \param angle Rotation of the coordinate system, around the origin, w.r.t. right-handed coordinate system (according to commonroad specs), in radians
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    void transform_coordinate_system(double scale, double angle, double translate_x, double translate_y) override;

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
     * \brief Translates all relevant parts of the data structure to a DDS object, which is returned
     * No interface was created for this function because the return type depends on the class
     */
    CommonroadDDSRectangle to_dds_msg();

    //Getter
    /**
     * \brief Get the orientation of the rectangle, if it exists, or a nullopt
     */
    std::optional<double> get_orientation();
    /**
     * \brief Get the center point of the rectangle, if it exists, or a nullopt (then: interpret this as origin)
     */
    const std::optional<Point>& get_center() const;
    /**
     * \brief Get the length of the rectangle
     */
    double get_length();
    /**
     * \brief Get the width of the rectangle
     */
    double get_width();
};