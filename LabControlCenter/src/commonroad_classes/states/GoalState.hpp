#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Position.hpp"
#include "commonroad_classes/datatypes/Interval.hpp"
#include "commonroad_classes/datatypes/IntervalOrExact.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/XMLTranslation.hpp"


/**
 * \class GoalState
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a GoalState specified in an XML file
 */
class GoalState : public InterfaceTransform, public InterfaceDraw
{
private:
    //Commonroad data
    std::optional<IntervalOrExact> time; //Time values should probably be within the range of double, we did not want to define an extra type for this - gets transformed in getter to nanoseconds view
    std::optional<Position> position; //Must not be defined 
    std::optional<Interval> orientation; //Must not be defined
    std::optional<Interval> velocity; //Must not be defined

    //Transformation scale of transform_coordinate_system is remembered to draw circles / arrows correctly scaled
    double transform_scale = 1.0;

public:
    /**
     * \brief Constructor, set up a goalstate object
     */
    GoalState(const xmlpp::Node* node);

    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * \param scale The factor by which to transform all number values related to position
     */
    void transform_coordinate_system(double scale) override;

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
     * \brief Setter for drawing lanelet references (Can also be constructed without this)
     * \param _draw_lanelet_refs Function that, given an lanelet reference and the typical drawing arguments, draws a lanelet reference
     */
    void set_lanelet_ref_draw_function(std::function<void (int, const DrawingContext&, double, double, double, double, double)> _draw_lanelet_refs);

    void to_dds_msg() {} 

    //TODO: Getter
};
