#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include <string>

#include "commonroad_classes/datatypes/IntervalOrExact.hpp"
#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class SignalState
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a SignalState specified in an XML file
 */
class SignalState : public InterfaceDraw
{
private:
    //Commonroad data
    std::optional<IntervalOrExact> time; //Time values should probably be within the range of double (-> not too large), we did not want to define an extra type for this - gets transformed in getter to nanoseconds view
    //These values are set to false if they do not exist
    bool horn;
    bool indicator_left;
    bool indicator_right;
    bool braking_lights;
    bool hazard_warning_lights;
    bool flashing_blue_lights;

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    SignalState(const xmlpp::Node* node);

    /**
     * \brief Takes a boolean string node and translates its value to a boolean
     * TODO: XML translation class instead - then also give a default return value
     * \param child_name Name of the child node which contains the boolean value
     */
    bool get_child_bool(std::string child_name);

    /**
     * \brief This function is used to draw the data structure that imports this interface
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     * \param orientation - optional: Rotation that needs to be applied before drawing
     * \param translate_x - optional: Translation in x-direction that needs to be applied before drawing
     * \param translate_y - optional: Translation in y-direction that needs to be applied before drawing
     */
    void draw(const DrawingContext& ctx, double scale = 1.0, double orientation = 0.0, double translate_x = 0.0, double translate_y = 0.0) override {}

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg(); 

    //TODO: Getter
};
