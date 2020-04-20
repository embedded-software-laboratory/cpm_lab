#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <iostream>
#include <vector>

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \class Crossing
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a crossing specified in an XML file
 * Is this specific class ever used outside the XML specs?
 * 2020 only
 */
class Crossing : public InterfaceDraw
{
private:
    std::vector<int> crossing_lanelets; //Lanelet ref

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    Crossing(const xmlpp::Node* node);

    //TODO: Getter

    /**
     * TODO: How to handle references? Do I just take them and draw somewhere else? In that case: We do not need a draw function
     * \brief This function is used to draw the data structure that imports this interface
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     * \param orientation - optional: Rotation that needs to be applied before drawing
     * \param translate_x - optional: Translation in x-direction that needs to be applied before drawing
     * \param translate_y - optional: Translation in y-direction that needs to be applied before drawing
     */
    void draw(const DrawingContext& ctx, double scale = 1.0, double orientation = 0.0, double translate_x = 0.0, double translate_y = 0.0) override;
};