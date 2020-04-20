#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include <map>
#include <iostream>

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

/**
 * \struct Incoming
 * \brief Specifies a part of the intersection, as in commonroad
 */
struct Incoming
{
    std::optional<int> incoming_lanelet; //Lanelet ref - must exist, optional in case of faulty XML-file
    std::optional<int> successors_right; //Lanelet ref
    std::optional<int> successors_straight; //Lanelet ref
    std::optional<int> successors_left; //Lanelet ref
    std::optional<int> is_left_of; //Incoming ref
};

/**
 * \class Intersection
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent an intersection specified in an XML file
 */
class Intersection : public InterfaceDraw
{
private:
    std::map<int, Incoming> incoming_map;

public:
     /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    Intersection(const xmlpp::Node* node);

    /**
     * \brief Get attribute value of child of node with name child_name
     * TODO: Maybe integrate in XMLTranslation
     * \param node The parent node
     * \param child_name Name of the child
     * \param warn Warn if the child or attribute does not exist
     */
    std::optional<int> get_child_attribute_ref(const xmlpp::Node* node, std::string child_name, bool warn);

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