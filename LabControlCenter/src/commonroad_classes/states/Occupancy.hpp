#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <cstdint>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/datatypes/IntervalOrExact.hpp"
#include "commonroad_classes/geometry/Shape.hpp"

/**
 * \class Occupancy
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent an Occupancy specified in an XML file
 */
class Occupancy
{
private:
    //Commonroad data
    std::optional<Shape> shape;
    std::optional<Shape> time; //Time values should probably be within the range of double, we did not want to define an extra type for this - gets transformed in getter to nanoseconds view

public:
    /**
     * \brief Constructor - we do not want the user to be able to set values after the class has been created
     */
    Occupancy(const xmlpp::Node* node){}

    /**
     * \brief Returns a DDS message created from the current scenario that contains all information relevant to the HLC
     * Due to the different return types for each class, no interface was defined for this function.
     * Still, it is required for all classes that are to be communicated via DDS to other members after the translation from XML
     * TODO: Change return type to whatever the name of the IDL type is
     */
    void to_dds_msg(); 

    //TODO: Getter
};
