#pragma once

#include <libxml++-2.6/libxml++/libxml++.h>

#include <optional>
//Optional is used for 3 reasons:
//1. Some values are optional according to the specification
//2. Some values might be missing if the file is not spec-conform, which is easy to handle when we do not require that they exist (though we still check for their existance)
//3. It is easier to set up an object piece by piece in the constructor, but that is not possible if the member object we want to set up does not have a default constructor (we would have to use the initializer list then)

#include "commonroad_classes/geometry/Shape.hpp"

#include "commonroad_classes/states/Occupancy.hpp"
#include "commonroad_classes/states/State.hpp"

#include "commonroad_classes/InterfaceDraw.hpp"
#include "commonroad_classes/InterfaceTransform.hpp"
#include "commonroad_classes/InterfaceTransformTime.hpp"
#include "commonroad_classes/XMLTranslation.hpp"

#include <sstream>
#include "commonroad_classes/SpecificationError.hpp"

#include "LCCErrorLogger.hpp"

#include "ObstacleSimulationData.hpp"

#include <cassert> //To make sure that the translation is performed on the right node types, which should haven been made sure by the programming (thus not an error, but an assertion is used)

/**
 * \enum ObstacleTypeEnvironment
 * \brief Specifies environment obstacle types, as in commonroad
 * \ingroup lcc_commonroad
 */
enum class ObstacleTypeEnvironment {Unknown, Building, Pillar, MedianStrip};

/**
 * \class EnvironmentObstacle
 * \brief This class, like all other classes in this folder, are heavily inspired by the current (2020) common road XML specification (https://gitlab.lrz.de/tum-cps/commonroad-scenarios/blob/master/documentation/XML_commonRoad_2020a.pdf)
 * It is used to store / represent a EnvironmentObstacle specified in an XML file
 * 2020 only!
 * \ingroup lcc_commonroad
 */
class EnvironmentObstacle : public InterfaceTransform
{
private:
    //! The obstacle type, e.g. a parked vehicle
    ObstacleTypeEnvironment type;
    //! The obstacle type as string
    std::string obstacle_type_text;
    //! Shape of the object, must exist
    std::optional<Shape> shape = std::nullopt;

    //! Transformation scale of transform_coordinate_system is remembered to draw text correctly scaled
    double transform_scale = 1.0;

    //! Remember line in commonroad file for logging
    int commonroad_line = 0;

public:
    /**
     * \brief The constructor gets an XML node and parses it once, translating it to the C++ data structure
     * An error is thrown in case the node is invalid / does not match the expected CommonRoad specs
     * \param node A (static) obstacle node
     */
    EnvironmentObstacle(
        const xmlpp::Node* node
    );

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

    //No draw function, obstacles are handled by LCC directly via simulation

    //Getter
    /**
     * \brief Returns a single trajectory point constructed from initial state, and further static obstacle information
     * Throws errors if expected types are missing
     */
    ObstacleSimulationData get_obstacle_simulation_data();

    /**
     * \brief Get the obstacle type
     */
    ObstacleTypeEnvironment get_type();
    /**
     * \brief Get the obstacle type as string
     */
    std::string get_obstacle_type_text();
    /**
     * \brief Get the obstacle shape, which must exist (optional due to no default constructor & potential translation issues)
     */
    const std::optional<Shape>& get_shape() const;
};