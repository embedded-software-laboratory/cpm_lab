#pragma once

/**
 * \class Point
 * \brief Auxiliary class from the XML specification: https://gitlab.lrz.de/tum-cps/commonroad-scenarios/-/blob/master/documentation/XML_commonRoad_XSD_2020a.xsd
 */
class Point
{
private:
    double x;
    double y;
    double z; //must not be set (then ??)
public:
    /**
     * \brief Constructor, set up a point object
     */
    Point(double x, double y, double z);

    //TODO: From interface
    void transform_to_lane_width(unsigned int width);
    void to_dds_msg(); 

    //TODO: Getter
};