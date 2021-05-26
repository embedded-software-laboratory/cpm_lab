// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#pragma once

#include <math.h>

/**
 * \class InterfaceTransform
 * \brief This interface requires the deriving classes to implement a transform function
 * It is mainly used for clarity, to define common behaviour
 * \ingroup lcc_commonroad
 */
class InterfaceTransform
{
public:
    /**
     * \brief This function is used to fit the imported XML scenario to a given min. lane width
     * The lane with min width gets assigned min. width by scaling the whole scenario up until it fits
     * This scale value is used for the whole coordinate system
     * ORDER: Translate, rotate, scale (as specified for commonroad)
     * \param scale The factor by which to transform all number values related to position, or the min lane width (for commonroadscenario) - 0 means: No transformation desired
     * \param angle Rotation of the coordinate system, around the origin, w.r.t. right-handed coordinate system (according to commonroad specs), in radians
     * \param translate_x Move the coordinate system's origin along the x axis by this value
     * \param translate_y Move the coordinate system's origin along the y axis by this value
     */
    virtual void transform_coordinate_system(double scale, double angle, double translate_x, double translate_y) = 0;

    /**
     * \brief This helper function is used to rotate a point (x, y) around the z axis, counterclockwise, w.r.t. the origin
     * \param x value of the point (x, y) in the x-axis
     * \param y value of the point (x, y) in the y-axis
     * \param angle rotation, in radians
     * \return Via given paramters, as reference: The transformed point (x', y')
     */
    static void rotate_point_around_z(double& x, double& y, double angle)
    {
        double x_old = x;
        double y_old = y;

        double cos_angle = std::cos(angle);
        double sin_angle = std::sin(angle);

        x = cos_angle * x_old + sin_angle * y_old;
        y = -1.0 * sin_angle * x_old + cos_angle * y_old;
    }

    /**
     * \brief This helper function is used to rotate an orientation value in radians around the z axis, counterclockwise, w.r.t. the origin
     * \param orientation orientation, in radians
     * \param angle rotation, in radians
     * \return The transformed orientation
     */
    static double rotate_orientation_around_z(double& orientation, double angle)
    {
        return std::fmod((orientation - angle), 2.0 * M_PI);
    }

    //! Destructor. Good practice
    virtual ~InterfaceTransform() {};
};