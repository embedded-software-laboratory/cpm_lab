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

#include <gtkmm.h>

#include <string>

using DrawingContext = ::Cairo::RefPtr< ::Cairo::Context >;

/**
 * \class InterfaceDraw
 * \brief This interface requires the deriving classes to implement a draw function
 * It is mainly used for clarity, to define common behaviour
 * \ingroup lcc_commonroad
 */
class InterfaceDraw
{
public:
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
    virtual void draw(const DrawingContext& ctx, double scale = 1.0, double global_orientation = 0.0, double global_translate_x = 0.0, double global_translate_y = 0.0, double local_orientation = 0.0) = 0;

    //! Destructor. Good practice
    virtual ~InterfaceDraw() {};

    //Utility functions that can be used by each class
    /**
     * \brief This function can be used to draw an arrow from (x_1, y_1) to (x_2, y_2) - within the currently set coordinate system - with a given scale.
     * Important: An arrow is only drawn if (x_1, y_1) != (x_2, y_2) (up to 0.001 difference)!
     * \param ctx A DrawingContext, used to draw on
     * \param x_1 Arrow start x
     * \param y_1 Arrow start y
     * \param x_2 Arrow pointer / end x
     * \param y_2 Arrow pointer / end y
     * \param scale Scale applies to the arrow thickness and the arrowhead size only
     */
    void draw_arrow(const DrawingContext& ctx, double x_1, double y_1, double x_2, double y_2, double scale)
    {
        //Only draw an arrow if the points are not the same
        if (abs(x_1 - x_2) <= 0.001 && abs(y_1 - y_2) <= 0.001) return;

        ctx->save();

        //Calculate arrowhead orientation (part that is orthogonal to the line)
        double x_orth_vec = (y_2 - y_1);
        double y_orth_vec = (x_1 - x_2);
        double orth_vec_len = std::sqrt(std::pow(x_orth_vec, 2) + std::pow(y_orth_vec, 2));
        x_orth_vec /= orth_vec_len;
        y_orth_vec /= orth_vec_len;

        double x_back_vec = x_2 - x_1;
        double y_back_vec = y_2 - y_1;
        double back_vec_len = std::sqrt(std::pow(x_back_vec, 2) + std::pow(y_back_vec, 2));
        x_back_vec /= back_vec_len;
        y_back_vec /= back_vec_len;

        ctx->set_line_width(0.03 * scale);
        ctx->move_to(x_1, y_1);
        ctx->line_to(x_2, y_2);
        ctx->line_to(x_2 + 0.1 * x_orth_vec * scale - 0.1 * x_back_vec * scale, y_2 + 0.1 * y_orth_vec * scale - 0.1 * y_back_vec * scale);
        ctx->line_to(x_2 - 0.1 * x_orth_vec * scale - 0.1 * x_back_vec * scale, y_2 - 0.1 * y_orth_vec * scale - 0.1 * y_back_vec * scale);
        ctx->line_to(x_2, y_2);
        ctx->fill_preserve();
        ctx->stroke();

        ctx->restore();
    }

    /**
     * \brief Helper function to draw text centered, given a rotation, at (x,y), with a given font size
     * \param ctx The cairo context of the LCC's Map View
     * \param x x coordinate 
     * \param y y coordinate
     * \param rotation Applied before translation to (x,y)
     * \param font_size Size of the font
     * \param text Text to draw
     */
    void draw_text_centered(const DrawingContext& ctx, double x, double y, double rotation, double font_size, std::string text)
    {
        ctx->save();

        ctx->rotate(rotation);
        const double scale = 0.01;
        ctx->scale(scale, -scale);
        ctx->set_font_size(font_size);
        ctx->move_to(0,0);
        Cairo::TextExtents extents;
        ctx->get_text_extents(text, extents);

        ctx->translate(x, y);

        //Draw rectangle around text
        //Move to first corner from center
        ctx->save();
        ctx->set_source_rgba(1, 1, 1, 0.5);
        ctx->set_line_width(0);
        ctx->move_to(- extents.width/2, - extents.height/2);
        ctx->line_to(- extents.width/2,   extents.height/2);
        ctx->line_to(  extents.width/2,   extents.height/2);
        ctx->line_to(  extents.width/2, - extents.height/2);
        ctx->line_to(- extents.width/2, - extents.height/2);
        ctx->fill_preserve();
        ctx->stroke();
        ctx->restore();

        //Draw text centered
        ctx->move_to(-extents.width/2 - extents.x_bearing, -extents.height/2 - extents.y_bearing);
        ctx->set_source_rgb(.2,.1,.1);
        ctx->show_text(text);

        //For relief, not that readable
        // ctx->move_to(-extents.width/2 - extents.x_bearing - 0.6, -extents.height/2 - extents.y_bearing - 0.4);
        // ctx->set_source_rgb(.1,.1,.1);
        // ctx->show_text(text);

        ctx->restore();
    }
};