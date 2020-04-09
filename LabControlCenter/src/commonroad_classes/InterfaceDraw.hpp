#pragma once

#include <gtkmm.h>

using DrawingContext = ::Cairo::RefPtr< ::Cairo::Context >;

/**
 * \class InterfaceDraw
 * \brief This interface requires the deriving classes to implement a draw function
 * It is mainly used for clarity, to define common behaviour
 */
class InterfaceDraw
{
public:
    /**
     * \brief This function is used to draw the data structure that imports this interface
     * If you want to set a color for drawing, perform this action on the context before using the draw function
     * \param ctx A DrawingContext, used to draw on
     * \param scale - optional: The factor by which to transform all number values related to position - this is not permanent, only for drawing (else, use InterfaceTransform's functions)
     */
    virtual void draw(const DrawingContext& ctx, double scale = 1.0) = 0;

    //Good practice
    virtual ~InterfaceDraw() {};
};