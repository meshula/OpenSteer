// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// Draw
//
// This is a first stab at a graphics module for OpenSteerDemo.  It is intended
// to encapsulate all functionality related to 3d graphics as well as windows
// and graphics input devices such as the mouse.
//
// However this is purely an OpenGL-based implementation.  No special effort
// has been made to keep the "OpenGL way" from leaking through.  Attempting to
// port this to another graphics substrate may run into modularity problems.
//
// In any case, all calls to the underlying graphics substrate should be made
// from this module only.
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 06-25-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_DRAW_H
#define OPENSTEER_DRAW_H


#include "Vec3.h"
#include "AbstractVehicle.h"


// ------------------------------------------------------------------------
// for convenience, names of a few common RGB colors as Vec3 values
// (XXX I know, I know, there should be a separate "Color" class XXX)

namespace OpenSteer {


    const Vec3 gBlack   (0, 0, 0);
    const Vec3 gWhite   (1, 1, 1);

    const Vec3 gRed     (1, 0, 0);
    const Vec3 gYellow  (1, 1, 0);
    const Vec3 gGreen   (0, 1, 0);
    const Vec3 gCyan    (0, 1, 1);
    const Vec3 gBlue    (0, 0, 1);
    const Vec3 gMagenta (1, 0, 1);

    const Vec3 gOrange (1, 0.5f, 0);

    inline Vec3 grayColor (const float g) {return Vec3 (g, g, g);}

    const Vec3 gGray10 = grayColor (0.1f);
    const Vec3 gGray20 = grayColor (0.2f);
    const Vec3 gGray30 = grayColor (0.3f);
    const Vec3 gGray40 = grayColor (0.4f);
    const Vec3 gGray50 = grayColor (0.5f);
    const Vec3 gGray60 = grayColor (0.6f);
    const Vec3 gGray70 = grayColor (0.7f);
    const Vec3 gGray80 = grayColor (0.8f);
    const Vec3 gGray90 = grayColor (0.9f);


    // ------------------------------------------------------------------------
    // warn when draw functions are called during OpenSteerDemo's update phase
    //
    // XXX perhaps this should be made to "melt away" when not in debug mode?

    void warnIfInUpdatePhase2( const char* name);

    inline void warnIfInUpdatePhase (const char* name)
    {
        if (OpenSteerDemo::phaseIsUpdate())
        {
            // void warnIfInUpdatePhase2 (const char* name); // moved declaration of function outside
            warnIfInUpdatePhase2 (name);
        }
    }


    // ------------------------------------------------------------------------
    // draw the three axes of a LocalSpace: three lines parallel to the
    // basis vectors of the space, centered at its origin, of lengths
    // given by the coordinates of "size".


    void drawAxes  (const AbstractLocalSpace& localSpace,
                    const Vec3& size,
                    const Vec3& color);


    // ------------------------------------------------------------------------
    // draw the edges of a box with a given position, orientation, size
    // and color.  The box edges are aligned with the axes of the given
    // LocalSpace, and it is centered at the origin of that LocalSpace.
    // "size" is the main diagonal of the box.


    void drawBoxOutline  (const AbstractLocalSpace& localSpace,
                          const Vec3& size,
                          const Vec3& color);


    // ------------------------------------------------------------------------
    // draw a (filled-in, polygon-based) square checkerboard grid on the XZ
    // (horizontal) plane.
    //
    // ("size" is the length of a side of the overall checkerboard, "subsquares"
    // is the number of subsquares along each edge (for example a standard
    // checkboard has eight), "center" is the 3d position of the center of the
    // grid, color1 and color2 are used for alternating subsquares.)


    void drawXZCheckerboardGrid (const float size,
                                 const int subsquares,
                                 const Vec3& center,
                                 const Vec3& color1,
                                 const Vec3& color2);


    // ------------------------------------------------------------------------
    // draw a square grid of lines on the XZ (horizontal) plane.
    //
    // ("size" is the length of a side of the overall grid, "subsquares" is the
    // number of subsquares along each edge (for example a standard checkboard
    // has eight), "center" is the 3d position of the center of the grid, lines
    // are drawn in the specified "color".)


    void drawXZLineGrid (const float size,
                         const int subsquares,
                         const Vec3& center,
                         const Vec3& color);


    // ------------------------------------------------------------------------
    // Circle/disk drawing utilities


    void drawCircleOrDisk (const float radius,
                           const Vec3& axis,
                           const Vec3& center,
                           const Vec3& color,
                           const int segments,
                           const bool filled,
                           const bool in3d);

    void drawXZCircleOrDisk (const float radius,
                             const Vec3& center,
                             const Vec3& color,
                             const int segments,
                             const bool filled);

    void draw3dCircleOrDisk (const float radius,
                             const Vec3& center,
                             const Vec3& axis,
                             const Vec3& color,
                             const int segments,
                             const bool filled);

    inline void drawXZCircle (const float radius,
                              const Vec3& center,
                              const Vec3& color,
                              const int segments)
    {
        warnIfInUpdatePhase ("drawXZCircle");
        drawXZCircleOrDisk (radius, center, color, segments, false);
    }

    inline void drawXZDisk (const float radius,
                            const Vec3& center,
                            const Vec3& color,
                            const int segments)
    {
        warnIfInUpdatePhase ("drawXZDisk");
        drawXZCircleOrDisk (radius, center, color, segments, true);
    }

    inline void draw3dCircle (const float radius,
                              const Vec3& center,
                              const Vec3& axis,
                              const Vec3& color,
                              const int segments)
    {
        warnIfInUpdatePhase ("draw3dCircle");
        draw3dCircleOrDisk (radius, center, axis, color, segments, false);
    }

    inline void draw3dDisk (const float radius,
                            const Vec3& center,
                            const Vec3& axis,
                            const Vec3& color,
                            const int segments)
    {
        warnIfInUpdatePhase ("draw3dDisk");
        draw3dCircleOrDisk (radius, center, axis, color, segments, true);
    }


    // draw a circular arc on the XZ plane, from a start point, around a center,
    // for a given arc length, in a given number of segments and color.  The
    // sign of arcLength determines the direction in which the arc is drawn.

    void drawXZArc (const Vec3& start,
                    const Vec3& center,
                    const float arcLength,
                    const int segments,
                    const Vec3& color);


    // ------------------------------------------------------------------------
    // draw a reticle at the center of the window.  Currently it is small
    // crosshair with a gap at the center, drawn in white with black borders


    void drawReticle (void);


    // ------------------------------------------------------------------------


    void drawBasic2dCircularVehicle (const AbstractVehicle& bv,
                                     const Vec3& color);

    void drawBasic3dSphericalVehicle (const AbstractVehicle& bv,
                                      const Vec3& color);


    // ------------------------------------------------------------------------


    void draw2dTextAt3dLocation (const char& text,
                                 const Vec3& location,
                                 const Vec3& color);

    void draw2dTextAt3dLocation (const std::ostringstream& text,
                                 const Vec3& location,
                                 const Vec3& color);

    void draw2dTextAt2dLocation (const char& text,
                                 const Vec3 location,
                                 const Vec3 color);

    void draw2dTextAt2dLocation (const std::ostringstream& text,
                                 const Vec3 location,
                                 const Vec3 color);

    // ------------------------------------------------------------------------
    // emit an OpenGL vertex based on a Vec3


    void glVertexVec3 (const Vec3& v);


    // ----------------------------------------------------------------------------
    // draw 3d "graphical annotation" lines, used for debugging


    void drawLine (const Vec3& startPoint,
                   const Vec3& endPoint,
                   const Vec3& color);


    // ----------------------------------------------------------------------------
    // draw 2d lines in screen space: x and y are the relevant coordinates


    void draw2dLine (const Vec3& startPoint,
                     const Vec3& endPoint,
                     const Vec3& color);


    // ----------------------------------------------------------------------------
    // draw a line with alpha blending

    void drawLineAlpha (const Vec3& startPoint,
                        const Vec3& endPoint,
                        const Vec3& color,
                        const float alpha);


    // ------------------------------------------------------------------------
    // deferred drawing of lines, circles and (filled) disks


    void deferredDrawLine (const Vec3& startPoint,
                           const Vec3& endPoint,
                           const Vec3& color);

    void deferredDrawCircleOrDisk (const float radius,
                                   const Vec3& axis,
                                   const Vec3& center,
                                   const Vec3& color,
                                   const int segments,
                                   const bool filled,
                                   const bool in3d);

    void drawAllDeferredLines (void);
    void drawAllDeferredCirclesOrDisks (void);


    // ------------------------------------------------------------------------
    // Draw a single OpenGL triangle given three Vec3 vertices.


    void drawTriangle (const Vec3& a,
                       const Vec3& b,
                       const Vec3& c,
                       const Vec3& color);


    // ------------------------------------------------------------------------
    // Draw a single OpenGL quadrangle given four Vec3 vertices, and color.


    void drawQuadrangle (const Vec3& a,
                         const Vec3& b,
                         const Vec3& c,
                         const Vec3& d,
                         const Vec3& color);


    // ----------------------------------------------------------------------------
    // draws a "wide line segment": a rectangle of the given width and color
    // whose mid-line connects two given endpoints


    void drawXZWideLine (const Vec3& startPoint,
                         const Vec3& endPoint,
                         const Vec3& color,
                         float width);


    // ----------------------------------------------------------------------------


    void drawCameraLookAt (const Vec3& cameraPosition,
                           const Vec3& pointToLookAt,
                           const Vec3& up);


    // ----------------------------------------------------------------------------
    // check for errors during redraw, report any and then exit


    void checkForDrawError (const char * locationDescription);


    // ----------------------------------------------------------------------------
    // do all initialization related to graphics


    void initializeGraphics (int argc, char **argv);


    // ----------------------------------------------------------------------------
    // run graphics event loop


    void runGraphics (void);


    // ----------------------------------------------------------------------------
    // accessors for GLUT's window dimensions


    float drawGetWindowHeight (void);
    float drawGetWindowWidth (void);


    // ----------------------------------------------------------------------------
    // return a normalized direction vector pointing from the camera towards a
    // given point on the screen: the ray that would be traced for that pixel


    Vec3 directionFromCameraToScreenPosition (int x, int y);



} // namespace OpenSteer


// ----------------------------------------------------------------------------
#endif // OPENSTEER_DRAW_H
