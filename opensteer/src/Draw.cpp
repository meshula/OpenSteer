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
// This is a first stab at a graphics module for SteerTest.  It is intended to
// encapsulate all functionality related to 3d graphics as well as windows and
// graphics input devices such as the mouse.
//
// However this is purely an OpenGL-based implementation.  No special effort
// has been made to keep the "OpenGL way" from leaking through.  Attempting to
// port this to another graphics substrate may run into modularity problems.
//
// In any case, all calls to the underlying graphics substrate should be made
// from this module only.
//
// 06-25-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>


// Include headers for OpenGL (gl.h), OpenGL Utility Library (glu.h) and
// OpenGL Utility Toolkit (glut.h).
//
// XXX In Mac OS X these headers are located in a different directory.
// XXX Need to revisit conditionalization on operating system.
#if defined(__APPLE__)
#include <GLUT/glut.h>
#else // !defined(__APPLE__)
#include <GL/glut.h>
#endif // !defined(__APPLE__)


#include "OpenSteer/SteerTest.h"


// ----------------------------------------------------------------------------
// XXX moved here from main.cpp 12-5-02


char* appVersionName = "SteerTest 0.7";


// The number of our GLUT window
int windowID; 

const int initialWindowWidth = 620;
const int initialWindowHeight = 420;


bool gMouseAdjustingCameraAngle = false;
bool gMouseAdjustingCameraRadius = false;
int gMouseAdjustingCameraLastX;
int gMouseAdjustingCameraLastY;


// ----------------------------------------------------------------------------
// initialize GL mode settings


void initGL (void)
{
    // background = dark gray
    glClearColor (0.3f, 0.3f, 0.3f, 0);

    // enable depth buffer clears
    glClearDepth (1.0f);

    // select smooth shading
    glShadeModel (GL_SMOOTH);

    // enable  and select depth test
    glDepthFunc (GL_LESS);
    glEnable (GL_DEPTH_TEST);

    // turn on backface culling
    glEnable (GL_CULL_FACE);
    glCullFace (GL_BACK);

    // enable blending and set typical "blend into frame buffer" mode
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // reset projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
}


// ----------------------------------------------------------------------------
// handler for window resizing


void reshapeFunc (int width, int height)
{
    // set viewport to full window
    glViewport(0, 0, width, height);

    // set perspective transformation
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    const GLfloat w = width;
    const GLfloat h = height;
    const GLfloat aspectRatio = (height == 0) ? 1 : w/h;
    const GLfloat fieldOfViewY = 45;
    const GLfloat hither = 1;  // put this on Camera so PlugIns can frob it
    const GLfloat yon = 400;   // put this on Camera so PlugIns can frob it
    gluPerspective (fieldOfViewY, aspectRatio, hither, yon);

    // leave in modelview mode
    glMatrixMode(GL_MODELVIEW);
}


// ----------------------------------------------------------------------------
// This is called (by GLUT) each time a mouse button pressed or released.


void mouseButtonFunc (int button, int state, int x, int y)
{
    // if the mouse button has just been released
    if (state == GLUT_UP)
    {
        // end any ongoing mouse-adjusting-camera session
        gMouseAdjustingCameraAngle = false;
        gMouseAdjustingCameraRadius = false;
    }

    // if the mouse button has just been pushed down
    if (state == GLUT_DOWN)
    {
        // names for relevant values of "button" and "state"
        const int  mods       = glutGetModifiers ();
        const bool modNone    = (mods == 0);
        const bool modCtrl    = (mods == GLUT_ACTIVE_CTRL);
        const bool modCtrlAlt = (mods == (GLUT_ACTIVE_CTRL | GLUT_ACTIVE_ALT));
        const bool mouseL     = (button == GLUT_LEFT_BUTTON);
        const bool mouseM     = (button == GLUT_MIDDLE_BUTTON);

        // mouse-left (with no modifiers): select vehicle
        if (modNone && mouseL)
        {
            SteerTest::selectVehicleNearestScreenPosition (x, y);
        }

        // control-mouse-left: begin adjusting camera angle
        if (modCtrl && mouseL)
        {
            gMouseAdjustingCameraLastX = x;
            gMouseAdjustingCameraLastY = y;
            gMouseAdjustingCameraAngle = true;
        }

        // control-mouse-middle: begin adjusting camera radius
        // (same for: control-alt-mouse-left and control-alt-mouse-middle)
        if ((modCtrl    && mouseM) ||
            (modCtrlAlt && mouseL) ||
            (modCtrlAlt && mouseM))
        {
            gMouseAdjustingCameraLastX = x;
            gMouseAdjustingCameraLastY = y;
            gMouseAdjustingCameraRadius = true;
        }
    }
}


// ----------------------------------------------------------------------------
// called when mouse moves and any buttons are down


void mouseMotionFunc (int x, int y)
{
    // are we currently in the process of mouse-adjusting the camera?
    if (gMouseAdjustingCameraAngle || gMouseAdjustingCameraRadius)
    {
        // speed factors to map from mouse movement in pixels to 3d motion
        const float dSpeed = 0.005f;
        const float rSpeed = 0.01f;

        // XY distance (in pixels) that mouse moved since last update
        const float dx = x - gMouseAdjustingCameraLastX;
        const float dy = y - gMouseAdjustingCameraLastY;
        gMouseAdjustingCameraLastX = x;
        gMouseAdjustingCameraLastY = y;

        Vec3 cameraAdjustment;

        // set XY values according to mouse motion on screen space
        if (gMouseAdjustingCameraAngle)
        {
            cameraAdjustment.x = dx * -dSpeed;
            cameraAdjustment.y = dy * +dSpeed;
        }

        // set Z value according vertical to mouse motion
        if (gMouseAdjustingCameraRadius)
        {
            cameraAdjustment.z = dy * rSpeed;
        }

        // pass adjustment vector to camera's mouse adjustment routine
        SteerTest::camera.mouseAdjustOffset (cameraAdjustment);
    }
}


// ----------------------------------------------------------------------------
// called when mouse moves and no buttons are down


void mousePassiveMotionFunc (int x, int y)
{
    SteerTest::mouseX = x;
    SteerTest::mouseY = y;
}


// ----------------------------------------------------------------------------
// called when mouse enters or exits the window


void mouseEnterExitWindowFunc (int state)
{
    if (state == GLUT_ENTERED) SteerTest::mouseInWindow = true;
    if (state == GLUT_LEFT)    SteerTest::mouseInWindow = false;
}


// ----------------------------------------------------------------------------
// draw PlugI name in upper lefthand corner of screen


void drawDisplayPlugInName (void)
{
    const float h = glutGet (GLUT_WINDOW_HEIGHT);
    const Vec3 screenLocation (10, h-20, 0);
    draw2dTextAt2dLocation (*SteerTest::nameOfSelectedPlugIn (),
                            screenLocation,
                            gWhite);
}


// ----------------------------------------------------------------------------
// draw camera mode name in lower lefthand corner of screen


void drawDisplayCameraModeName (void)
{
    std::ostringstream message;
    message << "Camera: " << SteerTest::camera.modeName () << std::ends;
    const Vec3 screenLocation (10, 10, 0);
    draw2dTextAt2dLocation (message, screenLocation, gWhite);
}


// ----------------------------------------------------------------------------
// helper for drawDisplayFPS


void writePhaseTimerReportToStream (float phaseTimer,
                                    std::ostringstream& stream)
{
    // write the timer value in seconds in floating point
    stream << std::setprecision (5) << std::setiosflags (std::ios::fixed);
    stream << phaseTimer;

    // restate value in another form
    stream << std::setprecision (0) << std::setiosflags (std::ios::fixed);
    stream << " (";

    // is there a "fixed frame rate target"?
    if (SteerTest::clock.targetFPS > 0)
    {
        // quantify time as a percentage of frame time
        stream << ((100 * phaseTimer) / (1.0f / SteerTest::clock.targetFPS));
        stream << "% of 1/";
        stream << SteerTest::clock.targetFPS;
        stream << "sec)\n";
    }
    else
    {
        // express as FPS (inverse of phase time)
        stream << 1 / phaseTimer;
        stream << " fps)\n";
    }
}


// ----------------------------------------------------------------------------
// draw text showing (smoothed, rounded) "frames per second" rate
// (and later a bunch of related stuff was dumped here, a reorg would be nice)


float gSmoothedFPS = 0;
float gSmoothedUsage = 0;

float gSmoothedTimerDraw = 0;
float gSmoothedTimerUpdate = 0;
float gSmoothedTimerOverhead = 0;


void drawDisplayFPS (void)
{
    const float elapsedTime = SteerTest::clock.elapsedRealTime;
    const float fps = (elapsedTime > 0) ? 1 / elapsedTime : 0;
    const float smoothRate = (gSmoothedFPS == 0) ? 1 : (elapsedTime * 0.4f);

    // skip several frames to allow frame rate to settle
    static int skipCount = 10;
    if (skipCount > 0)
    {
        skipCount--;
    }
    else
    {
        // is there a "fixed frame rate target" or is it as-fast-as-possible?
        const bool targetFrameRate = SteerTest::clock.targetFPS > 0;

        // "smooth" instantaneous FPS rate: start at current fps the first
        // time through, then blend fps into a running average thereafter
        blendIntoAccumulator (smoothRate, fps, gSmoothedFPS);

        // convert smoothed FPS value into labeled character string
        std::ostringstream fpsStr;
        fpsStr << "fps: " << (int) round (gSmoothedFPS);
        if (SteerTest::clock.paused) fpsStr << " Paused";
        fpsStr << std::ends;

        // draw the string in white at the lower left corner of the window
        const int lh = 16; // xxx
        const Vec3 screenLocation1 (10, 10 + lh, 0);
        draw2dTextAt2dLocation (fpsStr, screenLocation1, gWhite);

        // add "usage" message if fixed target frame rate is specified
        if (targetFrameRate)
        {
            // run time per frame over target frame time (as a percentage)
            const float usage =
                ((100 * SteerTest::clock.elapsedNonWaitRealTime) /
                 (1.0f / SteerTest::clock.targetFPS));

            // blend new usage value into running average
            blendIntoAccumulator (smoothRate, usage, gSmoothedUsage);

            // create usage description character string
            std::ostringstream usageStr;
            usageStr << std::setprecision (0);
            usageStr << std::setiosflags (std::ios::fixed);
            usageStr << gSmoothedUsage << "% usage of 1/";
            usageStr << SteerTest::clock.targetFPS << " time step";
            usageStr << std::ends;

            // display message in lower left corner of window
            // (draw in red if the instantaneous usage is 100% or more)
            const Vec3 screenLocation2 (10, 10 + 2*lh, 0);
            const Vec3 color = (usage >= 100) ? gRed : gGray60;
            draw2dTextAt2dLocation (usageStr, screenLocation2, color);
        }

        // get smoothed phase timer information
        const float ptd = SteerTest::phaseTimerDraw();
        const float ptu = SteerTest::phaseTimerUpdate();
        const float pto = SteerTest::phaseTimerOverhead();
        blendIntoAccumulator (smoothRate, ptd, gSmoothedTimerDraw);
        blendIntoAccumulator (smoothRate, ptu, gSmoothedTimerUpdate);
        blendIntoAccumulator (smoothRate, pto, gSmoothedTimerOverhead);

        // display phase timer information
        std::ostringstream timerStr;
        timerStr << "update: ";
        writePhaseTimerReportToStream (gSmoothedTimerUpdate, timerStr);
        timerStr << "draw:   ";
        writePhaseTimerReportToStream (gSmoothedTimerDraw, timerStr);
        timerStr << "other:  ";
        writePhaseTimerReportToStream (gSmoothedTimerOverhead, timerStr);
        timerStr << std::ends;
        const Vec3 screenLocation3 (10, lh * 7, 0);
        draw2dTextAt2dLocation (timerStr, screenLocation3, gGreen);
    }
}


// ------------------------------------------------------------------------
// cycle through frame rate presets  (XXX move this to SteerTest)


int selectNextPresetFrameRate (void)
{
    // cycle through this list of frame rate presets on each subsequent call
    int frameRatePresets[] = {0, 24, 60};
    static int frameRatePresetIndex = 0;
    frameRatePresetIndex = (frameRatePresetIndex + 1) % 3;

    // set SteerTest's clock's target frame rate, return that value
    return SteerTest::clock.targetFPS = frameRatePresets[frameRatePresetIndex];
}


// ------------------------------------------------------------------------
// This function is called (by GLUT) each time a key is pressed.
//
// XXX the bulk of this should be moved to SteerTest


void keyboardFunc (unsigned char key, int x, int y) 
{
    std::ostringstream message;

    // ascii codes
    const int tab = 9;
    const int space = 32;
    const int esc = 27; // escape key

    switch (key)
    {
    // reset selected PlugIn
    case 'r':
        SteerTest::resetSelectedPlugIn ();
        message << "reset PlugIn "
                << '"' << SteerTest::nameOfSelectedPlugIn () << '"'
                << std::ends;
        SteerTest::printMessage (message);
        break;

    // cycle selection to next vehicle
    case 's':
        SteerTest::printMessage ("select next vehicle/agent");
        SteerTest::selectNextVehicle ();
        break;

    // camera mode cycle
    case 'c':
        SteerTest::camera.selectNextMode ();
        message << "select camera mode "
                << '"' << SteerTest::camera.modeName () << '"' << std::ends;
        SteerTest::printMessage (message);
        break;

    // select next PlugIn
    case tab:
        SteerTest::selectNextPlugIn ();
        message << "select next PlugIn: "
                << '"' << SteerTest::nameOfSelectedPlugIn () << '"'
                << std::ends;
        SteerTest::printMessage (message);
        break;

    // toggle run/pause state
    case space:
        SteerTest::printMessage (SteerTest::clock.togglePausedState () ?
                                 "pause" : "run");
        break;

    // cycle through frame rate presets
    case 'f':
        message << "set frame rate to "
                << selectNextPresetFrameRate () << std::ends;
        SteerTest::printMessage (message);
        break;

    // print minimal help for single key commands
    case '?':
        SteerTest::keyboardMiniHelp ();
        break;

    // exit application with normal status 
    case esc:
        glutDestroyWindow (windowID);
        SteerTest::printMessage ("exit.");
        SteerTest::exit (0);

    default:
        message << "unrecognized single key command: " << key;
        message << " (" << (int)key << ")";//xxx perhaps only for debugging?
        message << std::ends;
        SteerTest::printMessage ("");
        SteerTest::printMessage (message);
        SteerTest::keyboardMiniHelp ();
    }
}


// ------------------------------------------------------------------------
// handles "special" keys,
// function keys are handled by the PlugIn


void specialFunc (int key, int x, int y)
{
    std::ostringstream message;

    switch (key)
    {
    case GLUT_KEY_F1:  SteerTest::functionKeyForPlugIn (1);  break;
    case GLUT_KEY_F2:  SteerTest::functionKeyForPlugIn (2);  break;
    case GLUT_KEY_F3:  SteerTest::functionKeyForPlugIn (3);  break;
    case GLUT_KEY_F4:  SteerTest::functionKeyForPlugIn (4);  break;
    case GLUT_KEY_F5:  SteerTest::functionKeyForPlugIn (5);  break;
    case GLUT_KEY_F6:  SteerTest::functionKeyForPlugIn (6);  break;
    case GLUT_KEY_F7:  SteerTest::functionKeyForPlugIn (7);  break;
    case GLUT_KEY_F8:  SteerTest::functionKeyForPlugIn (8);  break;
    case GLUT_KEY_F9:  SteerTest::functionKeyForPlugIn (9);  break;
    case GLUT_KEY_F10: SteerTest::functionKeyForPlugIn (10); break;
    case GLUT_KEY_F11: SteerTest::functionKeyForPlugIn (11); break;
    case GLUT_KEY_F12: SteerTest::functionKeyForPlugIn (12); break;

    case GLUT_KEY_RIGHT:
        SteerTest::clock.paused = true;
        const int targetFPS = SteerTest::clock.targetFPS;
        const float FPS = (targetFPS > 0) ? targetFPS : gSmoothedFPS;
        const float frameTime = 1 / FPS;
        SteerTest::clock.advanceSimulationTime (frameTime);
        message << "single step forward (frame time: "
                << frameTime << ")" << std::endl;
        SteerTest::printMessage (message);
        break;
    }
}


// ------------------------------------------------------------------------
// Main drawing function for SteerTest application,
// drives simulation as a side effect


void displayFunc (void)
{
    // clear color and depth buffers
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // run simulation and draw associated graphics
    SteerTest::updateSimulationAndRedraw ();

    // draw text showing (smoothed, rounded) "frames per second" rate
    drawDisplayFPS ();

    // draw the name of the selected PlugIn
    drawDisplayPlugInName ();

    // draw the name of the camera's current mode
    drawDisplayCameraModeName ();

    // draw crosshairs to indicate aimpoint (xxx for debugging only?)
    // drawReticle ();

    // check for errors in drawing module, if so report and exit
    checkForDrawError ("SteerTest::updateSimulationAndRedraw");

    // double buffering, swap back and front buffers
    glFlush ();
    glutSwapBuffers();
}


// ----------------------------------------------------------------------------
// draw string s right-justified in the upper righthand corner


//     // XXX display the total number of AbstractVehicles created
//     {
//         std::ostringstream s;
//         s << "vehicles: " << xxx::SerialNumberCounter << std::ends;

//         // draw string s right-justified in the upper righthand corner
//         const int h = glutGet (GLUT_WINDOW_HEIGHT);
//         const int w = glutGet (GLUT_WINDOW_WIDTH);
//         const int fontWidth = 9; // for GLUT_BITMAP_9_BY_15
//         const int fontHeight = 15; // for GLUT_BITMAP_9_BY_15
//         const int x = w - (fontWidth * s.pcount());
//         const int y = h - (fontHeight + 5);
//         const Vec3 screenLocation (x, y, 0);
//         draw2dTextAt2dLocation (s, screenLocation, gWhite);
//     }


// ----------------------------------------------------------------------------
// do all initialization related to graphics


void initializeGraphics (int argc, char **argv)
{
    // initialize GLUT state based on command line arguments
    glutInit (&argc, argv);  

    // display modes: RGB+Z and double buffered
    GLint mode = GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE;
    glutInitDisplayMode (mode);

    // create and initialize our window with GLUT tools
    glutInitWindowPosition (0, 0);
    glutInitWindowSize (initialWindowWidth, initialWindowHeight);
    windowID = glutCreateWindow (appVersionName);
    reshapeFunc (initialWindowWidth, initialWindowHeight);
    initGL ();

    // register our display function, make it the idle handler too
    glutDisplayFunc (&displayFunc);  
    glutIdleFunc (&displayFunc);

    // register handler for window reshaping
    glutReshapeFunc (&reshapeFunc);

    // register handler for keyboard events
    glutKeyboardFunc (&keyboardFunc);
    glutSpecialFunc (&specialFunc);

    // register handler for mouse button events
    glutMouseFunc (&mouseButtonFunc);

    // register handler to track mouse motion when any button down
    glutMotionFunc (mouseMotionFunc);

    // register handler to track mouse motion when no buttons down
    glutPassiveMotionFunc (mousePassiveMotionFunc);

    // register handler for when mouse enters or exists the window
    glutEntryFunc (mouseEnterExitWindowFunc);
}


// ----------------------------------------------------------------------------
// run graphics event loop


void runGraphics (void)
{
    glutMainLoop ();  
}


// ------------------------------------------------------------------------
// emit an OpenGL vertex based on a Vec3


inline void iglVertexVec3 (const Vec3& v)
{
    glVertex3f (v.x, v.y, v.z);
}


void glVertexVec3 (const Vec3& v)
{
    iglVertexVec3 (v);
}


// ----------------------------------------------------------------------------
// warn when draw functions are called during SteerTest's update phase


void warnIfInUpdatePhase2 (const char* name)
{
    std::ostringstream message;
    message << "use annotation (during simulation update, do not call ";
    message << name;
    message << ")";
    message << std::ends;
    SteerTest::printWarning (message);
}


// ----------------------------------------------------------------------------
// draw 3d "graphical annotation" lines, used for debugging


inline void iDrawLine (const Vec3& startPoint,
                       const Vec3& endPoint,
                       const Vec3& color)
{
    warnIfInUpdatePhase ("iDrawLine");
    glColor3f (color.x, color.y, color.z);
    glBegin (GL_LINES);
    glVertexVec3 (startPoint);
    glVertexVec3 (endPoint);
    glEnd ();
}


void drawLine (const Vec3& startPoint,
               const Vec3& endPoint,
               const Vec3& color)
{
    iDrawLine (startPoint, endPoint, color);
}


// ----------------------------------------------------------------------------
// draw a line with alpha blending

// see also glAlphaFunc
// glBlendFunc (GL_SRC_ALPHA)
// glEnable (GL_BLEND)


void drawLineAlpha (const Vec3& startPoint,
                    const Vec3& endPoint,
                    const Vec3& color,
                    const float alpha)
{
    warnIfInUpdatePhase ("drawLineAlpha");
    glColor4f (color.x, color.y, color.z, alpha);
    glBegin (GL_LINES);
    glVertexVec3 (startPoint);
    glVertexVec3 (endPoint);
    glEnd ();
}


// ----------------------------------------------------------------------------
// Draw a single OpenGL triangle given three Vec3 vertices.


inline void iDrawTriangle (const Vec3& a,
                           const Vec3& b,
                           const Vec3& c,
                           const Vec3& color)
{
    warnIfInUpdatePhase ("iDrawTriangle");
    glColor3f (color.x, color.y, color.z);
    glBegin (GL_TRIANGLES);
    {
        glVertexVec3 (a);
        glVertexVec3 (b);
        glVertexVec3 (c);
    }
    glEnd ();
}


void drawTriangle (const Vec3& a,
                   const Vec3& b,
                   const Vec3& c,
                   const Vec3& color)
{
    iDrawTriangle (a, b, c, color);
}


// ------------------------------------------------------------------------
// Draw a single OpenGL quadrangle given four Vec3 vertices, and color.


inline void iDrawQuadrangle (const Vec3& a,
                             const Vec3& b,
                             const Vec3& c,
                             const Vec3& d,
                             const Vec3& color)
{
    warnIfInUpdatePhase ("iDrawQuadrangle");
    glColor3f (color.x, color.y, color.z);
    glBegin (GL_QUADS);
    {
        glVertexVec3 (a);
        glVertexVec3 (b);
        glVertexVec3 (c);
        glVertexVec3 (d);
    }
    glEnd ();
}


void drawQuadrangle (const Vec3& a,
                     const Vec3& b,
                     const Vec3& c,
                     const Vec3& d,
                     const Vec3& color)
{
    iDrawQuadrangle (a, b, c, d, color);
}


// ------------------------------------------------------------------------
// Between matched sets of these two calls, assert that all polygons
// will be drawn "double sided", that is, without back-face culling


inline void beginDoubleSidedDrawing (void)
{
    glPushAttrib (GL_ENABLE_BIT);
    glDisable (GL_CULL_FACE);
}


inline void endDoubleSidedDrawing (void)
{
    glPopAttrib ();
}


// ------------------------------------------------------------------------
// General purpose circle/disk drawing routine.  Draws circles or disks (as
// specified by "filled" argument) and handles both special case 2d circles
// on the XZ plane or arbitrary circles in 3d space (as specified by "in3d"
// argument)


void drawCircleOrDisk (const float radius,
                       const Vec3& axis,
                       const Vec3& center,
                       const Vec3& color,
                       const int segments,
                       const bool filled,
                       const bool in3d)
{
    LocalSpace ls;
    if (in3d)
    {
        // define a local space with "axis" as the Y/up direction
        // (XXX should this be a method on  LocalSpace?)
        const Vec3 unitAxis = axis.normalize ();
        const Vec3 unitPerp = findPerpendicularIn3d (axis).normalize ();
        ls.setUp (unitAxis);
        ls.setForward (unitPerp);
        ls.setPosition (center);
        ls.setUnitSideFromForwardAndUp ();
    }
        
    // make disks visible (not culled) from both sides 
    if (filled) beginDoubleSidedDrawing ();

    // point to be rotated about the (local) Y axis, angular step size
    Vec3 pointOnCircle (radius, 0, 0);
    const float step = (2 * M_PI) / segments;

    // set drawing color
    glColor3f (color.x, color.y, color.z);

    // begin drawing a triangle fan (for disk) or line loop (for circle)
    glBegin (filled ? GL_TRIANGLE_FAN : GL_LINE_LOOP);

    // for the filled case, first emit the center point
    if (filled) iglVertexVec3 (in3d ? ls.position() : center);

    // rotate p around the circle in "segments" steps
    const int vertexCount = filled ? segments+1 : segments;
    for (int i = 0; i < vertexCount; i++)
    {
        // emit next point on circle, either in 3d (globalized out
        // of the local space), or in 2d (offset from the center)
        iglVertexVec3 (in3d ?
                           ls.globalizePosition (pointOnCircle) :
                           (Vec3) (pointOnCircle + center));

        // rotate point one more step around circle
        pointOnCircle = pointOnCircle.rotateAboutGlobalY (step);
    }

    // close drawing operation
    glEnd ();
    if (filled) endDoubleSidedDrawing ();
}


// ------------------------------------------------------------------------


void draw3dCircleOrDisk (const float radius,
                         const Vec3& center,
                         const Vec3& axis,
                         const Vec3& color,
                         const int segments,
                         const bool filled)
{
    // draw a circle-or-disk in the given local space
    drawCircleOrDisk (radius, axis, center, color, segments, filled, true);
}


// ------------------------------------------------------------------------
// drawing utility used by both drawXZCircle and drawXZDisk


void drawXZCircleOrDisk (const float radius,
                         const Vec3& center,
                         const Vec3& color,
                         const int segments,
                         const bool filled)
{
    // draw a circle-or-disk on the XZ plane
    drawCircleOrDisk (radius, Vec3::zero, center, color, segments, filled, false);
}


// ------------------------------------------------------------------------
// a simple 2d vehicle on the XZ plane


void drawBasic2dCircularVehicle (const AbstractVehicle& vehicle,
                                 const Vec3& color)
{
    // "aspect ratio" of body (as seen from above)
    const float x = 0.5f;
    const float y = sqrtXXX (1 - (x * x));

    // radius and position of vehicle
    const float r = vehicle.radius();
    const Vec3& p = vehicle.position();

    // shape of triangular body
    const Vec3 u = r * 0.05f * Vec3 (0, 1, 0); // slightly up
    const Vec3 f = r * vehicle.forward();
    const Vec3 s = r * vehicle.side() * x;
    const Vec3 b = r * vehicle.forward() * -y;

    // draw double-sided triangle (that is: no (back) face culling)
    beginDoubleSidedDrawing ();
    iDrawTriangle (p + f + u,
                   p + b - s + u,
                   p + b + s + u,
                   color);
    endDoubleSidedDrawing ();

    // draw the circular collision boundary
    drawXZCircle (r, p + u, gWhite, 20);
}


// ------------------------------------------------------------------------
// a simple 3d vehicle


void drawBasic3dSphericalVehicle (const AbstractVehicle& vehicle,
                                  const Vec3& color)
{
    // "aspect ratio" of body (as seen from above)
    const float x = 0.5f;
    const float y = sqrtXXX (1 - (x * x));

    // radius and position of vehicle
    const float r = vehicle.radius();
    const Vec3& p = vehicle.position();

    // body shape parameters
    const Vec3 f = r * vehicle.forward();
    const Vec3 s = r * vehicle.side() * x;
    const Vec3 u = r * vehicle.up() * x * 0.5f;
    const Vec3 b = r * vehicle.forward() * -y;

    // vertex positions
    const Vec3 nose   = p + f;
    const Vec3 side1  = p + b - s;
    const Vec3 side2  = p + b + s;
    const Vec3 top    = p + b + u;
    const Vec3 bottom = p + b - u;

    // colors
    const float j = +0.05f;
    const float k = -0.05f;
    const Vec3 color1 = color + Vec3 (j, j, k);
    const Vec3 color2 = color + Vec3 (j, k, j);
    const Vec3 color3 = color + Vec3 (k, j, j);
    const Vec3 color4 = color + Vec3 (k, j, k);
    const Vec3 color5 = color + Vec3 (k, k, j);

    // draw body
    iDrawTriangle (nose,  side1,  top,    color1);  // top, side 1
    iDrawTriangle (nose,  top,    side2,  color2);  // top, side 2
    iDrawTriangle (nose,  bottom, side1,  color3);  // bottom, side 1
    iDrawTriangle (nose,  side2,  bottom, color4);  // bottom, side 2
    iDrawTriangle (side1, side2,  top,    color5);  // top back
    iDrawTriangle (side2, side1,  bottom, color5);  // bottom back
}



// ------------------------------------------------------------------------
// draw a (filled-in, polygon-based) square checkerboard grid on the XZ
// (horizontal) plane.
//
// ("size" is the length of a side of the overall grid, "subsquares" is the
// number of subsquares along each edge (for example a standard checkboard
// has eight), "center" is the 3d position of the center of the grid,
// color1 and color2 are used for alternating subsquares.)


void drawXZCheckerboardGrid (const float size,
                             const int subsquares,
                             const Vec3& center,
                             const Vec3& color1,
                             const Vec3& color2)
{
    const float half = size/2;
    const float spacing = size / subsquares;

    beginDoubleSidedDrawing ();
    {
        bool flag1 = false;
        float p = -half;
        Vec3 corner;
        for (int i = 0; i < subsquares; i++)
        {
            bool flag2 = flag1;
            float q = -half;
            for (int j = 0; j < subsquares; j++)
            {
                corner.set (p, 0, q);
                corner += center;
                iDrawQuadrangle (corner,
                                 corner + Vec3 (spacing, 0,       0),
                                 corner + Vec3 (spacing, 0, spacing),
                                 corner + Vec3 (0,       0, spacing),
                                 flag2 ? color1 : color2);
                flag2 = !flag2;
                q += spacing;
            }
            flag1 = !flag1;
            p += spacing;
        }
    }
    endDoubleSidedDrawing ();
}


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
                     const Vec3& color)
{
    warnIfInUpdatePhase ("drawXZLineGrid");

    const float half = size/2;
    const float spacing = size / subsquares;

    // set grid drawing color
    glColor3f (color.x, color.y, color.z);

    // draw a square XZ grid with the given size and line count
    glBegin (GL_LINES);
    float q = -half;
    for (int i = 0; i < (subsquares + 1); i++)
    {
        const Vec3 x1 (q, 0, +half); // along X parallel to Z
        const Vec3 x2 (q, 0, -half);
        const Vec3 z1 (+half, 0, q); // along Z parallel to X
        const Vec3 z2 (-half, 0, q);

        iglVertexVec3 (x1 + center);
        iglVertexVec3 (x2 + center);
        iglVertexVec3 (z1 + center);
        iglVertexVec3 (z2 + center);

        q += spacing;
    }
    glEnd ();
}


// ------------------------------------------------------------------------
// draw the three axes of a LocalSpace: three lines parallel to the
// basis vectors of the space, centered at its origin, of lengths
// given by the coordinates of "size".


void drawAxes  (const AbstractLocalSpace& ls,
                const Vec3& size,
                const Vec3& color)
{
    const Vec3 x (size.x / 2, 0, 0);
    const Vec3 y (0, size.y / 2, 0);
    const Vec3 z (0, 0, size.z / 2);
 
    iDrawLine (ls.globalizePosition (x), ls.globalizePosition (x * -1), color);
    iDrawLine (ls.globalizePosition (y), ls.globalizePosition (y * -1), color);
    iDrawLine (ls.globalizePosition (z), ls.globalizePosition (z * -1), color);
}


// ------------------------------------------------------------------------
// draw the edges of a box with a given position, orientation, size
// and color.  The box edges are aligned with the axes of the given
// LocalSpace, and it is centered at the origin of that LocalSpace.
// "size" is the main diagonal of the box.
//
// use gGlobalSpace to draw a box aligned with global space


void drawBoxOutline  (const AbstractLocalSpace& localSpace,
                      const Vec3& size,
                      const Vec3& color)
{
    const Vec3 s = size / 2.0f;  // half of main diagonal

    const Vec3 a (+s.x, +s.y, +s.z);
    const Vec3 b (+s.x, -s.y, +s.z);
    const Vec3 c (-s.x, -s.y, +s.z);
    const Vec3 d (-s.x, +s.y, +s.z);

    const Vec3 e (+s.x, +s.y, -s.z);
    const Vec3 f (+s.x, -s.y, -s.z);
    const Vec3 g (-s.x, -s.y, -s.z);
    const Vec3 h (-s.x, +s.y, -s.z);

    const Vec3 A = localSpace.globalizePosition (a);
    const Vec3 B = localSpace.globalizePosition (b);
    const Vec3 C = localSpace.globalizePosition (c);
    const Vec3 D = localSpace.globalizePosition (d);

    const Vec3 E = localSpace.globalizePosition (e);
    const Vec3 F = localSpace.globalizePosition (f);
    const Vec3 G = localSpace.globalizePosition (g);
    const Vec3 H = localSpace.globalizePosition (h);

    iDrawLine (A, B, color);
    iDrawLine (B, C, color);
    iDrawLine (C, D, color);
    iDrawLine (D, A, color);

    iDrawLine (A, E, color);
    iDrawLine (B, F, color);
    iDrawLine (C, G, color);
    iDrawLine (D, H, color);

    iDrawLine (E, F, color);
    iDrawLine (F, G, color);
    iDrawLine (G, H, color);
    iDrawLine (H, E, color);
}


// ------------------------------------------------------------------------
// this comes up often enough to warrant its own warning function


inline void drawCameraLookAtCheck (const Vec3& cameraPosition,
                                   const Vec3& pointToLookAt,
                                   const Vec3& up)
{
    const Vec3 view = pointToLookAt - cameraPosition;
    const Vec3 perp = view.perpendicularComponent (up);
    if (perp == Vec3::zero)
        SteerTest::printWarning ("LookAt: degenerate camera");
}


// ------------------------------------------------------------------------
// Define scene's camera (viewing transformation) in terms of the camera's
// position, the point to look at (an "aim point" in the scene which will
// end up at the center of the camera's view), and an "up" vector defining
// the camera's "roll" around the "view axis" between cameraPosition and
// pointToLookAt (the image of the up vector will be vertical in the
// camera's view).


void drawCameraLookAt (const Vec3& cameraPosition,
                       const Vec3& pointToLookAt,
                       const Vec3& up)
{
    // check for valid "look at" parameters
    drawCameraLookAtCheck (cameraPosition, pointToLookAt, up);

    // use LookAt from OpenGL Utilities
    glLoadIdentity ();
    gluLookAt (cameraPosition.x, cameraPosition.y, cameraPosition.z,
               pointToLookAt.x,  pointToLookAt.y,  pointToLookAt.z,
               up.x,             up.y,             up.z);
}


// ------------------------------------------------------------------------
// draw a reticle at the center of the window.  Currently it is small
// crosshair with a gap at the center, drawn in white with black borders


void drawReticle (void)
{
    const int a = 10;
    const int b = 30;
    const float w = glutGet (GLUT_WINDOW_WIDTH)  * 0.5f;
    const float h = glutGet (GLUT_WINDOW_HEIGHT) * 0.5f;

    draw2dLine (Vec3 (w+a, h,   0), Vec3 (w+b, h,   0), gWhite);
    draw2dLine (Vec3 (w,   h+a, 0), Vec3 (w,   h+b, 0), gWhite);
    draw2dLine (Vec3 (w-a, h,   0), Vec3 (w-b, h,   0), gWhite);
    draw2dLine (Vec3 (w,   h-a, 0), Vec3 (w,   h-b, 0), gWhite);

    glLineWidth (3);
    draw2dLine (Vec3 (w+a, h,   0), Vec3 (w+b, h,   0), gBlack);
    draw2dLine (Vec3 (w,   h+a, 0), Vec3 (w,   h+b, 0), gBlack);
    draw2dLine (Vec3 (w-a, h,   0), Vec3 (w-b, h,   0), gBlack);
    draw2dLine (Vec3 (w,   h-a, 0), Vec3 (w,   h-b, 0), gBlack);
    glLineWidth (1);
}


// ------------------------------------------------------------------------
// code (from main.cpp) used to draw "forward ruler" on vehicle

//     // xxx --------------------------------------------------
//     {
//         const Vec3 p = gSelectedVehicle->position;
//         const Vec3 f = gSelectedVehicle->forward;
//         const Vec3 s = gSelectedVehicle->side * 0.25f;
//         for (float i = 0; i <= 5; i++)
//         {
//             drawLine (p + (f * +i) + s, p + (f * +i) - s, gGray60);
//             drawLine (p + (f * -i) + s, p + (f * -i) - s, gGray60);
//         }
//     }
//     // xxx --------------------------------------------------


// ------------------------------------------------------------------------
// OpenGL-specific routine for error check, report, and exit


void checkForGLError (const char* locationDescription)
{
    // normally (when no error) just return
    const int lastGlError = glGetError();
    if (lastGlError == GL_NO_ERROR) return;

    // otherwise print vaguely descriptive error message, then exit
    std::cerr << std::endl << "SteerTest: OpenGL error ";
    switch (lastGlError)
    {
    case GL_INVALID_ENUM:      std::cerr << "GL_INVALID_ENUM";      break;
    case GL_INVALID_VALUE:     std::cerr << "GL_INVALID_VALUE";     break;
    case GL_INVALID_OPERATION: std::cerr << "GL_INVALID_OPERATION"; break;
    case GL_STACK_OVERFLOW:    std::cerr << "GL_STACK_OVERFLOW";    break;
    case GL_STACK_UNDERFLOW:   std::cerr << "GL_STACK_UNDERFLOW";   break;
    case GL_OUT_OF_MEMORY:     std::cerr << "GL_OUT_OF_MEMORY";     break;
#ifndef _WIN32
    case GL_TABLE_TOO_LARGE:   std::cerr << "GL_TABLE_TOO_LARGE";   break;
#endif
    }
    if (locationDescription) std::cerr << " in " << locationDescription;
    std::cerr << std::endl << std::endl << std::flush;
    SteerTest::exit (1);
}


// ------------------------------------------------------------------------
// check for errors during redraw, report any and then exit


void checkForDrawError (const char * locationDescription)
{
    checkForGLError (locationDescription);
}


// ----------------------------------------------------------------------------
// accessors for GLUT's window dimensions


float drawGetWindowHeight (void) {return glutGet (GLUT_WINDOW_HEIGHT);}
float drawGetWindowWidth  (void) {return glutGet (GLUT_WINDOW_WIDTH);}


// ----------------------------------------------------------------------------
// return a normalized direction vector pointing from the camera towards a
// given point on the screen: the ray that would be traced for that pixel


Vec3 directionFromCameraToScreenPosition (int x, int y)
{
    // Get window height, viewport, modelview and projection matrices
    GLint vp[4];
    GLdouble mMat[16], pMat[16];
    glGetIntegerv (GL_VIEWPORT, vp);
    glGetDoublev (GL_MODELVIEW_MATRIX, mMat);
    glGetDoublev (GL_PROJECTION_MATRIX, pMat);
    GLdouble un0x, un0y, un0z, un1x, un1y, un1z;
    const float h = glutGet (GLUT_WINDOW_HEIGHT);

    // Unproject mouse position at near and far clipping planes
    gluUnProject (x, h-y, 0, mMat, pMat, vp, &un0x, &un0y, &un0z);
    gluUnProject (x, h-y, 1, mMat, pMat, vp, &un1x, &un1y, &un1z);

    // "direction" is the normalized difference between these far and near
    // unprojected points.  Its parallel to the "eye-mouse" selection line.
    const Vec3 diffNearFar (un1x-un0x, un1y-un0y, un1z-un0z);
    const Vec3 direction = diffNearFar.normalize ();
    return direction;
}


// ----------------------------------------------------------------------------
// deferred draw line
//
// For use during simulation phase.
// Stores description of lines to be drawn later.
//
// XXX This should be rewritten using STL container classes


class DeferredLine
{
public:

    static void addToBuffer (const Vec3& s,
                             const Vec3& e,
                             const Vec3& c)
    {
        if (index < size)
        {
            deferredLineArray[index].startPoint = s;
            deferredLineArray[index].endPoint = e;
            deferredLineArray[index].color = c;
            index++;
        }
        else
        {
            SteerTest::printWarning ("overflow in deferredDrawLine buffer");
        }
    }

    static void drawAll (void)
    {
        // draw all lines in the buffer
        for (int i = 0; i < index; i++)
        {
            DeferredLine& dl = deferredLineArray[i];
            iDrawLine (dl.startPoint, dl.endPoint, dl.color);
        }

        // reset buffer index
        index = 0;
    }

private:

    Vec3 startPoint;
    Vec3 endPoint;
    Vec3 color;

    static int index;
    static const int size;
    static DeferredLine deferredLineArray [];
};


int DeferredLine::index = 0;
const int DeferredLine::size = 1000;
DeferredLine DeferredLine::deferredLineArray [DeferredLine::size];


void deferredDrawLine (const Vec3& startPoint,
                       const Vec3& endPoint,
                       const Vec3& color)
{
    DeferredLine::addToBuffer (startPoint, endPoint, color);
}


void drawAllDeferredLines (void)
{
    DeferredLine::drawAll ();
}


// ----------------------------------------------------------------------------
// deferred draw circle
// XXX for now, just a modified copy of DeferredLine
//
// For use during simulation phase.
// Stores description of circles to be drawn later.
//
// XXX This should be rewritten using STL container classes


class DeferredCircle
{
public:

    static void addToBuffer (const float radius,
                             const Vec3& axis,
                             const Vec3& center,
                             const Vec3& color,
                             const int segments,
                             const bool filled,
                             const bool in3d)
    {
        if (index < size)
        {
            deferredCircleArray[index].radius   = radius;
            deferredCircleArray[index].axis     = axis;
            deferredCircleArray[index].center   = center;
            deferredCircleArray[index].color    = color;
            deferredCircleArray[index].segments = segments;
            deferredCircleArray[index].filled   = filled;
            deferredCircleArray[index].in3d     = in3d;
            index++;
        }
        else
        {
            SteerTest::printWarning ("overflow in deferredDrawCircle buffer");
        }
    }

    static void drawAll (void)
    {
        // draw all circles in the buffer
        for (int i = 0; i < index; i++)
        {
            DeferredCircle& dc = deferredCircleArray[i];
            drawCircleOrDisk (dc.radius, dc.axis, dc.center, dc.color,
                              dc.segments, dc.filled, dc.in3d);
        }

        // reset buffer index
        index = 0;
    }

private:

    float radius;
    Vec3 axis;
    Vec3 center;
    Vec3 color;
    int segments;
    bool filled;
    bool in3d;

    static int index;
    static const int size;
    static DeferredCircle deferredCircleArray [];
};


int DeferredCircle::index = 0;
const int DeferredCircle::size = 500;
DeferredCircle DeferredCircle::deferredCircleArray [DeferredCircle::size];


void deferredDrawCircleOrDisk (const float radius,
                               const Vec3& axis,
                               const Vec3& center,
                               const Vec3& color,
                               const int segments,
                               const bool filled,
                               const bool in3d)
{
    DeferredCircle::addToBuffer (radius, axis, center, color,
                                 segments, filled, in3d);
}


void drawAllDeferredCirclesOrDisks (void)
{
    DeferredCircle::drawAll ();
}


// ------------------------------------------------------------------------
// Functions for drawing text (in GLUT's 9x15 bitmap font) in a given
// color, starting at a location on the screen which can be specified
// in screen space (draw2dTextAt2dLocation) or as the screen space
// projection of a location in 3d model space (draw2dTextAt3dLocation)
//
// based on code originally from:
//   Introduction to OpenGL - L23a - Displaying Text
//   http://www.york.ac.uk/services/cserv/sw/graphics/OPENGL/L23a.html

// xxx  Note: I *think* "const char* const s" means that both the pointer s
// xxx  AND the char string it points to are declared read only.  I should
// xxx  check that this is really the case.  I added it based on something
// xxx  from Telespace (Pedestrian constructor) xxx

// xxx  and for THAT matter, why not just use reference ("&") args instead?


// // void draw2dTextAt3dLocation (const char* s,
// void draw2dTextAt3dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
//     // set text color and raster position
//     glColor3f (color.x, color.y, color.z);
//     glRasterPos3f (location.x, location.y, location.z);

//     // loop over each character in string (until null terminator)
//     int lines = 0;
//     for (const char* p = s; *p; p++)
//     {
//         if (*p == '\n')
//         {
//             // handle "new line" character, reset raster position
//             lines++;
//             const int fontHeight = 15; // for GLUT_BITMAP_9_BY_15
//             const int vOffset = lines * (fontHeight + 1);
//             glRasterPos3f (location.x, location.y-vOffset, location.z);

//         }
//         else
//         {
//             // otherwise draw character bitmap
//             glutBitmapCharacter (GLUT_BITMAP_9_BY_15, *p);
//         }
//     }
// }


// // void draw2dTextAt2dLocation (char* s,
// void draw2dTextAt2dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
//     // store OpenGL matrix mode
//     int savedMatrixMode;
//     glGetIntegerv (GL_MATRIX_MODE, &savedMatrixMode);

//     // clear projection transform
//     glMatrixMode (GL_PROJECTION);
//     glPushMatrix ();
//     glLoadIdentity ();

//     // set up orthogonal projection onto window's screen space
//     const float w = glutGet (GLUT_WINDOW_WIDTH);
//     const float h = glutGet (GLUT_WINDOW_HEIGHT);
//     glOrtho (0.0f, w, 0.0f, h, -1.0f, 1.0f);

//     // clear model transform
//     glMatrixMode (GL_MODELVIEW);
//     glPushMatrix ();
//     glLoadIdentity ();

//     // draw text at specified location (which is now interpreted as
//     // relative to screen space) and color
//     draw2dTextAt3dLocation (s, location, color);

//     // restore previous model/projection transformation state
//     glPopMatrix ();
//     glMatrixMode (GL_PROJECTION);
//     glPopMatrix ();

//     // restore OpenGL matrix mode
//     glMatrixMode (savedMatrixMode);
// }




// // for now these cannot be nested (would need to have a stack of saved
// // xxx  matrix modes instead of just a global).



// int gxxxsavedMatrixMode;


// inline void begin2dDrawing (void)
// {
//     // store OpenGL matrix mode
// //     int savedMatrixMode;
//     glGetIntegerv (GL_MATRIX_MODE, &gxxxsavedMatrixMode);

//     // clear projection transform
//     glMatrixMode (GL_PROJECTION);
//     glPushMatrix ();
//     glLoadIdentity ();

//     // set up orthogonal projection onto window's screen space
//     const float w = glutGet (GLUT_WINDOW_WIDTH);
//     const float h = glutGet (GLUT_WINDOW_HEIGHT);
//     glOrtho (0.0f, w, 0.0f, h, -1.0f, 1.0f);

//     // clear model transform
//     glMatrixMode (GL_MODELVIEW);
//     glPushMatrix ();
//     glLoadIdentity ();
// }


// inline void end2dDrawing (void)
// {
//     // restore previous model/projection transformation state
//     glPopMatrix ();
//     glMatrixMode (GL_PROJECTION);
//     glPopMatrix ();

//     // restore OpenGL matrix mode
//     glMatrixMode (gxxxsavedMatrixMode);
// }



// void draw2dTextAt3dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
//     // set text color and raster position
//     glColor3f (color.x, color.y, color.z);
//     glRasterPos3f (location.x, location.y, location.z);

//     // loop over each character in string (until null terminator)
//     int lines = 0;
//     for (const char* p = s; *p; p++)
//     {
//         if (*p == '\n')

//             // handle "new line" character, reset raster position
//             lines++;
//             const int fontHeight = 15; // for GLUT_BITMAP_9_BY_15
//             const int vOffset = lines * (fontHeight + 1);
//             glRasterPos3f (location.x, location.y-vOffset, location.z);

//         }
//         else
//         {
//             // otherwise draw character bitmap
//             glutBitmapCharacter (GLUT_BITMAP_9_BY_15, *p);
//         }
//     }
// }


// void draw2dTextAt2dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
// //     // store OpenGL matrix mode
// //     int savedMatrixMode;
// //     glGetIntegerv (GL_MATRIX_MODE, &savedMatrixMode);

// //     // clear projection transform
// //     glMatrixMode (GL_PROJECTION);
// //     glPushMatrix ();
// //     glLoadIdentity ();

// //     // set up orthogonal projection onto window's screen space
// //     const float w = glutGet (GLUT_WINDOW_WIDTH);
// //     const float h = glutGet (GLUT_WINDOW_HEIGHT);
// //     glOrtho (0.0f, w, 0.0f, h, -1.0f, 1.0f);

// //     // clear model transform
// //     glMatrixMode (GL_MODELVIEW);
// //     glPushMatrix ();
// //     glLoadIdentity ();

//     begin2dDrawing ();

//     // draw text at specified location (which is now interpreted as
//     // relative to screen space) and color
//     draw2dTextAt3dLocation (s, location, color);

// //     // restore previous model/projection transformation state
// //     glPopMatrix ();
// //     glMatrixMode (GL_PROJECTION);
// //     glPopMatrix ();

// //     // restore OpenGL matrix mode
// //     glMatrixMode (savedMatrixMode);

//     end2dDrawing ();

// }


// // for now these cannot be nested (would need to have a stack of saved
// // xxx  matrix modes instead of just a global).



// int gxxxsavedMatrixMode;


// inline void begin2dDrawing (void)
// {
//     // store OpenGL matrix mode
// //     int savedMatrixMode;
//     glGetIntegerv (GL_MATRIX_MODE, &gxxxsavedMatrixMode);

//     // clear projection transform
//     glMatrixMode (GL_PROJECTION);
//     glPushMatrix ();
//     glLoadIdentity ();

//     // set up orthogonal projection onto window's screen space
//     const float w = glutGet (GLUT_WINDOW_WIDTH);
//     const float h = glutGet (GLUT_WINDOW_HEIGHT);
//     glOrtho (0.0f, w, 0.0f, h, -1.0f, 1.0f);

//     // clear model transform
//     glMatrixMode (GL_MODELVIEW);
//     glPushMatrix ();
//     glLoadIdentity ();
// }


// inline void end2dDrawing (void)
// {
//     // restore previous model/projection transformation state
//     glPopMatrix ();
//     glMatrixMode (GL_PROJECTION);
//     glPopMatrix ();

//     // restore OpenGL matrix mode
//     glMatrixMode (gxxxsavedMatrixMode);
// }



// void draw2dTextAt3dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
//     // set text color and raster position
//     glColor3f (color.x, color.y, color.z);
//     glRasterPos3f (location.x, location.y, location.z);

//     // switch into 2d screen space in case we need to handle a new-line
//     begin2dDrawing ();
//     GLint rasterPosition[4];
//     glGetIntegerv(GL_CURRENT_RASTER_POSITION, rasterPosition);
//     glRasterPos2i (rasterPosition[0], rasterPosition[1]);

//     // loop over each character in string (until null terminator)
//     int lines = 0;
//     for (const char* p = s; *p; p++)
//     {
//         if (*p == '\n')
//         {
//             // handle new-line character, reset raster position
//             lines++;
//             const int fontHeight = 15; // for GLUT_BITMAP_9_BY_15
//             const int vOffset = lines * (fontHeight + 1);
//             glRasterPos2i (rasterPosition[0], rasterPosition[1] - vOffset);
//         }
//         else
//         {
//             // otherwise draw character bitmap
//             glutBitmapCharacter (GLUT_BITMAP_9_BY_15, *p);
//         }
//     }

//     // xxx
//     end2dDrawing ();
// }


// void draw2dTextAt2dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
//     begin2dDrawing ();

//     // draw text at specified location (which is now interpreted as
//     // relative to screen space) and color
//     draw2dTextAt3dLocation (s, location, color);

//     end2dDrawing ();
// }


// // for now these cannot be nested (would need to have a stack of saved
// // xxx  matrix modes instead of just a global).
// int gxxxsavedMatrixMode;


inline GLint begin2dDrawing (void)
{
    // store OpenGL matrix mode
    GLint originalMatrixMode;
    glGetIntegerv (GL_MATRIX_MODE, &originalMatrixMode);

    // clear projection transform
    glMatrixMode (GL_PROJECTION);
    glPushMatrix ();
    glLoadIdentity ();

    // set up orthogonal projection onto window's screen space
    const float w = glutGet (GLUT_WINDOW_WIDTH);
    const float h = glutGet (GLUT_WINDOW_HEIGHT);
    glOrtho (0.0f, w, 0.0f, h, -1.0f, 1.0f);

    // clear model transform
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glLoadIdentity ();

    // return original matrix mode for saving (stacking)
    return originalMatrixMode;
}


inline void end2dDrawing (GLint originalMatrixMode)
{
    // restore previous model/projection transformation state
    glPopMatrix ();
    glMatrixMode (GL_PROJECTION);
    glPopMatrix ();

    // restore OpenGL matrix mode
    glMatrixMode (originalMatrixMode);
}



// void draw2dTextAt3dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
//     // set text color and raster position
//     glColor3f (color.x, color.y, color.z);
//     glRasterPos3f (location.x, location.y, location.z);

//     // switch into 2d screen space in case we need to handle a new-line
//     GLint rasterPosition[4];
//     glGetIntegerv (GL_CURRENT_RASTER_POSITION, rasterPosition);
//     const GLint originalMatrixMode = begin2dDrawing ();

//     //xxx uncommenting this causes the "2d" version to print the wrong thing
//     //xxx with it out the first line of a multi-line "3d" string jiggles
//     //glRasterPos2i (rasterPosition[0], rasterPosition[1]);

//     // loop over each character in string (until null terminator)
//     int lines = 0;
//     for (const char* p = s; *p; p++)
//     {
//         if (*p == '\n')
//         {
//             // handle new-line character, reset raster position
//             lines++;
//             const int fontHeight = 15; // for GLUT_BITMAP_9_BY_15
//             const int vOffset = lines * (fontHeight + 1);
//             glRasterPos2i (rasterPosition[0], rasterPosition[1] - vOffset);
//         }
//         else
//         {
//             // otherwise draw character bitmap
//             glutBitmapCharacter (GLUT_BITMAP_9_BY_15, *p);
//         }
//     }

//     // switch back out of 2d screen space
//     end2dDrawing (originalMatrixMode);
// }


// void draw2dTextAt2dLocation (const char* const s,
//                              const Vec3 location,
//                              const Vec3 color)
// {
//     const GLint originalMatrixMode = begin2dDrawing ();

//     // draw text at specified location (which is now interpreted as
//     // relative to screen space) and color
//     draw2dTextAt3dLocation (s, location, color);

//     end2dDrawing (originalMatrixMode);
// }


void draw2dTextAt3dLocation (const char& text,
                             const Vec3& location,
                             const Vec3& color)
{
    // XXX NOTE: "it would be nice if" this had a 2d screenspace offset for
    // the origin of the text relative to the screen space projection of
    // the 3d point.

    // set text color and raster position
    glColor3f (color.x, color.y, color.z);
    glRasterPos3f (location.x, location.y, location.z);

    // switch into 2d screen space in case we need to handle a new-line
    GLint rasterPosition[4];
    glGetIntegerv (GL_CURRENT_RASTER_POSITION, rasterPosition);
    const GLint originalMatrixMode = begin2dDrawing ();

    //xxx uncommenting this causes the "2d" version to print the wrong thing
    //xxx with it out the first line of a multi-line "3d" string jiggles
    //glRasterPos2i (rasterPosition[0], rasterPosition[1]);

    // loop over each character in string (until null terminator)
    int lines = 0;
    for (const char* p = &text; *p; p++)
    {
        if (*p == '\n')
        {
            // handle new-line character, reset raster position
            lines++;
            const int fontHeight = 15; // for GLUT_BITMAP_9_BY_15
            const int vOffset = lines * (fontHeight + 1);
            glRasterPos2i (rasterPosition[0], rasterPosition[1] - vOffset);
        }
        else
        {
            // otherwise draw character bitmap
            glutBitmapCharacter (GLUT_BITMAP_9_BY_15, *p);
        }
    }

    // switch back out of 2d screen space
    end2dDrawing (originalMatrixMode);
}

void draw2dTextAt3dLocation (const std::ostringstream& text,
                             const Vec3& location,
                             const Vec3& color)
{
    draw2dTextAt3dLocation (*text.str().c_str(), location, color);
}


void draw2dTextAt2dLocation (const char& text,
                             const Vec3 location,
                             const Vec3 color)
{
    const GLint originalMatrixMode = begin2dDrawing ();

    // draw text at specified location (which is now interpreted as
    // relative to screen space) and color
    draw2dTextAt3dLocation (text, location, color);

    end2dDrawing (originalMatrixMode);
}


void draw2dTextAt2dLocation (const std::ostringstream& text,
                             const Vec3 location,
                             const Vec3 color)
{
    draw2dTextAt2dLocation (*text.str().c_str(), location, color);
}


// ------------------------------------------------------------------------
// draw 2d lines in screen space: x and y are the relevant coordinates


void draw2dLine (const Vec3& startPoint,
                 const Vec3& endPoint,
                 const Vec3& color)
{
    const GLint originalMatrixMode = begin2dDrawing ();

    iDrawLine (startPoint, endPoint, color);

    end2dDrawing (originalMatrixMode);
}


// ----------------------------------------------------------------------------
