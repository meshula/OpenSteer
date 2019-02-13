// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2005, Sony Computer Entertainment America
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
// OpenSteerDemo
//
// This class encapsulates the state of the OpenSteerDemo application and the
// services it provides to its plug-ins.  It is never instantiated, all of
// its members are static (belong to the class as a whole.)
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 11-14-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Annotation.h"
#include "OpenSteer/Color.h"
#include "OpenSteer/Vec3.h"

#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

GLFWwindow* demo_window = nullptr;

// ----------------------------------------------------------------------------
// keeps track of both "real time" and "simulation time"


OpenSteer::Clock OpenSteer::OpenSteerDemo::clock;


// ----------------------------------------------------------------------------
// camera automatically tracks selected vehicle


OpenSteer::Camera OpenSteer::OpenSteerDemo::camera;


// ----------------------------------------------------------------------------
// currently selected plug-in (user can choose or cycle through them)


OpenSteer::PlugIn* OpenSteer::OpenSteerDemo::selectedPlugIn = NULL;


// ----------------------------------------------------------------------------
// currently selected vehicle.  Generally the one the camera follows and
// for which additional information may be displayed.  Clicking the mouse
// near a vehicle causes it to become the Selected Vehicle.


OpenSteer::AbstractVehicle* OpenSteer::OpenSteerDemo::selectedVehicle = NULL;


// ----------------------------------------------------------------------------
// phase: identifies current phase of the per-frame update cycle


int OpenSteer::OpenSteerDemo::phase = OpenSteer::OpenSteerDemo::overheadPhase;


// ----------------------------------------------------------------------------
// graphical annotation: master on/off switch


bool OpenSteer::enableAnnotation = true;


// ----------------------------------------------------------------------------
// XXX apparently MS VC6 cannot handle initialized static const members,
// XXX so they have to be initialized not-inline.


const int OpenSteer::OpenSteerDemo::overheadPhase = 0;
const int OpenSteer::OpenSteerDemo::updatePhase = 1;
const int OpenSteer::OpenSteerDemo::drawPhase = 2;


// ----------------------------------------------------------------------------
// initialize OpenSteerDemo application

namespace {

    void printPlugIn (OpenSteer::PlugIn& pi) {std::cout << " " << pi << std::endl;} // XXX

} // anonymous namespace

void 
OpenSteer::OpenSteerDemo::initialize (void)
{
    // select the default PlugIn
    selectDefaultPlugIn ();

    {
        // XXX this block is for debugging purposes,
        // XXX should it be replaced with something permanent?

        std::cout << std::endl << "Known plugins:" << std::endl;   // xxx?
        PlugIn::applyToAll (printPlugIn);                          // xxx?
        std::cout << std::endl;                                    // xxx?

        // identify default PlugIn
        if (!selectedPlugIn) errorExit ("no default PlugIn");
        std::cout << std::endl << "Default plugin:" << std::endl;  // xxx?
        std::cout << " " << *selectedPlugIn << std::endl;          // xxx?
        std::cout << std::endl;                                    // xxx?
    }

    // initialize the default PlugIn
    openSelectedPlugIn ();
}


// ----------------------------------------------------------------------------
// main update function: step simulation forward and redraw scene


void 
OpenSteer::OpenSteerDemo::updateSimulationAndRedraw (void)
{
    // update global simulation clock
    clock.update ();

    //  start the phase timer (XXX to accurately measure "overhead" time this
    //  should be in displayFunc, or somehow account for time outside this
    //  routine)
    initPhaseTimers ();

    // run selected PlugIn (with simulation's current time and step size)
    updateSelectedPlugIn (clock.getTotalSimulationTime (),
                          clock.getElapsedSimulationTime ());

    // redraw selected PlugIn (based on real time)
    redrawSelectedPlugIn (clock.getTotalRealTime (),
                          clock.getElapsedRealTime ());
}


// ----------------------------------------------------------------------------
// exit OpenSteerDemo with a given text message or error code


void 
OpenSteer::OpenSteerDemo::errorExit (const char* message)
{
    printMessage (message);
#ifdef _MSC_VER
	MessageBox(0, message, "OpenSteerDemo Unfortunate Event", MB_ICONERROR);
#endif
    exit (-1);
}


void 
OpenSteer::OpenSteerDemo::exit (int exitCode)
{
    ::exit (exitCode);
}


// ----------------------------------------------------------------------------
// select the default PlugIn


void 
OpenSteer::OpenSteerDemo::selectDefaultPlugIn (void)
{
    PlugIn::sortBySelectionOrder ();
    selectedPlugIn = PlugIn::findDefault ();
}


// ----------------------------------------------------------------------------
// select the "next" plug-in, cycling through "plug-in selection order"


void 
OpenSteer::OpenSteerDemo::selectNextPlugIn (void)
{
    closeSelectedPlugIn ();
    selectedPlugIn = selectedPlugIn->next ();
    openSelectedPlugIn ();
}


// ----------------------------------------------------------------------------
// handle function keys an a per-plug-in basis


void 
OpenSteer::OpenSteerDemo::functionKeyForPlugIn (int keyNumber)
{
    selectedPlugIn->handleFunctionKeys (keyNumber);
}


// ----------------------------------------------------------------------------
// return name of currently selected plug-in


const char* 
OpenSteer::OpenSteerDemo::nameOfSelectedPlugIn (void)
{
    return (selectedPlugIn ? selectedPlugIn->name() : "no PlugIn");
}


// ----------------------------------------------------------------------------
// open the currently selected plug-in


void 
OpenSteer::OpenSteerDemo::openSelectedPlugIn (void)
{
    camera.reset ();
    selectedVehicle = NULL;
    selectedPlugIn->open ();
}


// ----------------------------------------------------------------------------
// do a simulation update for the currently selected plug-in


void 
OpenSteer::OpenSteerDemo::updateSelectedPlugIn (const float currentTime,
                                                const float elapsedTime)
{
    // switch to Update phase
    pushPhase (updatePhase);

    // service queued reset request, if any
    doDelayedResetPlugInXXX ();

    // if no vehicle is selected, and some exist, select the first one
    if (selectedVehicle == NULL)
    {
        const AVGroup& vehicles = allVehiclesOfSelectedPlugIn();
        if (vehicles.size() > 0) selectedVehicle = vehicles.front();
    }

    // invoke selected PlugIn's Update method
    selectedPlugIn->update (currentTime, elapsedTime);

    // return to previous phase
    popPhase ();
}


// ----------------------------------------------------------------------------
// redraw graphics for the currently selected plug-in


void 
OpenSteer::OpenSteerDemo::redrawSelectedPlugIn (const float currentTime,
                                                const float elapsedTime)
{
    // switch to Draw phase
    pushPhase (drawPhase);

    // invoke selected PlugIn's Draw method
    selectedPlugIn->redraw (currentTime, elapsedTime);

    // draw any annotation queued up during selected PlugIn's Update method
    drawAllDeferredLines ();
    drawAllDeferredCirclesOrDisks ();

    // return to previous phase
    popPhase ();
}


// ----------------------------------------------------------------------------
// close the currently selected plug-in


void 
OpenSteer::OpenSteerDemo::closeSelectedPlugIn (void)
{
    selectedPlugIn->close ();
    selectedVehicle = NULL;
}


// ----------------------------------------------------------------------------
// reset the currently selected plug-in


void 
OpenSteer::OpenSteerDemo::resetSelectedPlugIn (void)
{
    selectedPlugIn->reset ();
}


namespace {

    // ----------------------------------------------------------------------------
    // XXX this is used by CaptureTheFlag
    // XXX it was moved here from main.cpp on 12-4-02
    // XXX I'm not sure if this is a useful feature or a bogus hack
    // XXX needs to be reconsidered.


    bool gDelayedResetPlugInXXX = false;

} // anonymous namespace
    
    
void 
OpenSteer::OpenSteerDemo::queueDelayedResetPlugInXXX (void)
{
    gDelayedResetPlugInXXX = true;
}


void 
OpenSteer::OpenSteerDemo::doDelayedResetPlugInXXX (void)
{
    if (gDelayedResetPlugInXXX)
    {
        resetSelectedPlugIn ();
        gDelayedResetPlugInXXX = false;
    }
}


// ----------------------------------------------------------------------------
// return a group (an STL vector of AbstractVehicle pointers) of all
// vehicles(/agents/characters) defined by the currently selected PlugIn


const OpenSteer::AVGroup& 
OpenSteer::OpenSteerDemo::allVehiclesOfSelectedPlugIn (void)
{
    return selectedPlugIn->allVehicles ();
}


// ----------------------------------------------------------------------------
// select the "next" vehicle: the one listed after the currently selected one
// in allVehiclesOfSelectedPlugIn


void 
OpenSteer::OpenSteerDemo::selectNextVehicle (void)
{
    if (selectedVehicle != NULL)
    {
        // get a container of all vehicles
        const AVGroup& all = allVehiclesOfSelectedPlugIn ();
        const AVIterator first = all.begin();
        const AVIterator last = all.end();

        // find selected vehicle in container
        const AVIterator s = std::find (first, last, selectedVehicle);

        // normally select the next vehicle in container
        selectedVehicle = *(s+1);

        // if we are at the end of the container, select the first vehicle
        if (s == last-1) selectedVehicle = *first;

        // if the search failed, use NULL
        if (s == last) selectedVehicle = NULL;
    }
}


// ----------------------------------------------------------------------------
// select vehicle nearest the given screen position (e.g.: of the mouse)


void 
OpenSteer::OpenSteerDemo::selectVehicleNearestScreenPosition (int x, int y)
{
    selectedVehicle = findVehicleNearestScreenPosition (x, y);
}


// ----------------------------------------------------------------------------
// Find the AbstractVehicle whose screen position is nearest the current the
// mouse position.  Returns NULL if mouse is outside this window or if
// there are no AbstractVehicle.


OpenSteer::AbstractVehicle* 
OpenSteer::OpenSteerDemo::vehicleNearestToMouse (void)
{
    return (mouseInWindow ? 
            findVehicleNearestScreenPosition (mouseX, mouseY) :
            NULL);
}


// ----------------------------------------------------------------------------
// Find the AbstractVehicle whose screen position is nearest the given window
// coordinates, typically the mouse position.  Returns NULL if there are no
// AbstractVehicles.
//
// This works by constructing a line in 3d space between the camera location
// and the "mouse point".  Then it measures the distance from that line to the
// centers of each AbstractVehicle.  It returns the AbstractVehicle whose
// distance is smallest.
//
// xxx Issues: Should the distanceFromLine test happen in "perspective space"
// xxx or in "screen space"?  Also: I think this would be happy to select a
// xxx vehicle BEHIND the camera location.


OpenSteer::AbstractVehicle* 
OpenSteer::OpenSteerDemo::findVehicleNearestScreenPosition (int x, int y)
{
    /// @TODO this routine is dependent on directionFromCameraToScreenPosition
    /// which needs an unproject implementation
    return nullptr;
#if 0
    // find the direction from the camera position to the given pixel
    const Vec3 direction = directionFromCameraToScreenPosition (x, y, glutGet (GLUT_WINDOW_HEIGHT));

    // iterate over all vehicles to find the one whose center is nearest the
    // "eye-mouse" selection line
    float minDistance = FLT_MAX;       // smallest distance found so far
    AbstractVehicle* nearest = NULL;   // vehicle whose distance is smallest
    const AVGroup& vehicles = allVehiclesOfSelectedPlugIn();
    for (AVIterator i = vehicles.begin(); i != vehicles.end(); i++)
    {
        // distance from this vehicle's center to the selection line:
        const float d = distanceFromLine ((**i).position(),
                                          camera.position(),
                                          direction);

        // if this vehicle-to-line distance is the smallest so far,
        // store it and this vehicle in the selection registers.
        if (d < minDistance)
        {
            minDistance = d;
            nearest = *i;
        }
    }

    return nearest;
#endif
}


// ----------------------------------------------------------------------------
// for storing most recent mouse state


int OpenSteer::OpenSteerDemo::mouseX = 0;
int OpenSteer::OpenSteerDemo::mouseY = 0;
bool OpenSteer::OpenSteerDemo::mouseInWindow = false;


// ----------------------------------------------------------------------------
// set a certain initial camera state used by several plug-ins


void 
OpenSteer::OpenSteerDemo::init3dCamera (AbstractVehicle& selected)
{
    init3dCamera (selected, cameraTargetDistance, camera2dElevation);
}

void 
OpenSteer::OpenSteerDemo::init3dCamera (AbstractVehicle& selected,
                                  float distance,
                                  float elevation)
{
    position3dCamera (selected, distance, elevation);
    camera.fixedDistDistance = distance;
    camera.fixedDistVOffset = elevation;
    camera.mode = Camera::cmFixedDistanceOffset;
}


void 
OpenSteer::OpenSteerDemo::init2dCamera (AbstractVehicle& selected)
{
    init2dCamera (selected, cameraTargetDistance, camera2dElevation);
}

void 
OpenSteer::OpenSteerDemo::init2dCamera (AbstractVehicle& selected,
                                  float distance,
                                  float elevation)
{
    position2dCamera (selected, distance, elevation);
    camera.fixedDistDistance = distance;
    camera.fixedDistVOffset = elevation;
    camera.mode = Camera::cmFixedDistanceOffset;
}


void 
OpenSteer::OpenSteerDemo::position3dCamera (AbstractVehicle& selected)
{
    position3dCamera (selected, cameraTargetDistance, camera2dElevation);
}

void 
OpenSteer::OpenSteerDemo::position3dCamera (AbstractVehicle& selected,
                                            float distance,
                                            float /*elevation*/)
{
    selectedVehicle = &selected;
    if (&selected)
    {
        const Vec3 behind = selected.forward() * -distance;
        camera.setPosition (selected.position() + behind);
        camera.target = selected.position();
    }
}


void 
OpenSteer::OpenSteerDemo::position2dCamera (AbstractVehicle& selected)
{
    position2dCamera (selected, cameraTargetDistance, camera2dElevation);
}

void 
OpenSteer::OpenSteerDemo::position2dCamera (AbstractVehicle& selected,
                                            float distance,
                                            float elevation)
{
    // position the camera as if in 3d:
    position3dCamera (selected, distance, elevation);

    // then adjust for 3d:
    Vec3 position3d = camera.position();
    position3d.y += elevation;
    camera.setPosition (position3d);
}


// ----------------------------------------------------------------------------
// camera updating utility used by several plug-ins


void 
OpenSteer::OpenSteerDemo::updateCamera (const float currentTime,
                                        const float elapsedTime,
                                        const AbstractVehicle* selected)
{
    camera.vehicleToTrack = selected;
    camera.update (currentTime, elapsedTime, clock.getPausedState ());
}


// ----------------------------------------------------------------------------
// some camera-related default constants


const float OpenSteer::OpenSteerDemo::camera2dElevation = 8;
const float OpenSteer::OpenSteerDemo::cameraTargetDistance = 13;
const OpenSteer::Vec3 OpenSteer::OpenSteerDemo::cameraTargetOffset (0, OpenSteer::OpenSteerDemo::camera2dElevation, 
                                                                    0);


// ----------------------------------------------------------------------------
// ground plane grid-drawing utility used by several plug-ins


void 
OpenSteer::OpenSteerDemo::gridUtility (const Vec3& gridTarget)
{
    // round off target to the nearest multiple of 2 (because the
    // checkboard grid with a pitch of 1 tiles with a period of 2)
    // then lower the grid a bit to put it under 2d annotation lines
    const Vec3 gridCenter ((round (gridTarget.x * 0.5f) * 2),
                           (round (gridTarget.y * 0.5f) * 2) - .05f,
                           (round (gridTarget.z * 0.5f) * 2));

    // colors for checkboard
    const Color gray1(0.27f);
    const Color gray2(0.30f);

    // draw 50x50 checkerboard grid with 50 squares along each side
    drawXZCheckerboardGrid (50, 50, gridCenter, gray1, gray2);

    // alternate style
    // drawXZLineGrid (50, 50, gridCenter, gBlack);
}


// ----------------------------------------------------------------------------
// draws a gray disk on the XZ plane under a given vehicle


void 
OpenSteer::OpenSteerDemo::highlightVehicleUtility (const AbstractVehicle* vehicle)
{
    if (vehicle)
        drawXZDisk (vehicle->radius(), vehicle->position(), gGray60, 20);
}


// ----------------------------------------------------------------------------
// draws a gray circle on the XZ plane under a given vehicle


void 
OpenSteer::OpenSteerDemo::circleHighlightVehicleUtility (const AbstractVehicle* vehicle)
{
    if (vehicle) drawXZCircle (vehicle->radius () * 1.1f,
                                        vehicle->position(),
                                        gGray60,
                                        20);
}


// ----------------------------------------------------------------------------
// draw a box around a vehicle aligned with its local space
// xxx not used as of 11-20-02


void 
OpenSteer::OpenSteerDemo::drawBoxHighlightOnVehicle (const AbstractVehicle* v,
                                               const Color& color)
{
    if (v)
    {
        const float diameter = v->radius() * 2;
        const Vec3 size (diameter, diameter, diameter);
        drawBoxOutline (v, size, color);
    }
}


// ----------------------------------------------------------------------------
// draws a colored circle (perpendicular to view axis) around the center
// of a given vehicle.  The circle's radius is the vehicle's radius times
// radiusMultiplier.


void 
OpenSteer::OpenSteerDemo::drawCircleHighlightOnVehicle (const AbstractVehicle* v,
                                                  const float radiusMultiplier,
                                                  const Color& color)
{
    if (v)
    {
        const Vec3& cPosition = camera.position();
        draw3dCircle  (v->radius() * radiusMultiplier,  // adjusted radius
                       v->position(),                   // center
                       v->position() - cPosition,       // view axis
                       color,                          // drawing color
                       20);                            // circle segments
    }
}


// ----------------------------------------------------------------------------


void 
OpenSteer::OpenSteerDemo::printMessage (const char* message)
{
    std::cout << "OpenSteerDemo: " <<  message << std::endl << std::flush;
}


void 
OpenSteer::OpenSteerDemo::printMessage (const std::ostringstream& message)
{
    printMessage (message.str().c_str());
}


void 
OpenSteer::OpenSteerDemo::printWarning (const char* message)
{
    std::cout << "OpenSteerDemo: Warning: " <<  message << std::endl << std::flush;
}


void 
OpenSteer::OpenSteerDemo::printWarning (const std::ostringstream& message)
{
    printWarning (message.str().c_str());
}


// ------------------------------------------------------------------------
// print list of known commands
//
// XXX this list should be assembled automatically,
// XXX perhaps from a list of "command" objects created at initialization


void 
OpenSteer::OpenSteerDemo::keyboardMiniHelp (void)
{
    printMessage ("");
    printMessage ("defined single key commands:");
    printMessage ("  r      restart current PlugIn.");
    printMessage ("  s      select next vehicle.");
    printMessage ("  c      select next camera mode.");
    printMessage ("  f      select next preset frame rate");
    printMessage ("  Tab    select next PlugIn.");
    printMessage ("  a      toggle annotation on/off.");
    printMessage ("  Space  toggle between Run and Pause.");
    printMessage ("  ->     step forward one frame.");
    printMessage ("  Esc    exit.");
    printMessage ("");

    // allow PlugIn to print mini help for the function keys it handles
    selectedPlugIn->printMiniHelpForFunctionKeys ();
}


// ----------------------------------------------------------------------------
// manage OpenSteerDemo phase transitions (xxx and maintain phase timers)


int OpenSteer::OpenSteerDemo::phaseStackIndex = 0;
const int OpenSteer::OpenSteerDemo::phaseStackSize = 5;
int OpenSteer::OpenSteerDemo::phaseStack [OpenSteer::OpenSteerDemo::phaseStackSize];

namespace OpenSteer {
bool updatePhaseActive = false;
bool drawPhaseActive = false;
}

void 
OpenSteer::OpenSteerDemo::pushPhase (const int newPhase)
{
    updatePhaseActive = newPhase == OpenSteer::OpenSteerDemo::updatePhase;
    drawPhaseActive = newPhase == OpenSteer::OpenSteerDemo::drawPhase;

    // update timer for current (old) phase: add in time since last switch
    updatePhaseTimers ();

    // save old phase
    phaseStack[phaseStackIndex++] = phase;

    // set new phase
    phase = newPhase;

    // check for stack overflow
    if (phaseStackIndex >= phaseStackSize) errorExit ("phaseStack overflow");
}


void 
OpenSteer::OpenSteerDemo::popPhase (void)
{
    // update timer for current (old) phase: add in time since last switch
    updatePhaseTimers ();

    // restore old phase
    phase = phaseStack[--phaseStackIndex];
    updatePhaseActive = phase == OpenSteer::OpenSteerDemo::updatePhase;
    drawPhaseActive = phase == OpenSteer::OpenSteerDemo::drawPhase;
}


// ----------------------------------------------------------------------------


float OpenSteer::OpenSteerDemo::phaseTimerBase = 0;
float OpenSteer::OpenSteerDemo::phaseTimers [drawPhase+1];


void 
OpenSteer::OpenSteerDemo::initPhaseTimers (void)
{
    phaseTimers[drawPhase] = 0;
    phaseTimers[updatePhase] = 0;
    phaseTimers[overheadPhase] = 0;
    phaseTimerBase = clock.getTotalRealTime ();
}


void 
OpenSteer::OpenSteerDemo::updatePhaseTimers (void)
{
    const float currentRealTime = clock.realTimeSinceFirstClockUpdate();
    phaseTimers[phase] += currentRealTime - phaseTimerBase;
    phaseTimerBase = currentRealTime;
}


// ----------------------------------------------------------------------------


namespace {

    std::string const appVersionName("OpenSteerDemo 0.8.3");

    bool gMouseAdjustingCameraAngle = false;
    bool gMouseAdjustingCameraRadius = false;
    int gMouseAdjustingCameraLastX;
    int gMouseAdjustingCameraLastY;


    // ----------------------------------------------------------------------------
    // handler for window resizing

    void reshape( GLFWwindow* window, int width, int height )
    {
      GLfloat h = (GLfloat) height / (GLfloat) width;
      GLfloat xmax, znear, zfar;

      znear = 1.0f;
      zfar  = 400.0f;
      xmax  = znear * 0.5f;

      glViewport( 0, 0, (GLint) width, (GLint) height );
      glMatrixMode( GL_PROJECTION );
      glLoadIdentity();
      glFrustum( -xmax, xmax, -xmax*h, xmax*h, znear, zfar );
      glMatrixMode( GL_MODELVIEW );
    }


    // ----------------------------------------------------------------------------
    // This is called (by glfw) each time a mouse button pressed or released.

    void 
    mouseButtonFunc(GLFWwindow* window, int button, int action, int mods)
    {
        // if the mouse button has just been released
        if (action == GLFW_RELEASE)
        {
            // end any ongoing mouse-adjusting-camera session
            gMouseAdjustingCameraAngle = false;
            gMouseAdjustingCameraRadius = false;
        }

        // if the mouse button has just been pushed down
        if (action == GLFW_PRESS)
        {
            // names for relevant values of "button" and "state"
            const bool modNone    = (mods == 0);
            const bool modCtrl    = (mods & GLFW_MOD_CONTROL);
            const bool modAlt     = (mods & GLFW_MOD_ALT);
            const bool modCtrlAlt = modCtrl || modAlt;
            const bool mouseL     = (button == 0);
            const bool mouseM     = (button == 1);
            const bool mouseR     = (button == 2);

    #if __APPLE__ && __MACH__
            const bool macosx = true;
    #else
            const bool macosx = false;
    #endif

            // mouse-left (with no modifiers): select vehicle
            if (modNone && mouseL)
            {
                OpenSteer::OpenSteerDemo::selectVehicleNearestScreenPosition (OpenSteer::OpenSteerDemo::mouseX, OpenSteer::OpenSteerDemo::mouseY);
            }

            // control-mouse-left: begin adjusting camera angle (on Mac OS X
            // control-mouse maps to mouse-right for "context menu", this makes
            // OpenSteerDemo's control-mouse work work the same on OS X as on Linux
            // and Windows, but it precludes using a mouseR context menu)
            if ((modCtrl && mouseL) ||
               (modNone && mouseR && macosx))
            {
                gMouseAdjustingCameraLastX = OpenSteer::OpenSteerDemo::mouseX;
                gMouseAdjustingCameraLastY = OpenSteer::OpenSteerDemo::mouseY;
                gMouseAdjustingCameraAngle = true;
            }

            // control-mouse-middle: begin adjusting camera radius
            // (same for: control-alt-mouse-left and control-alt-mouse-middle,
            // and on Mac OS X it is alt-mouse-right)
            if ((modCtrl    && mouseM) ||
                (modCtrlAlt && mouseL) ||
                (modCtrlAlt && mouseM) ||
                (modAlt     && mouseR && macosx))
            {
                gMouseAdjustingCameraLastX = OpenSteer::OpenSteerDemo::mouseX;
                gMouseAdjustingCameraLastY = OpenSteer::OpenSteerDemo::mouseY;
                gMouseAdjustingCameraRadius = true;
            }
        }
    }


    // ----------------------------------------------------------------------------
    // called when mouse moves and no buttons are down

    void 
    mouseMotionFunc(GLFWwindow* window, double x, double y)
    {
        OpenSteer::OpenSteerDemo::mouseX = static_cast<int>(x);
        OpenSteer::OpenSteerDemo::mouseY = static_cast<int>(y);

        if (glfwGetMouseButton(window, 0))
        {
            // are we currently in the process of mouse-adjusting the camera?
            if (gMouseAdjustingCameraAngle || gMouseAdjustingCameraRadius)
            {
                // speed factors to map from mouse movement in pixels to 3d motion
                const float dSpeed = 0.005f;
                const float rSpeed = 0.01f;

                // XY distance (in pixels) that mouse moved since last update
                const float dx = static_cast<float>(x - gMouseAdjustingCameraLastX);
                const float dy = static_cast<float>(y - gMouseAdjustingCameraLastY);
                gMouseAdjustingCameraLastX = static_cast<int>(x);
                gMouseAdjustingCameraLastY = static_cast<int>(y);

                OpenSteer::Vec3 cameraAdjustment;

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
                OpenSteer::OpenSteerDemo::camera.mouseAdjustOffset (cameraAdjustment);
            }
        }
    }


    // ----------------------------------------------------------------------------
    // called when mouse enters or exits the window

    void mouseEnterExitWindowFunc(GLFWwindow* window, int entered)
    {
        OpenSteer::OpenSteerDemo::mouseInWindow = !!entered;
    }



    // ----------------------------------------------------------------------------
    // draw PlugI name in upper lefthand corner of screen


    void 
    drawDisplayPlugInName (void)
    {
        const float h = OpenSteer::drawGetWindowWidth();
        const OpenSteer::Vec3 screenLocation (10, h-20, 0);
        draw2dTextAt2dLocation (*OpenSteer::OpenSteerDemo::nameOfSelectedPlugIn (),
                                screenLocation,
                                OpenSteer::gWhite, OpenSteer::drawGetWindowWidth(), OpenSteer::drawGetWindowHeight());
    }


    // ----------------------------------------------------------------------------
    // draw camera mode name in lower lefthand corner of screen


    void 
    drawDisplayCameraModeName (void)
    {
        std::ostringstream message;
        message << "Camera: " << OpenSteer::OpenSteerDemo::camera.modeName () << std::ends;
        const OpenSteer::Vec3 screenLocation (10, 10, 0);
        OpenSteer::draw2dTextAt2dLocation (message, screenLocation, OpenSteer::gWhite, OpenSteer::drawGetWindowWidth(), OpenSteer::drawGetWindowHeight());
    }


    // ----------------------------------------------------------------------------
    // helper for drawDisplayFPS



    void 
    writePhaseTimerReportToStream (float phaseTimer,
                                              std::ostringstream& stream)
    {
        // write the timer value in seconds in floating point
        stream << std::setprecision (5) << std::setiosflags (std::ios::fixed);
        stream << phaseTimer;

        // restate value in another form
        stream << std::setprecision (0) << std::setiosflags (std::ios::fixed);
        stream << " (";

        // different notation for variable and fixed frame rate
        if (OpenSteer::OpenSteerDemo::clock.getVariableFrameRateMode())
        {
            // express as FPS (inverse of phase time)
            stream << 1 / phaseTimer;
            stream << " fps)\n";
        }
        else
        {
            // quantify time as a percentage of frame time
            const int fps = OpenSteer::OpenSteerDemo::clock.getFixedFrameRate ();
            stream << ((100 * phaseTimer) / (1.0f / fps));
            stream << "% of 1/";
            stream << fps;
            stream << "sec)\n";
        }
    }


    // ----------------------------------------------------------------------------
    // draw text showing (smoothed, rounded) "frames per second" rate
    // (and later a bunch of related stuff was dumped here, a reorg would be nice)
    //
    // XXX note: drawDisplayFPS has morphed considerably and should be called
    // something like displayClockStatus, and that it should be part of
    // OpenSteerDemo instead of Draw  (cwr 11-23-04)

    float gSmoothedTimerDraw = 0;
    float gSmoothedTimerUpdate = 0;
    float gSmoothedTimerOverhead = 0;

    void
    drawDisplayFPS (void)
    {
        // skip several frames to allow frame rate to settle
        static int skipCount = 10;
        if (skipCount > 0)
        {
            skipCount--;
        }
        else
        {
            // keep track of font metrics and start of next line
            const int lh = 16; // xxx line height
            const int cw = 9; // xxx character width
            OpenSteer::Vec3 screenLocation (10, 10, 0);

            // target and recent average frame rates
            const int targetFPS = OpenSteer::OpenSteerDemo::clock.getFixedFrameRate ();
            const float smoothedFPS = OpenSteer::OpenSteerDemo::clock.getSmoothedFPS ();

            // describe clock mode and frame rate statistics
            screenLocation.y += lh;
            std::ostringstream clockStr;
            clockStr << "Clock: ";
            if (OpenSteer::OpenSteerDemo::clock.getAnimationMode ())
            {
                clockStr << "animation mode (";
                clockStr << targetFPS << " fps,";
                clockStr << " display "<< OpenSteer::round(smoothedFPS) << " fps, ";
                const float ratio = smoothedFPS / targetFPS;
                clockStr << (int) (100 * ratio) << "% of nominal speed)";
            }
            else
            {
                clockStr << "real-time mode, ";
                if (OpenSteer::OpenSteerDemo::clock.getVariableFrameRateMode ())
                {
                    clockStr << "variable frame rate (";
                    clockStr << OpenSteer::round(smoothedFPS) << " fps)";
                }
                else
                {
                    clockStr << "fixed frame rate (target: " << targetFPS;
                    clockStr << " actual: " << OpenSteer::round(smoothedFPS) << ", ";

                    OpenSteer::Vec3 sp;
                    sp = screenLocation;
                    sp.x += cw * (int) clockStr.tellp ();

                    // create usage description character string
                    std::ostringstream xxxStr;
                    xxxStr << std::setprecision (0)
                           << std::setiosflags (std::ios::fixed)
                           << "usage: " << OpenSteer::OpenSteerDemo::clock.getSmoothedUsage ()
                           << "%"
                           << std::ends;

                    const int usageLength = ((int) xxxStr.tellp ()) - 1;
                    for (int i = 0; i < usageLength; i++) clockStr << " ";
                    clockStr << ")";

                    // display message in lower left corner of window
                    // (draw in red if the instantaneous usage is 100% or more)
                    const float usage = OpenSteer::OpenSteerDemo::clock.getUsage ();
                    const OpenSteer::Color color = (usage >= 100) ? OpenSteer::gRed : OpenSteer::gWhite;
                    draw2dTextAt2dLocation (xxxStr, sp, color, OpenSteer::drawGetWindowWidth(), OpenSteer::drawGetWindowHeight());
                }
            }
            if (OpenSteer::OpenSteerDemo::clock.getPausedState ())
                clockStr << " [paused]";
            clockStr << std::ends;
            draw2dTextAt2dLocation (clockStr, screenLocation, OpenSteer::gWhite, OpenSteer::drawGetWindowWidth(), OpenSteer::drawGetWindowHeight());

            // get smoothed phase timer information
            const float ptd = OpenSteer::OpenSteerDemo::phaseTimerDraw();
            const float ptu = OpenSteer::OpenSteerDemo::phaseTimerUpdate();
            const float pto = OpenSteer::OpenSteerDemo::phaseTimerOverhead();
            const float smoothRate = OpenSteer::OpenSteerDemo::clock.getSmoothingRate ();
            OpenSteer::blendIntoAccumulator (smoothRate, ptd, gSmoothedTimerDraw);
            OpenSteer::blendIntoAccumulator (smoothRate, ptu, gSmoothedTimerUpdate);
            OpenSteer::blendIntoAccumulator (smoothRate, pto, gSmoothedTimerOverhead);

            // display phase timer information
            screenLocation.y += lh * 4;
            std::ostringstream timerStr;
            timerStr << "update: ";
            writePhaseTimerReportToStream (gSmoothedTimerUpdate, timerStr);
            timerStr << "draw:   ";
            writePhaseTimerReportToStream (gSmoothedTimerDraw, timerStr);
            timerStr << "other:  ";
            writePhaseTimerReportToStream (gSmoothedTimerOverhead, timerStr);
            timerStr << std::ends;
            draw2dTextAt2dLocation (timerStr, screenLocation, OpenSteer::gGreen, OpenSteer::drawGetWindowWidth(), OpenSteer::drawGetWindowHeight());
        }
    }


    // ------------------------------------------------------------------------
    // cycle through frame rate presets  (XXX move this to OpenSteerDemo)


    void 
    selectNextPresetFrameRate (void)
    {
        // note that the cases are listed in reverse order, and that 
        // the default is case 0 which causes the index to wrap around
        static int frameRatePresetIndex = 0;
        switch (++frameRatePresetIndex)
        {
        case 3: 
            // animation mode at 60 fps
            OpenSteer::OpenSteerDemo::clock.setFixedFrameRate (60);
            OpenSteer::OpenSteerDemo::clock.setAnimationMode (true);
            OpenSteer::OpenSteerDemo::clock.setVariableFrameRateMode (false);
            break;
        case 2: 
            // real-time fixed frame rate mode at 60 fps
            OpenSteer::OpenSteerDemo::clock.setFixedFrameRate (60);
            OpenSteer::OpenSteerDemo::clock.setAnimationMode (false);
            OpenSteer::OpenSteerDemo::clock.setVariableFrameRateMode (false);
            break;
        case 1: 
            // real-time fixed frame rate mode at 24 fps
            OpenSteer::OpenSteerDemo::clock.setFixedFrameRate (24);
            OpenSteer::OpenSteerDemo::clock.setAnimationMode (false);
            OpenSteer::OpenSteerDemo::clock.setVariableFrameRateMode (false);
            break;
        case 0:
        default:
            // real-time variable frame rate mode ("as fast as possible")
            frameRatePresetIndex = 0;
            OpenSteer::OpenSteerDemo::clock.setFixedFrameRate (0);
            OpenSteer::OpenSteerDemo::clock.setAnimationMode (false);
            OpenSteer::OpenSteerDemo::clock.setVariableFrameRateMode (true);
            break;
        }
    }


    // ------------------------------------------------------------------------
    // This function is called (by glfw) each time a key is pressed.
    //
    void key( GLFWwindow* window, int key, int s, int action, int mods )
    {
        std::ostringstream message;

        switch (key)
        {
        // reset selected PlugIn
        case 'R':
            OpenSteer::OpenSteerDemo::resetSelectedPlugIn ();
            message << "reset PlugIn "
                    << '"' << OpenSteer::OpenSteerDemo::nameOfSelectedPlugIn () << '"'
                    << std::ends;
            OpenSteer::OpenSteerDemo::printMessage (message);
            break;

        // cycle selection to next vehicle
        case 'S':
            OpenSteer::OpenSteerDemo::printMessage ("select next vehicle/agent");
            OpenSteer::OpenSteerDemo::selectNextVehicle ();
            break;

        // camera mode cycle
        case 'C':
            OpenSteer::OpenSteerDemo::camera.selectNextMode ();
            message << "select camera mode "
                    << '"' << OpenSteer::OpenSteerDemo::camera.modeName () << '"' << std::ends;
            OpenSteer::OpenSteerDemo::printMessage (message);
            break;

        // select next PlugIn
        case GLFW_KEY_TAB:
            OpenSteer::OpenSteerDemo::selectNextPlugIn ();
            message << "select next PlugIn: "
                    << '"' << OpenSteer::OpenSteerDemo::nameOfSelectedPlugIn () << '"'
                    << std::ends;
            OpenSteer::OpenSteerDemo::printMessage (message);
            break;

        // toggle annotation state
        case 'A':
            OpenSteer::OpenSteerDemo::printMessage (OpenSteer::toggleAnnotationState () ?
                                                    "annotation ON" : "annotation OFF");
            break;

        // toggle run/pause state
        case GLFW_KEY_SPACE:
            OpenSteer::OpenSteerDemo::printMessage (OpenSteer::OpenSteerDemo::clock.togglePausedState () ?
                                                    "pause" : "run");
            break;

        // cycle through frame rate (clock mode) presets
        case 'F':
            selectNextPresetFrameRate ();
            message << "set clock to ";
            if (OpenSteer::OpenSteerDemo::clock.getAnimationMode ())
                message << "animation mode, fixed frame rate ("
                        << OpenSteer::OpenSteerDemo::clock.getFixedFrameRate () << " fps)";
            else
            {
                message << "real-time mode, ";
                if (OpenSteer::OpenSteerDemo::clock.getVariableFrameRateMode ())
                    message << "variable frame rate";
                else
                    message << "fixed frame rate ("
                            << OpenSteer::OpenSteerDemo::clock.getFixedFrameRate () << " fps)";
            }
            message << std::ends;
            OpenSteer::OpenSteerDemo::printMessage (message);
            break;

        // print minimal help for single key commands
        case '?':
            OpenSteer::OpenSteerDemo::keyboardMiniHelp ();
            break;

        // exit application with normal status 
        case GLFW_KEY_ESCAPE:
            OpenSteer::OpenSteerDemo::printMessage ("exit.");
            OpenSteer::OpenSteerDemo::exit (0);

        case GLFW_KEY_F1:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (1);  break;
        case GLFW_KEY_F2:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (2);  break;
        case GLFW_KEY_F3:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (3);  break;
        case GLFW_KEY_F4:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (4);  break;
        case GLFW_KEY_F5:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (5);  break;
        case GLFW_KEY_F6:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (6);  break;
        case GLFW_KEY_F7:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (7);  break;
        case GLFW_KEY_F8:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (8);  break;
        case GLFW_KEY_F9:  OpenSteer::OpenSteerDemo::functionKeyForPlugIn (9);  break;
        case GLFW_KEY_F10: OpenSteer::OpenSteerDemo::functionKeyForPlugIn (10); break;
        case GLFW_KEY_F11: OpenSteer::OpenSteerDemo::functionKeyForPlugIn (11); break;
        case GLFW_KEY_F12: OpenSteer::OpenSteerDemo::functionKeyForPlugIn (12); break;

        case GLFW_KEY_RIGHT:
            OpenSteer::OpenSteerDemo::clock.setPausedState (true);
            message << "single step forward (frame time: "
                    << OpenSteer::OpenSteerDemo::clock.advanceSimulationTimeOneFrame ()
                    << ")"
                    << std::endl;
            OpenSteer::OpenSteerDemo::printMessage (message);
            break;

        default:
            message << "unrecognized single key command: " << key;
            message << " (" << (int)key << ")";//xxx perhaps only for debugging?
            message << std::ends;
            OpenSteer::OpenSteerDemo::printMessage ("");
            OpenSteer::OpenSteerDemo::printMessage (message);
            OpenSteer::OpenSteerDemo::keyboardMiniHelp ();
        }
    }


    // ------------------------------------------------------------------------
    // Main drawing function for OpenSteerDemo application,
    // drives simulation as a side effect


    void 
    displayFunc (void)
    {
        // clear color and depth buffers
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // run simulation and draw associated graphics
        OpenSteer::OpenSteerDemo::updateSimulationAndRedraw ();

        // draw text showing (smoothed, rounded) "frames per second" rate
        drawDisplayFPS ();

        // draw the name of the selected PlugIn
        drawDisplayPlugInName ();

        // draw the name of the camera's current mode
        drawDisplayCameraModeName ();

        // draw crosshairs to indicate aimpoint (xxx for debugging only?)
        // drawReticle ();

        // check for errors in drawing module, if so report and exit
        OpenSteer::checkForDrawError ("OpenSteerDemo::updateSimulationAndRedraw");

        // double buffering, swap back and front buffers
        glFlush ();
    }


} // annonymous namespace



// ----------------------------------------------------------------------------
// do all initialization related to graphics


void 
OpenSteer::initializeGraphics (int argc, char **argv)
{
    if (!glfwInit())
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        exit( EXIT_FAILURE );
    }

    demo_window = glfwCreateWindow( 1200, 900, "OpenSteer", NULL, NULL );
    if (!demo_window)
    {
        fprintf( stderr, "Failed to open GLFW window\n" );
        glfwTerminate();
        exit( EXIT_FAILURE );
    }

    // Set callback functions
    glfwSetFramebufferSizeCallback(demo_window, reshape);
    glfwSetKeyCallback(demo_window, key);
    glfwSetCursorEnterCallback(demo_window, mouseEnterExitWindowFunc);
    glfwSetCursorPosCallback(demo_window, mouseMotionFunc);
    glfwSetMouseButtonCallback(demo_window, mouseButtonFunc);

    glfwMakeContextCurrent(demo_window);
    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
    glfwSwapInterval( 1 );

    int width, height;
    glfwGetFramebufferSize(demo_window, &width, &height);
    reshape(demo_window, width, height);
}


// ----------------------------------------------------------------------------
// run graphics event loop


void 
OpenSteer::runGraphics (void)
{
    
    // Main loop
    while( !glfwWindowShouldClose(demo_window) )
    {
        displayFunc();

        // Swap buffers
        glfwSwapBuffers(demo_window);
        glfwPollEvents();
    }
    glfwTerminate();
}



// ----------------------------------------------------------------------------
// accessors for GLUT's window dimensions


float 
OpenSteer::drawGetWindowHeight (void) 
{
    int width, height;
    glfwGetWindowSize(demo_window, &width, &height);
    return static_cast<float>(height);
}


float 
OpenSteer::drawGetWindowWidth  (void) 
{
    int width, height;
    glfwGetWindowSize(demo_window, &width, &height);
    return static_cast<float>(width);
}


