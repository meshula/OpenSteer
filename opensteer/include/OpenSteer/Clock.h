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
// discrete time simulation clock for SteerTest
//
// Keeps track of real clock time and simulation time.  Encapsulates the time
// API of the underlying operating system.  Can be put in either "as fast as
// possible" variable time step mode (where simulation time steps are based on
// real time elapsed between updates), or in fixed "target FPS" mode where the
// simulation steps are constrained to start on 1/FPS boundaries (e.g. on a 60
// hertz video game console).  Also handles the notion of "pausing" simulation
// time.
//
// Usage: allocate a clock, set its "paused" or "targetFPS" parameters, then
// call updateGlobalSimulationClock before each simulation step.
//
// 09-24-02 cwr: major overhaul
// 06-26-02 cwr: created
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_CLOCK_H
#define OPENSTEER_CLOCK_H

#ifdef _WIN32
#include <windows.h>
#endif



class Clock
{
public:

    // constructor
    Clock ();

    // ------------------------------------------------------- external API

    // update this clock, called once per simulation step ("frame")
    void update (void);

    // returns the number of seconds of real time (represented as a float)
    // since the clock was first updated.
    float realTimeSinceFirstClockUpdate (void);

    // toggles "pause" and "run" states
    bool togglePausedState (void) {return (paused = !paused);};

    // ("manually") force simulation time ahead, unrelated to the passage of
    // real time, currently used only for SteerTest's "single step forward"
    void advanceSimulationTime (const float seconds);

    // "wait" until next frame time
    void frameRateSync (void);

    // ------------------------------------------ externally-set parameters

    // is simulation running or paused?
    bool paused;

    // the desired rate of frames per second,
    // or zero to mean "as fast as possible"
    int targetFPS;

    // ---------------------------------- treat these as "read only" state

    // real "wall clock" time since launch
    float totalRealTime;

    // total time simulation has run
    float totalSimulationTime;

    // total time spent paused
    float totalPausedTime;

    // sum of (non-realtime driven) advances to simulation time
    float totalAdvanceTime;

    // interval since last simulation time
    // (xxx does this need to be stored in the instance? xxx)
    float elapsedSimulationTime;

    // interval since last clock update time 
    // (xxx does this need to be stored in the instance? xxx)
    float elapsedRealTime;

    // interval since last clock update,
    // exclusive of time spent waiting for frame boundary when targetFPS>0
    float elapsedNonWaitRealTime;

    // "manually" advance clock by this amount on next update
    float newAdvanceTime;

    // "Calendar time" when this clock was first updated
#ifdef _WIN32
    // from QueryPerformanceCounter on Windows
	LONGLONG basePerformanceCounter;
#else
    // from gettimeofday on Linux and Mac OS X
    int baseRealTimeSec;
    int baseRealTimeUsec;
#endif
};


// ----------------------------------------------------------------------------
#endif // OPENSTEER_CLOCK_H
