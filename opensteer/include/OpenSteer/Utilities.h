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
// Utilities for OpenSteering
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 07-09-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_UTILITIES_H
#define OPENSTEER_UTILITIES_H


#include <iostream>  // for ostream, <<, etc.
#include <cstdlib>   // for rand, etc.
#include <cfloat>    // for FLT_MAX, etc.
#include <cmath>     // for sqrt, etc.


// ----------------------------------------------------------------------------
// For the sake of Windows, apparently this is a "Linux/Unix thing"


#ifndef OPENSTEER_M_PI
#define OPENSTEER_M_PI 3.14159265358979323846f
#endif


namespace OpenSteer {

    // ----------------------------------------------------------------------------
    // Generic interpolation


    template<class T> inline T interpolate (float alpha, const T& x0, const T& x1)
    {
        return x0 + ((x1 - x0) * alpha);
    }


    // ----------------------------------------------------------------------------
    // Random number utilities


    // Returns a float randomly distributed between 0 and 1

    inline float frandom01 (void)
    {
        return (((float) rand ()) / ((float) RAND_MAX));
    }


    // Returns a float randomly distributed between lowerBound and upperBound

    inline float frandom2 (float lowerBound, float upperBound)
    {
        return lowerBound + (frandom01 () * (upperBound - lowerBound));
    }


    // ----------------------------------------------------------------------------
    // Constrain a given value (x) to be between two (ordered) bounds: min
    // and max.  Returns x if it is between the bounds, otherwise returns
    // the nearer bound.


    inline float clip (const float x, const float min, const float max)
    {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }


    // ----------------------------------------------------------------------------
    // remap a value specified relative to a pair of bounding values
    // to the corresponding value relative to another pair of bounds.
    // Inspired by (dyna:remap-interval y y0 y1 z0 z1)


    inline float remapInterval (float x,
                                float in0, float in1,
                                float out0, float out1)
    {
        // uninterpolate: what is x relative to the interval in0:in1?
        float relative = (x - in0) / (in1 - in0);

        // now interpolate between output interval based on relative x
        return interpolate (relative, out0, out1);
    }


    // Like remapInterval but the result is clipped to remain between
    // out0 and out1


    inline float remapIntervalClip (float x,
                                    float in0, float in1,
                                    float out0, float out1)
    {
        // uninterpolate: what is x relative to the interval in0:in1?
        float relative = (x - in0) / (in1 - in0);

        // now interpolate between output interval based on relative x
        return interpolate (clip (relative, 0, 1), out0, out1);
    }


    // ----------------------------------------------------------------------------
    // classify a value relative to the interval between two bounds:
    //     returns -1 when below the lower bound
    //     returns  0 when between the bounds (inside the interval)
    //     returns +1 when above the upper bound


    inline int intervalComparison (float x, float lowerBound, float upperBound)
    {
        if (x < lowerBound) return -1;
        if (x > upperBound) return +1;
        return 0;
    }



    // ----------------------------------------------------------------------------


    inline float scalarRandomWalk (const float initial, 
                                   const float walkspeed,
                                   const float min,
                                   const float max)
    {
        const float next = initial + (((frandom01() * 2) - 1) * walkspeed);
        if (next < min) return min;
        if (next > max) return max;
        return next;
    }


    // ----------------------------------------------------------------------------


    inline float square (float x)
    {
        return x * x;
    }


    // ----------------------------------------------------------------------------
    // for debugging: prints one line with a given C expression, an equals sign,
    // and the value of the expression.  For example "angle = 35.6"


    #define debugPrint(e) (std::cout << #e" = " << (e) << std::endl << std::flush)


    // ----------------------------------------------------------------------------
    // blends new values into an accumulator to produce a smoothed time series
    //
    // Modifies its third argument, a reference to the float accumulator holding
    // the "smoothed time series."
    //
    // The first argument (smoothRate) is typically made proportional to "dt" the
    // simulation time step.  If smoothRate is 0 the accumulator will not change,
    // if smoothRate is 1 the accumulator will be set to the new value with no
    // smoothing.  Useful values are "near zero".
    //
    // Usage:
    //         blendIntoAccumulator (dt * 0.4f, currentFPS, smoothedFPS);


    template<class T>
    inline void blendIntoAccumulator (const float smoothRate,
                                      const T& newValue,
                                      T& smoothedAccumulator)
    {
        smoothedAccumulator = interpolate (clip (smoothRate, 0, 1),
                                           smoothedAccumulator,
                                           newValue);
    }


    // ----------------------------------------------------------------------------
    // given a new Angle and an old angle, adjust the new for angle wraparound (the
    // 0->360 flip), returning a value equivalent to newAngle, but closest in
    // absolute value to oldAngle.  For radians fullCircle = OPENSTEER_M_PI*2, for degrees
    // fullCircle = 360.  Based on code in stuart/bird/fish demo's camera.cc
    //
    // (not currently used)

    /*
      inline float distance1D (const float a, const float b)
      {
          const float d = a - b;
          return (d > 0) ? d : -d;
      }


      float adjustForAngleWraparound (float newAngle,
                                      float oldAngle,
                                      float fullCircle)
      {
          // adjust newAngle for angle wraparound: consider its current value (a)
          // as well as the angle 2pi larger (b) and 2pi smaller (c).  Select the
          // one closer (magnitude of difference) to the current value of oldAngle.
          const float a = newAngle;
          const float b = newAngle + fullCircle;
          const float c = newAngle - fullCircle;
          const float ad = distance1D (a, oldAngle);
          const float bd = distance1D (b, oldAngle);
          const float cd = distance1D (c, oldAngle);

          if ((bd < ad) && (bd < cd)) return b;
          if ((cd < ad) && (cd < bd)) return c;
          return a;
      }
    */


    // ----------------------------------------------------------------------------
    // Functions to encapsulate cross-platform differences for several <cmath>
    // functions.  Specifically, the C++ standard says that these functions are
    // in the std namespace (std::sqrt, etc.)  Apparently the MS VC6 compiler (or
    // its header files) do not implement this correctly and the function names
    // are in the global namespace.  We hope these -XXX versions are a temporary
    // expedient, to be removed later.


    #ifdef _WIN32

    inline float floorXXX (float x)          {return floor (x);}
    inline float  sqrtXXX (float x)          {return sqrt (x);}
    inline float   sinXXX (float x)          {return sin (x);}
    inline float   cosXXX (float x)          {return cos (x);}
    inline float   absXXX (float x)          {return abs (x);}
    inline int     absXXX (int x)            {return abs (x);}
    inline float   maxXXX (float x, float y) {if (x > y) return x; else return y;}
    inline float   minXXX (float x, float y) {if (x < y) return x; else return y;}

    #else

    inline float floorXXX (float x)          {return std::floor (x);}
    inline float  sqrtXXX (float x)          {return std::sqrt (x);}
    inline float   sinXXX (float x)          {return std::sin (x);}
    inline float   cosXXX (float x)          {return std::cos (x);}
    inline float   absXXX (float x)          {return std::abs (x);}
    inline int     absXXX (int x)            {return std::abs (x);}
    inline float   maxXXX (float x, float y) {return std::max (x, y);}
    inline float   minXXX (float x, float y) {return std::min (x, y);}

    #endif


    // ----------------------------------------------------------------------------
    // round (x)  "round off" x to the nearest integer (as a float value)
    //
    // This is a Gnu-sanctioned(?) post-ANSI-Standard(?) extension (as in
    // http://www.opengroup.org/onlinepubs/007904975/basedefs/math.h.html)
    // which may not be present in all C++ environments.  It is defined in
    // math.h headers in Linux and Mac OS X, but apparently not in Win32:


    #ifdef _WIN32

    inline float round (float x)
    {
      if (x < 0)
          return -floorXXX (0.5f - x);
      else
          return  floorXXX (0.5f + x);
    }

    #else 
    
    inline float round( float x )
    {
        return ::round( x );
    }
    
    #endif

} // namespace OpenSteer
    
    
// ----------------------------------------------------------------------------
#endif // OPENSTEER_UTILITIES_H
