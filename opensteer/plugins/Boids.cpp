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
// OpenSteer Boids
// 
// 09-26-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/SteerTest.h"


// ----------------------------------------------------------------------------


class Boid : public SimpleVehicle
{
public:

    // type for a flock: an STL vector of Boid pointers
    typedef vector<Boid*> groupType;

    // per-boid reference to its flock
    const groupType& flock;

    // constructor: takes flock as argument
    Boid (const groupType& FLOCK) : flock (FLOCK) {reset ();}

    // reset state
    void reset (void)
    {
        // reset the vehicle
        SimpleVehicle::reset ();

        // initial speed
        setSpeed (0);

        // steering force is clipped to this magnitude
        setMaxForce (27);

        // velocity is clipped to this magnitude
        setMaxSpeed (9);

        // randomize initial orientation
        regenerateOrthonormalBasisUF (RandomUnitVector ());

        // randomize initial position
        setPosition (RandomVectorInUnitRadiusSphere () * 20);

        // trail is 1 second long with 20 points (10 dashes) along its length
        // setTrailParameters (10, 200);
    }

    // draw this boid into the scene
    void draw (void)
    {
        drawBasic3dSphericalVehicle (*this, gGray70);
        // drawTrail ();
    }

    // per frame simulation update
    void update (const float currentTime,
                 const float elapsedTime)
    {
        // combined steering force: boids are always flocking, plus
        // when near sphere boundary, gently seek toward the center
        const Vec3 combined = (steerToFlock () +
                               ((position().length() > 50) ?
                                steerForSeek (Vec3::zero) * 0.1 :
                                Vec3::zero));

        // apply force
        applySteeringForce (combined, elapsedTime);
    }

    // basic flocking
    Vec3 steerToFlock (void)
    {
        const float separationRadius = 9.5;
        const float separationAngle = -0.707;
        const float separationWeight = 9.2;

        const float alignmentRadius = 7.5;
        const float alignmentAngle = 0.2;
        const float alignmentWeight = 7.0;

        const float cohesionRadius = 13.00;
        const float cohesionAngle = -0.15;
        const float cohesionWeight = 10.0;

        const AVGroup& avflock = (AVGroup&)flock;

        const Vec3 separation = steerForSeparation (separationRadius,
                                                    separationAngle,
                                                    avflock);
        const Vec3 alignment = steerForAlignment (alignmentRadius,
                                                  alignmentAngle,
                                                  avflock);
        const Vec3 cohesion = steerForCohesion (cohesionRadius,
                                                cohesionAngle,
                                                avflock);

        const Vec3 separationW = separation * separationWeight;
        const Vec3 alignmentW = alignment * alignmentWeight;
        const Vec3 cohesionW = cohesion * cohesionWeight;

        // annotation
        // const float s = 0.1;
        // annotationLine (position, position + (separationW * s), gRed);
        // annotationLine (position, position + (alignmentW  * s), gOrange);
        // annotationLine (position, position + (cohesionW   * s), gYellow);

        return separationW + alignmentW + cohesionW;
    }
};


// ----------------------------------------------------------------------------
// PlugIn for SteerTest


class BoidsPlugIn : public PlugIn
{
public:
    
    const char* name (void) {return "Boids";}

    float selectionOrderSortKey (void) {return 0.03;}

    virtual ~BoidsPlugIn() {} // be more "nice" to avoid a compiler warning

    void open (void)
    {
        makeFlock (flockSize);

        // initialize camera
        Boid& firstBoid = **flock.begin();
        SteerTest::init3dCamera (firstBoid);
        SteerTest::camera.mode = Camera::cmFixed;
        SteerTest::camera.fixedDistDistance = SteerTest::cameraTargetDistance;
        SteerTest::camera.fixedDistVOffset = 0;
        SteerTest::camera.lookdownDistance = 20;
        SteerTest::camera.aimLeadTime = 0.5;
        SteerTest::camera.povOffset.set (0, 0.5, -2);
    }

    void update (const float currentTime, const float elapsedTime)
    {
        // update flock simulation for each boid
        for (iterator i = flock.begin(); i != flock.end(); i++)
        {
            (**i).update (currentTime, elapsedTime);
        }
    }

    void redraw (const float currentTime, const float elapsedTime)
    {
        // selected vehicle (user can mouse click to select another)
        AbstractVehicle& selected = *SteerTest::selectedVehicle;

        // vehicle nearest mouse (to be highlighted)
        AbstractVehicle& nearMouse = *SteerTest::vehicleNearestToMouse ();

        // update camera
        SteerTest::updateCamera (currentTime, elapsedTime, selected);

        // draw each boid in flock
        for (iterator i = flock.begin(); i != flock.end(); i++) (**i).draw ();

        // highlight vehicle nearest mouse
        SteerTest::drawCircleHighlightOnVehicle (nearMouse, 1, gGray70);

        // highlight selected vehicle
        SteerTest::drawCircleHighlightOnVehicle (selected, 1, gGray50);
    }

    void close (void)
    {
        // delete each member of the flock
        for (iterator i = flock.begin(); i != flock.end(); i++) delete (*i);
        flock.clear ();
    }

    void reset (void)
    {
        // reinitialize all boids in the flock
        initializeFlock ();

        // reset camera position
        SteerTest::position3dCamera (*SteerTest::selectedVehicle);

        // make camera jump immediately to new position
        SteerTest::camera.doNotSmoothNextMove ();
    }

    const AVGroup& allVehicles (void) {return (const AVGroup&)flock;}

    // create new flock of the given size
    void makeFlock (const int population)
    {
        for (int i = 0; i < population; i++)
            flock.push_back (new Boid (flock));
        initializeFlock ();
    }

    // initialize the flock's positions and orientations
    void initializeFlock (void)
    {
        for (iterator i = flock.begin(); i != flock.end(); i++) (**i).reset();
    }

    // flock: a group (STL vector) of pointers to all boids
    Boid::groupType flock;
    const static int flockSize = 200;
    typedef Boid::groupType::const_iterator iterator;
};


BoidsPlugIn gBoidsPlugIn;


// ----------------------------------------------------------------------------
