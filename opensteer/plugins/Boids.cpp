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
    typedef std::vector<Boid*> groupType;

    // per-boid reference to its flock
    const groupType& flock;

    // constructor: argument is a reference to a flock container which is 
    // still being filled with the newly constructed member of the flock
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
        const Vec3 flocking = steerToFlock ();
        if (position().length() < 50)
        {
            applySteeringForce (flocking, elapsedTime);
        }
        else
        {
            const Vec3 seek = xxxsteerForSeek (Vec3::zero);
            const Vec3 lateral = seek.perpendicularComponent (forward ());
            applySteeringForce (flocking + lateral, elapsedTime);
        }
    }

    // basic flocking
    Vec3 steerToFlock (void)
    {
        const float separationRadius =  5.0f;
        const float separationAngle  = -0.707f;
        const float separationWeight =  12.0f;

        const float alignmentRadius = 7.5f;
        const float alignmentAngle  = 0.7f;
        const float alignmentWeight = 8.0f;

        const float cohesionRadius = 9.0f;
        const float cohesionAngle  = -0.15f;
        const float cohesionWeight = 8.0f;

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

    // make boids "bank" as they fly
    void regenerateLocalSpace (const Vec3& newVelocity,
                               const float elapsedTime)
    {
        regenerateLocalSpaceForBanking (newVelocity, elapsedTime);
    }
};


// ----------------------------------------------------------------------------
// PlugIn for SteerTest


class BoidsPlugIn : public PlugIn
{
public:
// -------------------------------------------------- xxxBasicProximity
//     bool requestInitialSelection (void) {return true;}
// -------------------------------------------------- xxxBasicProximity
    
    const char* name (void) {return "Boids";}

    float selectionOrderSortKey (void) {return 0.03f;}

    virtual ~BoidsPlugIn() {} // be more "nice" to avoid a compiler warning

    void open (void)
    {
        makeFlock (200);

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
    typedef Boid::groupType::const_iterator iterator;
};


BoidsPlugIn gBoidsPlugIn;


// ----------------------------------------------------------------------------
