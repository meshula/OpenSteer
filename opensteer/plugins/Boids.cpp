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


#include <sstream>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/SteerTest.h"
#include "OpenSteer/Proximity.h"


// ----------------------------------------------------------------------------


typedef AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
typedef AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;


// ----------------------------------------------------------------------------


class Boid : public SimpleVehicle
{
public:

    // type for a flock: an STL vector of Boid pointers
    typedef std::vector<Boid*> groupType;

    // per-boid reference to its flock
    const groupType& flock;

    // a pointer to this boid's interface object for the proximity database
    ProximityToken* proximityToken;

    // constructor: argument is a reference to a flock container which is
    // still being filled with the newly constructed member of the flock (that
    // container is now redundant since the ProximityDatabase is holding the
    // relvant information, the flock argument needs to be removed.)
    Boid (const groupType& FLOCK, ProximityDatabase& pd) : flock (FLOCK)
    {
        // allocate a token for this boid in the proximity database
        proximityToken = pd.allocateToken (this);

        // reset all boid state
        reset ();
    }

    // destructor
    ~Boid ()
    {
        // delete this boid's token in the proximity database
        delete proximityToken;
    }

    // reset state
    void reset (void)
    {
        // reset the vehicle
        SimpleVehicle::reset ();

        // steering force is clipped to this magnitude
        setMaxForce (27);

        // velocity is clipped to this magnitude
        setMaxSpeed (9);

        // initial slow speed
        setSpeed (maxSpeed() * 0.3);

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

        // notify proximity database that our position has changed
        proximityToken->updateForNewPosition (position());
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

        const float maxRadius = std::max (separationRadius,
                                          std::max (alignmentRadius,
                                                    cohesionRadius));

        // find all flockmates within maxRadius using proximity database
        neighbors.clear();
        proximityToken->findNeighbors (position(), maxRadius, neighbors);

        // determine each of the three component behaviors of flocking
        const Vec3 separation = steerForSeparation (separationRadius,
                                                    separationAngle,
                                                    neighbors);
        const Vec3 alignment  = steerForAlignment  (alignmentRadius,
                                                    alignmentAngle,
                                                    neighbors);
        const Vec3 cohesion   = steerForCohesion   (cohesionRadius,
                                                    cohesionAngle,
                                                    neighbors);

        // apply weights to components (save in variables for annotation)
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

    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more mp-safe)
    static AVGroup neighbors;
};


AVGroup Boid::neighbors;


// ----------------------------------------------------------------------------
// PlugIn for SteerTest


class BoidsPlugIn : public PlugIn
{
public:
    
    const char* name (void) {return "Boids";}

    float selectionOrderSortKey (void) {return 0.03f;}

    virtual ~BoidsPlugIn() {} // be more "nice" to avoid a compiler warning

    void open (void)
    {
        // make the database used to accelerate proximity queries
        // XXX the sphere boundary diameter is 100 and they frequently
        // XXX go outside that.  Using 110, but need to link constants
        // XXX here and in Boid::update
        pd = new LQProximityDatabase<AbstractVehicle*> (Vec3::zero,
                                                        Vec3 (110, 110, 110),
                                                        Vec3 (15, 15, 15));
        // make default-sized flock
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

        // display status in the upper left corner of the window
        std::ostringstream status;
        status << "Flock size: " << population << std::ends;
        const float h = drawGetWindowHeight ();
        const Vec3 screenLocation (10, h-50, 0);
        draw2dTextAt2dLocation (status, screenLocation, gGray80);
        // std::ostringstream status2;
        // status2  << "PD content count: " << pd->getPopulation ()<<std::endl;
        // const Vec3 screenLocation2 (10, h-70, 0);
        // draw2dTextAt2dLocation (status2, screenLocation2, gGray80);
    }

    void close (void)
    {
        // delete each member of the flock
        for (iterator i = flock.begin(); i != flock.end(); i++) delete (*i);
        flock.clear ();

        // delete the proximity database
        delete pd;
        pd = NULL;
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

    void handleFunctionKeys (int keyNumber)
    {
        switch (keyNumber)
        {
        case 1:
            addBoidToFlock ();
            break;
        case 2:
            if (population > 0) removeBoidFromFlock ();
            break;
        }
    }

    void printMiniHelpForFunctionKeys (void)
    {
        std::ostringstream message;
        message << "Function keys handled by ";
        message << '"' << name() << '"' << ':' << std::ends;
        SteerTest::printMessage (message);
        SteerTest::printMessage ("  F1     add a boid to the flock.");
        SteerTest::printMessage ("  F2     remove a boid from the flock.");
        SteerTest::printMessage ("");
    }

    void addBoidToFlock (void)
    {
        population++;
        Boid* boid = new Boid (flock, *pd);
        flock.push_back (boid);
        if (population == 1) SteerTest::selectedVehicle = boid;
    }

    void removeBoidFromFlock (void)
    {
        // save a pointer to the last boid, then remove it from the flock
        const Boid* boid = flock.back();
        flock.pop_back();
        population--;

        // if it is SteerTest's selected vehicle, unselect it
        if (boid == SteerTest::selectedVehicle)
            SteerTest::selectedVehicle = NULL;

        // delete the Boid
        delete boid;
    }

    // create new flock of the given size
    void makeFlock (const int size)
    {
        for (int i = 0; i < size; i++) addBoidToFlock ();
    }

    // initialize the flock's positions and orientations
    void initializeFlock (void)
    {
        for (iterator i = flock.begin(); i != flock.end(); i++) (**i).reset();
    }

    // return an AVGroup containing each boid of the flock
    const AVGroup& allVehicles (void) {return (const AVGroup&)flock;}

    // flock: a group (STL vector) of pointers to all boids
    Boid::groupType flock;
    typedef Boid::groupType::const_iterator iterator;

    // pointer to database used to accelerate proximity queries
    ProximityDatabase* pd;

    // keep track of current flock size
    int population;
};


BoidsPlugIn gBoidsPlugIn;


// ----------------------------------------------------------------------------
