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
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Proximity.h"


// Include names declared in the OpenSteer namespace into the
// namespaces to search to find names.
using namespace OpenSteer;


// ----------------------------------------------------------------------------


typedef OpenSteer::AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
typedef OpenSteer::AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;


// ----------------------------------------------------------------------------


class Boid : public OpenSteer::SimpleVehicle
{
public:

    // type for a flock: an STL vector of Boid pointers
    typedef std::vector<Boid*> groupType;


    // enumerate demos of various constraints on the flock
    enum ConstraintType {none, insideSphere, outsideSphere, outsideSpheres,
                         rectangle, outsideBox, insideBox};


    // constructor
    Boid (ProximityDatabase& pd)
    {
        // allocate a token for this boid in the proximity database
        proximityToken = NULL;
        newPD (pd);

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
        setSpeed (maxSpeed() * 0.3f);

        // randomize initial orientation
        regenerateOrthonormalBasisUF (RandomUnitVector ());

        // randomize initial position
        setPosition (RandomVectorInUnitRadiusSphere () * 20);

        // notify proximity database that our position has changed
        proximityToken->updateForNewPosition (position());
    }


    // draw this boid into the scene
    void draw (void)
    {
        drawBasic3dSphericalVehicle (*this, gGray70);
        // drawTrail ();
    }


    // per frame simulation update
    void update (const float currentTime, const float elapsedTime)
    {
        // steer to flock and perhaps to stay within the spherical boundary
        applySteeringForce (steerToFlock () + handleBoundary(), elapsedTime);

        // notify proximity database that our position has changed
        proximityToken->updateForNewPosition (position());
    }


    // basic flocking
    Vec3 steerToFlock (void)
    {
        // avoid obstacles if needed
        // XXX this should probably be moved elsewhere
        const Vec3 avoidance = steerToAvoidObstacles (1.0f, obstacles);
        if (avoidance != Vec3::zero) return avoidance;

        const float separationRadius =  5.0f;
        const float separationAngle  = -0.707f;
        const float separationWeight =  12.0f;

        const float alignmentRadius = 7.5f;
        const float alignmentAngle  = 0.7f;
        const float alignmentWeight = 8.0f;

        const float cohesionRadius = 9.0f;
        const float cohesionAngle  = -0.15f;
        const float cohesionWeight = 8.0f;

        const float maxRadius = maxXXX (separationRadius,
                                        maxXXX (alignmentRadius,
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


    // Take action to stay within sphereical boundary.  Returns steering
    // value (which is normally zero) and may take other side-effecting
    // actions such as kinematically changing the Boid's position.
    //
    // XXX not much here since I removed "steer back when outside" 11-11-04
    //
    Vec3 handleBoundary (void)
    {
        // while inside the sphere do nothing
        if (position().length() < worldRadius) return Vec3::zero;

        // once outside, select strategy
        // wrap around (teleport)
        setPosition (position().sphericalWrapAround (Vec3::zero, worldRadius));
        return Vec3::zero;
    }


    // make boids "bank" as they fly
    void regenerateLocalSpace (const Vec3& newVelocity,
                               const float elapsedTime)
    {
        regenerateLocalSpaceForBanking (newVelocity, elapsedTime);
    }

    // switch to new proximity database -- just for demo purposes
    void newPD (ProximityDatabase& pd)
    {
        // delete this boid's token in the old proximity database
        delete proximityToken;

        // allocate a token for this boid in the proximity database
        proximityToken = pd.allocateToken (this);
    }


    // cycle through various boundary conditions
    // XXX probably need to be re-thought
    static void nextBoundaryCondition (void)
    {
        debugPrint ((int)constraint);
        constraint = (ConstraintType) ((int) constraint + 1);
        switch (constraint)
        {
        default:
            constraint = none; // wrap-around, falls through to first case:
        case none:
            break;
        case insideSphere:
            break;
        case outsideSphere:
            break;
        case outsideSpheres:
            break;
        case rectangle:
            break;
        case outsideBox:
            break;
        case insideBox:
            break;
        }
    }
    static ConstraintType constraint;
    static ObstacleGroup obstacles;


    // a pointer to this boid's interface object for the proximity database
    ProximityToken* proximityToken;

    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    static AVGroup neighbors;

    static float worldRadius;

    // xxx perhaps this should be a call to a general purpose annotation for
    // xxx "local xxx axis aligned box in XZ plane" -- same code in in
    // xxx CaptureTheFlag.cpp
    void annotateAvoidObstacle (const float minDistanceToCollision)
    {
        const Vec3 boxSide = side() * radius();
        const Vec3 boxFront = forward() * minDistanceToCollision;
        const Vec3 FR = position() + boxFront - boxSide;
        const Vec3 FL = position() + boxFront + boxSide;
        const Vec3 BR = position()            - boxSide;
        const Vec3 BL = position()            + boxSide;
        const Vec3 white (1,1,1);
        annotationLine (FR, FL, white);
        annotationLine (FL, BL, white);
        annotationLine (BL, BR, white);
        annotationLine (BR, FR, white);
    }
};


AVGroup Boid::neighbors;
float Boid::worldRadius = 50.0f;
ObstacleGroup Boid::obstacles;
Boid::ConstraintType Boid::constraint;


// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo


class BoidsPlugIn : public PlugIn
{
public:
    
    const char* name (void) {return "Boids";}

    float selectionOrderSortKey (void) {return 0.03f;}

    virtual ~BoidsPlugIn() {} // be more "nice" to avoid a compiler warning

    void open (void)
    {
        // make the database used to accelerate proximity queries
        cyclePD = -1;
        nextPD ();

        // make default-sized flock
        population = 0;
        for (int i = 0; i < 200; i++) addBoidToFlock ();

        // initialize camera
        OpenSteerDemo::init3dCamera (*OpenSteerDemo::selectedVehicle);
        OpenSteerDemo::camera.mode = Camera::cmFixed;
        OpenSteerDemo::camera.fixedDistDistance = OpenSteerDemo::cameraTargetDistance;
        OpenSteerDemo::camera.fixedDistVOffset = 0;
        OpenSteerDemo::camera.lookdownDistance = 20;
        OpenSteerDemo::camera.aimLeadTime = 0.5;
        OpenSteerDemo::camera.povOffset.set (0, 0.5, -2);

        // set up obstacles
        initObstacles ();
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
        AbstractVehicle& selected = *OpenSteerDemo::selectedVehicle;

        // vehicle nearest mouse (to be highlighted)
        AbstractVehicle& nearMouse = *OpenSteerDemo::vehicleNearestToMouse ();

        // update camera
        OpenSteerDemo::updateCamera (currentTime, elapsedTime, selected);

        // draw each boid in flock
        for (iterator i = flock.begin(); i != flock.end(); i++) (**i).draw ();

        // highlight vehicle nearest mouse
        OpenSteerDemo::drawCircleHighlightOnVehicle (nearMouse, 1, gGray70);

        // highlight selected vehicle
        OpenSteerDemo::drawCircleHighlightOnVehicle (selected, 1, gGray50);

        // display status in the upper left corner of the window
        std::ostringstream status;
        status << "[F1/F2] " << population << " boids";
        status << "\n[F3]    PD type: ";
        switch (cyclePD)
        {
        case 0: status << "LQ bin lattice"; break;
        case 1: status << "brute force";    break;
        }
        status << "\n[F4]    Obstacles: ";
        switch (Boid::constraint)
        {
        case Boid::none:
            status << "none (wrap-around at sphere boundary)" ; break;
        case Boid::insideSphere:
            status << "inside a sphere" ; break;
        case Boid::outsideSphere:
            status << "inside a sphere, outside another" ; break;
        case Boid::outsideSpheres:
            status << "inside a sphere, outside several" ; break;
        case Boid::rectangle:
            status << "inside a sphere, with a rectangle" ; break;
        case Boid::outsideBox:
            status << "inside a sphere, outside a box" ; break;
        case Boid::insideBox:
            status << "inside a box" ; break;
        }
        status << std::endl;
        const float h = drawGetWindowHeight ();
        const Vec3 screenLocation (10, h-50, 0);
        draw2dTextAt2dLocation (status, screenLocation, gGray80);

        drawObstacles ();
    }

    void close (void)
    {
        // delete each member of the flock
        while (population > 0) removeBoidFromFlock ();

        // delete the proximity database
        delete pd;
        pd = NULL;
    }

    void reset (void)
    {
        // reset each boid in flock
        for (iterator i = flock.begin(); i != flock.end(); i++) (**i).reset();

        // reset camera position
        OpenSteerDemo::position3dCamera (*OpenSteerDemo::selectedVehicle);

        // make camera jump immediately to new position
        OpenSteerDemo::camera.doNotSmoothNextMove ();
    }

    // for purposes of demonstration, allow cycling through various
    // types of proximity databases.  this routine is called when the
    // OpenSteerDemo user pushes a function key.
    void nextPD (void)
    {
        // save pointer to old PD
        ProximityDatabase* oldPD = pd;

        // allocate new PD
        const int totalPD = 2;
        switch (cyclePD = (cyclePD + 1) % totalPD)
        {
        case 0:
            {
                const Vec3 center;
                const float div = 10.0f;
                const Vec3 divisions (div, div, div);
                const float diameter = Boid::worldRadius * 1.1f * 2;
                const Vec3 dimensions (diameter, diameter, diameter);
                typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
                pd = new LQPDAV (center, dimensions, divisions);
                break;
            }
        case 1:
            {
                pd = new BruteForceProximityDatabase<AbstractVehicle*> ();
                break;
            }
        }

        // switch each boid to new PD
        for (iterator i=flock.begin(); i!=flock.end(); i++) (**i).newPD(*pd);

        // delete old PD (if any)
        delete oldPD;
    }

    void handleFunctionKeys (int keyNumber)
    {
        switch (keyNumber)
        {
        case 1:  addBoidToFlock ();               break;
        case 2:  removeBoidFromFlock ();          break;
        case 3:  nextPD ();                       break;
        case 4:  Boid::nextBoundaryCondition ();  break;
        }
        updateObstacles ();
    }

    void printMiniHelpForFunctionKeys (void)
    {
        std::ostringstream message;
        message << "Function keys handled by ";
        message << '"' << name() << '"' << ':' << std::ends;
        OpenSteerDemo::printMessage (message);
        OpenSteerDemo::printMessage ("  F1     add a boid to the flock.");
        OpenSteerDemo::printMessage ("  F2     remove a boid from the flock.");
        OpenSteerDemo::printMessage ("  F3     use next proximity database.");
        OpenSteerDemo::printMessage ("  F4     next flock boundary condition.");
        OpenSteerDemo::printMessage ("");
    }

    void addBoidToFlock (void)
    {
        population++;
        Boid* boid = new Boid (*pd);
        flock.push_back (boid);
        if (population == 1) OpenSteerDemo::selectedVehicle = boid;
    }

    void removeBoidFromFlock (void)
    {
        if (population > 0)
        {
            // save a pointer to the last boid, then remove it from the flock
            const Boid* boid = flock.back();
            flock.pop_back();
            population--;

            // if it is OpenSteerDemo's selected vehicle, unselect it
            if (boid == OpenSteerDemo::selectedVehicle)
                OpenSteerDemo::selectedVehicle = NULL;

            // delete the Boid
            delete boid;
        }
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

    // which of the various proximity databases is currently in use
    int cyclePD;


    SphereObstacle insideBigSphere;
    SphereObstacle outsideSphere0;
    SphereObstacle outsideSphere1;
    SphereObstacle outsideSphere2;
    SphereObstacle outsideSphere3;
    SphereObstacle outsideSphere4;
    SphereObstacle outsideSphere5;
    SphereObstacle outsideSphere6;
    RectangleObstacle rectangle;
    BoxObstacle outsideBigBox, insideBigBox;

    void initObstacles (void)
    {
        Boid::constraint = Boid::none;

        insideBigSphere.radius = Boid::worldRadius;
        insideBigSphere.setSeenFrom (Obstacle::inside);

        outsideSphere0.radius = Boid::worldRadius * 0.5f;

        const float r = Boid::worldRadius * 0.33f;
        outsideSphere1.radius = r;
        outsideSphere2.radius = r;
        outsideSphere3.radius = r;
        outsideSphere4.radius = r;
        outsideSphere5.radius = r;
        outsideSphere6.radius = r;

        const float p = Boid::worldRadius * 0.5f;
        const float m = -p;
        const float z = 0.0f;
        outsideSphere1.center.set (p, z, z);
        outsideSphere2.center.set (m, z, z);
        outsideSphere3.center.set (z, p, z);
        outsideSphere4.center.set (z, m, z);
        outsideSphere5.center.set (z, z, p);
        outsideSphere6.center.set (z, z, m);

        const Vec3 tiltF = Vec3 (1.0f, 1.0f, 0.0f).normalize ();
        const Vec3 tiltS (0.0f, 0.0f, 1.0f);
        const Vec3 tiltU = Vec3 (-1.0f, 1.0f, 0.0f).normalize ();

        rectangle.width = 50.0f;
        rectangle.height = 80.0f;
        rectangle.setSeenFrom (Obstacle::both);
        rectangle.setForward (tiltF);
        rectangle.setSide (tiltS);
        rectangle.setUp (tiltU);

        outsideBigBox.width = 50.0f;
        outsideBigBox.height = 80.0f;
        outsideBigBox.depth = 20.0f;
        outsideBigBox.setForward (tiltF);
        outsideBigBox.setSide (tiltS);
        outsideBigBox.setUp (tiltU);

        insideBigBox = outsideBigBox;
        insideBigBox.setSeenFrom (Obstacle::inside);

        updateObstacles ();
    }

    // update Boid::obstacles list when Boid::constraint changes
    void updateObstacles (void)
    {
        // first clear out obstacle list, and add back big sphere
        Boid::obstacles.clear ();
        Boid::obstacles.push_back (&insideBigSphere);

        // add back obstacles based on mode
        switch (Boid::constraint)
        {
        case Boid::none:
            Boid::obstacles.clear (); // this is only one with no big sphere
            break;
        case Boid::insideSphere:
            break;
        case Boid::outsideSphere:
            Boid::obstacles.push_back (&outsideSphere0);
            break;
        case Boid::outsideSpheres:
            Boid::obstacles.push_back (&outsideSphere1);
            Boid::obstacles.push_back (&outsideSphere2);
            Boid::obstacles.push_back (&outsideSphere3);
            Boid::obstacles.push_back (&outsideSphere4);
            Boid::obstacles.push_back (&outsideSphere5);
            Boid::obstacles.push_back (&outsideSphere6);
            break;
        case Boid::rectangle:
            Boid::obstacles.push_back (&rectangle);
            break;
        case Boid::outsideBox:
            Boid::obstacles.push_back (&outsideBigBox);
            break;
        case Boid::insideBox:
            Boid::obstacles.push_back (&insideBigBox);
            break;
        }
    }


    void drawObstacles (void)
    {
        Vec3 light (0.2f, 0.2f, 0.4f);
        Vec3 dark (0.1f, 0.1f, 0.2f);
        switch (Boid::constraint)
        {
        case Boid::none:
            break;
        case Boid::insideSphere:
            tempDrawSphere (insideBigSphere, light);
            break;
        case Boid::outsideSphere:
            tempDrawSphere (insideBigSphere, light);
            tempDrawSphere (outsideSphere0, dark);
            break;
        case Boid::outsideSpheres:
            tempDrawSphere (insideBigSphere, light);
            tempDrawSphere (outsideSphere1, dark);
            tempDrawSphere (outsideSphere2, dark);
            tempDrawSphere (outsideSphere3, dark);
            tempDrawSphere (outsideSphere4, dark);
            tempDrawSphere (outsideSphere5, dark);
            tempDrawSphere (outsideSphere6, dark);
            break;
        case Boid::rectangle:
            tempDrawSphere (insideBigSphere, light);
            {
                float w = rectangle.width / 2;
                float h = rectangle.height / 2;
                Vec3 v1 = rectangle.globalizePosition (Vec3 (w, h, 0));
                Vec3 v2 = rectangle.globalizePosition (Vec3 (-w, h, 0));
                Vec3 v3 = rectangle.globalizePosition (Vec3 (-w, -h, 0));
                Vec3 v4 = rectangle.globalizePosition (Vec3 (w, -h, 0));
                drawLine (v1, v2, dark);
                drawLine (v2, v3, dark);
                drawLine (v3, v4, dark);
                drawLine (v4, v1, dark);
            }
            break;
        case Boid::outsideBox:
            tempDrawSphere (insideBigSphere, light);
            tempDrawBox (outsideBigBox, dark);
            break;
        case Boid::insideBox:
            tempDrawSphere (insideBigSphere, light);
            tempDrawBox (insideBigBox, dark);
            break;
        }
    }

    // XXX temporary DrawSphere utility - clean up and move to Draw.cpp
    // XXX 
    // XXX this utility should provide a "filled-p" switch to slect
    // XXX between wire frame and filled triangles.  It should also
    // XXX support optional backface culling, drawing only triangles
    // XXX oriented towards (or away from) a given POV.
    //
    void tempDrawSphere (const SphereObstacle& sphere,
                         const Vec3& color)
    {
        drawIcosahedralGeodesicSphere (sphere.radius, sphere.center, color);
    }


    // XXX move to Vec3 or utilities
    inline Vec3 projectOnSphere (const Vec3& v,
                                 const float radius,
                                 const Vec3& center)
    {
        const Vec3 unitRadial = (v - center).normalize ();
        return center + (unitRadial * radius);
    }

    // XXX move to Vec3 or utilities
    inline Vec3 midpointOnSphere (const Vec3& a, 
                                  const Vec3& b,
                                  const float radius,
                                  const Vec3& center)
    {
        return projectOnSphere ((a + b) * 0.5f, radius, center);
    }


    // XXX clean up and move to Draw.cpp
    // XXX min edge length should be a parameter
    void drawOctahedralGeodesicSphere (const float radius,
                                       const Vec3& center,
                                       const Vec3& color)
    {
        const Vec3 x = center + (radius * Vec3 (1.0f, 0.0f, 0.0f));
        const Vec3 y = center + (radius * Vec3 (0.0f, 1.0f, 0.0f));
        const Vec3 z = center + (radius * Vec3 (0.0f, 0.0f, 1.0f));

        // start from faces of an octahedron
        drawMeshedTriangleOnSphere ( x,  y,  z, radius, center, 10.0f, color);
        drawMeshedTriangleOnSphere (-x,  y,  z, radius, center, 10.0f, color);
        drawMeshedTriangleOnSphere ( x, -y,  z, radius, center, 10.0f, color);
        drawMeshedTriangleOnSphere (-x, -y,  z, radius, center, 10.0f, color);
        drawMeshedTriangleOnSphere ( x,  y, -z, radius, center, 10.0f, color);
        drawMeshedTriangleOnSphere (-x,  y, -z, radius, center, 10.0f, color);
        drawMeshedTriangleOnSphere ( x, -y, -z, radius, center, 10.0f, color);
        drawMeshedTriangleOnSphere (-x, -y, -z, radius, center, 10.0f, color);
    }


    // XXX clean up and move to Draw.cpp
    // XXX min edge length should be a parameter
    void drawIcosahedralGeodesicSphere (const float radius,
                                        const Vec3& center,
                                        const Vec3& color)
    {
        // Geometry based on Paul Bourke's excellent article:
        //   Platonic Solids (Regular polytopes in 3D)
        //   http://astronomy.swin.edu.au/~pbourke/polyhedra/platonic/

        // define the icosahedron's 12 vertices:
        // XXX better to pre-scale a and b, but just for the moment try:
        const float phi = (1.0f + sqrtXXX(5.0f)) * 0.5f; // "golden ratio"
        const float a = 0.5;
        const float b = 1.0f / (2.0f * phi);
        const Vec3 v1 = projectOnSphere(center+Vec3(0, b, -a), radius, center);
        const Vec3 v2 = projectOnSphere(center+Vec3(b, a, 0), radius, center);
        const Vec3 v3 = projectOnSphere(center+Vec3(-b, a, 0), radius, center);
        const Vec3 v4 = projectOnSphere(center+Vec3(0, b, a), radius, center);
        const Vec3 v5 = projectOnSphere(center+Vec3(0, -b, a), radius, center);
        const Vec3 v6 = projectOnSphere(center+Vec3(-a, 0, b), radius, center);
        const Vec3 v7 = projectOnSphere(center+Vec3(0, -b, -a), radius, center);
        const Vec3 v8 = projectOnSphere(center+Vec3(a, 0, -b), radius, center);
        const Vec3 v9 = projectOnSphere(center+Vec3(a, 0, b), radius, center);
        const Vec3 v10 = projectOnSphere(center+Vec3(-a, 0, -b), radius, center);
        const Vec3 v11 = projectOnSphere(center+Vec3(b, -a, 0), radius, center);
        const Vec3 v12 = projectOnSphere(center+Vec3(-b, -a, 0), radius, center);

        // draw the icosahedron's 20 triangular faces:
        const float min = 10.0f;
        drawMeshedTriangleOnSphere (v1, v2, v3,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v4, v3, v2,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v4, v5, v6,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v4, v9, v5,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v1, v7, v8,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v1, v10, v7,  radius, center, min, color);
        drawMeshedTriangleOnSphere (v5, v11, v12, radius, center, min, color);
        drawMeshedTriangleOnSphere (v7, v12, v11, radius, center, min, color);
        drawMeshedTriangleOnSphere (v3, v6, v10,  radius, center, min, color);
        drawMeshedTriangleOnSphere (v12, v10, v6, radius, center, min, color);
        drawMeshedTriangleOnSphere (v2, v8, v9,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v11, v9, v8,  radius, center, min, color);
        drawMeshedTriangleOnSphere (v4, v6, v3,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v4, v2, v9,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v1, v3, v10,  radius, center, min, color);
        drawMeshedTriangleOnSphere (v1, v8, v2,   radius, center, min, color);
        drawMeshedTriangleOnSphere (v7, v10, v12, radius, center, min, color);
        drawMeshedTriangleOnSphere (v7, v11, v8,  radius, center, min, color);
        drawMeshedTriangleOnSphere (v5, v12, v6,  radius, center, min, color);
        drawMeshedTriangleOnSphere (v5, v9, v11,  radius, center, min, color);
     }


    // Draws line from A to B but not from B to A: assumes each edge
    // will be drawn in both directions, picks just one direction for
    // drawing according to an arbitary but reproducable criterion.
    void drawMeshedTriangleLine (const Vec3& a, 
                                 const Vec3& b,
                                 const Vec3& color)
    {
        if (a.x != b.x)
        {
            if (a.x > b.x) drawLine (a, b, color);
        }
        else
        {
            if (a.y != b.y)
            {
                if (a.y > b.y) drawLine (a, b, color); 
            }
            else
            {
                if (a.z > b.z) drawLine (a, b, color); 
            }
        }
    }

    void drawMeshedTriangleOnSphere (const Vec3& a, 
                                     const Vec3& b,
                                     const Vec3& c,
                                     const float radius,
                                     const Vec3& center,
                                     const float max, // max Edge Length
                                     const Vec3& color)
    {
        // measure edge lengths
        const float abLength = (a - b).length ();
        const float bcLength = (b - c).length ();
        const float caLength = (c - a).length ();

        // below length threshold?
        if ((abLength < max) && (bcLength < max) && (caLength < max))
        {
            // if all are short, draw edges
            // (note this will generally draw each edge twice)
            drawMeshedTriangleLine (a, b, color);
            drawMeshedTriangleLine (b, c, color);
            drawMeshedTriangleLine (c, a, color);
        }
        else // otherwise, subdivide and recurse
        {
            // find midpoints
            const Vec3 ab = midpointOnSphere (a, b, radius, center);
            const Vec3 bc = midpointOnSphere (b, c, radius, center);
            const Vec3 ca = midpointOnSphere (c, a, radius, center);

            // recurse
            drawMeshedTriangleOnSphere ( a, ab, ca, radius, center, max, color);
            drawMeshedTriangleOnSphere (ab,  b, bc, radius, center, max, color);
            drawMeshedTriangleOnSphere (ca, bc,  c, radius, center, max, color);
            drawMeshedTriangleOnSphere (ab, bc, ca, radius, center, max, color);
        }
    }

    void tempDrawBox (const BoxObstacle& box, const Vec3 color)
    {
        const float w = box.width / 2;
        const float h = box.height / 2;
        const float d = box.depth / 2;
        const Vec3 p = box.position ();
        const Vec3 s = box.side ();
        const Vec3 u = box.up ();
        const Vec3 f = box.forward ();

        const Vec3 v1 = box.globalizePosition (Vec3 ( w,  h,  d));
        const Vec3 v2 = box.globalizePosition (Vec3 (-w,  h,  d));
        const Vec3 v3 = box.globalizePosition (Vec3 (-w, -h,  d));
        const Vec3 v4 = box.globalizePosition (Vec3 ( w, -h,  d));

        const Vec3 v5 = box.globalizePosition (Vec3 ( w,  h, -d));
        const Vec3 v6 = box.globalizePosition (Vec3 (-w,  h, -d));
        const Vec3 v7 = box.globalizePosition (Vec3 (-w, -h, -d));
        const Vec3 v8 = box.globalizePosition (Vec3 ( w, -h, -d));

        drawLine (v1, v2, color);
        drawLine (v2, v3, color);
        drawLine (v3, v4, color);
        drawLine (v4, v1, color);

        drawLine (v5, v6, color);
        drawLine (v6, v7, color);
        drawLine (v7, v8, color);
        drawLine (v8, v5, color);

        drawLine (v1, v5, color);
        drawLine (v2, v6, color);
        drawLine (v3, v7, color);
        drawLine (v4, v8, color);
    }
};


BoidsPlugIn gBoidsPlugIn;



// ----------------------------------------------------------------------------
