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
        // steer to flock and avoid obstacles if any
        applySteeringForce (steerToFlock (), elapsedTime);

        // wrap around to contrain boid within the spherical boundary
        sphericalWrapAround ();

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


    // constrain this boid to stay within sphereical boundary.
    void sphericalWrapAround (void)
    {
        // when outside the sphere
        if (position().length() > worldRadius)
        {
            // wrap around (teleport)
            setPosition (position().sphericalWrapAround (Vec3::zero,
                                                         worldRadius));
            if (this == OpenSteerDemo::selectedVehicle)
            {
                OpenSteerDemo::position3dCamera
                    (*OpenSteerDemo::selectedVehicle); 
                OpenSteerDemo::camera.doNotSmoothNextMove ();
            }
        }
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


    // group of all obstacles to be avoided by each Boid
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
        switch (constraint)
        {
        case none:
            status << "none (wrap-around at sphere boundary)" ; break;
        case insideSphere:
            status << "inside a sphere" ; break;
        case outsideSphere:
            status << "inside a sphere, outside another" ; break;
        case outsideSpheres:
            status << "inside a sphere, outside several" ; break;
        case rectangle:
            status << "inside a sphere, with a rectangle" ; break;
        case outsideBox:
            status << "inside a sphere, outside a box" ; break;
        case insideBox:
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
        case 1:  addBoidToFlock ();         break;
        case 2:  removeBoidFromFlock ();    break;
        case 3:  nextPD ();                 break;
        case 4:  nextBoundaryCondition ();  break;
        }
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

    // --------------------------------------------------------
    // the rest of this plug-in supports the various obstacles:
    // --------------------------------------------------------

    // enumerate demos of various constraints on the flock
    enum ConstraintType {none, insideSphere, outsideSphere, outsideSpheres,
                         rectangle, outsideBox, insideBox};

    ConstraintType constraint;

    // select next "boundary condition / constraint / obstacle"
    void nextBoundaryCondition (void)
    {
        constraint = (ConstraintType) ((int) constraint + 1);
        updateObstacles ();
    }


    SphereObstacle insideBigSphere;
    SphereObstacle outsideSphere0;
    SphereObstacle outsideSphere1;
    SphereObstacle outsideSphere2;
    SphereObstacle outsideSphere3;
    SphereObstacle outsideSphere4;
    SphereObstacle outsideSphere5;
    SphereObstacle outsideSphere6;
    RectangleObstacle bigRectangle;
    BoxObstacle outsideBigBox, insideBigBox;


    void initObstacles (void)
    {
        constraint = none;

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

        bigRectangle.width = 50.0f;
        bigRectangle.height = 80.0f;
        bigRectangle.setSeenFrom (Obstacle::both);
        bigRectangle.setForward (tiltF);
        bigRectangle.setSide (tiltS);
        bigRectangle.setUp (tiltU);

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


    // update Boid::obstacles list when constraint changes
    void updateObstacles (void)
    {
        // first clear out obstacle list
        Boid::obstacles.clear ();

        // add back obstacles based on mode
        switch (constraint)
        {
        default:
            // reset for wrap-around, fall through to first case:
            constraint = none;
        case none:
            break;
        case insideSphere:
            Boid::obstacles.push_back (&insideBigSphere);
            break;
        case outsideSphere:
            Boid::obstacles.push_back (&insideBigSphere);
            Boid::obstacles.push_back (&outsideSphere0);
            break;
        case outsideSpheres:
            Boid::obstacles.push_back (&insideBigSphere);
            Boid::obstacles.push_back (&outsideSphere1);
            Boid::obstacles.push_back (&outsideSphere2);
            Boid::obstacles.push_back (&outsideSphere3);
            Boid::obstacles.push_back (&outsideSphere4);
            Boid::obstacles.push_back (&outsideSphere5);
            Boid::obstacles.push_back (&outsideSphere6);
            break;
        case rectangle:
            Boid::obstacles.push_back (&insideBigSphere);
            Boid::obstacles.push_back (&bigRectangle);
            break;
        case outsideBox:
            Boid::obstacles.push_back (&insideBigSphere);
            Boid::obstacles.push_back (&outsideBigBox);
            break;
        case insideBox:
            Boid::obstacles.push_back (&insideBigBox);
            break;
        }
    }


    void drawObstacles (void)
    {
        const float max = 10.0f;
        const Vec3 vp = OpenSteerDemo::camera.position ();
        const Vec3 light (0.2f, 0.2f, 0.4f);
        const Vec3 dark (0.1f, 0.1f, 0.2f);
        const bool fill = false; // draw in wireframe

        switch (constraint)
        {
        case none:
            break;
        case insideSphere:
            drawSphereObstacle (insideBigSphere, max, vp, fill, light);
            break;
        case outsideSphere:
            drawSphereObstacle (insideBigSphere, max, vp, fill, light);
            drawSphereObstacle (outsideSphere0,  max, vp, fill, dark);
            break;
        case outsideSpheres:
            drawSphereObstacle (insideBigSphere, max, vp, fill, light);
            drawSphereObstacle (outsideSphere1,  max, vp, fill, dark);
            drawSphereObstacle (outsideSphere2,  max, vp, fill, dark);
            drawSphereObstacle (outsideSphere3,  max, vp, fill, dark);
            drawSphereObstacle (outsideSphere4,  max, vp, fill, dark);
            drawSphereObstacle (outsideSphere5,  max, vp, fill, dark);
            drawSphereObstacle (outsideSphere6,  max, vp, fill, dark);
            break;
        case rectangle:
            drawSphereObstacle (insideBigSphere, max, vp, fill, light);
            tempDrawRectangle (bigRectangle, dark);
            break;
        case outsideBox:
            drawSphereObstacle (insideBigSphere, max, vp, fill, light);
            tempDrawBox (outsideBigBox, dark);
            break;
        case insideBox:
            tempDrawBox (insideBigBox, dark);
            break;
        }
    }


    // utility to draw a SphereObstacle (culling via seenFrom)
    void drawSphereObstacle (const SphereObstacle& so,
                             const float maxEdgeLength,
                             const Vec3& viewpoint,
                             const bool filled,
                             const Vec3& color)
    {
        const DrawSphereHelper s (so, maxEdgeLength, viewpoint, filled, color);
        s.draw ();
    }


    class DrawSphereHelper
    {
    public:
        Vec3 center;
        float radius;
        Vec3 color;
        float maxEdgeLength;
        bool filled;
        bool drawFrontFacing;
        bool drawBackFacing;
        Vec3 viewpoint;

        // default constructor (at origin, radius=1, white, wireframe, nocull)
        DrawSphereHelper () : center (Vec3::zero), radius (1.0f),
                              color (gWhite), maxEdgeLength (1.0f),
                              filled (false),
                              drawFrontFacing (true), drawBackFacing (true),
                              viewpoint (Vec3::zero)
        {}

        // utility constructor for drawing SphereObstacles
        DrawSphereHelper (const SphereObstacle& so,
                          const float mel,
                          const Vec3& vp,
                          const bool f,
                          const Vec3& c) : center (so.center),
                                           radius (so.radius),
                                           color (c),
                                           maxEdgeLength (mel),
                                           filled (f),
                                           drawFrontFacing (true),
                                           drawBackFacing (true),
                                           viewpoint (vp)
        {
            switch (so.seenFrom ())
            {
            case Obstacle::both:                            break;
            case Obstacle::inside: drawFrontFacing = false; break;
            case Obstacle::outside: drawBackFacing = false; break;
            }
        }

        // draw as an icosahedral geodesic sphere
        void draw (void) const
        {
            // Geometry based on Paul Bourke's excellent article:
            //   Platonic Solids (Regular polytopes in 3D)
            //   http://astronomy.swin.edu.au/~pbourke/polyhedra/platonic/
            const float sqrt5 = sqrtXXX (5.0f);
            const float phi = (1.0f + sqrt5) * 0.5f; // "golden ratio"
            // ratio of edge length to radius
            const float ratio = sqrtXXX (10.0f + (2.0f * sqrt5)) / (4.0f * phi);
            const float a = (radius / ratio) * 0.5;
            const float b = (radius / ratio) / (2.0f * phi);

            // define the icosahedron's 12 vertices:
            const Vec3 v1  = center + Vec3 ( 0,  b, -a);
            const Vec3 v2  = center + Vec3 ( b,  a,  0);
            const Vec3 v3  = center + Vec3 (-b,  a,  0);
            const Vec3 v4  = center + Vec3 ( 0,  b,  a);
            const Vec3 v5  = center + Vec3 ( 0, -b,  a);
            const Vec3 v6  = center + Vec3 (-a,  0,  b);
            const Vec3 v7  = center + Vec3 ( 0, -b, -a);
            const Vec3 v8  = center + Vec3 ( a,  0, -b);
            const Vec3 v9  = center + Vec3 ( a,  0,  b);
            const Vec3 v10 = center + Vec3 (-a,  0, -b);
            const Vec3 v11 = center + Vec3 ( b, -a,  0);
            const Vec3 v12 = center + Vec3 (-b, -a,  0);

            // draw the icosahedron's 20 triangular faces:
            drawMeshedTriangleOnSphere (v1, v2, v3);
            drawMeshedTriangleOnSphere (v4, v3, v2);
            drawMeshedTriangleOnSphere (v4, v5, v6);
            drawMeshedTriangleOnSphere (v4, v9, v5);
            drawMeshedTriangleOnSphere (v1, v7, v8);
            drawMeshedTriangleOnSphere (v1, v10, v7);
            drawMeshedTriangleOnSphere (v5, v11, v12);
            drawMeshedTriangleOnSphere (v7, v12, v11);
            drawMeshedTriangleOnSphere (v3, v6, v10);
            drawMeshedTriangleOnSphere (v12, v10, v6);
            drawMeshedTriangleOnSphere (v2, v8, v9);
            drawMeshedTriangleOnSphere (v11, v9, v8);
            drawMeshedTriangleOnSphere (v4, v6, v3);
            drawMeshedTriangleOnSphere (v4, v2, v9);
            drawMeshedTriangleOnSphere (v1, v3, v10);
            drawMeshedTriangleOnSphere (v1, v8, v2);
            drawMeshedTriangleOnSphere (v7, v10, v12);
            drawMeshedTriangleOnSphere (v7, v11, v8);
            drawMeshedTriangleOnSphere (v5, v12, v6);
            drawMeshedTriangleOnSphere (v5, v9, v11);
        }


        // given two points, take midpoint and project onto this sphere
        inline Vec3 midpointOnSphere (const Vec3& a, const Vec3& b) const
        {
            const Vec3 midpoint = (a + b) * 0.5f;
            const Vec3 unitRadial = (midpoint - center).normalize ();
            return center + (unitRadial * radius);
        }


        // given three points on the surface of this sphere, if the triangle
        // is "small enough" draw it, otherwise subdivide it into four smaller
        // triangles and recursively draw each of them.
        void drawMeshedTriangleOnSphere (const Vec3& a, 
                                         const Vec3& b,
                                         const Vec3& c) const
        {
            // if all edges are short enough
            if ((((a - b).length ()) < maxEdgeLength) &&
                (((b - c).length ()) < maxEdgeLength) &&
                (((c - a).length ()) < maxEdgeLength))
            {
                // draw triangle
                drawTriangleOnSphere (a, b, c);
            }
            else // otherwise subdivide and recurse
            {
                // find edge midpoints
                const Vec3 ab = midpointOnSphere (a, b);
                const Vec3 bc = midpointOnSphere (b, c);
                const Vec3 ca = midpointOnSphere (c, a);

                // recurse on four sub-triangles
                drawMeshedTriangleOnSphere ( a, ab, ca);
                drawMeshedTriangleOnSphere (ab,  b, bc);
                drawMeshedTriangleOnSphere (ca, bc,  c);
                drawMeshedTriangleOnSphere (ab, bc, ca);
            }
        }


        // draw one mesh element for drawMeshedTriangleOnSphere
        void drawTriangleOnSphere (const Vec3& a, 
                                   const Vec3& b,
                                   const Vec3& c) const
        {
            // draw triangle, subject to the camera orientation criteria
            // (according to drawBackFacing and drawFrontFacing)
            const Vec3 triCenter = (a + b + c) / 3.0f;
            const Vec3 triNormal = triCenter - center; // not unit length
            const Vec3 view = triCenter - viewpoint;
            const float dot = view.dot (triNormal); // project normal on view
            const bool seen = ((dot>0.0f) ? drawBackFacing : drawFrontFacing);
            if (seen)
            {
                if (filled)
                {
                    // draw filled triangle
                    if (drawFrontFacing)
                        drawTriangle (c, b, a, color);
                    else
                        drawTriangle (a, b, c, color);
                }
                else
                {
                    // draw triangle edges (use trick to avoid drawing each
                    // edge twice (for each adjacent triangle) unless we are
                    // culling and this tri is near the sphere's silhouette)
                    const float unitDot = view.dot (triNormal.normalize ());
                    const float t = 0.05f; // near threshold
                    const bool nearSilhouette = (unitDot<t) || (unitDot>-t);
                    if (nearSilhouette && !(drawBackFacing&&drawFrontFacing))
                    {
                        drawLine (a, b, color);
                        drawLine (b, c, color);
                        drawLine (c, a, color);
                    }
                    else
                    {
                        drawMeshedTriangleLine (a, b, color);
                        drawMeshedTriangleLine (b, c, color);
                        drawMeshedTriangleLine (c, a, color);
                    }
                }
            }
        }


        // Draws line from A to B but not from B to A: assumes each edge
        // will be drawn in both directions, picks just one direction for
        // drawing according to an arbitary but reproducable criterion.
        void drawMeshedTriangleLine (const Vec3& a, 
                                     const Vec3& b,
                                     const Vec3& color) const
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

    };


    void tempDrawRectangle (const RectangleObstacle& rect, const Vec3 color)
    {
        float w = rect.width / 2;
        float h = rect.height / 2;

        Vec3 v1 = rect.globalizePosition (Vec3 ( w,  h, 0));
        Vec3 v2 = rect.globalizePosition (Vec3 (-w,  h, 0));
        Vec3 v3 = rect.globalizePosition (Vec3 (-w, -h, 0));
        Vec3 v4 = rect.globalizePosition (Vec3 ( w, -h, 0));

        drawLine (v1, v2, color);
        drawLine (v2, v3, color);
        drawLine (v3, v4, color);
        drawLine (v4, v1, color);
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
