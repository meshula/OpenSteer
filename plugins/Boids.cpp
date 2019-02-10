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
#include "OpenSteer/Color.h"
#include "OpenSteer/UnusedParameter.h"

#ifdef WIN32
// Windows defines these as macros :(
#undef min
#undef max
#endif

#ifndef NO_LQ_BIN_STATS
#include <iomanip> // for setprecision
#include <limits> // for numeric_limits::max()
#endif // NO_LQ_BIN_STATS


namespace {

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
            drawBasic3dSphericalVehicle (this, gGray70);
            // drawTrail ();
        }


        // per frame simulation update
        void update (const float currentTime, const float elapsedTime)
        {
            OPENSTEER_UNUSED_PARAMETER(currentTime);
            
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

    #ifndef NO_LQ_BIN_STATS
            // maintain stats on max/min/ave neighbors per boids
            size_t count = neighbors.size();
            if (maxNeighbors < count) maxNeighbors = count;
            if (minNeighbors > count) minNeighbors = count;
            totalNeighbors += count;
    #endif // NO_LQ_BIN_STATS

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


    // ---------------------------------------------- xxxcwr111704_terrain_following
        // control orientation for this boid
        void regenerateLocalSpace (const Vec3& newVelocity,
                                   const float elapsedTime)
        {
            // 3d flight with banking
            regenerateLocalSpaceForBanking (newVelocity, elapsedTime);

            // // follow terrain surface
            // regenerateLocalSpaceForTerrainFollowing (newVelocity, elapsedTime);
        }


        // XXX experiment:
        // XXX   herd with terrain following
        // XXX   special case terrain: a sphere at the origin, radius 40
        void regenerateLocalSpaceForTerrainFollowing  (const Vec3& newVelocity,
                                                       const float /* elapsedTime */)
        {

            // XXX this is special case code, these should be derived from arguments //
            const Vec3 surfaceNormal = position().normalize();                       //
            const Vec3 surfacePoint = surfaceNormal * 40.0f;                         //
            // XXX this is special case code, these should be derived from arguments //

            const Vec3 newUp = surfaceNormal;
            const Vec3 newPos = surfacePoint;
            const Vec3 newVel = newVelocity.perpendicularComponent(newUp);
            const float newSpeed = newVel.length();
            const Vec3 newFor = newVel / newSpeed;

            setSpeed (newSpeed);
            setPosition (newPos);
            setUp (newUp);
            setForward (newFor);
            setUnitSideFromForwardAndUp ();
        }
    // ---------------------------------------------- xxxcwr111704_terrain_following

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
            const Color white (1,1,1);
            annotationLine (FR, FL, white);
            annotationLine (FL, BL, white);
            annotationLine (BL, BR, white);
            annotationLine (BR, FR, white);
        }

    #ifndef NO_LQ_BIN_STATS
            static size_t minNeighbors, maxNeighbors, totalNeighbors;
    #endif // NO_LQ_BIN_STATS
    };


    AVGroup Boid::neighbors;
    float Boid::worldRadius = 50.0f;
    ObstacleGroup Boid::obstacles;
    #ifndef NO_LQ_BIN_STATS
    size_t Boid::minNeighbors, Boid::maxNeighbors, Boid::totalNeighbors;
    #endif // NO_LQ_BIN_STATS


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
    #ifndef NO_LQ_BIN_STATS
            Boid::maxNeighbors = Boid::totalNeighbors = 0;
            Boid::minNeighbors = std::numeric_limits<int>::max();
    #endif // NO_LQ_BIN_STATS

            // update flock simulation for each boid
            for (iterator i = flock.begin(); i != flock.end(); i++)
            {
                (**i).update (currentTime, elapsedTime);
            }
        }

        void redraw (const float currentTime, const float elapsedTime)
        {
            // selected vehicle (user can mouse click to select another)
            AbstractVehicle* selected = OpenSteerDemo::selectedVehicle;

            // vehicle nearest mouse (to be highlighted)
            AbstractVehicle* nearMouse = OpenSteerDemo::vehicleNearestToMouse ();

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
            case outsideSpheresNoBig:
                status << "outside several spheres, with wrap-around" ; break;
            case rectangle:
                status << "inside a sphere, with a rectangle" ; break;
            case rectangleNoBig:
                status << "a rectangle, with wrap-around" ; break;
            case outsideBox:
                status << "inside a sphere, outside a box" ; break;
            case insideBox:
                status << "inside a box" ; break;
            }
            status << std::endl;
            const float h = drawGetWindowHeight ();
            const Vec3 screenLocation (10, h-50, 0);
            draw2dTextAt2dLocation (status, screenLocation, gGray80, drawGetWindowWidth(), drawGetWindowHeight());

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
            case 5:  printLQbinStats ();        break;
            }
        }

        void printLQbinStats (void)
        {
    #ifndef NO_LQ_BIN_STATS
            int min, max; float average;
            Boid& aBoid = **(flock.begin());
            aBoid.proximityToken->getBinPopulationStats (min, max, average);
            std::cout << std::setprecision (2)
                      << std::setiosflags (std::ios::fixed);
            std::cout << "Bin populations: min, max, average: "
                      << min << ", " << max << ", " << average
                      << " (non-empty bins)" << std::endl; 
            std::cout << "Boid neighbors:  min, max, average: "
                      << Boid::minNeighbors << ", "
                      << Boid::maxNeighbors << ", "
                      << ((float)Boid::totalNeighbors) / ((float)population)
                      << std::endl;
    #endif // NO_LQ_BIN_STATS
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
        enum ConstraintType {none, insideSphere,
                             outsideSphere, outsideSpheres, outsideSpheresNoBig,
                             rectangle, rectangleNoBig,
                             outsideBox, insideBox};

        ConstraintType constraint;

        // select next "boundary condition / constraint / obstacle"
        void nextBoundaryCondition (void)
        {
            constraint = (ConstraintType) ((int) constraint + 1);
            updateObstacles ();
        }

        class SO : public SphereObstacle
        {void draw (const bool filled, const Color& color, const Vec3& vp) const
            {drawSphereObstacle (*this, 10.0f, filled, color, vp);}};

        class RO : public RectangleObstacle
        {void draw (const bool, const Color& color, const Vec3&) const
            {tempDrawRectangle (*this, color);}};

        class BO : public BoxObstacle
        {void draw (const bool, const Color& color, const Vec3&) const
            {tempDrawBox (*this, color);}};

        RO bigRectangle;
        BO outsideBigBox, insideBigBox;
        SO insideBigSphere, outsideSphere0, outsideSphere1, outsideSphere2,
           outsideSphere3, outsideSphere4, outsideSphere5, outsideSphere6;


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
            case outsideSpheresNoBig:
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
            case rectangleNoBig:
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
            for (ObstacleIterator o = Boid::obstacles.begin();
                 o != Boid::obstacles.end();
                 o++)
            {
                (**o).draw (false, // draw in wireframe
                            ((*o == &insideBigSphere) ?
                             Color (0.2f, 0.2f, 0.4f) :
                             Color (0.1f, 0.1f, 0.2f)),
                            OpenSteerDemo::camera.position ());
            }
        }


        static void tempDrawRectangle (const RectangleObstacle& rect, const Color& color)
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


        static void tempDrawBox (const BoxObstacle& box, const Color& color)
        {
            const float w = box.width / 2;
            const float h = box.height / 2;
            const float d = box.depth / 2;

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

} // anonymous namespace
