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
// Obstacle and SphericalObstacle
//
// for use with obstacle avoidance
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 09-05-02 cwr: created
//
//
// ----------------------------------------------------------------------------
//
// XXX 4-23-03: Temporary work around.  Put steerToAvoid method here to
// XXX support generic steerToAvoidObstacle method in SteerLibraryMixin.
// XXX That in turn requires making Obstacles know about AbstractVehicle.
// XXX The final implementation will probably be significantly different.
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_OBSTACLE_H
#define OPENSTEER_OBSTACLE_H


#include "OpenSteer/Vec3.h"

// XXX 4-23-03: Temporary work around (see comment above)
#include "OpenSteer/AbstractVehicle.h"


namespace OpenSteer {


    // ----------------------------------------------------------------------------
    // Obstacle: a pure virtual base class for an abstract shape in space, to be
    // used with obstacle avoidance.
    //
    // XXX this should define generic methods for querying the obstacle shape


    class AbstractObstacle
    {
    public:

        // seenFrom (eversion): does this obstacle contrain vehicle to stay
        // inside it or outside it (or both)?  "Inside" describes a clear space
        // within a solid (for example, the interior of a room inside its
        // walls). "Ouitside" describes a solid chunk in the midst of clear
        // space.
        enum seenFromState {outside, inside, both};
        virtual seenFromState seenFrom (void) const = 0;
        virtual void setSeenFrom (seenFromState s) = 0;

        // XXX 4-23-03: Temporary work around (see comment above)
        virtual Vec3 steerToAvoid (const AbstractVehicle& v,
                                   const float minTimeToCollision) const = 0;

        typedef struct {
            int intersect;
            float distance;
            Vec3 surfacePoint;
            Vec3 surfaceNormal;
            // xxx should this be "Obstacle*"now?
            // SphericalObstacle* obstacle;
            // xxx no, replace it with centerXXX for now,
            //    later use surfacePoint/surfaceNormal
            Vec3 centerXXX;
            // xxx new
            AbstractObstacle* obstacle;
        } PathIntersection;

        virtual void
        findIntersectionWithVehiclePath (const AbstractVehicle& vehicle,
                                         PathIntersection& intersection)
        /* const ??? */
            = 0 ;
    };


    // an STL vector of AbstractObstacle pointers and an iterator for it:
    typedef std::vector<AbstractObstacle*> ObstacleGroup;
    typedef ObstacleGroup::const_iterator ObstacleIterator;


    // ----------------------------------------------------------------------------
    // Obstacle is a utility base class providing some shared functionality


    class Obstacle : public AbstractObstacle
    {
    public:

        Obstacle (void) : _seenFrom (outside) {}

        // apply steerToAvoid to the nearest obstacle in an ObstacleGroup
        static Vec3 steerToAvoidObstacles (const AbstractVehicle& vehicle,
                                           const float minTimeToCollision,
                                           const ObstacleGroup& obstacles);


        seenFromState seenFrom (void) const {return _seenFrom;}
        void setSeenFrom (seenFromState s) {_seenFrom = s;}
    private:
        seenFromState _seenFrom;
    };



    // ----------------------------------------------------------------------------
    // SphericalObstacle a simple concrete type of obstacle


    class SphericalObstacle : public Obstacle
    {
    public:
        float radius;
        Vec3 center;

        // constructors
        SphericalObstacle (float r, Vec3 c) : radius(r), center (c) {}
        SphericalObstacle (void) : radius(1), center (Vec3::zero) {}

        Vec3 steerToAvoid (const AbstractVehicle& v,
                           const float minTimeToCollision) const;


        void findIntersectionWithVehiclePath (const AbstractVehicle& vehicle,
                                              PathIntersection& intersection);
    };

} // namespace OpenSteer
    
    
// ----------------------------------------------------------------------------
#endif // OPENSTEER_OBSTACLE_H
