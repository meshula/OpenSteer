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


#include "Vec3.h"

// XXX 4-23-03: Temporary work around (see comment above)
#include "AbstractVehicle.h"


namespace OpenSteer {


    // ----------------------------------------------------------------------------
    // Obstacle: a pure virtual base class for an abstract shape in space, to be
    // used with obstacle avoidance.
    //
    // XXX this should define generic methods for querying the obstacle shape


    class Obstacle
    {
    public:
        enum seenFromState {outside, inside, both};
        virtual seenFromState seenFrom (void) const = 0;
        virtual void setSeenFrom (seenFromState s) = 0;

        // XXX 4-23-03: Temporary work around (see comment above)
        virtual Vec3 steerToAvoid (const AbstractVehicle& v,
                                   const float minTimeToCollision) const = 0;
    };


    // an STL vector of Obstacle pointers and an iterator for it:
    typedef std::vector<Obstacle*> ObstacleGroup;
    typedef ObstacleGroup::const_iterator ObstacleIterator;



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

        seenFromState seenFrom (void) const {return _seenFrom;}
        void setSeenFrom (seenFromState s) {_seenFrom = s;}


        // XXX 4-23-03: Temporary work around (see comment above)
        //
        // Checks for intersection of the given spherical obstacle with a
        // volume of "likely future vehicle positions": a cylinder along the
        // current path, extending minTimeToCollision seconds along the
        // forward axis from current position.
        //
        // If they intersect, a collision is imminent and this function returns
        // a steering force pointing laterally away from the obstacle's center.
        //
        // Returns a zero vector if the obstacle is outside the cylinder
        //
        // xxx couldn't this be made more compact using localizePosition?

        Vec3 steerToAvoid (const AbstractVehicle& v,
                           const float minTimeToCollision) const
        {
            // minimum distance to obstacle before avoidance is required
            const float minDistanceToCollision = minTimeToCollision * v.speed();
            const float minDistanceToCenter = minDistanceToCollision + radius;

            // contact distance: sum of radii of obstacle and vehicle
            const float totalRadius = radius + v.radius ();

            // obstacle center relative to vehicle position
            const Vec3 localOffset = center - v.position ();

            // distance along vehicle's forward axis to obstacle's center
            const float forwardComponent = localOffset.dot (v.forward ());
            const Vec3 forwardOffset = forwardComponent * v.forward ();

            // offset from forward axis to obstacle's center
            const Vec3 offForwardOffset = localOffset - forwardOffset;

            // test to see if sphere overlaps with obstacle-free corridor
            const bool inCylinder = offForwardOffset.length() < totalRadius;
            const bool nearby = forwardComponent < minDistanceToCenter;
            const bool inFront = forwardComponent > 0;

            // if all three conditions are met, steer away from sphere center
            if (inCylinder && nearby && inFront)
            {
                return offForwardOffset * -1;
            }
            else
            {
                return Vec3::zero;
            }
        }

    private:
        seenFromState _seenFrom;
    };

} // namespace OpenSteer
    
    
// ----------------------------------------------------------------------------
#endif // OPENSTEER_OBSTACLE_H
