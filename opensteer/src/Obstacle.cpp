// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2004, Sony Computer Entertainment America
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
// OpenSteer Obstacle classes
// 
// 10-28-04 cwr: split off from Obstacle.h 
//
//
// ----------------------------------------------------------------------------


#include "OpenSteer/Obstacle.h"


// ----------------------------------------------------------------------------
// static method to find avoidance steering for a given vehicle,
// minTimeToCollision and obstacle group


OpenSteer::Vec3
OpenSteer::Obstacle::
steerToAvoidObstacles (const AbstractVehicle& vehicle,
                       const float minTimeToCollision,
                       const ObstacleGroup& obstacles)
{
    Vec3 avoidance;
    Obstacle::PathIntersection nearest, next;
    const float minDistanceToCollision = minTimeToCollision * vehicle.speed();

    next.intersect = false;
    nearest.intersect = false;

    // test all obstacles for intersection with my forward axis,
    // select the one whose point of intersection is nearest
    for (ObstacleIterator o = obstacles.begin(); o != obstacles.end(); o++)
    {
        // find nearest point (if any) where vehicle path intersects obstacle
        // o, storing the results in PathIntersection object "next"
        (**o).findIntersectionWithVehiclePath (vehicle, next);

        // if this is the first intersection found, or it is the nearest found
        // so far, store it in PathIntersection object "nearest"
        if ((nearest.intersect == false) ||
            ((next.intersect != false) &&
             (next.distance < nearest.distance)))
            nearest = next;
    }

    // when a nearest intersection was found
    if ((nearest.intersect != false) &&
        (nearest.distance < minDistanceToCollision))
    {
        // XXX at this point we WANT to call steerToAvoid on that obstacle

        // avoidance = nearest.obstacle->steerToAvoid (vehicle, minTimeToCollision);

        // XXX but that is not doing the same thing as this old inline
        // XXX code, so I will leave it in until I find the difference

        // compute avoidance steering force: take offset from obstacle to me,
        // take the component of that which is lateral (perpendicular to my
        // forward direction), set length to maxForce, add a bit of forward
        // component (in capture the flag, we never want to slow down)
        const Vec3 offset = vehicle.position() - nearest.centerXXX;
        avoidance = offset.perpendicularComponent (vehicle.forward());
        avoidance = avoidance.normalize ();
        avoidance *= vehicle.maxForce ();
        avoidance += vehicle.forward() * vehicle.maxForce () * 0.75;
    }

    return avoidance;
}


// ----------------------------------------------------------------------------

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


OpenSteer::Vec3 
OpenSteer::SphericalObstacle::steerToAvoid (const AbstractVehicle& v,
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


// ----------------------------------------------------------------------------


void 
OpenSteer::
SphericalObstacle::
findIntersectionWithVehiclePath (const AbstractVehicle& vehicle,
                                 PathIntersection& intersection)
{
    // This routine is based on the Paul Bourke's derivation in:
    //   Intersection of a Line and a Sphere (or circle)
    //   http://www.swin.edu.au/astronomy/pbourke/geometry/sphereline/

    float b, c, d, p, q, s;
    Vec3 lc;

    // initialize pathIntersection object
    intersection.intersect = false;
    intersection.centerXXX = center;
    // xxx new
    intersection.obstacle = this;

    // find "local center" (lc) of sphere in boid's coordinate space
    lc = vehicle.localizePosition (center);

    // compute line-sphere intersection parameters
    b = -2 * lc.z;
    c = square (lc.x) + square (lc.y) + square (lc.z) - 
        square (radius + vehicle.radius());
    d = (b * b) - (4 * c);

    // when the path does not intersect the sphere
    if (d < 0) return;

    // otherwise, the path intersects the sphere in two points with
    // parametric coordinates of "p" and "q".  (If "d" is zero the two
    // points are coincident, the path is tangent)
    s = sqrtXXX (d);
    p = (-b + s) / 2;
    q = (-b - s) / 2;

    // both intersections are behind us, so no potential collisions
    if ((p < 0) && (q < 0)) return; 

    // at least one intersection is in front, so intersects our forward
    // path
    intersection.intersect = true;
    intersection.distance =
        ((p > 0) && (q > 0)) ?
        // both intersections are in front of us, find nearest one
        ((p < q) ? p : q) :
        // otherwise one is ahead and one is behind: we are INSIDE obstacle
        (seenFrom () == outside ?
         // inside a solid obstacle, so distance to obstacle is zero
         0.0f :
         // hollow obstacle (or "both"), pick point that is in front
         ((p > 0) ? p : q));
    return;
}

// ----------------------------------------------------------------------------
