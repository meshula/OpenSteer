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
// Obstacle
// static method to apply steerToAvoid to nearest obstacle in an ObstacleGroup


OpenSteer::Vec3
OpenSteer::Obstacle::
steerToAvoidObstacles (const AbstractVehicle& vehicle,
                       const float minTimeToCollision,
                       const ObstacleGroup& obstacles)
{
    PathIntersection nearest, next;

    // test all obstacles in group for an intersection with the vehicle's
    // future path, select the one whose point of intersection is nearest
    next.intersect = false;
    nearest.intersect = false;
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

    // if nearby intersection found, steer away from it, otherwise no steering
    return nearest.steerToAvoidIfNeeded (vehicle, minTimeToCollision);
}


// ----------------------------------------------------------------------------
// Obstacle
// compute steering to avoid this obstacle, if needed 


OpenSteer::Vec3 
OpenSteer::Obstacle::steerToAvoid (const AbstractVehicle& vehicle,
                                   const float minTimeToCollision) const
{
    // if nearby intersection found, steer away from it, otherwise no steering
    PathIntersection pi;
    findIntersectionWithVehiclePath (vehicle, pi);
    return pi.steerToAvoidIfNeeded (vehicle, minTimeToCollision);
}


// ----------------------------------------------------------------------------
// PathIntersection
// determine steering once path intersections have been found


OpenSteer::Vec3 
OpenSteer::Obstacle::PathIntersection::
steerToAvoidIfNeeded (const AbstractVehicle& vehicle,
                      const float minTimeToCollision) const
{
    // if nearby intersection found, steer away from it, otherwise no steering
    const float minDistanceToCollision = minTimeToCollision * vehicle.speed();
    if (intersect && (distance < minDistanceToCollision))
    {
        // compute avoidance steering force: take the component of
        // steerHint which is lateral (perpendicular to vehicle's
        // forward direction), set its length to vehicle's maxForce
        Vec3 lateral = steerHint.perpendicularComponent (vehicle.forward ());
        return lateral.normalize () * vehicle.maxForce ();
    }
    else
    {
        return Vec3::zero;
    }
}


// ----------------------------------------------------------------------------
// SphericalObstacle
// find first intersection of a vehicle's path with this obstacle


void 
OpenSteer::
SphericalObstacle::
findIntersectionWithVehiclePath (const AbstractVehicle& vehicle,
                                 PathIntersection& intersection) const
{
    // This routine is based on the Paul Bourke's derivation in:
    //   Intersection of a Line and a Sphere (or circle)
    //   http://www.swin.edu.au/astronomy/pbourke/geometry/sphereline/
    // But the computation is done in the vehicle's local space,
    // the line in question is the Z (Forward) axis of the space

    float b, c, d, p, q, s;
    Vec3 lc;

    // initialize pathIntersection object to "no intersection found"
    intersection.intersect = false;

    // find sphere's "local center" (lc) in the vehicle's coordinate space
    lc = vehicle.localizePosition (center);

    // compute line-sphere intersection parameters
    const float r = radius + vehicle.radius();
    b = -2 * lc.z;
    c = square (lc.x) + square (lc.y) + square (lc.z) - square (r);
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
    intersection.obstacle = this;
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
    intersection.surfacePoint =
        vehicle.position() + (vehicle.forward() * intersection.distance);
    intersection.surfaceNormal = (intersection.surfacePoint-center).normalize();
    intersection.steerHint = intersection.surfaceNormal;
}


// ----------------------------------------------------------------------------
// RectangleObstacle
// find first intersection of a vehicle's path with this obstacle


void 
OpenSteer::
RectangleObstacle::
findIntersectionWithVehiclePath (const AbstractVehicle& vehicle,
                                 PathIntersection& intersection) const
{
    // initialize pathIntersection object to "no intersection found"
    intersection.intersect = false;

    const Vec3 lp =  localizePosition (vehicle.position ());
    const Vec3 ld = localizeDirection (vehicle.forward ());

    // no obstacle intersection if path is parallel to rectangle's plane
    if (ld.dot (Vec3::forward) == 0.0f) return;

    // no obstacle intersection if vehicle is heading away from rectangle
    if ((lp.z > 0.0f) && (ld.z > 0.0f)) return;
    if ((lp.z < 0.0f) && (ld.z < 0.0f)) return;

    // no obstacle intersection if obstacle "not seen" from vehicle's side
    if ((seenFrom () == outside) && (lp.z < 0.0f)) return;
    if ((seenFrom () == inside)  && (lp.z > 0.0f)) return;

    // find intersection of path with rectangle's plane (XY plane)
    const float ix = lp.x - (ld.x * lp.z / ld.z);
    const float iy = lp.y - (ld.y * lp.z / ld.z);
    const Vec3 planeIntersection (ix, iy, 0.0f);

    // no obstacle intersection if plane intersection is outside rectangle
    const float r = vehicle.radius ();
    const float w = r + (width * 0.5f);
    const float h = r + (height * 0.5f);
    if ((ix > w) || (ix < -w) || (iy > h) || (iy < -h)) return;

    // otherwise, the vehicle path DOES intersect this rectangle
    const Vec3 pin = planeIntersection.normalize ();
    const Vec3 gpin = globalizeDirection (pin);
    const float sideSign = (lp.z > 0.0f) ? +1.0f : -1.0f;
    const Vec3 opposingNormal = forward () * sideSign;
    intersection.intersect = true;
    intersection.obstacle = this;
    intersection.distance = (lp - planeIntersection).length ();
    intersection.steerHint = opposingNormal + gpin;
    intersection.surfacePoint = globalizePosition (planeIntersection);
    intersection.surfaceNormal = opposingNormal;
}


// ----------------------------------------------------------------------------
