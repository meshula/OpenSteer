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
// SteerLibraryMixin
//
// This mixin (class with templated superclass) adds the "steering library"
// functionality to a given base class.  SteerLibraryMixin assumes its base
// class supports the AbstractVehicle interface.
//
// 02-06-03 cwr: create mixin (from "SteerMass")
// 06-03-02 cwr: removed TS dependencies
// 11-21-01 cwr: created
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_STEERLIBRARY_H
#define OPENSTEER_STEERLIBRARY_H


#include "AbstractVehicle.h"
#include "Pathway.h"
#include "Obstacle.h"
#include "Utilities.h"


// ----------------------------------------------------------------------------


template <class Super>
class SteerLibraryMixin : public Super
{
public:

    // Constructor: initializes state
    SteerLibraryMixin ()
    {
        // set inital state
        reset ();
    }

    // reset state
    void reset (void)
    {
        // initial state of wander behavior
        WanderSide = 0;
        WanderUp = 0;

        // default to non-gaudyPursuitAnnotation
        gaudyPursuitAnnotation = false;
    }

    // -------------------------------------------------- steering behaviors

    // Wander behavior
    float WanderSide;
    float WanderUp;
    Vec3 steerForWander (float dt);

    // Seek behavior
    Vec3 steerForSeek (const Vec3& target);

    // Flee behavior
    Vec3 steerForFlee (const Vec3& target);

    // xxx proposed, experimental new seek/flee [cwr 9-16-02]
    Vec3 xxxsteerForFlee (const Vec3& target);
    Vec3 xxxsteerForSeek (const Vec3& target);

    // Path Following behavior
    Vec3 steerForPathFollowing (const float predictionTime, Pathway& path);

    // ------------------------------------------------------------------------
    // Obstacle Avoidance behavior
    //
    // Returns a steering force to avoid a given obstacle.  The purely
    // lateral steering force will turn our vehicle towards a silhouette edge
    // of the obstacle.  Avoidance is required when (1) the obstacle
    // intersects the vehicle's current path, (2) it is in front of the
    // vehicle, and (3) is within minTimeToCollision seconds of travel at the
    // vehicle's current velocity.  Returns a zero vector value (Vec3::zero)
    // when no avoidance is required.


    Vec3 steerToAvoidObstacle (const Obstacle& obstacle,
                               const float minTimeToCollision);

    // ------------------------------------------------------------------------
    // Unaligned collision avoidance behavior: avoid colliding with other
    // nearby vehicles moving in unconstrained directions.  Determine which
    // (if any) other other vehicle we would collide with first, then steers
    // to avoid the site of that potential collision.  Returns a steering
    // force vector, which is zero length if there is no impending collision.


    Vec3 steerToAvoidNeighbors (const float minTimeToCollision,
                                const AVGroup& others);


    // Given two vehicles, based on their current positions and velocities,
    // determine the time until nearest approach
    float predictNearestApproachTime (AbstractVehicle& other);

    // Given the time until nearest approach (predictNearestApproachTime)
    // determine position of each vehicle at that time, and the distance
    // between them
    float computeNearestApproachPositions (AbstractVehicle& other,
                                           float time);


    /// XXX globals only for the sake of graphical annotation
    Vec3 hisPositionAtNearestApproach;
    Vec3 ourPositionAtNearestApproach;


    // ------------------------------------------------------------------------
    // avoidance of "close neighbors" -- used only by steerToAvoidNeighbors
    //
    // XXX  Does a hard steer away from any other agent who comes withing a
    // XXX  critical distance.  Ideally this should be replaced with a call
    // XXX  to steerForSeparation.


    Vec3 steerToAvoidCloseNeighbors (const float minSeparationDistance,
                                     const AVGroup& others);


    // ------------------------------------------------------------------------
    // used by boid behaviors


    bool inBoidNeighborhood (const Vec3& queryLocation,
                             const float minDistance,
                             const float maxDistance,
                             const float cosMaxAngle,
                             float& distanceWeight,
                             Vec3& unitOffset);


    // ------------------------------------------------------------------------
    // Separation behavior -- determines the direction away from nearby boids


    Vec3 steerForSeparation (const float maxDistance,
                             const float cosMaxAngle,
                             const AVGroup& flock);


    // ------------------------------------------------------------------------
    // Alignment behavior

    Vec3 steerForAlignment (const float maxDistance,
                            const float cosMaxAngle,
                            const AVGroup& flock);


    // ------------------------------------------------------------------------
    // Cohesion behavior


    Vec3 steerForCohesion (const float maxDistance,
                           const float cosMaxAngle,
                           const AVGroup& flock);


    // ------------------------------------------------------------------------
    // pursuit of another vehicle (& version with ceiling on prediction time)


    Vec3 steerForPursuit (const AbstractVehicle& quarry);

    Vec3 steerForPursuit (const AbstractVehicle& quarry,
                          const float maxPredictionTime);

    // for annotation
    bool gaudyPursuitAnnotation;


    // ------------------------------------------------------------------------
    // evasion of another vehicle


    Vec3 steerForEvasion (const AbstractVehicle& menace,
                          const float maxPredictionTime);


    // ------------------------------------------------------------------------
    // tries to maintain a given speed, returns a maxForce-clipped steering
    // force along the forward/backward axis


    Vec3 steerForTargetSpeed (const float targetSpeed);


    // ----------------------------------------------------------- utilities
    // XXX these belong somewhere besides the steering library
    // XXX above AbstractVehicle, below SimpleVehicle
    // XXX ("utility vehicle"?)

    // xxx cwr experimental 9-9-02 -- names OK?
    bool isAhead (const Vec3& target) const {return isAhead (target, 0.707f);};
    bool isAside (const Vec3& target) const {return isAside (target, 0.707f);};
    bool isBehind (const Vec3& target) const {return isBehind (target, -0.707f);};

    bool isAhead (const Vec3& target, float cosThreshold) const
    {
        const Vec3 targetDirection = (target - position ()).normalize ();
        return forward().dot(targetDirection) > cosThreshold;
    };
    bool isAside (const Vec3& target, float cosThreshold) const
    {
        const Vec3 targetDirection = (target - position ()).normalize ();
        const float dp = forward().dot(targetDirection);
        return (dp < cosThreshold) && (dp > -cosThreshold);
    };
    bool isBehind (const Vec3& target, float cosThreshold) const
    {
        const Vec3 targetDirection = (target - position()).normalize ();
        return forward().dot(targetDirection) < cosThreshold;
    };


    // xxx cwr 9-6-02 temporary to support old code
    typedef struct {
        int intersect;
        float distance;
        Vec3 surfacePoint;
        Vec3 surfaceNormal;
        SphericalObstacle* obstacle;
    } PathIntersection;

    // xxx experiment cwr 9-6-02
    void findNextIntersectionWithSphere (SphericalObstacle& obs,
                                         PathIntersection& intersection);


    // ------------------------------------------------ graphical annotation

    void annotationForPathFollowing (const Vec3& futurePosition,
                                     const Vec3& onPath,
                                     const bool withinPath);

    void annotationForAvoidSphere (const float minDistanceToCollision);

    void annotationForAvoidNeighbor (const AbstractVehicle& threat,
                                     const float steer,
                                     const Vec3& ourFuture,
                                     const Vec3& threatFuture);

    void annotationForAvoidCloseNeighbor (const AbstractVehicle& other,
                                          const float additionalDistance);
};


// ----------------------------------------------------------------------------


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForWander (float dt)
{
    // random walk WanderSide and WanderUp between -1 and +1
    const float speed = 12 * dt; // maybe this (12) should be an argument?
    WanderSide = scalarRandomWalk (WanderSide, speed, -1, +1);
    WanderUp   = scalarRandomWalk (WanderUp,   speed, -1, +1);

    // return a pure lateral steering vector: (+/-Side) + (+/-Up)
    return (side() * WanderSide) + (up() * WanderUp);
}


// ----------------------------------------------------------------------------
// Seek behavior


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForSeek (const Vec3& target)
{
    const Vec3 desiredVelocity = target - position();
    return desiredVelocity - velocity();
}


// ----------------------------------------------------------------------------
// Flee behavior


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForFlee (const Vec3& target)
{
    const Vec3 desiredVelocity = position - target;
    return desiredVelocity - velocity();
}


// ----------------------------------------------------------------------------
// xxx proposed, experimental new seek/flee [cwr 9-16-02]


template<class Super>
Vec3
SteerLibraryMixin<Super>::
xxxsteerForFlee (const Vec3& target)
{
//  const Vec3 offset = position - target;
    const Vec3 offset = position() - target;
    const Vec3 desiredVelocity = offset.truncateLength (maxSpeed ()); //xxxnew
    return desiredVelocity - velocity();
}


template<class Super>
Vec3
SteerLibraryMixin<Super>::
xxxsteerForSeek (const Vec3& target)
{
//  const Vec3 offset = target - position;
    const Vec3 offset = target - position();
    const Vec3 desiredVelocity = offset.truncateLength (maxSpeed ()); //xxxnew
    return desiredVelocity - velocity();
}


// ----------------------------------------------------------------------------
// Path Following behavior


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForPathFollowing (const float predictionTime, Pathway& path)
{
    // predict our future position
    const Vec3 futurePosition = predictFuturePosition (predictionTime);

    // find the point on the path nearest the predicted future position
    Vec3 tangent;
    bool withinPath;
    const Vec3 onPath = path.mapPointToPath (futurePosition,
                                             tangent,     // output argument
                                             withinPath); // output argument

    annotationForPathFollowing (futurePosition, onPath, withinPath);

    if (withinPath)
    {
        // our predicted future position was in the path,
        // return zero steering.
        return Vec3::zero;
    }
    else
    {
        // our predicted future position was outside the path, need to
        // steer towards it.  Decide whether we are going up or down
        // the path.  Choose a target point on the path, somewhat
        // "ahead: of our current position.
        const Vec3 nextPosition = position () + forward ();
        float thisPathDistance = path.mapPointToPathDistance (position ());
        float nextPathDistance = path.mapPointToPathDistance (nextPosition);
        // xxx!:
        float nudge = (thisPathDistance < nextPathDistance) ? 10 : -10;
        Vec3 target = path.mapPathDistanceToPoint (thisPathDistance + nudge);
        return steerForSeek (target);
    }
}


// ----------------------------------------------------------------------------
// Obstacle Avoidance behavior
//
// Returns a steering force to avoid a given obstacle.  The purely lateral
// steering force will turn our vehicle towards a silhouette edge of the
// obstacle.  Avoidance is required when (1) the obstacle intersects the
// vehicle's current path, (2) it is in front of the vehicle, and (3) is
// within minTimeToCollision seconds of travel at the vehicle's current
// velocity.  Returns a zero vector value (Vec3::zero) when no avoidance is
// required.
//
// XXX The current (4-23-03) scheme is to dump all the work on the various
// XXX Obstacle classes, making them provide a "steer vehicle to avoid me"
// XXX method.  This may well change.


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerToAvoidObstacle (const Obstacle& obstacle,
                      const float minTimeToCollision)
{
    const Vec3 avoidance = obstacle.steerToAvoid (*this, minTimeToCollision);

    // XXX more annotation modularity problems (assumes spherical obstacle)
    if (avoidance != Vec3::zero)
        annotationForAvoidSphere (minTimeToCollision * speed());

    return avoidance;
}


// ----------------------------------------------------------------------------
// Unaligned collision avoidance behavior: avoid colliding with other nearby
// vehicles moving in unconstrained directions.  Determine which (if any)
// other other vehicle we would collide with first, then steers to avoid the
// site of that potential collision.  Returns a steering force vector, which
// is zero length if there is no impending collision.


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerToAvoidNeighbors (const float minTimeToCollision,
                       const AVGroup& others)
{
    // first priority is to prevent immediate interpenetration
    const Vec3 separation = steerToAvoidCloseNeighbors (0, others);
    if (separation != Vec3::zero) return separation;

    // otherwise, go on to consider potential future collisions
    float steer = 0;
    AbstractVehicle* threat = NULL;

    // Time (in seconds) until the most immediate collision threat found
    // so far.  Initial value is a threshold: don't look more than this
    // many frames into the future.
    float minTime = minTimeToCollision;

    // xxx solely for annotation
    Vec3 xxxThreatPositionAtNearestApproach;
    Vec3 xxxOurPositionAtNearestApproach;

    // for each of the other vehicles, determine which (if any)
    // pose the most immediate threat of collision.
    for (AVIterator i = others.begin(); i != others.end(); i++)
    {
        AbstractVehicle& other = **i;
        if (&other != this)
        {	
            // avoid when future positions are this close (or less)
            const float collisionDangerThreshold = radius() * 2;

            // predicted time until nearest approach of "this" and "other"
            const float time = predictNearestApproachTime (other);

            // If the time is in the future, sooner than any other
            // threatened collision...
            if ((time >= 0) && (time < minTime))
            {
                // if the two will be close enough to collide,
                // make a note of it
                if (computeNearestApproachPositions (other, time)
                    < collisionDangerThreshold)
                {
                    minTime = time;
                    threat = &other;
                    xxxThreatPositionAtNearestApproach
                        = hisPositionAtNearestApproach;
                    xxxOurPositionAtNearestApproach
                        = ourPositionAtNearestApproach;
                }
            }
        }
    }

    // if a potential collision was found, compute steering to avoid
    if (threat != NULL)
    {
        // parallel: +1, perpendicular: 0, anti-parallel: -1
        float parallelness = forward().dot(threat->forward());
        float angle = 0.707f;

        if (parallelness < -angle)
        {
            // anti-parallel "head on" paths:
            // steer away from future threat position
            Vec3 offset = xxxThreatPositionAtNearestApproach - position();
            float sideDot = offset.dot(side());
            steer = (sideDot > 0) ? -1 : 1;
        }
        else
        {
            if (parallelness > angle)
            {
                // parallel paths: steer away from threat
                Vec3 offset = threat->position() - position();
                float sideDot = offset.dot(side());
                steer = (sideDot > 0) ? -1 : 1;
            }
            else
            {
                // perpendicular paths: steer behind threat
                // (only the slower of the two does this)
                if (threat->speed() <= speed())
                {
                    float sideDot = side().dot(threat->velocity());
                    steer = (sideDot > 0) ? -1 : 1;
                }
            }
        }

        annotationForAvoidNeighbor (*threat,
                                    steer,
                                    xxxOurPositionAtNearestApproach,
                                    xxxThreatPositionAtNearestApproach);
    }

    return side() * steer;
}



// Given two vehicles, based on their current positions and velocities,
// determine the time until nearest approach
//
// XXX should this return zero if they are already in contact?

template<class Super>
float
SteerLibraryMixin<Super>::
predictNearestApproachTime (AbstractVehicle& other)
{
    // imagine we are at the origin with no velocity,
    // compute the relative velocity of the other vehicle
    const Vec3 myVelocity = velocity();
    const Vec3 otherVelocity = other.velocity();
    const Vec3 relVelocity = otherVelocity - myVelocity;
    const float relSpeed = relVelocity.length();

    // for parallel paths, the vehicles will always be at the same distance,
    // so return 0 (aka "now") since "there is no time like the present"
    if (relSpeed == 0) return 0;

    // Now consider the path of the other vehicle in this relative
    // space, a line defined by the relative position and velocity.
    // The distance from the origin (our vehicle) to that line is
    // the nearest approach.

    // Take the unit tangent along the other vehicle's path
    const Vec3 relTangent = relVelocity / relSpeed;

    // find distance from its path to origin (compute offset from
    // other to us, find length of projection onto path)
    const Vec3 relPosition = position() - other.position();
    const float projection = relTangent.dot(relPosition);

    return projection / relSpeed;
}


// Given the time until nearest approach (predictNearestApproachTime)
// determine position of each vehicle at that time, and the distance
// between them


template<class Super>
float
SteerLibraryMixin<Super>::
computeNearestApproachPositions (AbstractVehicle& other,
                                 float time)
{
    const Vec3    myTravel =       forward () *       speed () * time;
    const Vec3 otherTravel = other.forward () * other.speed () * time;

    const Vec3    myFinal =       position () +    myTravel;
    const Vec3 otherFinal = other.position () + otherTravel;

    // xxx for annotation
    ourPositionAtNearestApproach = myFinal;
    hisPositionAtNearestApproach = otherFinal;

    return Vec3::distance (myFinal, otherFinal);
}



// ----------------------------------------------------------------------------
// avoidance of "close neighbors" -- used only by steerToAvoidNeighbors
//
// XXX  Does a hard steer away from any other agent who comes withing a
// XXX  critical distance.  Ideally this should be replaced with a call
// XXX  to steerForSeparation.


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerToAvoidCloseNeighbors (const float minSeparationDistance,
                            const AVGroup& others)
{
    // for each of the other vehicles...
    for (AVIterator i = others.begin(); i != others.end(); i++)    
    {
        AbstractVehicle& other = **i;
        if (&other != this)
        {
            const float sumOfRadii = radius() + other.radius();
            const float minCenterToCenter = minSeparationDistance + sumOfRadii;
            const Vec3 offset = other.position() - position();
            const float currentDistance = offset.length();

            if (currentDistance < minCenterToCenter)
            {
                annotationForAvoidCloseNeighbor (other, minSeparationDistance);
                return (-offset).perpendicularComponent (forward());
            }
        }
    }

    // otherwise return zero
    return Vec3::zero;
}


// ----------------------------------------------------------------------------
// used by boid behaviors


template<class Super>
bool
SteerLibraryMixin<Super>::
inBoidNeighborhood (const Vec3& queryLocation,
                    const float minDistance,
                    const float maxDistance,
                    const float cosMaxAngle,
                    float& distanceWeight,
                    Vec3& unitOffset)
{
    const Vec3 offset = queryLocation - position();
    const float distance = offset.length ();
    unitOffset = offset / distance;
    const float forwardness = forward().dot (unitOffset);

    // conical weighting function (linear falloff)
    const float d = maxDistance - distance; //xxx NAME???
    distanceWeight = d / maxDistance;

    if (distance < minDistance)
    {
        return true;
    }
    else
    {
        if ((distance < maxDistance) && (forwardness > cosMaxAngle))
        {
            return true;
        }
        else
        {
            distanceWeight = 0;
            return false;
        }
    }
}



// ----------------------------------------------------------------------------
// Separation behavior (xxx used by Boids xxx)
// xxx used by Boids -- determines the direction away from nearby boids


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForSeparation (const float maxDistance,
                    const float cosMaxAngle,
                    const AVGroup& flock)
{
    // for each of the other vehicles...
    Vec3 steering;
    int neighbors = 0;
    for (AVIterator i = flock.begin(); i != flock.end(); i++)
    {
        AbstractVehicle& other = **i;
        if (&other != this)
        {
            float distanceWeight;
            Vec3 unitOffset;
            if (inBoidNeighborhood (other.position(),
                                    radius()*3,
                                    maxDistance,
                                    cosMaxAngle,
                                    distanceWeight,
                                    unitOffset))
            {
                // XXX ------------------------------ do it like in psboids
                // // repulsive force, diminished with distance
                // steering += unitOffset * -distanceWeight;
                // XXX ------------------------------ do it like in psboids
                const Vec3 offset = other.position() - position();
                const float distanceSquared = offset.dot(offset);
                steering += (offset / -distanceSquared);
                // XXX ------------------------------ do it like in psboids

                // count neighbors
                neighbors++;
            }
        }
    }

    // XXX ------------------------------ do it like in psboids
    //     if (neighbors > 0) steering = steering / neighbors;
    // XXX ------------------------------ do it like in psboids
    if (neighbors > 0)
    {
        steering = (steering / neighbors).normalize();
    }
    // XXX ------------------------------ do it like in psboids

    return steering;
}

// ----------------------------------------------------------------------------
// Alignment behavior

template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForAlignment (const float maxDistance,
                   const float cosMaxAngle,
                   const AVGroup& flock)
{
    // for each of the other vehicles...
    Vec3 steering;
    int neighbors = 0;
    for (AVIterator i = flock.begin(); i != flock.end(); i++)
    {
        AbstractVehicle& other = **i;
        if (&other != this)
        {
            float distanceWeight;
            Vec3 unitOffset;
            if (inBoidNeighborhood (other.position(),
                                    radius()*3,
                                    maxDistance,
                                    cosMaxAngle,
                                    distanceWeight,
                                    unitOffset))
            {
                // const Vec3 alignError = forward() - other.forward;
                // steering += alignError * -distanceWeight;
                // XXX ------------------------------ do it like in psboids
                // const Vec3 myVelocity = forward() * speed;
                // const Vec3 otherVelocity = other.forward() * other.speed;
                // const Vec3 velocityDifference = myVelocity - otherVelocity;
                // XXX ------------------------------ do it like in psboids

                // XXX ------------------------------ do it like in psboids
                // // corrective force, diminished with distance
                // steering += velocityDifference * -distanceWeight;
                // XXX ------------------------------ do it like in psboids
                steering += other.forward();
                // XXX ------------------------------ do it like in psboids

                // count neighbors
                neighbors++;
            }

        }
    }

    // XXX ------------------------------ do it like in psboids
    //     if (neighbors > 0) steering = steering / neighbors;
    // XXX ------------------------------ do it like in psboids
    if (neighbors > 0)
    {
        steering = steering / neighbors;
        steering -= forward();
        steering = steering.normalize();
    }
    // XXX ------------------------------ do it like in psboids

    return steering;
}


// ----------------------------------------------------------------------------
// Cohesion behavior


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForCohesion (const float maxDistance,
                  const float cosMaxAngle,
                  const AVGroup& flock)
{
    // for each of the other vehicles...
    Vec3 steering;
    int neighbors = 0;
    for (AVIterator i = flock.begin(); i != flock.end(); i++)
    {
        AbstractVehicle& other = **i;
        if (&other != this)
        {

            float distanceWeight;
            Vec3 unitOffset;
            if (inBoidNeighborhood (other.position(),
                                    radius()*3,
                                    maxDistance,
                                    cosMaxAngle,
                                    distanceWeight,
                                    unitOffset))
            {
                // XXX ------------------------------ do it like in psboids
                // // attractive force, diminished with distance
                // steering += unitOffset * distanceWeight;
                // XXX ------------------------------ do it like in psboids
                steering += other.position();
                // XXX ------------------------------ do it like in psboids

                // count neighbors
                neighbors++;
            }
        }
    }

    // XXX ------------------------------ do it like in psboids
    //     if (neighbors > 0) steering = steering / neighbors;
    // XXX ------------------------------ do it like in psboids
    if (neighbors > 0)
    {
        steering = steering / neighbors;
        steering -= position();
        steering = steering.normalize();
    }
    // XXX ------------------------------ do it like in psboids

    return steering;
}



// ----------------------------------------------------------------------------
// pursuit of another vehicle (& version with ceiling on prediction time)


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForPursuit (const AbstractVehicle& quarry)
{
    return steerForPursuit (quarry, FLT_MAX);
}


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForPursuit (const AbstractVehicle& quarry,
                 const float maxPredictionTime)
{
    // offset from this to quarry, that distance, unit vector toward quarry
    const Vec3 offset = quarry.position() - position();
    const float distance = offset.length ();
    const Vec3 unitOffset = offset / distance;

    // how parallel are the paths of "this" and the quarry
    // (1 means parallel, 0 is pependicular, -1 is anti-parallel)
    const float parallelness = forward().dot (quarry.forward());

    // how "forward" is the direction to the quarry
    // (1 means dead ahead, 0 is directly to the side, -1 is straight back)
    const float forwardness = forward().dot (unitOffset);

    const float directTravelTime = distance / speed ();
    const int f = intervalComparison (forwardness,  -0.707f, 0.707f);
    const int p = intervalComparison (parallelness, -0.707f, 0.707f);

    float timeFactor = 0; // to be filled in below
    Vec3 color;           // to be filled in below (xxx just for debugging)

    // Break the pursuit into nine cases, the cross product of the
    // quarry being [ahead, aside, or behind] us and heading
    // [parallel, perpendicular, or anti-parallel] to us.
    switch (f)
    {
    case +1:
        switch (p)
        {
        case +1:          // ahead, parallel
            timeFactor = 4;
            color = gBlack;
            break;
        case 0:           // ahead, perpendicular
            timeFactor = 1.8f;
            color = gGray50;
            break;
        case -1:          // ahead, anti-parallel
            timeFactor = 0.85f;
            color = gWhite;
            break;
        }
        break;
    case 0:
        switch (p)
        {
        case +1:          // aside, parallel
            timeFactor = 1;
            color = gRed;
            break;
        case 0:           // aside, perpendicular
            timeFactor = 0.8f;
            color = gYellow;
            break;
        case -1:          // aside, anti-parallel
            timeFactor = 4;
            color = gGreen;
            break;
        }
        break;
    case -1:
        switch (p)
        {
        case +1:          // behind, parallel
            timeFactor = 0.5f;
            color= gCyan;
            break;
        case 0:           // behind, perpendicular
            timeFactor = 2;
            color= gBlue;
            break;
        case -1:          // behind, anti-parallel
            timeFactor = 2;
            color = gMagenta;
            break;
        }
        break;
    }

    // estimated time until intercept of quarry
    const float et = directTravelTime * timeFactor;

    // xxx experiment, if kept, this limit should be an argument
    const float etl = (et > maxPredictionTime) ? maxPredictionTime : et;

    // estimated position of quarry at intercept
    const Vec3 target = quarry.predictFuturePosition (etl);

    // annotation
    annotationLine (position(),
                    target,
                    gaudyPursuitAnnotation ? color : gGray40);

    return steerForSeek (target);
}

// ----------------------------------------------------------------------------
// evasion of another vehicle


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForEvasion (const AbstractVehicle& menace,
                 const float maxPredictionTime)
{
    // offset from this to menace, that distance, unit vector toward menace
    const Vec3 offset = menace.position - position;
    const float distance = offset.length ();

    const float roughTime = distance / menace.speed();
    const float predictionTime = ((roughTime > maxPredictionTime) ?
                                  maxPredictionTime :
                                  roughTime);

    const Vec3 target = menace.predictFuturePosition (predictionTime);

    return steerForFlee (target);
}


// ----------------------------------------------------------------------------
// tries to maintain a given speed, returns a maxForce-clipped steering
// force along the forward/backward axis


template<class Super>
Vec3
SteerLibraryMixin<Super>::
steerForTargetSpeed (const float targetSpeed)
{
    float speedError = targetSpeed - speed ();

    if (speedError > +maxForce ()) speedError = +maxForce ();
    if (speedError < -maxForce ()) speedError = -maxForce ();

    return forward () * speedError;
}



// ----------------------------------------------------------------------------
// xxx experiment cwr 9-6-02


template<class Super>
void
SteerLibraryMixin<Super>::
findNextIntersectionWithSphere (SphericalObstacle& obs,
                                PathIntersection& intersection)
{
    // xxx"SphericalObstacle& obs" should be "const SphericalObstacle&
    // obs" but then it won't let me store a pointer to in inside the
    // PathIntersection

    // This routine is based on the Paul Bourke's derivation in:
    //   Intersection of a Line and a Sphere (or circle)
    //   http://www.swin.edu.au/astronomy/pbourke/geometry/sphereline/

    float b, c, d, p, q, s;
    Vec3 lc;

    // initialize pathIntersection object
    intersection.intersect = false;
    intersection.obstacle = &obs;

    // find "local center" (lc) of sphere in boid's coordinate space
    lc = localizePosition (obs.center);

    // computer line-sphere intersection parameters
    b = -2 * lc.z;
    c = square (lc.x) + square (lc.y) + square (lc.z) - 
        square (obs.radius + radius());
    d = (b * b) - (4 * c);

    // when the path does not intersect the sphere
    if (d < 0) return;

    // otherwise, the path intersects the sphere in two points with
    // parametric coordinates of "p" and "q".
    // (If "d" is zero the two points are coincident, the path is tangent)
    s = sqrtXXX (d);
    p = (-b + s) / 2;
    q = (-b - s) / 2;

    // both intersections are behind us, so no potential collisions
    if ((p < 0) && (q < 0)) return; 

    // at least one intersection is in front of us
    intersection.intersect = true;
    intersection.distance =
        ((p > 0) && (q > 0)) ?
        // both intersections are in front of us, find nearest one
        ((p < q) ? p : q) :
        // otherwise only one intersections is in front, select it
        ((p > 0) ? p : q);
    return;
}


// ----------------------------------------------------------------------------


template<class Super>
void
SteerLibraryMixin<Super>::
annotationForPathFollowing (const Vec3& futurePosition,
                            const Vec3& onPath,
                            const bool withinPath)
{
    const Vec3 yellow (1, 1, 0);
    const Vec3 lightOrange (1.0f, 0.5f, 0.0f);
    const Vec3 darkOrange  (0.5f, 0.2f, 0.0f);
    const Vec3 orange (withinPath ? darkOrange : lightOrange);

    annotationLine (futurePosition, position(), yellow);
    annotationLine (futurePosition, onPath, orange);
}


// ----------------------------------------------------------------------------


template<class Super>
void
SteerLibraryMixin<Super>::
annotationForAvoidSphere (const float minDistanceToCollision)
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


// ----------------------------------------------------------------------------


template<class Super>
void
SteerLibraryMixin<Super>::
annotationForAvoidNeighbor (const AbstractVehicle& threat,
                            const float steer,
                            const Vec3& ourFuture,
                            const Vec3& threatFuture)
{
    const Vec3 red (1,0,0);
    const Vec3 cyan (0,1,1);
    const Vec3 white (1,1,1);
    const Vec3 magenta (1,0,1);
    annotationLine (position(), threat.position(), magenta);
    annotationLine (position(), position() + (side() * steer * 3), cyan);
    annotationLine (position(), ourFuture, white);
    annotationLine (threat.position(), threatFuture, white);
    annotationLine (ourFuture, threatFuture, red);
}


// ----------------------------------------------------------------------------
// XXX This is misplaced, this annotation is specific to the Pedestrian PlugIn
// XXX but it need to be called from inside the steering behavior.  This all
// XXX needs to be redesigned.


template<class Super>
void
SteerLibraryMixin<Super>::
annotationForAvoidCloseNeighbor (const AbstractVehicle& other,
                                 const float additionalDistance)
{
    // draw the word "Ouch!" above colliding vehicles
    const float headOn = forward().dot(other.forward()) < 0;
    const Vec3 green (0.4f, 0.8f, 0.1f);
    const Vec3 red (1, 0.1f, 0);
    const Vec3 color = headOn ? red : green;
    const char* string = headOn ? "OUCH!" : "pardon me";
    const Vec3 location = position() + Vec3 (0, 0.5f, 0);
    draw2dTextAt3dLocation (*string, location, color);
}


// ----------------------------------------------------------------------------
#endif // OPENSTEER_STEERLIBRARY_H
