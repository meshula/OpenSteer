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
// Capture the Flag   (a portion of the traditional game)
//
// The "Capture the Flag" sample steering problem, proposed by Marcin
// Chady of the Working Group on Steering of the IGDA's AI Interface
// Standards Committee (http://www.igda.org/Committees/ai.htm) in this
// message (http://sourceforge.net/forum/message.php?msg_id=1642243):
//
//     "An agent is trying to reach a physical location while trying
//     to stay clear of a group of enemies who are actively seeking
//     him. The environment is littered with obstacles, so collision
//     avoidance is also necessary."
//
// Note that the enemies do not make use of their knowledge of the 
// seeker's goal by "guarding" it.  
//
// XXX hmm, rename them "attacker" and "defender"?
//
// 08-12-02 cwr: created 
//
//
// ----------------------------------------------------------------------------


#include <iomanip>
#include <sstream>
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/SteerTest.h"


// ----------------------------------------------------------------------------
// This PlugIn uses two vehicle types: CtfSeeker and CtfEnemy.  They have a
// common base class: CtfBase which is a specialization of SimpleVehicle.


class CtfBase : public SimpleVehicle
{
public:
    // constructor
    CtfBase () {reset ();}

    // reset state
    void reset (void);

    // draw this character/vehicle into the scene
    void draw (void);

    void randomizeStartingPositionAndHeading (void);
    enum seekerState {running, tagged, atGoal};

    Vec3 steerToAvoidAllObstacles (const float minTimeToCollision,
                                   bool& potentialCollision);

    Vec3 bodyColor;    // for draw method

    bool avoiding; // xxx store steer sub-state for anotation

    // dynamic obstacle registry
    static void addOneObstacle (void);
    static void removeOneObstacle (void);
    float minDistanceToObstacle (const Vec3 point);
    static int obstacleCount;
    static const int maxObstacleCount;
    static SphericalObstacle obstacles [];

    void drawHomeBase (void);
};


class CtfSeeker : public CtfBase
{
public:

    // constructor
    CtfSeeker () {reset ();}

    // reset state
    void reset (void);

    // per frame simulation update
    void update (const float currentTime, const float elapsedTime);

    // is there a clear path to the goal?
    bool clearPathToGoal (void);

    Vec3 steeringForSeeker (void);
    void updateState (const float currentTime);
    void draw (void);
    Vec3 steerToEvadeAllDefenders (void);
    Vec3 XXXsteerToEvadeAllDefenders (void);
    void adjustObstacleAvoidanceLookAhead (const bool clearPath);
    void clearPathAnnotation (const float threshold,
                              const float behindcThreshold,
                              const Vec3& goalDirection);

    seekerState state;
    bool evading; // xxx store steer sub-state for anotation
    float lastRunningTime; // for auto-reset
};


class CtfEnemy : public CtfBase
{
public:

    // constructor
    CtfEnemy () {reset ();}

    // reset state
    void reset (void);

    // per frame simulation update
    void update (const float currentTime, const float elapsedTime);
};


// ----------------------------------------------------------------------------
// globals
// (perhaps these should be member variables of a Vehicle or PlugIn class)


const int CtfBase::maxObstacleCount = 100;

const Vec3 gHomeBaseCenter (0, 0, 0);
const float gHomeBaseRadius = 1.5;

const float gMinStartRadius = 30;
const float gMaxStartRadius = 40;

const float gBrakingRate = 0.75;

const Vec3 evadeColor     (0.6f, 0.6f, 0.3f); // annotation
const Vec3 seekColor      (0.3f, 0.6f, 0.6f); // annotation
const Vec3 clearPathColor (0.3f, 0.6f, 0.3f); // annotation

const float gAvoidancePredictTimeMin  = 0.9f;
const float gAvoidancePredictTimeMax  = 2;
float gAvoidancePredictTime = gAvoidancePredictTimeMin;

bool enableAttackSeek  = true; // for testing (perhaps retain for UI control?)
bool enableAttackEvade = true; // for testing (perhaps retain for UI control?)

CtfSeeker* gSeeker = NULL;


// count the number of times the simulation has reset (e.g. for overnight runs)
int resetCount = 0;


// ----------------------------------------------------------------------------
// state for SteerTest PlugIn
//
// XXX consider moving this inside CtfPlugIn
// XXX consider using STL (any advantage? consistency?)


CtfSeeker* ctfSeeker;
const int ctfEnemyCount = 4;
CtfEnemy* ctfEnemies [ctfEnemyCount];


// ----------------------------------------------------------------------------
// reset state


void CtfBase::reset (void)
{
    SimpleVehicle::reset ();  // reset the vehicle 

    setSpeed (3);             // speed along Forward direction.
    setMaxForce (3.0);        // steering force is clipped to this magnitude
    setMaxSpeed (3.0);        // velocity is clipped to this magnitude

    avoiding = false;         // not actively avoiding

    randomizeStartingPositionAndHeading ();  // new starting position

    clearTrailHistory ();     // prevent long streaks due to teleportation
}


void CtfSeeker::reset (void)
{
    CtfBase::reset ();
    bodyColor.set (0.4f, 0.4f, 0.6f); // blueish
    gSeeker = this;
    state = running;
    evading = false;
}


void CtfEnemy::reset (void)
{
    CtfBase::reset ();
    bodyColor.set (0.6f, 0.4f, 0.4f); // redish
}


// ----------------------------------------------------------------------------
// draw this character/vehicle into the scene


void CtfBase::draw (void)
{
    drawBasic2dCircularVehicle (*this, bodyColor);
    drawTrail ();
}


// ----------------------------------------------------------------------------


void CtfBase::randomizeStartingPositionAndHeading (void)
{
    // randomize position on a ring between inner and outer radii
    // centered around the home base
    const float rRadius = frandom2 (gMinStartRadius, gMaxStartRadius);
    const Vec3 randomOnRing = RandomUnitVectorOnXZPlane () * rRadius;
    setPosition (gHomeBaseCenter + randomOnRing);

    // are we are too close to an obstacle?
    if (minDistanceToObstacle (position()) < radius()*5)
    {
        // if so, retry the randomization (this recursive call may not return
        // if there is too little free space)
        randomizeStartingPositionAndHeading ();
    }
    else
    {
        // otherwise, if the position is OK, randomize 2D heading
        randomizeHeadingOnXZPlane ();
    }
}


// ----------------------------------------------------------------------------


void CtfEnemy::update (const float currentTime, const float elapsedTime)
{
    // determine upper bound for pursuit prediction time
    const float seekerToGoalDist = Vec3::distance (gHomeBaseCenter,
                                                   gSeeker->position());
    const float adjustedDistance = seekerToGoalDist - radius()-gHomeBaseRadius;
    const float seekerToGoalTime = MAX (0, adjustedDistance) /gSeeker->speed();
    const float maxPredictionTime = seekerToGoalTime * 0.9;

    // determine steering (pursuit, obstacle avoidance, or braking)
    Vec3 steer (0, 0, 0);
    if (gSeeker->state == running)
    {
        bool needToAvoid;
        Vec3 avoidance =
            steerToAvoidAllObstacles (gAvoidancePredictTimeMin, needToAvoid);
        steer = (needToAvoid ?
                 avoidance :
                 steerForPursuit (*gSeeker, maxPredictionTime));
    }
    else
    {
        applyBrakingForce (gBrakingRate, elapsedTime);
    }
    applySteeringForce (steer, elapsedTime);

    // annotation
    annotationVelocityAcceleration ();
    recordTrailVertex (currentTime, position());


    // detect and record interceptions ("tags") of seeker
    const float seekerToMeDist = Vec3::distance (position(), 
                                                 gSeeker->position());
    const float sumOfRadii = radius() + gSeeker->radius();
    if (seekerToMeDist < sumOfRadii)
    {
        if (gSeeker->state == running) gSeeker->state = tagged;

        // annotation:
        if (gSeeker->state == tagged)
        {
            const Vec3 color (0.8f, 0.5f, 0.5f);
            annotationXZDisk (sumOfRadii,
                        (position() + gSeeker->position()) / 2,
                        color,
                        20);
        }
    }
}


// ----------------------------------------------------------------------------
// are there any enemies along the corridor between us and the goal?


bool CtfSeeker::clearPathToGoal (void)
{
    const float sideThreshold = radius() * 8.0;
    const float behindThreshold = radius() * 2;

    const Vec3 goalOffset = gHomeBaseCenter - position();
    const float goalDistance = goalOffset.length ();
    const Vec3 goalDirection = goalOffset / goalDistance;

    const bool goalIsAside = isAside (gHomeBaseCenter, 0.5);

    // for annotation: loop over all and save result, instead of early return 
    bool xxxReturn = true;

    // loop over enemies
    for (int i = 0; i < ctfEnemyCount; i++)
    {
        // short name for this enemy
        const CtfEnemy& e = *ctfEnemies[i];
        const float eDistance = Vec3::distance (position(), e.position());
        const float timeEstimate = 0.3 * eDistance / e.speed(); //xxx
        const Vec3 eFuture = e.predictFuturePosition (timeEstimate);
        const Vec3 eOffset = eFuture - position();
        const float alongCorridor = goalDirection.dot (eOffset);
        const bool inCorridor = ((alongCorridor > -behindThreshold) && 
                                 (alongCorridor < goalDistance));
        const float eForwardDistance = forward().dot (eOffset);

        // xxx temp move this up before the conditionals
        annotationXZCircle (e.radius(), eFuture, clearPathColor, 20); //xxx

        // consider as potential blocker if within the corridor
        if (inCorridor)
        {
            const Vec3 perp = eOffset - (goalDirection * alongCorridor);
            const float acrossCorridor = perp.length();
            if (acrossCorridor < sideThreshold)
            {
                // not a blocker if behind us and we are perp to corridor
                const float eFront = eForwardDistance + e.radius ();

                //annotationLine (position, forward*eFront, gGreen); // xxx
                //annotationLine (e.position, forward*eFront, gGreen); // xxx

                // xxx
                // std::ostringstream message;
                // message << "eFront = " << std::setprecision(2)
                //         << std::setiosflags(std::ios::fixed) << eFront << std::ends;
                // draw2dTextAt3dLocation (*message.str(), eFuture, gWhite);

                const bool eIsBehind = eFront < -behindThreshold;
                const bool eIsWayBehind = eFront < (-2 * behindThreshold);
                const bool safeToTurnTowardsGoal =
                    ((eIsBehind && goalIsAside) || eIsWayBehind);

                if (! safeToTurnTowardsGoal)
                {
                    // this enemy blocks the path to the goal, so return false
                    annotationLine (position(), e.position(), clearPathColor);
                    // return false;
                    xxxReturn = false;
                }
            }
        }
    }

    // no enemies found along path, return true to indicate path is clear
    // clearPathAnnotation (sideThreshold, behindThreshold, goalDirection);
    // return true;
    //if (xxxReturn)
        clearPathAnnotation (sideThreshold, behindThreshold, goalDirection);
    return xxxReturn;
}


// ----------------------------------------------------------------------------


void CtfSeeker::clearPathAnnotation (const float sideThreshold,
                                     const float behindThreshold,
                                     const Vec3& goalDirection)
{
    const Vec3 behindSide = side() * sideThreshold;
    const Vec3 behindBack = forward() * -behindThreshold;
    const Vec3 pbb = position() + behindBack;
    const Vec3 gun = localRotateForwardToSide (goalDirection);
    const Vec3 gn = gun * sideThreshold;
    const Vec3 hbc = gHomeBaseCenter;
    annotationLine (pbb + gn,         hbc + gn,         clearPathColor);
    annotationLine (pbb - gn,         hbc - gn,         clearPathColor);
    annotationLine (hbc - gn,         hbc + gn,         clearPathColor);
    annotationLine (pbb - behindSide, pbb + behindSide, clearPathColor);
}


// ----------------------------------------------------------------------------


Vec3 CtfBase::steerToAvoidAllObstacles (const float minTimeToCollision,
                                        bool& potentialCollision)
{
    Vec3 avoidance (0, 0, 0);
    PathIntersection nearest, next;
    const float minDistanceToCollision = minTimeToCollision * speed();

    avoiding = false;
    next.intersect = false;
    nearest.intersect = false;
    potentialCollision = false;

    // test all obstacles for intersection with my forward axis,
    // select the one whose point of intersection is nearest
    for (int i = 0; i < obstacleCount; i++)
    {
        // xxx this should be a generic call on Obstacle, rather than this
        // xxx code which presumes the obstacle is spherical
        findNextIntersectionWithSphere (obstacles[i], next);

        if ((nearest.intersect == false) ||
            ((next.intersect != false) &&
             (next.distance < nearest.distance)))
            nearest = next;
    }

    // when a nearest intersection was found
    if ((nearest.intersect != false) &&
        (nearest.distance < minDistanceToCollision))
    {
        // set the return argument to indicate collision avoidance is needed
        potentialCollision = true;

        // save state for annotation
        avoiding = true;

        // show the corridor being checked for collisions
        annotationForAvoidSphere (minDistanceToCollision);

        // compute avoidance steering force: take offset from obstacle to me,
        // take the component of that which is lateral (perpendicular to my
        // forward direction), set length to maxForce, add a bit of forward
        // component (in capture the flag, we never want to slow down)
        const Vec3 offset = position() - nearest.obstacle->center;
        avoidance = offset.perpendicularComponent (forward());
        avoidance = avoidance.normalize ();
        avoidance *= maxForce ();
        avoidance += forward() * maxForce () * 0.75;
    }

    return avoidance;
}


// ----------------------------------------------------------------------------


Vec3 CtfSeeker::steerToEvadeAllDefenders (void)
{
    Vec3 evade (0, 0, 0);
    const float goalDistance = Vec3::distance (gHomeBaseCenter, position());

    // sum up weighted evasion
    for (int i = 0; i < ctfEnemyCount; i++)
    {
        const CtfEnemy& e = *ctfEnemies[i];
        const Vec3 eOffset = e.position() - position();
        const float eDistance = eOffset.length();

        const float eForwardDistance = forward().dot (eOffset);
        const float behindThreshold = radius() * 2;
        const bool behind = eForwardDistance < behindThreshold;
        if ((!behind) || (eDistance < 5))
        {
            if (eDistance < (goalDistance * 1.2)) //xxx
            {
                // const float timeEstimate = 0.5 * eDistance / e.speed;//xxx
                const float timeEstimate = 0.15 * eDistance / e.speed();//xxx
                const Vec3 future =
                    e.predictFuturePosition (timeEstimate);

                annotationXZCircle (e.radius(), future, evadeColor, 20); // xxx

                const Vec3 offset = future - position();
                const Vec3 lateral = offset.perpendicularComponent (forward());
                const float d = lateral.length();
                const float weight = -1000 / (d * d);
                evade += (lateral / d) * weight;
            }
        }
    }
    return evade;
}


Vec3 CtfSeeker::XXXsteerToEvadeAllDefenders (void)
{
    // sum up weighted evasion
    Vec3 evade (0, 0, 0);
    for (int i = 0; i < ctfEnemyCount; i++)
    {
        const CtfEnemy& e = *ctfEnemies[i];
        const Vec3 eOffset = e.position() - position();
        const float eDistance = eOffset.length();

        // xxx maybe this should take into account e's heading? xxx
        const float timeEstimate = 0.5 * eDistance / e.speed(); //xxx
        const Vec3 eFuture = e.predictFuturePosition (timeEstimate);

        // annotation
        annotationXZCircle (e.radius(), eFuture, evadeColor, 20);

        // steering to flee from eFuture (enemy's future position)
        const Vec3 flee = xxxsteerForFlee (eFuture);

        const float eForwardDistance = forward().dot (eOffset);
        const float behindThreshold = radius() * -2;

        const float distanceWeight = 4 / eDistance;
        const float forwardWeight = ((eForwardDistance > behindThreshold) ?
                                     1 : 0.5);

        const Vec3 adjustedFlee = flee * distanceWeight * forwardWeight;

        evade += adjustedFlee;
    }
    return evade;
}


// ----------------------------------------------------------------------------


Vec3 CtfSeeker::steeringForSeeker (void)
{
    // determine if obstacle avodiance is needed
    bool needed;
    const float pt = gAvoidancePredictTime;
    const bool clearPath = clearPathToGoal ();
    adjustObstacleAvoidanceLookAhead (clearPath);
    const Vec3 obstacleAvoidance = steerToAvoidAllObstacles (pt, needed);

    if (needed)
    {
        // use pure obstacle avoidance if needed
        return obstacleAvoidance;
    }
    else
    {
        // otherwise seek home base and perhaps evade defenders
        const Vec3 seek = xxxsteerForSeek (gHomeBaseCenter);
        if (clearPath)
        {
            // we have a clear path (defender-free corridor), use pure seek

            // xxx experiment 9-16-02
            Vec3 s = limitMaxDeviationAngle (seek, 0.707f, forward());

            annotationLine (position(), position() + (s * 0.2f), seekColor);
            return s;
        }
        else
        {
            if (0) // xxx testing new evade code xxx
            {
                // combine seek and (forward facing portion of) evasion
                const Vec3 evade = steerToEvadeAllDefenders ();
                const Vec3 steer = 
                    seek + limitMaxDeviationAngle (evade, 0.5f, forward());

                // annotation: show evasion steering force
                annotationLine (position(),position()+(steer*0.2f),evadeColor);
                return steer;
            }
            else

            {
                const Vec3 evade = XXXsteerToEvadeAllDefenders ();
                const Vec3 steer = limitMaxDeviationAngle (seek + evade,
                                                           0.707f, forward());

                annotationLine (position(),position()+seek, gRed);
                annotationLine (position(),position()+evade, gGreen);

                // annotation: show evasion steering force
                annotationLine (position(),position()+(steer*0.2f),evadeColor);
                return steer;
            }
        }
    }
}


// ----------------------------------------------------------------------------
// adjust obstacle avoidance look ahead time: make it large when we are far
// from the goal and heading directly towards it, make it small otherwise.


void CtfSeeker::adjustObstacleAvoidanceLookAhead (const bool clearPath)
{
    if (clearPath)
    {
        evading = false;
        const float goalDistance = Vec3::distance (gHomeBaseCenter,position());
        const bool headingTowardGoal = isAhead (gHomeBaseCenter, 0.98f);
        const bool isNear = (goalDistance/speed()) < gAvoidancePredictTimeMax;
        const bool useMax = headingTowardGoal && !isNear;
        gAvoidancePredictTime =
            (useMax ? gAvoidancePredictTimeMax : gAvoidancePredictTimeMin);
    }
    else
    {
        evading = true;
        gAvoidancePredictTime = gAvoidancePredictTimeMin;
    }
}


// ----------------------------------------------------------------------------


void CtfSeeker::updateState (const float currentTime)
{
    // if we reach the goal before being tagged, switch to atGoal state
    if (state == running)
    {
        const float baseDistance = Vec3::distance (position(),gHomeBaseCenter);
        if (baseDistance < (radius() + gHomeBaseRadius)) state = atGoal;
    }

    // update lastRunningTime (holds off reset time)
    if (state == running)
    {
        lastRunningTime = currentTime;
    }
    else
    {
        const float resetDelay = 4;
        const float resetTime = lastRunningTime + resetDelay;
        if (currentTime > resetTime) 
        {
            // xxx a royal hack (should do this internal to CTF):
            SteerTest::queueDelayedResetPlugInXXX ();
        }
    }
}


// ----------------------------------------------------------------------------


void CtfSeeker::draw (void)
{
    // first call the draw method in the base class
    CtfBase::draw();

    // select string describing current seeker state
    char* seekerStateString = "";
    switch (state)
    {
    case running:
        if (avoiding)
            seekerStateString = "avoid obstacle";
        else if (evading)
            seekerStateString = "seek and evade";
        else
            seekerStateString = "seek goal";
        break;
    case tagged: seekerStateString = "tagged"; break;
    case atGoal: seekerStateString = "reached goal"; break;
    }

    // annote seeker with its state as text
    const Vec3 textOrigin = position() + Vec3 (0, 0.25, 0);
    std::ostringstream annote;
    annote << seekerStateString << std::endl;
    annote << std::setprecision(2) << std::setiosflags(std::ios::fixed)
           << speed() << std::ends;
    draw2dTextAt3dLocation (annote, textOrigin, gWhite);

    // display status in the upper left corner of the window
    std::ostringstream status;
    status << seekerStateString << std::endl;
    status << obstacleCount << " obstacles" << std::endl;
    status << resetCount << " restarts" << std::ends;
    const float h = drawGetWindowHeight ();
    const Vec3 screenLocation (10, h-50, 0);
    draw2dTextAt2dLocation (status, screenLocation, gGray80);
}


// ----------------------------------------------------------------------------
// update method for goal seeker


void CtfSeeker::update (const float currentTime, const float elapsedTime)
{
    // do behavioral state transitions, as needed
    updateState (currentTime);

    // determine and apply steering/braking forces
    Vec3 steer (0, 0, 0);
    if (state == running)
    {
        steer = steeringForSeeker ();
    }
    else
    {
        applyBrakingForce (gBrakingRate, elapsedTime);
    }
    applySteeringForce (steer, elapsedTime);


    // annotation
    annotationVelocityAcceleration ();
    recordTrailVertex (currentTime, position());
}


// ----------------------------------------------------------------------------
// dynamic obstacle registry
//
// xxx need to combine guts of addOneObstacle and minDistanceToObstacle,
// xxx perhaps by having the former call the latter, or change the latter to
// xxx be "nearestObstacle": give it a position, it finds the nearest obstacle
// xxx (but remember: obstacles a not necessarilty spheres!)


int CtfBase::obstacleCount = 0;
SphericalObstacle CtfBase::obstacles [maxObstacleCount];

#define testOneObstacleOverlap(radius, center)               \
{                                                            \
    float d = Vec3::distance (c, center);                    \
    float clearance = d - (r + (radius));                    \
    if (minClearance > clearance) minClearance = clearance;  \
}


void CtfBase::addOneObstacle (void)
{
    if (obstacleCount < maxObstacleCount)
    {
        // pick a random center and radius,
        // loop until no overlap with other obstacles and the home base
        float r;
        Vec3 c;
        float minClearance;
        const float requiredClearance = gSeeker->radius() * 4; // 2 x diameter
        do
        {
            r = frandom2 (1.5, 4);
            c = randomVectorOnUnitRadiusXZDisk () * gMaxStartRadius * 1.1f;
            minClearance = FLT_MAX;

            for (int i = 0; i < obstacleCount; i++)
            {
                testOneObstacleOverlap (obstacles[i].radius,
                                        obstacles[i].center);
            }
            testOneObstacleOverlap (gHomeBaseRadius - requiredClearance,
                                    gHomeBaseCenter);
        }
        while (minClearance < requiredClearance);

        // add new non-overlapping obstacle to registry
        obstacles[obstacleCount].radius = r;
        obstacles[obstacleCount].center = c;
        obstacleCount++;
    }
}


float CtfBase::minDistanceToObstacle (const Vec3 point)
{
    float r = 0;
    Vec3 c = point;
    float minClearance = FLT_MAX;
    for (int i = 0; i < obstacleCount; i++)
    {
        testOneObstacleOverlap (obstacles[i].radius,
                                obstacles[i].center);
    }
    return minClearance;
}


void CtfBase::removeOneObstacle (void)
{
    if (obstacleCount > 0) obstacleCount--;
}


// ----------------------------------------------------------------------------
// PlugIn for SteerTest


class CtfPlugIn : public PlugIn
{
public:

    const char* name (void) {return "Capture the Flag";}

    float selectionOrderSortKey (void) {return 0.01f;}

    virtual ~CtfPlugIn() {} // be more "nice" to avoid a compiler warning

    void open (void)
    {
        // create the seeker ("hero"/"attacker")
        ctfSeeker = new CtfSeeker;
        all.push_back (ctfSeeker);

        // create the specified number of enemies, 
        // storing pointers to them in an array.
        for (int i = 0; i<ctfEnemyCount; i++)
        {
            ctfEnemies[i] = new CtfEnemy;
            all.push_back (ctfEnemies[i]);
        }

        // initialize camera
        SteerTest::init2dCamera (*ctfSeeker);
        SteerTest::camera.mode = Camera::cmFixedDistanceOffset;
        SteerTest::camera.fixedTarget.set (15, 0, 0);
        SteerTest::camera.fixedPosition.set (80, 60, 0);
    }

    void update (const float currentTime, const float elapsedTime)
    {
        // update the seeker
        ctfSeeker->update (currentTime, elapsedTime);
      
        // update each enemy
        for (int i = 0; i < ctfEnemyCount; i++)
        {
            ctfEnemies[i]->update (currentTime, elapsedTime);
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

        // draw "ground plane" centered between base and selected vehicle
        const Vec3 goalOffset = gHomeBaseCenter-SteerTest::camera.position();
        const Vec3 goalDirection = goalOffset.normalize ();
        const Vec3 cameraForward = SteerTest::camera.xxxls().forward();
        const float goalDot = cameraForward.dot (goalDirection);
        const float blend = remapIntervalClip (goalDot, 1, 0, 0.5, 0);
        const Vec3 gridCenter = interpolate (blend,
                                             selected.position(),
                                             gHomeBaseCenter);
        SteerTest::gridUtility (gridCenter);

        // draw the seeker, obstacles and home base
        ctfSeeker->draw();
        drawObstacles ();
        drawHomeBase();

        // draw each enemy
        for (int i = 0; i < ctfEnemyCount; i++) ctfEnemies[i]->draw ();

        // highlight vehicle nearest mouse
        SteerTest::highlightVehicleUtility (nearMouse);
    }

    void close (void)
    {
        // delete seeker
        delete (ctfSeeker);
        ctfSeeker = NULL;

        // delete each enemy
        for (int i = 0; i < ctfEnemyCount; i++)
        {
            delete (ctfEnemies[i]);
            ctfEnemies[i] = NULL;
        }

        // clear the group of all vehicles
        all.clear();
    }

    void reset (void)
    {
        // count resets
        resetCount++;

        // reset the seeker ("hero"/"attacker") and enemies
        ctfSeeker->reset ();
        for (int i = 0; i<ctfEnemyCount; i++) ctfEnemies[i]->reset ();

        // reset camera position
        SteerTest::position2dCamera (*ctfSeeker);

        // make camera jump immediately to new position
        SteerTest::camera.doNotSmoothNextMove ();
    }

    void handleFunctionKeys (int keyNumber)
    {
        switch (keyNumber)
        {
        case 1: CtfBase::addOneObstacle ();    break;
        case 2: CtfBase::removeOneObstacle (); break;
        }
    }

    void printMiniHelpForFunctionKeys (void)
    {
        std::ostringstream message;
        message << "Function keys handled by ";
        message << '"' << name() << '"' << ':' << std::ends;
        SteerTest::printMessage (message);
        SteerTest::printMessage ("  F1     add one obstacle.");
        SteerTest::printMessage ("  F2     remove one obstacle.");
        SteerTest::printMessage ("");
    }

    const AVGroup& allVehicles (void) {return (const AVGroup&) all;}

    void drawHomeBase (void)
    {
        const Vec3 up (0, 0.01f, 0);
        const Vec3 atColor (0.3f, 0.3f, 0.5f);
        const Vec3 noColor = gGray50;
        const bool reached = ctfSeeker->state == CtfSeeker::atGoal;
        const Vec3 baseColor = (reached ? atColor : noColor);
        drawXZDisk (gHomeBaseRadius,    gHomeBaseCenter, baseColor, 40);
        drawXZDisk (gHomeBaseRadius/15, gHomeBaseCenter+up, gBlack, 20);
    }

    void drawObstacles (void)
    {
        for (int i = 0; i < CtfBase::obstacleCount; i++)
        {
            const Vec3 color (0.8f, 0.6f, 0.4f);
            drawXZCircle (CtfBase::obstacles[i].radius,
                          CtfBase::obstacles[i].center,
                          color,
                          40);
        }
    }

    // a group (STL vector) of all vehicles in the PlugIn
    std::vector<CtfBase*> all;
};


CtfPlugIn gCtfPlugIn;


// ----------------------------------------------------------------------------
