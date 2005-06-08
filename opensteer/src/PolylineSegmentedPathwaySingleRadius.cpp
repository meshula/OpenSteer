#include "OpenSteer/PolylineSegmentedPathwaySingleRadius.h"


// Include std::swap
#include <algorithm>


// Include OpenSteer::mapPointToPathway, OpenSteer::mapDistanceToPointOnPathCenterLine
#include "OpenSteer/QueryPathAlike.h"

// Include OpenSteer::PointToPathMapping, OpenSteer::PathDistanceToPointMapping, OpenSteer::PointToPathDistanceMapping
#include "OpenSteer/QueryPathAlikeMappings.h"

// Include OpenSteer::HasSingleRadius
#include "OpenSteer/QueryPathAlikeUtilities.h"

// Include OPENSTEER_UNUSED_PARAMETER
#include "OpenSteer/UnusedParameter.h"


OpenSteer::PolylineSegmentedPathwaySingleRadius::PolylineSegmentedPathwaySingleRadius()
    : path_(), radius_ ( 0.0f )
{
    
}



OpenSteer::PolylineSegmentedPathwaySingleRadius::PolylineSegmentedPathwaySingleRadius( float r )
    : path_(), radius_( r )
{
    
}



OpenSteer::PolylineSegmentedPathwaySingleRadius::PolylineSegmentedPathwaySingleRadius( size_type numOfPoints,
                                                                                       Vec3 const points[],
                                                                                       float r,
                                                                                       bool closeCycle )
    : path_( numOfPoints, points, closeCycle ), radius_( r )
{
    
}



OpenSteer::PolylineSegmentedPathwaySingleRadius::PolylineSegmentedPathwaySingleRadius( PolylineSegmentedPathwaySingleRadius const& other )
    : path_( other.path_ ), radius_( other.radius_ )
{
    
}



OpenSteer::PolylineSegmentedPathwaySingleRadius::~PolylineSegmentedPathwaySingleRadius()
{
    // Nothing to do.
}



OpenSteer::PolylineSegmentedPathwaySingleRadius& 
OpenSteer::PolylineSegmentedPathwaySingleRadius::operator=( PolylineSegmentedPathwaySingleRadius other )
{
    swap( other );
    return *this;
}




void 
OpenSteer::PolylineSegmentedPathwaySingleRadius::swap( PolylineSegmentedPathwaySingleRadius& other )
{
    path_.swap( other.path_ );
    std::swap( radius_, other.radius_ );
}




void 
OpenSteer::PolylineSegmentedPathwaySingleRadius::movePoints( size_type startIndex,
                                                             size_type numOfPoints,
                                                             Vec3 const newPointValues[] )
{
    path_.movePoints( startIndex, numOfPoints, newPointValues );
}




void 
OpenSteer::PolylineSegmentedPathwaySingleRadius::setPathway( size_type numOfPoints,
                                                             Vec3 const points[],
                                                             float r,
                                                             bool closedCycle )
{
    path_.setPath( numOfPoints, points, closedCycle );
    setRadius( r );
}




void 
OpenSteer::PolylineSegmentedPathwaySingleRadius::setRadius( float r )
{
    radius_ = r;
}



float 
OpenSteer::PolylineSegmentedPathwaySingleRadius::radius() const
{
    return radius_;
}



bool
OpenSteer::PolylineSegmentedPathwaySingleRadius::isValid() const 
{
    return pointCount() > 1;
}



OpenSteer::Vec3 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapPointToPath (const Vec3& point,
                                                                 Vec3& tangent,
                                                                 float& outside) const
{
    PointToPathMapping< PolylineSegmentedPathwaySingleRadius, HasSingleRadius > mapping;
    mapPointToPathway( *this, point, mapping );
    tangent = mapping.tangent;
    outside = mapping.distancePointToPath;
    return mapping.pointOnPathBoundary;
}



OpenSteer::Vec3 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapPathDistanceToPoint (float pathDistance) const
{
    PathDistanceToPointMapping< PolylineSegmentedPathwaySingleRadius, HasSingleRadius > mapping;
    mapDistanceToPointOnPathCenterLine( *this, pathDistance, mapping );
    return mapping.pointOnPathCenterLine;
}



float 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapPointToPathDistance (const Vec3& point) const
{
    PointToPathDistanceMapping< PolylineSegmentedPathwaySingleRadius, HasSingleRadius > mapping;
    mapPointToPathway( *this, point, mapping );
    return mapping.distanceOnPath;
}



bool 
OpenSteer::PolylineSegmentedPathwaySingleRadius::isCyclic() const
{
    return path_.isCyclic();
}



float 
OpenSteer::PolylineSegmentedPathwaySingleRadius::length() const
{
    return path_.length();
}



OpenSteer::SegmentedPathway::size_type 
OpenSteer::PolylineSegmentedPathwaySingleRadius::pointCount() const 
{
    return path_.pointCount();
}



OpenSteer::Vec3 
OpenSteer::PolylineSegmentedPathwaySingleRadius::point( size_type pointIndex ) const
{
    return path_.point( pointIndex );
}




OpenSteer::PolylineSegmentedPathwaySingleRadius::size_type 
OpenSteer::PolylineSegmentedPathwaySingleRadius::segmentCount() const
{
    return path_.segmentCount();
}



float 
OpenSteer::PolylineSegmentedPathwaySingleRadius::segmentLength( size_type segmentIndex ) const
{
    return path_.segmentLength( segmentIndex );
}



OpenSteer::Vec3 
OpenSteer::PolylineSegmentedPathwaySingleRadius::segmentStart( size_type segmentIndex ) const
{
    return path_.segmentStart( segmentIndex );
}



OpenSteer::Vec3 
OpenSteer::PolylineSegmentedPathwaySingleRadius::segmentEnd( size_type segmentIndex ) const
{
    return path_.segmentEnd( segmentIndex );
}



float 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapPointToSegmentDistance( size_type segmentIndex, 
                                                                            Vec3 const& point ) const
{
    return path_.mapPointToSegmentDistance( segmentIndex, point );
}



OpenSteer::Vec3 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapSegmentDistanceToPoint( size_type segmentIndex, 
                                                                            float segmentDistance ) const
{
    return path_.mapSegmentDistanceToPoint( segmentIndex, segmentDistance );
}



float 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapSegmentDistanceToRadius( size_type segmentIndex, 
                                                                             float distanceOnSegment ) const
{
    OPENSTEER_UNUSED_PARAMETER(segmentIndex);
    OPENSTEER_UNUSED_PARAMETER(distanceOnSegment);
    return radius_;
}



OpenSteer::Vec3 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapSegmentDistanceToTangent( size_type segmentIndex, 
                                                                              float segmentDistance ) const
{
    return path_.mapSegmentDistanceToTangent( segmentIndex, segmentDistance );
}


/*
void 
OpenSteer::PolylineSegmentedPathwaySingleRadius::mapPointToSegmentDistanceAndPointAndTangentAndRadius( size_type segmentIndex,
                                                                                              Vec3 const& point,
                                                                                              float& distance,
                                                                                              Vec3& pointOnPath,
                                                                                              Vec3& tangent,
                                                                                              float& radius) const
{
    path_.mapPointToSegmentDistanceAndPointAndTangent( segmentIndex, point, distance, pointOnPath, tangent );
    radius = radius_;
}
*/

