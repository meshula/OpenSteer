#ifndef OPENSTEER_QUERYPATHALIKE_H
#define OPENSTEER_QUERYPATHALIKE_H


// Include std::numeric_limits< float >::max
#include <limits>



// Include OpenSteer::Vec3
#include "OpenSteer/Vec3.h"

// Include OpenSteer::distance
#include "OpenSteer/Vec3Utilities.h"

// Include OpenSteer::clamp, OpenSteer::modulo
#include "OpenSteer/Utilities.h"



namespace OpenSteer {

    /**
     * Maps @a queryPoint to a path alike @a pathway and returns the queried 
     * data in @a mapping.
     *
     * @c PointToPathwayMapping must provide the following member functions to
     * set queried values. Provide empty member functions if you aren't 
     * interested in the specified values. A good compiler should optimize the
     * empty member function calls and the associated calculations for its
     * parameters away.
     *
     * <code> void setPointOnPathCenterLine( Vec3 const& ) </code>
     * <code> void setPointOnPathBoundary( Vec3 const& ) </code>
     * <code> void setRadius( float ) </code>
     * <code> void setTangent( Vec3 const& ) </code>
     * <code> void setSegmentIndex( typename SegmentedPathAlike::size_type ) </code>
     * <code> void setDistancePointToPath( float ) </code>
     * <code> void setDistancePointToPathCenterLine( float ) </code>
     * <code> void setDistanceOnPath( float ) </code>
     * <code> void setDistanceOnSegment( float ) </code>
     *
     * To query for the distance on the path the two following member functions
     * of @c PointToPathwayMapping must be provided too:
     *
     * <code> void setDistanceOnPathFlag( float ) </code> and
     * <code> float distanceOnPathFlag() const </code>.
     *
     * If the distance along the path shouldn't be extracted provide empty 
     * versions of these member functions.
     *
     * Provide a member function <code>float segmentDistanceRadius( PathAlike const& pathAlike, typename PathAlike::size_type segmentIndex, float distanceOnSegment ) const</code>
     * for your path alike class so radius information can be extracted. A path
     * without an associated radius for example would just return @c 0.0f all 
     * the time.
     *
     * In @c QueryPathAlikeUtilities.h provides some base classes to inherit
     * from to automatically get some of the functionality described above.
     */
    template< class SegmentedPathAlike, class PointToPathwayMapping >
    void mapPointToPathway( SegmentedPathAlike const& pathway, 
                            Vec3 const& queryPoint, 
                            PointToPathwayMapping& mapping )
    {
        float minDistancePointToPath = std::numeric_limits< float >::max();
        mapping.setDistanceOnPathFlag( 0.0f );
            
        typedef typename SegmentedPathAlike::size_type size_type;
        size_type const segmentCount = pathway.segmentCount();
        for (  size_type segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex ) {
            
            
            float segmentDistance = pathway.mapPointToSegmentDistance( segmentIndex, queryPoint );
            Vec3 pointOnPathCenterLine( pathway.mapSegmentDistanceToPoint( segmentIndex, segmentDistance ) );
            float radius = mapping.segmentDistanceRadius( pathway, segmentIndex, segmentDistance );
            float distancePointToPath = distance( queryPoint, pointOnPathCenterLine )  - radius;
            
            /*
            float segmentDistance = 0.0f;
            Vec3 pointOnPathCenterLine( 0.0f, 0.0f, 0.0f );
            Vec3 tangent( 0.0f, 0.0f, 0.0f );
            float radius = 0.0f;
            mapPointToSegmentDistanceAndPointAndTangentAndRadius( segmentIndex, queryPoint, segmentDistance, pointOnPathCenterLine, tangent, radius );
            float distancePointToPath = distance( queryPoint, pointOnPathCenterLine )  - radius;
            */
           if ( distancePointToPath < minDistancePointToPath ) {
               minDistancePointToPath = distancePointToPath;
               mapping.setPointOnPathCenterLine( pointOnPathCenterLine );
               mapping.setPointOnPathBoundary( pointOnPathCenterLine + ( ( queryPoint - pointOnPathCenterLine ).normalize() * radius ) );
               mapping.setRadius( radius );
               mapping.setTangent( pathway.mapSegmentDistanceToTangent( segmentIndex, segmentDistance ) );
               // mapping.setTangent( tangent );
               mapping.setSegmentIndex( segmentIndex );
               mapping.setDistancePointToPath( distancePointToPath );
               mapping.setDistancePointToPathCenterLine( distancePointToPath + radius );
               mapping.setDistanceOnPath( mapping.distanceOnPathFlag() + segmentDistance );
               mapping.setDistanceOnSegment( segmentDistance );
           }
           
           mapping.setDistanceOnPathFlag( mapping.distanceOnPathFlag() + pathway.segmentLength( segmentIndex ) );
        }
    }



    /**
     * Maps @a distanceOnPath to a path alike @a pathway and returns the queried
     * data in @a mapping.
     *
     * @c DistanceToPathwayMapping must provide the following member functions 
     * to set queried values. Provide empty member functions if you aren't 
     * interested in the specified values. A good compiler should optimize the
     * empty member function calls and the associated calculations for its
     * parameters away.
     *
     * <code> void setPointOnPathCenterLine( Vec3 const& ) </code>
     * <code> void setRadius( float ) </code>
     * <code> void setTangent( Vec3 const& ) </code>
     * <code> void setSegmentIndex( typename SegmentedPathAlike::size_type ) </code>
     * <code> void setDistanceOnPath( float ) </code>
     * <code> void setDistanceOnSegment( float ) </code>
     *
     * Provide a member function <code>float segmentDistanceRadius( PathAlike const& pathAlike, typename PathAlike::size_type segmentIndex, float distanceOnSegment ) const </code>
     * for your path alike class so radius information can be extracted. A path
     * without an associated radius for example would just return @c 0.0f all 
     * the time.
     */
    template< class SegmentedPathAlike, class DistanceToPathwayMapping >
    void mapDistanceToPointOnPathCenterLine( SegmentedPathAlike const& pathway, 
                                             float distanceOnPath, 
                                             DistanceToPathwayMapping& mapping )
    {
        float const pathLength = pathway.length();
        if ( pathway.isCyclic() ) {
            distanceOnPath = modulo( distanceOnPath, pathLength );       
            if ( 0.0f > distanceOnPath ) {
                distanceOnPath = pathLength + distanceOnPath;
            }       
        } else {    
            if ( 0.0f > distanceOnPath ) {
                distanceOnPath = pathLength + distanceOnPath;
            }    
            distanceOnPath = clamp( distanceOnPath, 0.0f, pathLength );
        }
        
        float remainingDistance = distanceOnPath;
        typedef typename SegmentedPathAlike::size_type size_type;
        size_type segmentIndex = 0;        
        size_type const maxSegmentIndex = pathway.segmentCount() - 1;
        while( remainingDistance > pathway.segmentLength( segmentIndex ) ) {
            remainingDistance -= pathway.segmentLength( segmentIndex );
            if ( segmentIndex == maxSegmentIndex ) { 
                break; 
            }
            ++segmentIndex;
        }
        
        mapping.setPointOnPathCenterLine( pathway.mapSegmentDistanceToPoint( segmentIndex, remainingDistance ) );
        // mapping.setPointOnPathBoundary();
        mapping.setRadius( mapping.segmentDistanceRadius( pathway, segmentIndex, remainingDistance ) );
        mapping.setTangent( pathway.mapSegmentDistanceToTangent( segmentIndex, remainingDistance ) );
        mapping.setSegmentIndex( segmentIndex );
        // mapping.setDistancePointToPath();
        // mapping.setDistancePointToPathCenterLine();
        mapping.setDistanceOnPath( distanceOnPath );
        mapping.setDistanceOnSegment( remainingDistance );
    }    
    
} // namespace OpenSteer

#endif // OPENSTEER_QUERYPATHALIKE_H
