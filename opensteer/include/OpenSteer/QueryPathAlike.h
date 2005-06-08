/**
 * OpenSteer -- Steering Behaviors for Autonomous Characters
 *
 * Copyright (c) 2002-2005, Sony Computer Entertainment America
 * Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *
 * @author Bjoern Knafla <bknafla@uni-kassel.de>
 */
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
    /*
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
     */


    template< class PathAlike, class Mapping, class BaseDataExtractionPolicy = PointToPathAlikeBaseDataExtractionPolicy< PathAlike > >
    class PointToPathAlikeMapping {
    public:
        
        static void map( PathAlike const& pathAlike, Vec3 const& queryPoint, Mapping& mapping ) {
            float minDistancePointToPath = std::numeric_limits< float >::max();
            mapping.setDistanceOnPathFlag( 0.0f );
            
            typedef typename PathAlike::size_type size_type;
            size_type const segmentCount = pathAlike.segmentCount();
            for ( size_type segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex ) {
                
                float segmentDistance = 0.0f;
                float radius = 0.0f;
                float distancePointToPath = 0.0f;
                Vec3 pointOnPathCenterLine( 0.0f, 0.0f, 0.0f );
                Vec3 tangent( 0.0f, 0.0f, 0.0f );
                
                BaseDataExtractionPolicy::extract( pathAlike, segmentIndex, queryPoint, segmentDistance, radius, distancePointToPath, pointOnPathCenterLine, tangent );
                
                if ( distancePointToPath < minDistancePointToPath ) {
                    minDistancePointToPath = distancePointToPath;
                    mapping.setPointOnPathCenterLine( pointOnPathCenterLine );
                    mapping.setPointOnPathBoundary( pointOnPathCenterLine + ( ( queryPoint - pointOnPathCenterLine ).normalize() * radius ) );
                    mapping.setRadius( radius );
                    mapping.setTangent( tangent );
                    mapping.setSegmentIndex( segmentIndex );
                    mapping.setDistancePointToPath( distancePointToPath );
                    mapping.setDistancePointToPathCenterLine( distancePointToPath + radius );
                    mapping.setDistanceOnPath( mapping.distanceOnPathFlag() + segmentDistance );
                    mapping.setDistanceOnSegment( segmentDistance );
                }
                
                mapping.setDistanceOnPathFlag( mapping.distanceOnPathFlag() + pathAlike.segmentLength( segmentIndex ) );
            }
        }
        
    }; // class MapPointToPathway
    
    
    template< class PathAlike, class Mapping >
    void mapPointToPathAlike( PathAlike const& pathAlike, Vec3 const& point, Mapping& mapping ) {
        PointToPathAlikeMapping< PathAlike, Mapping >::map( pathAlike, point, mapping );
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
    /*
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
    */
    
    
    template< class PathAlike, class Mapping, class BaseDataExtractionPolicy = DistanceToPathAlikeBaseDataExtractionPolicy< PathAlike > > 
    class DistanceToPathAlikeMapping {
    public:
    
        static void map( PathAlike const& pathAlike, float distanceOnPath, Mapping& mapping ) {
            float const pathLength = pathAlike.length();
            if ( pathAlike.isCyclic() ) {
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
            typedef typename PathAlike::size_type size_type;
            size_type segmentIndex = 0;        
            size_type const maxSegmentIndex = pathAlike.segmentCount() - 1;
            while( remainingDistance > pathAlike.segmentLength( segmentIndex ) ) {
                remainingDistance -= pathAlike.segmentLength( segmentIndex );
                if ( segmentIndex == maxSegmentIndex ) { 
                    break; 
                }
                ++segmentIndex;
            }
            
            Vec3 pointOnPathCenterLine( 0.0f, 0.0f, 0.0f );
            Vec3 tangent( 0.0f, 0.0f, 0.0f );
            float radius = 0.0f;
            
            BaseDataExtractionPolicy::extract( pathAlike, segmentIndex, remainingDistance, pointOnPathCenterLine, tangent, radius );
            
            
            mapping.setPointOnPathCenterLine( pointOnPathCenterLine );
            // mapping.setPointOnPathBoundary();
            mapping.setRadius( radius );
            mapping.setTangent( tangent );
            mapping.setSegmentIndex( segmentIndex );
            // mapping.setDistancePointToPath();
            // mapping.setDistancePointToPathCenterLine();
            mapping.setDistanceOnPath( distanceOnPath );
            mapping.setDistanceOnSegment( remainingDistance );            
        }
        
    }; // class DistanceToPathAlikeMapping
    
    
    template< class PathAlike, class Mapping >
    void mapDistanceToPathAlike( PathAlike const& pathAlike, float distance, Mapping& mapping ) {
        DistanceToPathAlikeMapping< PathAlike, Mapping >::map( pathAlike, distance, mapping );
    }
    
    
} // namespace OpenSteer

#endif // OPENSTEER_QUERYPATHALIKE_H
