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
#ifndef OPENSTEER_QUERYPATHALIKEMAPPINGS_H
#define OPENSTEER_QUERYPATHALIKEMAPPINGS_H


// Include OpenSteer::HasNoRadius, OpenSteer::ExtractPathDistance, OpenSteer::DontExtractPathDistance
#include "OpenSteer/QueryPathAlikeUtilities.h"

// Include OpenSteer::Vec3
#include "OpenSteer/Vec3.h"

// Include OpenSteer::size_t
#include "OpenSteer/StandardTypes.h"



namespace OpenSteer {
    
    /**
     * Maps points to a path to extract the point on the path boundary, the 
     * tangent at that point and the distance of the given point to the nearest
     * path boundary.
     */
    class PointToPathMapping 
        : public DontExtractPathDistance {
        
    public:
        PointToPathMapping() : pointOnPathBoundary( 0.0f, 0.0f, 0.0f ), tangent( 0.0f, 0.0f, 0.0f ), distancePointToPath( 0.0f ) {}
            
        void setPointOnPathCenterLine( Vec3 const& ) {}
        void setPointOnPathBoundary( Vec3 const& point ) {
            pointOnPathBoundary = point;
        }
        void setRadius( float ) {}
        void setTangent( Vec3 const& t) {
            tangent = t;
        }
        void setSegmentIndex( size_t ) {}
        void setDistancePointToPath( float distance ) {
            distancePointToPath = distance;
        }
        void setDistancePointToPathCenterLine( float ) {}
        void setDistanceOnPath( float ) {}
        void setDistanceOnSegment( float ) {}
            
        
        Vec3 pointOnPathBoundary; 
        Vec3 tangent;
        float distancePointToPath;
            
            
    }; // class PointToPathMapping
    
    
    /**
     * Maps a distance along the path center line to a point on the path center
     * line.
     */
    class PathDistanceToPointMapping 
        :  public DontExtractPathDistance {
            
    public:
        
        void setPointOnPathCenterLine( Vec3 const& vec ){
            pointOnPathCenterLine = vec;
        }
        void setRadius( float ) {}
        void setTangent( Vec3 const& ){}
        void setSegmentIndex( size_t ){}
        void setDistanceOnPath( float ){}
        void setDistanceOnSegment( float ){}
            
            
        Vec3 pointOnPathCenterLine; 
            
            
    }; // class PathDistanceToPointMapping
    
    
    
    /**
     * Maps a point to the nearest point on the path center line
     * and extracts the distance from the start of the path to this
     * center line point.
     */
    class PointToPathDistanceMapping
        : public ExtractPathDistance {
    public:
        PointToPathDistanceMapping() : distanceOnPath( 0.0f ) {}
            
        void setPointOnPathCenterLine( Vec3 const& ) {}
        void setPointOnPathBoundary( Vec3 const&  ) {}
        void setRadius( float ) {}
        void setTangent( Vec3 const& ) {}
        void setSegmentIndex( size_t ) {}
        void setDistancePointToPath( float  ) {}
        void setDistancePointToPathCenterLine( float ) {}
        void setDistanceOnPath( float distance ) {
            distanceOnPath = distance;
        }
        void setDistanceOnSegment( float ) {}
            
        float distanceOnPath;
    }; // class PointToPathDistanceMapping
    
    
} // namespace OpenSteer



#endif // OPENSTEER_QUERYPATHALIKEMAPPINGS_H
