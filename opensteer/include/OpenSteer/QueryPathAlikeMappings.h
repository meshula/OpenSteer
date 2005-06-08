#ifndef OPENSTEER_QUERYPATHALIKEMAPPINGS_H
#define OPENSTEER_QUERYPATHALIKEMAPPINGS_H


// Include OpenSteer::HasNoRadius, OpenSteer::ExtractPathDistance, OpenSteer::DontExtractPathDistance
#include "OpenSteer/QueryPathAlikeUtilities.h"

// Include OpenSteer::Vec3
#include "OpenSteer/Vec3.h"



namespace OpenSteer {
    
    /**
     * Maps points to a path to extract the point on the path boundary, the 
     * tangent at that point and the distance of the given point to the nearest
     * path boundary. Use @c HasNoRadius, @c HasSegmentOrSegmentPointRadius,
     * @c HasSingleRadius or @c HasSegmentRadii as arguments for 
     * @c RadiusSwitch. The class provided must have the following member
     * function 
     * <code>float segmentDistanceRadius( PathAlike const& pathAlike, typename PathAlike::size_type segmentIndex, float ) const </code>.
     */
    template< class PathAlike, template< class T > class RadiusSwitch >
    class PointToPathMapping 
        : public RadiusSwitch< PathAlike >, public DontExtractPathDistance {
        
    public:
        PointToPathMapping() : pointOnPathBoundary(), tangent(), distancePointToPath( 0.0f ) {}
            
        void setPointOnPathCenterLine( Vec3 const& ) {}
        void setPointOnPathBoundary( Vec3 const& point ) {
            pointOnPathBoundary = point;
        }
        void setRadius( float ) {}
        void setTangent( Vec3 const& t) {
            tangent = t;
        }
        void setSegmentIndex( typename PathAlike::size_type ) {}
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
     * line. Use @c HasNoRadius, @c HasSegmentOrSegmentPointRadius,
     * @c HasSingleRadius or @c HasSegmentRadii as arguments for 
     * @c RadiusSwitch. The class provided must have the following member
     * function 
     * <code>float segmentDistanceRadius( PathAlike const& pathAlike, typename PathAlike::size_type segmentIndex, float ) const </code>.
     */
    template< class PathAlike, template< class T > class RadiusSwitch >
    class PathDistanceToPointMapping 
        : public RadiusSwitch< PathAlike >, public DontExtractPathDistance {
            
    public:
        
        void setPointOnPathCenterLine( Vec3 const& vec ){
            pointOnPathCenterLine = vec;
        }
        void setRadius( float ) {}
        void setTangent( Vec3 const& ){}
        void setSegmentIndex( typename PathAlike::size_type ){}
        void setDistanceOnPath( float ){}
        void setDistanceOnSegment( float ){}
            
            
        Vec3 pointOnPathCenterLine; 
            
            
    }; // class PathDistanceToPointMapping
    
    
    
    /**
     * Maps a point to the nearest point on the path center line
     * and extracts the distance from the start of the path to this
     * center line point. Use @c HasNoRadius, @c HasSegmentOrSegmentPointRadius,
     * @c HasSingleRadius or @c HasSegmentRadii as arguments for 
     * @c RadiusSwitch. The class provided must have the following member
     * function 
     * <code>float segmentDistanceRadius( PathAlike const& pathAlike, typename PathAlike::size_type segmentIndex, float ) const </code>.
     */
    template< class PathAlike, template< class T > class RadiusSwitch >
    class PointToPathDistanceMapping
        : public RadiusSwitch< PathAlike >, public ExtractPathDistance {
    public:
        PointToPathDistanceMapping() : distanceOnPath( 0.0f ) {}
            
        void setPointOnPathCenterLine( Vec3 const& ) {}
        void setPointOnPathBoundary( Vec3 const&  ) {}
        void setRadius( float ) {}
        void setTangent( Vec3 const& ) {}
        void setSegmentIndex( typename PathAlike::size_type ) {}
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
