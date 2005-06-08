#ifndef OPENSTEER_SEGMENTEDPATH_H
#define OPENSTEER_SEGMENTEDPATH_H

// Include OpenSteer::Path
#include "OpenSteer/Path.h"

// Include OpenSteer::size_t
#include "OpenSteer/StandardTypes.h"

namespace OpenSteer {
    
    /**
     * Path build by segments between points on the path.
     */
    class SegmentedPath : public Path {
    public:
        typedef size_t size_type;
        
        virtual ~SegmentedPath() = 0;
        
        
        /**
         * Returns the number of points defining the segments.
         *
         * This also includes the duplicated first point if the path is cyclic.
         */
        virtual size_type pointCount() const = 0;
        
        /**
         * Returns the point @a pointIndex.
         *
         * If the path is cyclic also the last point that is the duplicated
         * first one is accessible.
         */
        virtual Vec3 point( size_type pointIndex ) const = 0;
        
        
        
        /**
         * Returns the number of segments that build the pathway.
         */
        virtual size_type segmentCount() const = 0;
        
        /**
         * Returns the length of segment @a segmentIndex.
         */
        virtual float segmentLength( size_type segmentIndex ) const = 0;
        
        /**
         * Returns the start point of the segment @a segmentIndex.
         */
        virtual Vec3 segmentStart( size_type segmentIndex ) const = 0;
        
        /**
         * Returns the end point of segment @a segmentIndex.
         */
        virtual Vec3 segmentEnd( size_type segmentIndex ) const = 0;
        
        
        /**
         * Maps @a point to the nearest point on the center line of segment
         * @a segmentIndex and returns the distance from the segment start to 
         * this point.
         */
        virtual float mapPointToSegmentDistance( size_type segmentIndex, 
                                                 Vec3 const& point ) const = 0;
        
        
        /**
         * Maps @a segmentDistance to the center line of segment @a segmentIndex
         * and returns the reached point.
         *
         * If @a segmentDistance is negative the mapping is done backward from
         * the end of the path.
         *
         * If @a segmentDistance is greater or smaller than the segment length
         * is is clamped to @c 0.0f or @c segmentLength().
         */
        virtual Vec3 mapSegmentDistanceToPoint( size_type segmentIndex, 
                                                float segmentDistance ) const = 0;
        
        /**
         * Maps @a segmentDistance to the centerline of the segment 
         * @a segmentIndex and returns the tangent of the pathway at the reached
         * point.
         *
         * If @a segmentDistance is negative the mapping is done backward from
         * the end of the path.
         *
         * If @a segmentDistance is greater or smaller than the segment length
         * is is clamped to @c 0.0f or @c segmentLength().
         */
        virtual Vec3 mapSegmentDistanceToTangent( size_type segmentIndex, 
                                                  float segmentDistance ) const = 0;
        
        
        /**
         * Combines @c mapSegmentDistanceToPoint and 
         * @c mapSegmentDistanceToTangent.
         */
        /*
        virtual void mapDistanceToSegmentPointAndTangent( size_type segmentIndex,
                                                          float distance,
                                                          Vec3& pointOnPath,
                                                          Vec3& tangent ) const = 0;
        */
        
        /**
         * Combines @c mapPointToSegmentDistance, @c mapSegmentDistanceToPoint, 
         * and @c mapSegmentDistanceToTangent.
         */
        /*
        virtual void mapPointToSegmentDistanceAndPointAndTangent( size_type segmentIndex,
                                                                  Vec3 const& point,
                                                                  float& distance,
                                                                  Vec3& pointOnPath,
                                                                  Vec3& tangent ) const = 0;
        */
    }; // class SegmentedPath
    
    
} // namespace OpenSteer


#endif // OPENSTEER_SEGMENTEDPATH_H
