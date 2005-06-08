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
 *
 * Collection of helper classes to inherit from to create mappings as used by  
 * @c OpenSteer::mapDistanceToPointOnPathCenterLine and 
 * @c OpenSteer::mapPointToPathway.
 */

#ifndef OPENSTEER_QUERYPATHALIKEUTILITIES_H
#define OPENSTEER_QUERYPATHALIKEUTILITIES_H


namespace OpenSteer {
    
    /**
     * Inherit from it to create a mapping class used by  
     * @c OpenSteer::mapDistanceToPointOnPathCenterLine and 
     * @c OpenSteer::mapPointToPathway that work on a path alike which stores
     * no radius information.
     */
    template< class PathAlike >
    class HasNoRadius {
    public:
        float segmentDistanceRadius( PathAlike const&, 
                                     typename PathAlike::size_type, 
                                     float ) const {
            return 0.0f;
        }
    
    protected:
        ~HasNoRadius() {
            // Nothing to do.
        }
        
    }; // class HasNoRadius
    
    
    /**
     * Inherit from it to create a mapping class used by  
     * @c OpenSteer::mapDistanceToPointOnPathCenterLine and 
     * @c OpenSteer::mapPointToPathway that work on a path alike which stores
     * different radii for every point or segment point.
     */
    template< class PathAlike >
    class HasSegmentOrSegmentPointRadius {
    public:
        float segmentDistanceRadius( PathAlike const& pathAlike, 
                                     typename PathAlike::size_type segmentIndex, 
                                     float distanceOnSegment ) const {
            return pathAlike.mapSegmentDistanceToRadius( segmentIndex, distanceOnSegment );
        }
        
    protected:
        ~HasSegmentOrSegmentPointRadius() {
            // Nothing to do.
        }
    }; // class HasSegmentOrSegmentPointRadius
    
    
    /**
     * Inherit from it to create a mapping class used by  
     * @c OpenSteer::mapDistanceToPointOnPathCenterLine and 
     * @c OpenSteer::mapPointToPathway that work on a path alike which stores
     * a single radius for the whole path alike.
     */
    template< class PathAlike >
    class HasSingleRadius {
    public:
        float segmentDistanceRadius( PathAlike const& pathAlike, 
                                     typename PathAlike::size_type, 
                                     float ) const {
            return pathAlike.radius();
        }
        
    protected:
        ~HasSingleRadius() {
            // Nothing to do.
        }
    }; // class HasSingleRadius
    
    
    
    /**
     * Inherit from it to create a mapping class used by  
     * @c OpenSteer::mapDistanceToPointOnPathCenterLine and 
     * @c OpenSteer::mapPointToPathway that work on a path alike which stores
     * different radii for every segment.
     */
    template< class PathAlike >
    class HasSegmentRadii {
    public:
        float segmentDistanceRadius( PathAlike const& pathAlike, 
                                     typename PathAlike::size_type segmentIndex, 
                                     float ) const {
             return pathAlike.segmentRadius( segmentIndex );
         }
            
    protected:
        ~HasSegmentRadii() {
            // Nothing to do.
        }
    }; // class HasSegmentRadii    
    
    
    
    /**
     * Inherit from it to create a mapping class used by 
     * @c OpenSteer::mapDistanceToPointOnPathCenterLine and 
     * @c OpenSteer::mapPointToPathway that calculates and extracts the
     * distance along the path alike.
     */    
    class ExtractPathDistance {
    public:
        void setDistanceOnPathFlag( float distance ) {
            distanceOnPathFlag_ = distance;
        }
        
        
        float distanceOnPathFlag() const {
            return distanceOnPathFlag_;
        }
        
    protected:
        ExtractPathDistance() : distanceOnPathFlag_( 0.0f ) {
            // Nothing to do.
        }
        
        explicit ExtractPathDistance( float distance ) : distanceOnPathFlag_( distance ) {
            // Nothing to do.
        }
        
        ~ExtractPathDistance() {
            // Nothing to do.
        }
        
    private:
        float distanceOnPathFlag_;
    }; // class ExtractPathDistance
    
    
    /**
     * Inherit from it to create a mapping class used by 
     * @c OpenSteer::mapDistanceToPointOnPathCenterLine and 
     * @c OpenSteer::mapPointToPathway that shouldn't calculate and extract the
     * distance along the path alike.
     */
    class DontExtractPathDistance {
    public:
        void setDistanceOnPathFlag( float ) {
            // Nothing to do.
        }
        
        float distanceOnPathFlag() const {
            return 0.0f;
        };
        
    protected:
        ~DontExtractPathDistance() {
            // Nothing to do.
        }
    }; // class DontExtractPathDistance
    
    
} // namespace OpenSteer


#endif // OPENSTEER_QUERYPATHALIKEUTILITIES_H
