#ifndef OPENSTEER_PATH_H
#define OPENSTEER_PATH_H





namespace OpenSteer {

    // Forward declaration.
    class Vec3;
    
    
    /**
     * Path in space that might be cyclic.
     */
    class Path {
    public:
        
        virtual ~Path() = 0;
        
        
        /**
         * Returns @c true if the path is valid, @c false otherwise.
         */
        virtual bool isValid() const = 0;
        
        /**
         * Given an arbitrary point ("A"), returns the nearest point ("P") on
		 * this path.  Also returns, via output arguments, the path tangent at
		 * P and a measure of how far A is outside the Pathway's "tube".  Note
		 * that a negative distance indicates A is inside the Pathway.
         *
         * @todo Move it into a non-member function?
         */
		virtual Vec3 mapPointToPath (const Vec3& point,
                                     Vec3& tangent,
                                     float& outside) const = 0;
        
		/**
         * Given a distance along the path, convert it to a point on the path.
         * If @c isValid is @c false the behavior is undefined.
         *
         * @todo Move it into a non-member function?
         */
		virtual Vec3 mapPathDistanceToPoint (float pathDistance) const = 0;
        
		/**
         * Given an arbitrary point, convert it to a distance along the path.
         * If @c isValid is @c false the behavior is undefined.
         *
         * @todo Move it into a non-member function?
         */
		virtual float mapPointToPathDistance (const Vec3& point) const = 0;
        
        /**
         * Returns @c true f the path is closed, otherwise @c false.
         */
        virtual bool isCyclic() const = 0;
        
        /**
         * Returns the length of the path.
         */
        virtual float length() const = 0;
        
    }; // class Path
    
} // namespace OpenSteer


#endif // OPENSTEER_PATH_H
