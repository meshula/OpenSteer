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
// Proximity 
//
// Data structures for accelerating proximity/locality/neighborhood queries
//
// 06-20-01 cwr: created
//
//
// ----------------------------------------------------------------------------


#ifndef OPENSTEER_PROXIMITY_H
#define OPENSTEER_PROXIMITY_H


#include <vector>
#include "OpenSteer/Vec3.h"
#include "OpenSteer/lq.h"   // XXX temp?


// ----------------------------------------------------------------------------
// "tokens" are the objects manipulated by the spatial database


template <class ContentType>
class AbstractTokenForProximityDatabase
{
public:

    virtual ~AbstractTokenForProximityDatabase () {}

    // the client object calls this each time its position changes
    virtual void updateForNewPosition (const Vec3& position) = 0;

    // find all neighbors within the given sphere (as center and radius)
    virtual void findNeighbors (const Vec3& center,
                                const float radius,
                                std::vector<ContentType>& results) = 0;

};


// ----------------------------------------------------------------------------
// abstract type for all kinds of proximity databases


template <class ContentType>
class AbstractProximityDatabase
{
public:

    // type for the "tokens" manipulated by this spatial database
    typedef AbstractTokenForProximityDatabase<ContentType> tokenType;

    // allocate a token to represent a given client object in this database
    virtual tokenType* allocateToken (ContentType parentObject) = 0;

    // insert
    // XXX maybe this should return an iterator?
    // XXX see http://www.sgi.com/tech/stl/set.html
    // virtual void insert (const ContentType& x) = 0;

    // XXX name?
    // returns the number of tokens in the proximity database
    virtual int getPopulation (void) = 0;
};


// ----------------------------------------------------------------------------


/*

  template <class ContentType>
  class BruteForceProximityDatabase :
                                  public AbstractProximityDatabase<ContentType>
  {
  public:

      // constructor
      BruteForceProximityDatabase (void)
      {
      }

      // destructor
      virtual ~BruteForceProximityDatabase ()
      {
      }

      // "token" to represent objects stored in the database
      class tokenType : public AbstractTokenForProximityDatabase<ContentType>
      {
      public:

          // constructor
          tokenType (ContentType parentObject, BruteForceProximityDatabase& pd)
          {
              object = parentObject;
              bfpd = pd;
          }

          // the client object calls this each time its position changes
          void updateForNewPosition (const Vec3& newPosition)
          {
              position = newPosition;
          }

          // find all neighbors within the given sphere (as center and radius)
          void findNeighbors (const Vec3& center,
                              const float radius,
                              std::vector<ContentType>& results)
          {
              bfpd->XXXfindNeighbors (center, radius, results);
          }

      private:
          BruteForceProximityDatabase* bfpd;
          ContentType object;
          Vec3 position;
      };

      typedef std::vector<tokenType*> tokenVector;
      typedef tokenVector::const_iterator tokenIterator;    


      // find all neighbors within the given sphere (as center and radius)
      void XXXfindNeighbors (const Vec3& center,
                             const float radius,
                             std::vector<ContentType>& results)
      {
          // loop over all tokens
          for (tokenIterator i = bfpd->group.begin();
               i != bfpd->group.end();
               i++)
          {
              tokenType& other = **i;
              const float d = Vec3::distance (center, other.position);
              if (d < radius)
              {
                  results.push_back ((ContentType) clientObject);
              }
          }
      }


      // allocate a token to represent a given client object in this database
      tokenType* allocateToken (ContentType parentObject)
      {
          return new tokenType (parentObject, *this);
      }

  private:
      // STL vector containing all tokens in database
      tokenVector group;
  };

*/


// ----------------------------------------------------------------------------


template <class ContentType>
class LQProximityDatabase : public AbstractProximityDatabase<ContentType>
{
public:

    // constructor
    LQProximityDatabase (const Vec3& center,
                         const Vec3& dimensions,
                         const Vec3& divisions)
    {
        const Vec3 halfsize (dimensions * 0.5f);
        const Vec3 origin (center - halfsize);

        lq = lqCreateDatabase (origin.x, origin.y, origin.z, 
                               dimensions.x, dimensions.y, dimensions.z,  
                               (int) round (divisions.x),
                               (int) round (divisions.y),
                               (int) round (divisions.z));
    }

    // destructor
    virtual ~LQProximityDatabase ()
    {
        free (lq);
        lq = NULL;
    }

    // "token" to represent objects stored in the database
    class tokenType : public AbstractTokenForProximityDatabase<ContentType>
    {
    public:

        // constructor
        tokenType (ContentType parentObject, LQProximityDatabase& lqsd)
        {
            lqInitClientProxy (&proxy, parentObject);
            lq = lqsd.lq;
        }

        // destructor
        virtual ~tokenType (void)
        {
            lqRemoveFromBin (&proxy);
        }

        // the client object calls this each time its position changes
        void updateForNewPosition (const Vec3& p)
        {
            lqUpdateForNewLocation (lq, &proxy, p.x, p.y, p.z);
        }

        // find all neighbors within the given sphere (as center and radius)
        void findNeighbors (const Vec3& center,
                            const float radius,
                            std::vector<ContentType>& results)
        {
            lqMapOverAllObjectsInLocality (lq, 
                                           center.x, center.y, center.z,
                                           radius,
                                           perNeighborCallBackFunction,
                                           (void*)&results);
        }

        // called by LQ for each clientObject in the specified neighborhood:
        // push that clientObject onto the ContentType vector in void*
        // clientQueryState
        static void perNeighborCallBackFunction  (void* clientObject,
                                                  float distanceSquared,
                                                  void* clientQueryState)
        {
            typedef std::vector<ContentType> ctv;
            ctv& results = *((ctv*) clientQueryState);
            results.push_back ((ContentType) clientObject);
        }

    private:
        lqClientProxy proxy;
        lqDB* lq;
    };


    // allocate a token to represent a given client object in this database
    tokenType* allocateToken (ContentType parentObject)
    {
        return new tokenType (parentObject, *this);
    }

    // count the number of tokens currently in the database
    int getPopulation (void)
    {
        int count = 0;
        lqMapOverAllObjects (lq, counterCallBackFunction, &count);
        return count;
    }
    
    static void counterCallBackFunction  (void* clientObject,
                                          float distanceSquared,
                                          void* clientQueryState)
    {
        int& counter = *(int*)clientQueryState;
        counter++;
    }



private:
    lqDB* lq;
};


// ----------------------------------------------------------------------------
#endif // OPENSTEER_PROXIMITY_H