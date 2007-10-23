// Length Constraint
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _LENGTH_CONSTRAINT_H_
#define _LENGTH_CONSTRAINT_H_

#include <Math/Vector.h>

namespace OpenEngine {
namespace Physics {

using OpenEngine::Math::Vector;

/**
 * Length Constraint.
 *
 * Represents a line with a desired length and caching of old position.
 *
 * @class LengthConstraint LengthConstraint.h "LengthConstraint.h"
 */
class LengthConstraint {
public:

    // Actual position
    Vector<3,float>* p1;        //!< first end point
    Vector<3,float>* p2;        //!< second end point

    // Old positions are used for delta movement.
    Vector<3,float>* oldP1;     //!< old first end point
    Vector<3,float>* oldP2;     //!< old second end point

    // The actual constraint length
    float length;               //!< length of line between end points
    
    /**
     * Create a length constraint from two points.
     *
     * @param p1 First end point.
     * @param p2 Second end point.
     * @param oldP1 Old value of first end point.
     * @param oldP2 Old value of second end point.
     */
    LengthConstraint(Vector<3,float>* p1, Vector<3,float>* p2,
                     Vector<3,float>* oldP1, Vector<3,float>* oldP2)
        : p1(p1), p2(p2), oldP1(oldP1), oldP2(oldP2) {
        Vector<3,float> line = (*p1) - (*p2);
        length = line.GetLength();
    }

    ~LengthConstraint() {}
};

} // NS Physics
} // NS OpenEngine

#endif
