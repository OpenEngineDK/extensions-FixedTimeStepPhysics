// Rigid body interface.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _I_RIGID_BODY_H_
#define _I_RIGID_BODY_H_

#include <Scene/ISceneNode.h>

namespace OpenEngine {
namespace Physics {

using namespace OpenEngine::Scene;

/**
 * Rigid body.
 *
 * @class RigidBody RigidBody.h Physics/RigidBody.h
 */
class IRigidBody {
public:
    
    IRigidBody() {}

    virtual ~IRigidBody() {}

    /**
     * Set the time step length.
     *
     * @param length Step length.
     */
    virtual void SetStepLength(const float length) = 0;

    /**
     * Perform a time step.
     *
     * @param root Root node of the physics scene to do testing against.
     */
    virtual void TimeStep(ISceneNode* root) = 0;

    /**
     * Apply interpolated results of the physics calculations.
     *
     * @param percentage Percentage of elapsed frame to interpolate by.
     */
    virtual void ApplyTransformation(const float percentage) = 0;

};

} // NS Physics
} // NS OpenEngine

#endif // _I_RIGID_BODY_H_
