// FixedTimeStepPhysics processor.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Physics/FixedTimeStepPhysics.h>

namespace OpenEngine {
namespace Physics {

/**
 * Physic processor constructor.
 *
 * @param root Optional root of the physics scene.
 */
FixedTimeStepPhysics::FixedTimeStepPhysics(ISceneNode* root)
    : paused(false), root(root) {
    
}

/**
 * Destructor
 */
FixedTimeStepPhysics::~FixedTimeStepPhysics() {

}

/**
 * Set the root of the physics scene.
 *
 * @param root Root of scene, NULL unsets the scene.
 */
void FixedTimeStepPhysics::SetSceneRoot(ISceneNode* root) {
    this->root = root;
}

/**
 * Adding a rigid body to the physics processor.
 *
 * As long as the rigid body is in the list it will be processed by
 * calls to its TimeStep method.
 *
 * @param body Rigid body to add.
 */
void FixedTimeStepPhysics::AddRigidBody(IRigidBody* body) {
    if (body != NULL)
        rigidList.push_back(body);
}

list<IRigidBody*> FixedTimeStepPhysics::GetRigidBodies() {
    return rigidList;
}

/**
 *
 */
void FixedTimeStepPhysics::Handle(InitializeEventArg arg) {
    // Pass the tick time in seconds to all rigid bodies.
    float step = 0.05;
    list<IRigidBody*>::iterator itr;
    for (itr = rigidList.begin(); itr!=rigidList.end(); itr++)
        (*itr)->SetStepLength(step);
}

/**
 * On physics process all rigid bodies are update according to 
 * their constraints.
 */
void FixedTimeStepPhysics::Handle(ProcessEventArg arg) {
    if (paused || root == NULL) return;
    list<IRigidBody*>::iterator itr;
    for (itr = rigidList.begin(); itr!=rigidList.end(); itr++)
        (*itr)->TimeStep(root);
}

/**
 *
 */
void FixedTimeStepPhysics::Handle(DeinitializeEventArg arg) {

}

/**
 * Pause/resume the physics module.
 */
void FixedTimeStepPhysics::TogglePause() {
    paused = !paused;
}

} // NS Physics
} // NS OpenEngine
