// Program name and description
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _FIXED_TIME_STEP_PHYSICS_H_
#define _FIXED_TIME_STEP_PHYSICS_H_

#include <Core/IModule.h>
#include <Core/IListener.h>
#include <Physics/IRigidBody.h>
#include <Math/Vector.h>
#include <list>
#include <math.h>
#include <Utils/Timer.h>

namespace OpenEngine {
namespace Physics {

using OpenEngine::Core::IModule;
using OpenEngine::Core::InitializeEventArg;
using OpenEngine::Core::ProcessEventArg;
using OpenEngine::Core::DeinitializeEventArg;
using OpenEngine::Core::IListener;

using OpenEngine::Math::Vector;
using OpenEngine::Utils::Timer;

using namespace std;

/**
 * Physic module handling all added Ridget containers.
 *
 * @class FixedTimeStepPhysics FixedTimeStepPhysics.h Physics/FixedTimeStepPhysics.h
 */
class FixedTimeStepPhysics : public IModule {
private:
    //! Paused flag
    bool paused;

    //! Physic scene root
    ISceneNode* root;

    //! @todo Change to more general rigid body container.
    list<IRigidBody*> rigidList;
    
public:
    FixedTimeStepPhysics(ISceneNode* root = NULL);
    ~FixedTimeStepPhysics();

    void SetSceneRoot(ISceneNode* root);

    void AddRigidBody(IRigidBody* body);
    list<IRigidBody*> GetRigidBodies();

    void Handle(InitializeEventArg arg);
    void Handle(ProcessEventArg arg);
    void Handle(DeinitializeEventArg arg);

    void TogglePause();

};

class FixedTimeStepPhysicsTimer : public IListener<ProcessEventArg> {
    Timer timer;
    FixedTimeStepPhysics& physics;
public:
    FixedTimeStepPhysicsTimer(FixedTimeStepPhysics& physics) : physics(physics) {
        timer.Start();
    }
    virtual void Handle(ProcessEventArg arg) {
        // apply the fixed verlet integration
        unsigned int t = timer.GetElapsedIntervalsAndReset(50);
        while (t--) physics.Handle(arg);

        // apply interpolation
        float newp = min(1.0f, (float)timer.GetElapsedTime() / 50);
        list<IRigidBody*> rigidList = physics.GetRigidBodies();
        list<IRigidBody*>::iterator itr;
        for(itr = rigidList.begin(); itr != rigidList.end(); itr++) {
            (*itr)->ApplyTransformation(newp);
        }
    }
};

} // NS Physics
} // NS OpenEngine

#endif // _FIXED_TIME_STEP_PHYSICS_H_
