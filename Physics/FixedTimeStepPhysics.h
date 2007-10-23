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
#include <Physics/IRigidBody.h>
#include <Math/Vector.h>
#include <list>

namespace OpenEngine {
namespace Physics {

using OpenEngine::Core::IModule;
using OpenEngine::Math::Vector;

using namespace OpenEngine::Scene;
using namespace std;

/**
 * Physic module handling all added Ridget containers.
 *
 * @class Physic Physic.h Physics/Physic.h
 */
class FixedTimeStepPhysics : public IModule {
private:

    // Private module that interpolates the physic calculations in the
    // independent engine loop.
    class PhysicInterpolations : public IModule {
    private:
        FixedTimeStepPhysics& pmod;
    public:
        PhysicInterpolations(FixedTimeStepPhysics& pmod) : pmod(pmod) {}
        ~PhysicInterpolations() {}
        void Process(const float dt, const float p) {
            list<IRigidBody*>::iterator itr;
            for(itr = pmod.rigidList.begin(); itr != pmod.rigidList.end(); itr++)
                (*itr)->ApplyTransformation(p);
        }
        void Initialize() {}
        void Deinitialize() {}
        bool IsTypeOf(const std::type_info& inf) { return false; }
    };

    //! Paused flag
    bool paused;

    //! Physic scene root
    ISceneNode* root;

    //! @todo Change to more general rigid body container.
    list<IRigidBody*> rigidList;
    
    //! Interpolation module
    PhysicInterpolations* imod;

public:
    FixedTimeStepPhysics(ISceneNode* root = NULL);
    ~FixedTimeStepPhysics();

    void SetSceneRoot(ISceneNode* root);

    void AddRigidBody(IRigidBody* body);

    void Initialize();
    void Process(const float deltaTime, const float percent);
    void Deinitialize();
    bool IsTypeOf(const std::type_info& inf);

    void TogglePause();

};

} // NS Physics
} // NS OpenEngine

#endif // _FIXED_TIME_STEP_PHYSICS_H_
