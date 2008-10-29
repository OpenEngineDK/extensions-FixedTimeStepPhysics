// RigidBox.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _RIGID_BOX_H_
#define _RIGID_BOX_H_

#include <Physics/IOERigidBody.h>
#include <Physics/LengthConstraint.h>
#include <Physics/PhysicsRenderNode.h>
#include <Geometry/FaceSet.h>
#include <Scene/TransformationNode.h>
#include <Scene/GeometryNode.h>
#include <Math/Vector.h>
#include <Scene/QuadNode.h>
#include <Scene/BSPNode.h>

namespace OpenEngine {
namespace Physics {

using namespace OpenEngine::Physics;
using namespace OpenEngine::Geometry;
using namespace OpenEngine::Scene;

/**
 * Rigid Box.
 *
 * @class RigidBox RigidBox.h "RigidBox.h"
 */
class RigidBox : public IOERigidBody {
private:

    //! box constants
    static const unsigned int NUM_PARTICLES = 9;

    //! Step length
    float step;

    //! Root of the physics scene
    ISceneNode* root;

    //! Rotation matrix
    Matrix<3,3,float> matrix;

    //! Old and new direction quaternions 
    Quaternion<float> oldDirection;
    Quaternion<float> newDirection;

    //! Old and new position vectors
    Vector<3,float> oldPosition;
    Vector<3,float> newPosition;

    //! Damping constant
    float damping;


    //! List of length constraints keeping the box together.
    vector<LengthConstraint*> consList;

    Vector<3,float>* center;                      //!< Box center
    Vector<3,float>  offset;                      //!< Offset from origin to initial center
    Vector<3,float>  particle[NUM_PARTICLES];     //!< Box corners + center
    Vector<3,float>  oldParticle[NUM_PARTICLES];  //!< Old box corners + center
    Vector<3,float>  initParticle[NUM_PARTICLES]; //!< Initial values in order to reset

    //!< Accumulated forces for each particle
    Vector<3,float> accuForce[NUM_PARTICLES];
    
    //!< Added forces for each particle
    Vector<3,float> addedForce[NUM_PARTICLES]; 

    // Gravity
    Vector<3,float> gravity;

    //!< Transformation node to apply modification on.
    TransformationNode* tnode;

    //!< Render node of this box
    PhysicsRenderNode* gNode;

    // Private helper functions
    void CalculateConstraints();
    void SatisfySingleIntersection(LengthConstraint* lc, FacePtr face);
    void SatisfyDoubleIntersection(LengthConstraint* lc, FacePtr face1, FacePtr face2);
    void SetRenderState();
    void ListConstraints();
    string ToString();

    // Algorithmic steps for the time-step processing
    void AccumulateForces();
    void Verlet();
    void SatisfyConstraints();
    void CalculateTransformation();    

public:

    RigidBox(Box box);
    ~RigidBox();
    
    void SetStepLength(const float length);

    Vector<3,float> GetCenter() const;
    void SetCenter(Vector<3,float> position);

    Vector<3,float> GetGravity() const;
    void SetGravity(Vector<3,float> grav);

    TransformationNode* GetTransformationNode();
    void SetTransformationNode(TransformationNode*);

    void AddForce( Vector<3,float> force );
    void AddForce( Vector<3,float> force, unsigned int index );
    void ResetForces();

    void TimeStep(ISceneNode* root);

    void ApplyTransformation(const float percentage);

    Matrix<3,3,float> GetRotationMatrix();

    RenderNode* GetRigidBoxNode();

};

} // NS Physics
} // NS OpenEngine

#endif // _RIGID_BOX_H_

