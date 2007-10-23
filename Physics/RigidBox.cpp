// Rigid box.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#include <Physics/RigidBox.h>
#include <Physics/FaceCollector.h>
#include <Geometry/Box.h>
#include <Geometry/Face.h>
#include <Geometry/Line.h>
#include <Logging/Logger.h>

namespace OpenEngine {
namespace Physics {

using namespace OpenEngine::Math;
using namespace OpenEngine::Geometry;

/**
 * Construct a new rigid box.
 *
 * @param box Geometrical box to construct from.
 */
RigidBox::RigidBox(Box box) : damping(0.05f) {
    gNode = new PhysicsRenderNode();

    particle[0] = box.GetCenter();
    particle[1] = box.GetCorner(1,1,1); // x  y  z
    particle[2] = box.GetCorner(1,1,0); // x  y -z
    particle[3] = box.GetCorner(1,0,1); // and so on..
    particle[4] = box.GetCorner(1,0,0);
    particle[5] = box.GetCorner(0,1,1);
    particle[6] = box.GetCorner(0,1,0);
    particle[7] = box.GetCorner(0,0,1);
    particle[8] = box.GetCorner(0,0,0);
    
    // Initialise old particle positions to current.
    for (unsigned int i=0; i<NUM_PARTICLES; i++)
        initParticle[i] = oldParticle[i] = particle[i];

    center = &particle[0];
    offset = Vector<3,float>(0.0f) - *center;

    CalculateConstraints();

    ResetForces();
}

/**
 * Destructor.
 */
RigidBox::~RigidBox() {

}

/**
 * Set the time step length.
 *
 * @param length Step length.
 */
void RigidBox::SetStepLength(const float length) {
    step = length;
}

/**
 * Get the box center in world space coordinates.
 *
 * @return Current center of the box.
 */
Vector<3,float> RigidBox::GetCenter() const {
    return *center;
}

/**
 * Set the box center in world space.
 *
 * @param position The new center of the box.
 */
void RigidBox::SetCenter(Vector<3,float> position) {
    Vector<3,float> offset = position - (*center);
    for(unsigned int i=0; i<NUM_PARTICLES; i++) {
        particle[i] += offset;
        oldParticle[i] += offset;
    }
    newPosition += offset;
    oldPosition += offset;
}

/**
 * Set attached transformation node.
 *
 * Sets the transformation node the resulting physics calculations 
 * should be applied on. The transformation node's position and
 * orientation is overwritten every time 
 * ApplyTransformation(percentage) is called.
 *
 * @param node Transformation node to modify.
 */
void RigidBox::SetTransformationNode(TransformationNode* node) {
    tnode = node;
}

/**
 * Get the transformation node this rigid box is modifying.
 * 
 * @return Transformation node.
 */
TransformationNode* RigidBox::GetTransformationNode() {
    return tnode;
}

/**
 * Get the last computed rotation matrix.
 *
 * Vectors of the box orientation can be read from this matrix.
 * @code
 * Matrix<3,3,float> m = GetRotationMatrix();
 * m.GetRow(0); // vector following the x-axis
 * m.GetRow(1); // vector following the y-axis
 * m.GetRow(2); // vector following the z-axis
 * @endcode
 *
 * @return Rotation matrix of the box.
 */
Matrix<3,3,float> RigidBox::GetRotationMatrix() {
    return matrix;
}

/**
 * Add force to all particles of the box.
 *
 * Adds force in the direction of the force vector supplied with power
 * equal to the length of the force vector.
 *
 * @param force Vector specifying direction and power.
 */
void RigidBox::AddForce(Vector<3,float> force) {
    for (unsigned int i=0; i<NUM_PARTICLES; i++)
        addedForce[i] += force;
}

/**
 * Add force to a single particle of the box.
 *
 * Adds the force in the direction of the force vector supplied with
 * power equal to the length of the force vector.
 *
 * @param force Vector specifying direction and power.
 * @param index Index of the affected particle.
 */
void RigidBox::AddForce(Vector<3,float> force, unsigned int index) {
    if (index >= NUM_PARTICLES) return;
    addedForce[index] += force;

    // Debug, draw green lines and point illustrating the forces
    if (gNode != NULL) {
        Vector<3,float> color(0,1.0,0);
        Vector<3,float> p = particle[index];
        gNode->AddPoint(p,color,8.0);
        gNode->AddLine(Line(p,p+force),color,2.0);
    }
}

/**
 * Reset all forces applied to the box.
 *
 * Results in the box will stay still until new forces are applied.
 */
void RigidBox::ResetForces() {
    //Init to zero vector.
    for (unsigned int i=0; i<NUM_PARTICLES; i++) {
        accuForce[i] = addedForce[i] = Vector<3,float>(0.0f);
        particle[i] = oldParticle[i] = initParticle[i];
    }
}

/**
 * Apply interpolated results to the transformation node.
 *
 * @param percentage Percentage of elapsed frame to interpolate by.
 */
void RigidBox::ApplyTransformation(const float percentage) {
    tnode->SetRotation(Quaternion<float>(oldDirection, newDirection, percentage));
    tnode->SetPosition(oldPosition + (newPosition - oldPosition) * percentage);
}

/**
 * Perform a time step.
 *
 * @param root Root node of the physics scene to do testing against.
 */
void RigidBox::TimeStep(ISceneNode* root) {
    this->root = root;
    AccumulateForces();
    Verlet();
    SatisfyConstraints();
    CalculateTransformation();
}


/**
 * Apply the force changes to the particles.
 *
 * This is called once before the Verlet integration step.
 */
void RigidBox::AccumulateForces() {
    for (unsigned int i=0; i<NUM_PARTICLES; i++) {
        accuForce[i]  = addedForce[i] + Vector<3,float>(0,-9.82*10,0);
        addedForce[i] = Vector<3,float>(0.0f);
    }
}

/**
 * Perform one step of Verlet integration.
 *
 * For theoretical doc see http://www.teknikus.dk/tj/gdc2001.htm
 */
void RigidBox::Verlet() {
    // Calc deltaTime powered by 2.
    float step2 = step * step;
    // Iterate through all particles and do Verlet integration.
    for (unsigned int i=0; i<NUM_PARTICLES; i++) {
        // Save current position
        Vector<3,float> temp = particle[i];
        // Verlet integration step accelerates accumulated forces 
        // and adds this to delta movement of the particle.
        // particle = particle + deltaPosition + (accuForce * (deltaTime^2)
        Vector<3,float> result = ((particle[i] - oldParticle[i]) + (accuForce[i] * (step2)));
        // Add the resulting force to the particle. Decreasing it by a few percent creates a 
        // natural drag which makes objects decelerate.
        particle[i] += result * (1 - damping);
        // Update old position
        oldParticle[i] = temp;
    }
}

/**
 * Satisfy constraints on particles.
 *
 * This is called once after performing Verlet integration.
 */
void RigidBox::SatisfyConstraints() {
    FaceCollector collector;
    // Iterate through all length constraints.
    vector<LengthConstraint*>::iterator cons;
    for (cons = consList.begin(); cons != consList.end(); cons++ ) {
        LengthConstraint* lc = *cons;
        FaceSet faces = collector.Collect(root, lc);
        
        if (faces.Size() == 0) continue;

        if( faces.Size() == 1 ) {
            FaceList::iterator itr = faces.begin();
            FacePtr face1 = *itr;
            SatisfySingleIntersection(lc, face1);
        }
        else if( faces.Size() == 2 ) {
            FaceList::iterator itr = faces.begin();
            FacePtr face1 = *itr++;
             FacePtr face2 = *itr;
             SatisfyDoubleIntersection(lc, face1, face2);
        }
        //         else
        //             logger.warning << "More than 2 faces intersects with constraint." << logger.end;
    }
    
    // Start satisfying C2 - Expand constraints
    for (cons = consList.begin(); cons != consList.end(); cons++) {
        LengthConstraint* lc = *cons;
        Vector<3,float> delta = (*lc->p2) - (*lc->p1);
        float constraintLength = lc->length;
        float deltalength = sqrt(delta*delta);
        float diff = (deltalength - constraintLength) / deltalength;
        // Expand constraint in both directions.
        (*lc->p1) += (delta*diff) *  0.5;
        (*lc->p2) += (delta*diff) * -0.5;
    }
}

/**
 * Calculate the transformation information.
 *
 * This is where the three axis of the rigid box are constructed in
 * matrix representation and the old and new interpolations updated.
 *
 * This is called once after satisfying all constraints post to a
 * Verlet integration.
 */
void RigidBox::CalculateTransformation() {
    // each side in the box specifies a rotating axis.
    Vector<3,float> x( particle[4] - particle[8]); x.Normalize();
    Vector<3,float> y( particle[6] - particle[8]); y.Normalize();
    Vector<3,float> z( particle[7] - particle[8]); z.Normalize();

    // calculate the new rotation matrix
    matrix =  Matrix<3,3,float>(x,y,z);
    
    // save the old information
    oldDirection = newDirection;
    oldPosition = newPosition;

    // calculate the new information
    newDirection = Quaternion<float>(matrix);
    newPosition = *center + newDirection.RotateVector(offset);

    // Set debugging state to render
    SetRenderState();
}

/**
 * Get a rendering node for this rigid box.
 *
 * When added to the scene tree the node will draw the box described
 * by the particles and forces applied to them.
 *
 * @note The implementation of this method is optional and only
 *       intended for easing implementation and debugging.
 *       If you implement it you may use the Frustum::GetFrustumNode()
 *       as a source of inspiration.
 *
 * @return Render node of the box.
 */
IRenderNode* RigidBox::GetRigidBoxNode() {
    return gNode;
}


void RigidBox::SatisfySingleIntersection(LengthConstraint* lc, FacePtr face){
    // Calculate the distance from constraint p1 to plane.
    Vector<3,float>* correctionPoint = NULL;

    // If the distance is behind the plane correct p1
    if (face->ComparePointPlane(*(lc->p1)) == -1) {
        correctionPoint = lc->p1;
    }
    else if (face->ComparePointPlane(*(lc->p2)) == -1) {
        // p1 was not in front of the plane so try p2.
        correctionPoint = lc->p2;
    }
    
    if (correctionPoint != NULL) {
        float distance = -(face->hardNorm * (*correctionPoint - face->vert[0]));
        
        Vector<3,float> newPoint = *correctionPoint + face->hardNorm * distance;
        //  if( face->Contains(newPoint) )
            *correctionPoint = newPoint;
    }
}

void RigidBox::SatisfyDoubleIntersection(LengthConstraint* lc, FacePtr face1, FacePtr face2){
    // find the intersection line for the two planes that the length constraint goes through
    Vector<3,float> plane1norm = face1->hardNorm;
    Vector<3,float> plane2norm = face2->hardNorm;
    if (plane1norm.GetLength() == 0 ||
        plane2norm.GetLength() == 0) {
        logger.warning << "one of the planes norms does not define a plane" << logger.end;
        return;
    }
    // calculate the direction vector for the intersecting line
    Vector<3,float> intersectionLineDirection = plane1norm % plane2norm;
    if (intersectionLineDirection.GetLength() == 0) {
        logger.warning << "planes are parallel" << logger.end;
        return;
    }
    else
        intersectionLineDirection.Normalize();
    // calculate a point on the intersecting line
    Vector<3,float> plane1point = face1->vert[0];
    Vector<3,float> plane2point = face2->vert[0];
	Vector<3,float> perp = plane1norm % intersectionLineDirection; //unit vector in P1, perp to intersection
	Vector<3,float> intersectionLinePoint = plane1point + 
        ((((plane2point-plane1point)*(plane2norm))/(perp*(plane2norm)))*perp);
    Line* constraint = new Line(*lc->p1, *lc->p2);
    Line* d = constraint->ShortestLineBetweenLineAndRay
        (intersectionLineDirection,intersectionLinePoint);

    if( d == NULL ){
        logger.warning << "d IS NULL!! - skip" << logger.end;
        return; //temp hack
    }

    Vector<3,float> p = d->point1; // on the constraint
    Vector<3,float> q = d->point2; // on plane intersection
    if( !( face1->ComparePointPlane(p) == -1 &&
           face2->ComparePointPlane(p) == -1) ) return; // if p is not behind both planes abort
    
    Vector<3,float> delta = q-p;
    if (delta == Vector<3,float>(0.0f)) return; // p and q does not define a line
    
    float length  = (p - *lc->p1).GetLength();
    float consLen = (*lc->p2 - *lc->p1).GetLength();
    float percentP2  = length / consLen;
    float percentP1 = 1.0f - percentP2;
    float lambda = (delta * (q-p)) / 
        ((delta*delta)*(percentP2*percentP2 + percentP1*percentP1));
    
    //    logger.info << "lambda: " << lambda << logger.end;

    Vector<3,float> res1 = delta * (percentP1  * lambda);
    Vector<3,float> res2 = delta * (percentP2 * lambda);
    *lc->p1 += res1;
    *lc->p2 += res2;
    //*lc->oldP1 += res1;
    //*lc->oldP2 += res2;
}

void RigidBox::SetRenderState() {
    gNode->Clear();
    // Debug, draw red lines for the constraints
    if (gNode == NULL) return;
    vector<LengthConstraint*>::iterator cons = consList.begin();
    for (unsigned int i=0; cons != consList.end(); cons++, i++) {
        LengthConstraint* lc = *cons;
        Vector<3,float> color;
        if( i == 9 )
            color = Vector<3,float>(1.0,0.0,0.0);
        else if( i == 16 )
            color = Vector<3,float>(0.0,1.0,0.0);
        else if( i == 31 )
            color = Vector<3,float>(0.0,0.0,1.0);
        else if( i == 34 )
            color = Vector<3,float>(1.0,1.0,0.0);
        else 
            color = Vector<3,float>(0.5, 0.5, 1.0);
        gNode->AddLine(Line(*(lc->p1),*(lc->p2)),color,2.0);
    }
}


/**
 * To string RigidBox
 * @return string printing center and all particles.
 */
std::string RigidBox::ToString() {
    std::string str = "Center: " + (*center).ToString();
    for (unsigned int i = 1; i<NUM_PARTICLES;i++)
        str += "\n" + particle[i].ToString();
    str += "\n-------";
    return str;
}


/**
 * Logs all constraints.
 */
void RigidBox::ListConstraints() {
    vector<LengthConstraint*>::iterator cons;
    int i=0;
    for (cons = consList.begin(); cons != consList.end(); cons++) {
        LengthConstraint* lc = *cons;
        logger.info << "Constraint[" << i++ << "] (p1,p2): (" << lc->p1->ToString();
        logger.info << "," << lc->p2->ToString() << ")" << logger.end;
    }
}

/**
 * Calculates all constraints.
 * All sides, diagonals and center-corner constraints are constructed here. 
 */
void RigidBox::CalculateConstraints() {
    consList.clear();
    //Calculate all length constaints in the box
    for (unsigned int i=0; i<NUM_PARTICLES; i++)
        for (unsigned int j=0; j<NUM_PARTICLES; j++) {
            if (i >= j) continue;
            consList.push_back(new LengthConstraint(&particle[i],&particle[j],
                                                    &oldParticle[i],&oldParticle[j]));
        }
}

} // NS Physics
} // NS OpenEngine
