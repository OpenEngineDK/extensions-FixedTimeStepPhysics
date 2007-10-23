// FaceCollector
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _FACE_COLLECTOR_H_
#define _FACE_COLLECTOR_H_

#include <Physics/LengthConstraint.h>
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
 * Face Collector.
 * Collects faces that intersects with length constraint.
 *
 * @class FaceCollector FaceCollector.h "FaceCollector.h"
 */
class FaceCollector : public ISceneNodeVisitor {
    LengthConstraint* lc;
    FaceSet faces;
    void AddFacesFromSpan(BSPNode* node) {
        FaceSet* faceList = node->GetSpan();
        for (FaceList::iterator itr = faceList->begin(); itr != faceList->end(); itr++ ) {
            FacePtr face = *itr;
            Vector<3,float>* current = face->Intersection( *lc->p1, *lc->p2 );
            if (current != NULL)
                faces.Add(face);
        }
    }
public:
    FaceCollector() : lc(NULL) { }
    /**
     * Collect all faces intersecting the line defined by a length constraint.
     *
     * @param root Scene root to search in.
     * @param lc Length constraint.
     */
    FaceSet& Collect(ISceneNode* root, LengthConstraint* lc) {
        faces.Empty();
        this->lc = lc;
        root->Accept(*this);
        return faces;
    }
    void VisitQuadNode(QuadNode* node) {
        if (node->GetBoundingBox().Intersects( Line( *(lc->p1), *(lc->p2) ) ) )
            node->VisitSubNodes(*this);
    }
    void VisitBSPNode(BSPNode* node) {
        if( node->GetDivider() == NULL ) return;
        int dist1 = node->GetDivider()->ComparePointPlane( *(lc->p1) );
        int dist2 = node->GetDivider()->ComparePointPlane( *(lc->p2) );
        if( dist1 == 0 && dist2 == 0 ) { //both on the plane
            if( node->GetFront() != NULL )
                node->GetFront()->Accept(*this);
            if( node->GetBack() != NULL )
                node->GetBack()->Accept(*this);
        }
        else if( (dist1 == 1 && dist2 == 0) || //p2 on plane
                 (dist1 == 0 && dist2 == 1) || //p1 on plane
                 (dist1 == 1 && dist2 == 1) ) { //both infront of plane
            AddFacesFromSpan(node);
            if( node->GetFront() != NULL )
                node->GetFront()->Accept(*this);
        }
        else if( (dist1 == -1 && dist2 == 0) || //p2 on plane
                 (dist1 == 0 && dist2 == -1) || //p1 on plane
                 (dist1 == -1 && dist2 == -1) ) { //both behind of plane
            AddFacesFromSpan(node);
            if( node->GetBack() != NULL )
                node->GetBack()->Accept(*this);
        }
        else if( (dist1 == 1 && dist2 == -1) ||
                 (dist1 == -1 && dist2 == 1) ) { //line spaning across the plane
            AddFacesFromSpan(node);
            if( node->GetFront() != NULL )
                node->GetFront()->Accept(*this);
            if( node->GetBack() != NULL )
                node->GetBack()->Accept(*this);
        }
    }
};

} // NS Physics
} // NS OpenEngine

#endif
