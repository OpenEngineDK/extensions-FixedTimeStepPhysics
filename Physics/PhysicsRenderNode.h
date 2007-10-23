// Physics Render node.
// -------------------------------------------------------------------
// Copyright (C) 2007 OpenEngine.dk (See AUTHORS) 
// 
// This program is free software; It is covered by the GNU General 
// Public License version 2 or any later version. 
// See the GNU General Public License for more details (see LICENSE). 
//--------------------------------------------------------------------

#ifndef _FTS_PHYSICS_RENDER_NODE_H_
#define _FTS_PHYSICS_RENDER_NODE_H_

#include <Renderers/IRenderer.h>
#include <Renderers/IRenderNode.h>
#include <Geometry/Line.h>
#include <Math/Vector.h>
#include <list>
#include <map>

using OpenEngine::Geometry::Line;
using OpenEngine::Geometry::FacePtr;
using OpenEngine::Math::Vector;
using OpenEngine::Renderers::IRenderer;
using OpenEngine::Renderers::IRenderingView;
using OpenEngine::Renderers::IRenderNode;

/**
 * Physics Render node.
 * A node that modules can use to queue geometry the is
 * to be rendered. As example the physics module uses it
 * ro render the bounding box and forces for debugging.
 *
 * @class PhysicsRenderNode PhysicsRenderNode.h
 *
 * @see IGenericNode
 */
class PhysicsRenderNode : public IRenderNode {
private:
    /**
     *  Internal representation of faces.
     */
    struct GFace {
        FacePtr face;
        Vector<3,float> color;
        float width;
        GFace(FacePtr face, Vector<3,float> color, float width) : face(face), color(color), width(width) {}
    };

    /**
     *  Internal representation of lines.
     */
    struct GLine {
        Line line;
        Vector<3,float> color;
        float width;
        GLine(Line& line, Vector<3,float> color, float width) : line(line), color(color), width(width) {}
    };

    /**
     *  Internal representation of points.
     */
    struct GPoint {
        Vector<3,float> point;
        Vector<3,float> color;
        float size;
        GPoint(Vector<3,float> point, Vector<3,float> color, float size) : point(point), color(color), size(size) {}
    };

    std::list<GFace> faces; // internal list of faces queued
    std::list<GLine> lines; // internal list of lines queued
    std::list<GPoint> points; // internal list of points queued

    /**
     *  Draw faces.
     */
    void ApplyFaces(IRenderer* renderer) {
        std::list<GFace>::iterator faceItr = faces.begin();
        for (; faceItr != faces.end(); faceItr++) {
            GFace gface = *faceItr;
            renderer->DrawFace(gface.face, gface.color, gface.width);
        }
    }

    /**
     *  Draw lines.
     */
    void ApplyLines(IRenderer* renderer) {
        std::list<GLine>::iterator lineItr = lines.begin();
        for (; lineItr != lines.end(); lineItr++) {
            GLine gline = *lineItr;
            renderer->DrawLine(gline.line,gline.color,gline.width);
        }
    }

    /**
     *  Draw points.
     */
    void ApplyPoints(IRenderer* renderer) {
        std::list<GPoint>::iterator pointItr = points.begin();
        for (; pointItr != points.end(); pointItr++) {
            GPoint gpoint = *pointItr;
            renderer->DrawPoint(gpoint.point,gpoint.color,gpoint.size);
        }
    }

public:
    /**
     * Draw the geometry, this is called by the renderer
     */
    virtual void Apply(IRenderingView* renderingView) {
        IRenderer* renderer = renderingView->GetRenderer();
        ApplyFaces(renderer);
        ApplyLines(renderer);
        ApplyPoints(renderer);
    }

    /**
     * Queue face to be drawed by the renderer
     */
    void AddFace(FacePtr face,Vector<3,float> color, float width = 1) {
        faces.push_back(GFace(face,color,width));
    }

    /**
     * Queue line to be drawed by the renderer
     */
    void AddLine(Line line,Vector<3,float> color, float width = 1) {
        lines.push_back(GLine(line,color,width));
    }

    /**
     * Queue point to be drawed by the renderer
     */
    void AddPoint(Vector<3,float> point, Vector<3,float> color, float size = 1) {
        points.push_back(GPoint(point,color,size));
    }

    /**
     * Clear all drawing queues.
     */
    void Clear() {
        points.clear();
        faces.clear();
        lines.clear();
    }
};

#endif // _FTS_PHYSICS_RENDER_NODE_H_
