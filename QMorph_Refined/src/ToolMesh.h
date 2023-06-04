#pragma once

#ifndef _TOOL_MESH_H_
#define _TOOL_MESH_H_

#include<map>
#include<vector>
#include<queue>
#include<sstream>
#include <initializer_list>
#include <Mesh/BaseMesh.h>
#include <Mesh/Edge.h>
#include <Mesh/Face.h>
#include <Mesh/HalfEdge.h>
#include <Mesh/Vertex.h>
#include <Mesh/iterators.h>
#include <Geometry/Point.h>
#ifdef _DEBUG
#include <DebuggerConnector.h>
#endif
using namespace std;


constexpr double PI = 3.14159265358979;
#define MAX_BOUND = 5
#define EPSILON FLT_EPSILON
const double constEpsilon = 10.0;
const double seamEpsilon = 30.0;
const double constAngle = 135.0;

using namespace MeshLib;

// definition of interface Component
class Component {
public:
	virtual Component* getPointer() {
		return this;
	}
};

class CToolVertex : public CVertex, public Component {
public:
	int frontNum = 0;
	bool isSide = false;
	int sideNum = 0;
	bool markDelete = false;
	bool isSingular = false;
	~CToolVertex()
	{
	}

	virtual CToolVertex* getPointer() {
		return this;
	}
};

class CToolEdge : public CEdge, public Component {
public:
	bool disconnected = false;
	~CToolEdge()
	{
	}

	virtual CToolEdge* getPointer() {
		return this;
	}
};

class CToolHalfedge : public CHalfEdge, public Component {
public:
	bool isFront = false;
	int classNum = -1;
	bool needTopEdge = true;
	bool isSideEdge = false;
	CToolHalfedge* prevFe = NULL;
	CToolHalfedge* nextFe = NULL;
	CToolHalfedge* leftSide = NULL;
	CToolHalfedge* rightSide = NULL;
	CToolHalfedge* topEdge = NULL;
	CToolHalfedge* feReference = NULL;

	int crossFieldMatching = 100; //ccw from current face to sym face
	
	~CToolHalfedge()
	{
		assert(!isSideEdge);
	}
	void attributeCopyTo(CToolHalfedge* he) {
		he->isFront = isFront;
		he->classNum = classNum;
		he->needTopEdge = needTopEdge;
		he->isSideEdge = isSideEdge;
		he->prevFe = prevFe;
		he->nextFe = nextFe;
		he->leftSide = leftSide;
		he->rightSide = rightSide;
		he->topEdge = topEdge;
		he->feReference = feReference;
	}

	virtual CToolHalfedge* getPointer() {
		return this;
	}
};

class CToolFace : public CFace {
public:
	std::vector<CPoint> crossFieldDirection;
};


typedef CToolVertex* VertexHandle;
typedef CToolEdge* EdgeHandle;
typedef CToolHalfedge* HalfedgeHandle;
typedef CToolFace* FaceHandle;
typedef CPoint Point;

class CToolMesh : public CBaseMesh<CToolVertex,CToolEdge,CToolFace,CToolHalfedge>
{
public:
	typedef VertexOutHalfedgeIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> VertexOHalfedgeIter;
	typedef VertexInHalfedgeIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> VertexIHalfedgeIter;
	typedef VertexVertexIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> VertexVertexIter;
	typedef VertexEdgeIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> VertexEdgeIter;
	typedef VertexFaceIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> VertexFaceIter;

	typedef FaceVertexIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> FaceVertexIter;
	typedef FaceEdgeIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> FaceEdgeIter;
	typedef FaceHalfedgeIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> FaceHalfedgeIter;
	
	typedef MeshVertexIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> VertexIter;
	typedef MeshHalfEdgeIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> HalfedgeIter;
	typedef MeshFaceIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> FaceIter;
	typedef MeshEdgeIterator<CToolVertex, CToolEdge, CToolFace, CToolHalfedge> EdgeIter;

	Point normalVertex(VertexHandle vertex);
	Point bisector(HalfedgeHandle he1, HalfedgeHandle he1_next);

	double angle(Point cp1, VertexHandle pivot, Point cp2);
	double angle(HalfedgeHandle bhe, HalfedgeHandle bhe_next);
	double angle(Point cpoint, HalfedgeHandle he);
	
	int deleteVertexMergeFace(VertexHandle);
	int deleteEdgeMergeFace(EdgeHandle); // return 0 : success; return 1 : degenerate
	VertexHandle addVertexOnEdge(EdgeHandle oldEdge);

	// Assumes adjacent faces of the edge are all triangles.
	VertexHandle splitEdge(HalfedgeHandle, Point&);

	// Assumes the two adjacent faces of the edge are both triangles.
	EdgeHandle swapEdge(EdgeHandle);
	HalfedgeHandle edgeRecovery(VertexHandle, VertexHandle); //return a CTHEdgeHandle* source former param and target latter
	list<HalfedgeHandle>* calculateRambdaSet(VertexHandle, VertexHandle);
	list<HalfedgeHandle>* calculateRambdaSet(int i, VertexHandle Nc, VertexHandle Nd); //Deprecated
protected:
	void clearFace(vector<HalfedgeHandle> heList);
public:
	void clearFace(initializer_list<HalfedgeHandle> heList);
	int numQuad(EdgeHandle edge);
	int numQuad(VertexHandle vert);
	int numTriangles(VertexHandle vert);
	bool isQuad(FaceHandle face);
	double getWeight(EdgeHandle he);
	VertexHandle edgeVertex1(EdgeHandle edge);
	VertexHandle edgeVertex2(EdgeHandle edge);
	VertexHandle mergeEdge(VertexHandle va, VertexHandle vb_to_be_deleted);
	void highlight(EdgeHandle edge);
	void highlight(HalfedgeHandle he);
	void highlight(VertexHandle vert);
	void highlight(CPoint p1, CPoint p2);
	void highlightCrossField();
	void highlightEdgeCrossField();
	void highlightSingularCrossField();
	void highlightVertexCrossField();
	//void highlight(initializer_list<HalfedgeHandle> heList);
	//void highlight(initializer_list<VertexHandle> vertList);
	void highlight(initializer_list<Component*> componentList);
	void highlight(CPoint point);
	void topology_assert(bool expr, initializer_list<Component*> componentList = initializer_list<Component*>());
	void updateDebug();
	
	// getter & setters definitions
	void setFront(HalfedgeHandle tar, bool);
	bool isFront(HalfedgeHandle he);
	bool isFront(VertexHandle v);
	// only set attribute of side
	void setSideEdge(HalfedgeHandle side, HalfedgeHandle fe);
	// safe wrap of setLeftSide & setRightSide
	void setSide(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle upSide);
	bool isSideEdge(HalfedgeHandle he);
	HalfedgeHandle getFeReference(HalfedgeHandle he);
	const Point getPoint(VertexHandle v) const;
	void setPoint(VertexHandle v, const Point& p);
	HalfedgeHandle getPrevFe(HalfedgeHandle he);
	void setPrevFe(HalfedgeHandle fe, HalfedgeHandle prev);
	HalfedgeHandle getNextFe(HalfedgeHandle he);
	void setNextFe(HalfedgeHandle fe, HalfedgeHandle next);
	HalfedgeHandle getLeftSide(HalfedgeHandle he);
	HalfedgeHandle getRightSide(HalfedgeHandle he);
protected:
	void setLeftSide(HalfedgeHandle he, HalfedgeHandle left);
	void setRightSide(HalfedgeHandle he, HalfedgeHandle right);
public:
	HalfedgeHandle getTopEdge(HalfedgeHandle he);
	void setTopEdge(HalfedgeHandle he, HalfedgeHandle top);
	bool getNeedTopEdge(HalfedgeHandle he);
	void setNeedTopEdge(HalfedgeHandle he, bool need);
	int getClass(HalfedgeHandle he);
	void buildQuad(HalfedgeHandle left, HalfedgeHandle bottom, HalfedgeHandle right, HalfedgeHandle top) {
		clearFace({ left,bottom,right,top });
	}
	void buildQuad(HalfedgeHandle left, HalfedgeHandle bottom, HalfedgeHandle right) {
		buildQuad(left, bottom, right, edgeRecovery(halfedgeTarget(right), halfedgeSource(left)));
	}
private:
	void setClass(HalfedgeHandle he, int cls);
public:

	CToolMesh();
	int faceEdges(FaceHandle face) {
		int count = 0;
		for (FaceHalfedgeIter fiter(face); !fiter.end(); ++fiter) {
			count++;
		}
		return count;
	}

	void calculateCrossField();

	int frontEdgeSize(HalfedgeHandle fe);

	void splitFace(VertexHandle v1, VertexHandle v2);
	
	double length(HalfedgeHandle he);

	HalfedgeHandle halfedge_handle(VertexHandle v) {
		return vertexHalfedge(v);
	}
	HalfedgeHandle halfedge_handle(FaceHandle v) {
		return faceHalfedge(v);
	}
	HalfedgeHandle halfedge_handle(EdgeHandle v, int i) {
		return edgeHalfedge(v, i);
	}

	HalfedgeHandle sourceTargetHalfedge(VertexHandle v1, VertexHandle v2) {
		return vertexHalfedge(v1, v2);
	}

	FaceHandle face_handle(HalfedgeHandle he) {
		return halfedgeFace(he);
	}

	void disconnect(EdgeHandle edge1, EdgeHandle edge2);
	void disconnect(EdgeHandle edge1);
	bool isDisconnected(EdgeHandle edge);
	void unsetHalfedge(VertexHandle v, HalfedgeHandle he);
	std::vector<FaceHandle> objIdFaceMap;
	std::vector<VertexHandle> objIdVertexMap;
	void write_obj_set_map(string filename) {
		for (FaceIter fIter(this); !fIter.end(); fIter++) {
			objIdFaceMap.push_back(*fIter);
		}
		for (VertexIter vIter(this); !vIter.end(); vIter++) {
			objIdVertexMap.push_back(*vIter);
		}
		this->write_obj(filename.c_str());
	}

	CPoint edgeCrossField(EdgeHandle edge, int id);

	CPoint vertexCrossField(VertexHandle vertex, int id);
	CPoint nearestCrossField(VertexHandle vertex, CPoint direction);
protected:
	int nextVid = 0;
	int nextFid = 0;
#ifdef _DEBUG
	DebuggerConnector debug;
#endif
	double acos_limited(double x) {
		if (x > 1.0) {
			cerr << "WARNING: acos_limited: x > 1.0" << endl;
			return 0.0;
		}
		if (x < -1.0) {
			cerr << "WARNING: acos_limited: x < -1.0" << endl;
			return M_PI;
		}
		return acos(x);
	}
	
};

typedef CToolMesh CTMesh;
#endif