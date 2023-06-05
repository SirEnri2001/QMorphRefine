#pragma once
#ifndef _QMorph
#define _QMorph

#include "ToolMesh.h"
#include "Smoother.h"
#include <queue>

typedef enum SideDefineResult {
	Succeeded,
	NoSuitable,
	FrontEdgeContact,
	FrontEdgeContactDegenerate,
	SideEdgeContact,
	QuadContactByVertex,
	QuadContactByEdge
} SideDefineResult;

class VertexInFrontHeIterator
{
public:
	VertexInFrontHeIterator(CTMesh* pMesh, VertexHandle v,HalfedgeHandle fe1,HalfedgeHandle fe2)
	{
		m_pMesh = pMesh;
		m_vertex = v;
		m_halfedge = pMesh->halfedge_handle(v);
		leftFhe = fe1;
		rightFhe = fe2;
		assert(leftFhe);
		assert(rightFhe);
		m_halfedge = leftFhe;
	};

	~VertexInFrontHeIterator() {};

	// Reset the iterator
	void reset() {
		m_halfedge = leftFhe;
	}

	void operator++()	//prefix
	{
		if (m_halfedge == m_pMesh->halfedgePrev(rightFhe))
			m_halfedge = NULL;
		else
			m_halfedge = m_pMesh->halfedgeSym(m_pMesh->halfedgeNext(m_halfedge));
	};

	void operator++(int)	//postfix
	{
		if (m_halfedge == m_pMesh->halfedgePrev(rightFhe))
			m_halfedge = NULL;
		else
			m_halfedge = m_pMesh->halfedgeSym(m_pMesh->halfedgeNext(m_halfedge));
	};

	HalfedgeHandle value() { return m_halfedge; };

	bool end() { return !m_halfedge; };

	HalfedgeHandle operator*() { return value(); };

private:
	CTMesh* m_pMesh;
	VertexHandle m_vertex;
	HalfedgeHandle m_halfedge;

	HalfedgeHandle leftFhe;
	HalfedgeHandle rightFhe;
};

class QMorph
{
public:
	QMorph(CTMesh* tarMesh);
	int doQMorphProcess();
	int doEdgeRecovery();
	int doSideDefine();
	int doClearQuard();
	int doCornerGenerate();
	int initFrontEdgeGroup();
	int doSmooth(int epoch); //when debug, set easySmooth to true
	void highlightAllSides();
	void initQuadTree() {
		/*Rectangle* rootRec = new Rectangle(0, 16, 0, 16);
		QuadTree* root = new QuadTree(*rootRec);
		root->pointList = new PointList();
		Point rootPoint(1, 2);
		root->pointList->point = rootPoint;
		root->parent = NULL;
		root->pointList->next = NULL;
		root->pointList->Insert(Point(2, 9));
		root->pointList->Insert(Point(2, 14));
		root->pointList->Insert(Point(5, 9));
		root->pointList->Insert(Point(5, 11));
		root->pointList->Insert(Point(6, 14));
		root->pointList->Insert(Point(7, 9));
		root->pointList->Insert(Point(7, 11));
		root->pointList->Insert(Point(9, 6));
		root->pointList->Insert(Point(9, 12));
		root->CreateChildren();
		root->leftTopTree->CreateChildren();
		root->leftTopTree->rightBottomTree->CreateChildren();
		root->leftTopTree->rightTopTree->CreateChildren();*/
	}
private:
	CTMesh* mesh;
	Smoother smoother;

	list<HalfedgeHandle> frontEdgeGroups;
	int frontEdgeGroupSize(HalfedgeHandle he) {
		assert(mesh->isFront(he));
		int count = 0;
		HalfedgeHandle heIter = he;
		do {
			count++;
		} while (
			heIter = mesh->getNextFe(heIter), heIter != he);
		return count;
	}

	void pushHeadFrontEdgeGroup(HalfedgeHandle he) {
		assert(mesh->isFront(he));
		frontEdgeGroups.push_front(he);
	}

	void pushTailFrontEdgeGroup(HalfedgeHandle he) {
		assert(mesh->isFront(he));
		frontEdgeGroups.push_back(he);
	}

	// param he: any frontedge in the group
	void removeFrontEdgeGroup(HalfedgeHandle he) {
		HalfedgeHandle heIter = he;
		do {
			frontEdgeGroups.remove(heIter);
		} while (
			heIter = mesh->getNextFe(heIter), heIter != he);
	}

	bool isFrontEdgeGroupIndex(HalfedgeHandle he) {
		return find(frontEdgeGroups.begin(), frontEdgeGroups.end(), he) != frontEdgeGroups.end();
	}

	HalfedgeHandle popFrontEdgeGroup() {
		HalfedgeHandle he = frontEdgeGroups.front();
		frontEdgeGroups.pop_front();
		return he;
	}

public:
	HalfedgeHandle getFrontEdgeGroup() {
		if (frontEdgeGroups.size() > 0) {
			return frontEdgeGroups.front();
		}
		else {
			return NULL;
		}
	}
	bool switchFrontEdgeGroup() {
		pushTailFrontEdgeGroup(getFrontEdgeGroup());
		popFrontEdgeGroup();
		return getFrontEdgeGroup() != NULL;
	}
private:

	void updateHeadFrontEdgeGroup(HalfedgeHandle he) {
		popFrontEdgeGroup();
		pushHeadFrontEdgeGroup(he);
	}

	// these functions only return its finding of side he & its topology state
	SideDefineResult verticalSideSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resultUpSide);
	SideDefineResult horizontalSideSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resultUpSide);
	SideDefineResult verticalSideSplitSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resultUpSide);
	SideDefineResult horizontalSideSplitSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resultUpSide);

	int frontEdgeSideDefine(HalfedgeHandle lfe, HalfedgeHandle rfe);
	int sideDefineQuadFallback(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle downSide);
	int generateCorner(HalfedgeHandle lfe, HalfedgeHandle rfe);
	void findPointForDebug(Point coord, Point target);
	void highlightAllFes();
	int countFeToFe(HalfedgeHandle fe1, HalfedgeHandle fe2);
	bool proceedNextFeLoop(bool);
	void updateClass(HalfedgeHandle);
	void updateFeClassification(HalfedgeHandle feList = HalfedgeHandle());
	int doSeam();
	int seperateFrontLoop(HalfedgeHandle cutPos);
public:
	string toGmshString() {
		initFrontEdgeGroup();
		std::stringstream _os;
		for (CTMesh::VertexIter iter(mesh); !iter.end(); iter++) {
			if (mesh->isBoundary(*iter)) {
				_os << "p" << (*iter)->getId() << " = gmsh.model.geo.addPoint("
					<< mesh->getPoint(*iter)[0] << ", " << mesh->getPoint(*iter)[1] << ", " << mesh->getPoint(*iter)[2] << ")\n";
			}
		}
		int curve = 1;
		for (HalfedgeHandle he : frontEdgeGroups) {
			he = mesh->halfedgeSym(he);
			int i = 1;
			HalfedgeHandle heIter = he;
			do {
				_os << "l" << i << " = gmsh.model.geo.addLine(p" << mesh->halfedgeSource(heIter)->getId() << ", p"
					<< mesh->halfedgeTarget(heIter)->getId() << ")" << endl;
				i++;
			} while (heIter = mesh->halfedgePrev(heIter), heIter != he);
			_os << "c" << curve << " = gmsh.model.geo.addCurveLoop([";
			for (int j = 1; j < i; j++) {
				_os << "l" << j << ", ";
			}
			_os << "])" << endl;
			curve++;
		}
		_os << "pl = gmsh.model.geo.addPlaneSurface([";
		for (int j = 1; j < curve; j++) {
			_os <<"c" << j << ", ";
		}
		_os<< "])";
		return _os.str();
	}
#if COMPILE_DEPRECATED
	int frontEdgeSideDefine(int i, HalfedgeHandle lfe, HalfedgeHandle rfe); //Deprecated
	bool needSeam(HalfedgeHandle ek, HalfedgeHandle el); //Deprecated
	void seam(HalfedgeHandle ej, HalfedgeHandle el); //Deprecated
	void splitSeam1(HalfedgeHandle, HalfedgeHandle); //Deprecated
	void splitSeam2(HalfedgeHandle, HalfedgeHandle); //Deprecated
	void seamLastQuad(HalfedgeHandle ek, HalfedgeHandle el); //Deprecated
	HalfedgeHandle getWellCover(HalfedgeHandle wellBottom); //Deprecated
	HalfedgeHandle fillDeepWell(HalfedgeHandle wellBottom); //Deprecated
	void frontEdgeRecovery(HalfedgeHandle); //Deprecated
	void flipFix(); //Deprecated
#endif
};
//Point normalOfVertex(VertexHandle vertex);
#endif // !_QMorph
