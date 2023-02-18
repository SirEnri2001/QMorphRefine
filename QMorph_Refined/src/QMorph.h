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
	VertexInFrontHeIterator(CTMesh* pMesh, VertexHandle v)
	{
		m_pMesh = pMesh;
		m_vertex = v;
		m_halfedge = pMesh->halfedge_handle(v);
		leftFhe = NULL;
		rightFhe = NULL;
		for (CTMesh::VertexIHalfedgeIter heh(pMesh, v); !heh.end(); ++heh)
		{
			if (pMesh->isFront(*heh)) {
				leftFhe = *heh;
				break;
			}
		}

		for (CTMesh::VertexOHalfedgeIter heh(pMesh, v); !heh.end(); ++heh)
		{
			if (pMesh->isFront(*heh)) {
				rightFhe = *heh;
				break;
			}
		}
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
