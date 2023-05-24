#include <list>
#include<iostream>
#include<math.h>
#include"QMorph.h"
#include"util.h"

bool debug = false;
int globalIter = 0;
QMorph::QMorph(CTMesh* tarMesh)
{
	assert(tarMesh != NULL);
	mesh = tarMesh;
	smoother.setMesh(mesh);
}

void QMorph::findPointForDebug(Point coord, Point target) {
	if (coord[0] > target[0] - 0.005 && coord[0]<target[0] + 0.005 &&
		coord[1]>target[1] - 0.005 && coord[1] < target[1] + 0.005 &&
		coord[2]>target[2] - 0.005 && coord[2] < target[2] + 0.005) {
		cout << "gotcha!" << endl;
		//mesh->write_obj("debug.obj");
	}
}

void QMorph::highlightAllFes() {
	for (CTMesh::HalfedgeIter he_iter(mesh); !he_iter.end(); he_iter++) {
		if (mesh->isFront(*he_iter)) {
			mesh->highlight(mesh->halfedgeEdge(*he_iter));
		}
	}

	for (auto v_iter = mesh->vertices().begin(); v_iter != mesh->vertices().end(); v_iter++) {
		if (mesh->isFront(*v_iter)) {
			mesh->highlight(*v_iter);
		}
	}
	mesh->updateDebug();
}

void QMorph::highlightAllSides() {
	for (CTMesh::HalfedgeIter he_iter(mesh); !he_iter.end(); he_iter++) {
		if (mesh->isSideEdge(*he_iter)) {
			mesh->highlight(mesh->halfedgeEdge(*he_iter));
		}
	}
	mesh->updateDebug();
}

int QMorph::initFrontEdgeGroup()
{
	int i = 0;

	for (CTMesh::HalfedgeIter heiter(mesh); !heiter.end(); heiter++) {
		if (mesh->isBoundary(*heiter) && !mesh->isFront(mesh->halfedgeSym(*heiter))) {
			HalfedgeHandle curHe = *heiter;
			do {
				HalfedgeHandle curFe = mesh->halfedgeSym(curHe);
				HalfedgeHandle nextFe = mesh->halfedgeSym(mesh->halfedgePrev(curHe));
				mesh->setFront(curFe, true);
				mesh->setFront(nextFe, true);
				mesh->topology_assert(mesh->halfedgeTarget(curFe) == mesh->halfedgeSource(nextFe), { curFe, nextFe });
				mesh->setNextFe(mesh->halfedgeSym(curHe), mesh->halfedgeSym(mesh->halfedgePrev(curHe)));
				curHe = mesh->halfedgePrev(curHe);
				i++;
			} while (curHe != *heiter);
			pushTailFrontEdgeGroup(mesh->halfedgeSym(*heiter));
		}
	}
	if (i % 2 != 0)
	{
		HalfedgeHandle longestHE = getFrontEdgeGroup();
		HalfedgeHandle he = longestHE;
		do {
			if (eDist(mesh->halfedgeSource(he)->point(), mesh->halfedgeTarget(he)->point()) >
				eDist(mesh->halfedgeSource(longestHE)->point(), mesh->halfedgeTarget(longestHE)->point())) {
				longestHE = he;
			}
		} while ((he = mesh->getNextFe(he)) != longestHE);
		HalfedgeHandle prevFe, nextFe;
		VertexHandle v1, v2, vNew;
		prevFe = mesh->getPrevFe(longestHE);
		nextFe = mesh->getNextFe(longestHE);

		v1 = mesh->halfedgeSource(longestHE);
		v2 = mesh->halfedgeTarget(longestHE);
		mesh->setFront(longestHE, false);
		vNew = mesh->splitEdge(longestHE,
			(mesh->getPoint(mesh->halfedgeTarget(longestHE)) + mesh->getPoint(mesh->halfedgeSource(longestHE))) / 2);

		mesh->setFront(mesh->sourceTargetHalfedge(v1, vNew), true);
		mesh->setFront(mesh->sourceTargetHalfedge(vNew, v2), true);

		mesh->setNextFe(prevFe, mesh->sourceTargetHalfedge(v1, vNew));
		mesh->setNextFe(mesh->sourceTargetHalfedge(v1, vNew), mesh->sourceTargetHalfedge(vNew, v2));
		mesh->setNextFe(mesh->sourceTargetHalfedge(vNew, v2), nextFe);
		updateHeadFrontEdgeGroup(prevFe);

	}
	return 0;
}

SideDefineResult QMorph::verticalSideSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resultUpSide) {
	mesh->topology_assert(mesh->halfedgeTarget(lfe) == mesh->halfedgeSource(rfe), { lfe, rfe });
	HalfedgeHandle minAngleHe = lfe;
	VertexHandle pivotVertex = mesh->halfedgeTarget(lfe);
	Point pivot = mesh->getPoint(pivotVertex);
	Point bisector = mesh->bisector(lfe, rfe);
	VertexInFrontHeIterator pivotIter(mesh, pivotVertex,lfe,rfe);
	double minAngle = 360.0;
	bool traversed = false;
	for (; !pivotIter.end(); pivotIter++) {
		if (mesh->isBoundary(mesh->halfedgeSym(*pivotIter))) {
			continue;
		}
		traversed = true;
		double angle = mesh->angle(bisector + pivot, mesh->halfedgeSym(*pivotIter));
		if (angle < minAngle) {
			minAngle = angle;
			minAngleHe = *pivotIter;
		}
	}
	if (!traversed) {
		return SideDefineResult::NoSuitable;
	}
	resultUpSide = mesh->halfedgeSym(minAngleHe);
	if (mesh->halfedgeSource(mesh->getPrevFe(lfe)) == mesh->halfedgeSource(minAngleHe)
		|| mesh->halfedgeTarget(mesh->getNextFe(rfe)) == mesh->halfedgeSource(minAngleHe)) {
		// size 3 front edge group
		return SideDefineResult::FrontEdgeContactDegenerate;
	}
	if (mesh->isFront(mesh->halfedgeSource(minAngleHe))) {
		return SideDefineResult::FrontEdgeContact;
	}
	if (mesh->numQuad(mesh->halfedgeSource(minAngleHe)) > 0) {
		return SideDefineResult::QuadContactByVertex;
	}
	if (mesh->halfedgeSource(minAngleHe)->isSide) {
		return SideDefineResult::SideEdgeContact;
	}
	if (minAngle < constEpsilon) {
		return SideDefineResult::Succeeded;
	}
	else {
		return SideDefineResult::NoSuitable;
	}
}

SideDefineResult QMorph::horizontalSideSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resMinAH) {
	mesh->topology_assert(mesh->halfedgeTarget(lfe) == mesh->halfedgeSource(rfe), { lfe, rfe });
	HalfedgeHandle minAngleHe = lfe;
	VertexHandle pivotVertex = mesh->halfedgeTarget(lfe);
	Point pivot = mesh->getPoint(pivotVertex);
	Point bisector = mesh->bisector(lfe, rfe);
	VertexInFrontHeIterator pivotIter(mesh, pivotVertex,lfe,rfe);
	bool traversed = false;
	double minAngle = 360.0;
	// traverse all quadrilateral surround the vertex
	//(mAV)<--.
	//   |   / \
	//   |  /   \
	//   | /     \
	//   v--minAH>(pV)
	VertexHandle minAngleVertex = mesh->halfedgeSource(lfe);
	for (pivotIter.reset(); !pivotIter.end(); pivotIter++)
	{
		if (mesh->isFront(mesh->halfedgePrev(*pivotIter))
			||mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(mesh->halfedgePrev(*pivotIter))))) {
			continue;
		}
		traversed = true;
		VertexHandle curVertex =
			mesh->halfedgeTarget(mesh->halfedgeNext(mesh->halfedgeSym(mesh->halfedgePrev(*pivotIter))));
		double angle = mesh->angle(bisector + pivot, pivotVertex, mesh->getPoint(curVertex));
		if (angle < minAngle) {
			minAngle = angle;
			minAngleHe = *pivotIter;
			minAngleVertex = curVertex;
		}
	}
	if (!traversed) {
		return SideDefineResult::NoSuitable;
	}
	resMinAH = minAngleHe;
	if (minAngleVertex == mesh->halfedgeSource(mesh->getPrevFe(lfe)) ||
		minAngleVertex == mesh->halfedgeTarget(mesh->getNextFe(rfe))) {
		return SideDefineResult::FrontEdgeContactDegenerate;
	}
	if (mesh->isFront(minAngleVertex)) {
		return SideDefineResult::FrontEdgeContact;
	}
	if (mesh->numQuad(minAngleVertex)) {
		return SideDefineResult::QuadContactByVertex;
	}
	if (minAngleVertex->isSide) {
		return SideDefineResult::SideEdgeContact;
	}
	if (minAngle < constEpsilon
		&& 2.0 * eDist(mesh->getPoint(minAngleVertex), mesh->getPoint(mesh->halfedgeTarget(lfe))) <
		sqrt(3) * (mesh->length(lfe) + mesh->length(rfe))) {  //safisty second condition
		return SideDefineResult::Succeeded;
	}
	else {
		return SideDefineResult::NoSuitable;
	}
}

SideDefineResult QMorph::horizontalSideSplitSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resMinAH) {
	mesh->topology_assert(mesh->halfedgeTarget(lfe) == mesh->halfedgeSource(rfe), { lfe, rfe });
	HalfedgeHandle minAngleHe = lfe;
	VertexHandle pivotVertex = mesh->halfedgeTarget(lfe);
	Point pivot = mesh->getPoint(pivotVertex);
	Point bisector = mesh->bisector(lfe, rfe);
	Point leftLocal = mesh->getPoint(mesh->halfedgeSource(lfe)) - pivot;
	Point rightLocal = mesh->getPoint(mesh->halfedgeTarget(rfe)) - pivot;
	Point cs = leftLocal.cross(bisector);
	cout << cs[0] << cs[1] << cs[2];
	VertexInFrontHeIterator pivotIter(mesh, pivotVertex,lfe,rfe);
	double minAngle = 360.0;
	
	HalfedgeHandle he2 = minAngleHe;


	for (pivotIter.reset(); !pivotIter.end(); pivotIter++) {
		if ((bisector.cross(mesh->getPoint(mesh->halfedgeSource(*pivotIter)) - pivot))
			.dot(bisector.cross(mesh->getPoint(mesh->halfedgeTarget(mesh->halfedgeNext(*pivotIter))) - pivot)) < 0) {
			he2 = *pivotIter;
		}
	}
	//(mAV)<----.
	//   |    /   \
	//   |   /     he1
	//   | /          \
	//   v--minAH(he2)-->(pV)
	resMinAH = he2;
	HalfedgeHandle he1 = mesh->halfedgeNext(he2);
	// 2 halfedges in a patch: --he2-->.--he1-->
	if (mesh->halfedgeSource(he2)!=mesh->halfedgeSource(lfe) && mesh->isFront(mesh->halfedgePrev(he2))) {
		return SideDefineResult::FrontEdgeContactDegenerate;
	}
	if (mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(mesh->halfedgePrev(he2))))) {
		return SideDefineResult::QuadContactByEdge;
	}
	if (mesh->halfedgeSource(he2)->isSide||mesh->halfedgeTarget(he1)->isSide) {
		return SideDefineResult::SideEdgeContact;
	}
	return SideDefineResult::Succeeded;
	//Note: if lfe & rfe are on one corner, lfe->target == rfe->source, and no degeneracy detected, 
	// then this function must return Succeeded
}

SideDefineResult QMorph::verticalSideSplitSeek(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle& resultUpSide) {
	mesh->topology_assert(mesh->halfedgeTarget(lfe) == mesh->halfedgeSource(rfe), { lfe, rfe });
	mesh->topology_assert(mesh->halfedgeNext(lfe) != rfe, { lfe, rfe });
	HalfedgeHandle minAngleHe = lfe;
	VertexHandle pivotVertex = mesh->halfedgeTarget(lfe);
	Point pivot = mesh->getPoint(pivotVertex);
	Point bisector = mesh->bisector(lfe, rfe);
	VertexInFrontHeIterator pivotIter(mesh, pivotVertex,lfe,rfe);
	double minAngle = 360.0;

	for (; !pivotIter.end(); pivotIter++) {
		if (mesh->isBoundary(mesh->halfedgeSym(*pivotIter))) {
			continue;
		}
		double angle = mesh->angle(bisector + pivot, mesh->halfedgeSym(*pivotIter));
		if (angle < minAngle) {
			minAngle = angle;
			minAngleHe = *pivotIter;
		}
	}
	resultUpSide = mesh->halfedgeSym(minAngleHe);
	mesh->topology_assert(minAngleHe != lfe, { lfe });
	if (mesh->halfedgeSource(mesh->getPrevFe(lfe)) == mesh->halfedgeSource(minAngleHe)
		|| mesh->halfedgeTarget(mesh->getNextFe(rfe)) == mesh->halfedgeSource(minAngleHe)) {
		// size 3 front edge group
		return SideDefineResult::FrontEdgeContactDegenerate;
	}
	if (mesh->isFront(mesh->halfedgeSource(minAngleHe))) {
		return SideDefineResult::FrontEdgeContact;
	}
	return SideDefineResult::Succeeded;
}


int QMorph::frontEdgeSideDefine(HalfedgeHandle lfe, HalfedgeHandle rfe) {
	if (mesh->isQuad(mesh->halfedgeFace(lfe))) {
		mesh->setSide(lfe, rfe, mesh->halfedgeNext(lfe));
		return 0;
	}
	if (mesh->isQuad(mesh->halfedgeFace(rfe))) {
		mesh->setSide(lfe, rfe, mesh->halfedgeSym(mesh->halfedgePrev(rfe)));
		return 0;
	}
	HalfedgeHandle resultUpSide;
	switch (verticalSideSeek(lfe, rfe, resultUpSide))
	{
	case SideDefineResult::FrontEdgeContact:
		seperateFrontLoop(resultUpSide);
		return 1;
	case SideDefineResult::Succeeded:
		mesh->setSide(lfe, rfe, resultUpSide);
		return 0;
	default:
		break;
	}

	switch (horizontalSideSeek(lfe, rfe, resultUpSide)) {
	case SideDefineResult::FrontEdgeContact:
		mesh->swapEdge(mesh->halfedgeEdge(mesh->halfedgePrev(resultUpSide)));
		seperateFrontLoop(mesh->halfedgeNext(resultUpSide));
		return 1;
	case SideDefineResult::Succeeded:
		mesh->swapEdge(mesh->halfedgeEdge(mesh->halfedgePrev(resultUpSide)));
		mesh->setSide(lfe, rfe, mesh->halfedgeNext(resultUpSide));
		return 0;
	default:
		break;
	}
	if (horizontalSideSplitSeek(lfe, rfe, resultUpSide) == SideDefineResult::Succeeded) {
		HalfedgeHandle he2 = resultUpSide;
		HalfedgeHandle he1 = mesh->halfedgeNext(he2);
		VertexHandle pivotVertex = mesh->halfedgeTarget(lfe);
		// solve the 2x2 linear system
		Point v1 = mesh->getPoint(mesh->halfedgeSource(he2)) - mesh->getPoint(mesh->halfedgeTarget(he2));
		Point v2 = mesh->getPoint(mesh->halfedgeTarget(he1)) - mesh->getPoint(mesh->halfedgeTarget(he2));
		Point bisector = mesh->bisector(lfe, rfe);
		double* km = solveKMEquation(v1, v2, bisector);
		Point p = bisector * (km[0] + EPSILON * 10.0) + mesh->getPoint(pivotVertex);
		VertexHandle spliter = mesh->splitEdge(mesh->halfedgePrev(he2), p);
		HalfedgeHandle newHe = mesh->sourceTargetHalfedge(pivotVertex, spliter);
		assert(newHe);
		mesh->setSide(lfe, rfe, newHe);
		return 0;
	}

	switch (verticalSideSplitSeek(lfe, rfe, resultUpSide))
	{
	case SideDefineResult::FrontEdgeContact:
	case SideDefineResult::FrontEdgeContactDegenerate:
		seperateFrontLoop(resultUpSide);
		return 1;
	case SideDefineResult::Succeeded:
		mesh->topology_assert(!mesh->isQuad(mesh->halfedgeFace(resultUpSide)), { resultUpSide });
		mesh->topology_assert(!mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(resultUpSide))), { resultUpSide });
		mesh->setSide(lfe, rfe, NULL);
		VertexHandle v = mesh->splitEdge(resultUpSide,
			(mesh->getPoint(mesh->halfedgeSource(resultUpSide)) + mesh->getPoint(mesh->halfedgeTarget(resultUpSide))) / 2);
		resultUpSide = mesh->sourceTargetHalfedge(mesh->halfedgeTarget(lfe), v);
		mesh->setSide(lfe, rfe, resultUpSide);
		return 0;
	}
	assert(false);
}

int QMorph::doSideDefine() {
	HalfedgeHandle lfe, rfe;
	int i = 0;
	// traverse all front edge
	lfe = getFrontEdgeGroup();
	i = 0;
	lfe = getFrontEdgeGroup();
	do {
		reportIter(i++, "doSideDefine");
		rfe = mesh->getNextFe(lfe);
		if (!mesh->getNeedTopEdge(lfe) && !mesh->getNeedTopEdge(rfe)) {
			continue;
		}
		int retVal = frontEdgeSideDefine(lfe, rfe);
		if (retVal != 0) {
			return -1;
		}

	} while (lfe = mesh->getNextFe(lfe), lfe != getFrontEdgeGroup());
	return 0;
}

int QMorph::doEdgeRecovery() {
	HalfedgeHandle iter = getFrontEdgeGroup();
	int i = 0;
	do {
		reportIter(i++, "doEdgeRecovery");
		if (mesh->getNeedTopEdge(iter)) {
			HalfedgeHandle frontHe = iter;
			// left and right side & topology check
			mesh->topology_assert(mesh->getLeftSide(frontHe) && mesh->getRightSide(frontHe), { frontHe });
			mesh->topology_assert(mesh->halfedgeSource(frontHe) == mesh->halfedgeTarget(mesh->getLeftSide(frontHe)), { frontHe, mesh->getLeftSide(frontHe) });
			mesh->topology_assert(mesh->halfedgeTarget(frontHe) == mesh->halfedgeSource(mesh->getRightSide(frontHe)), { frontHe, mesh->getRightSide(frontHe) });
			mesh->buildQuad(mesh->getLeftSide(frontHe), frontHe, mesh->getRightSide(frontHe));
			mesh->setTopEdge(frontHe, mesh->halfedgeNext(mesh->halfedgeNext(frontHe)));
		}
	} while (iter = mesh->getNextFe(iter), iter != getFrontEdgeGroup());
	return 0;
}


int QMorph::doSmooth(int epoch = 3) {
	int i = 0;
	while (i < epoch) {
		reportIter(i++, "doSmooth");
		for (CTMesh::VertexIter vIter(mesh); !vIter.end(); vIter++) {
			int numQuad = mesh->numQuad(*vIter);
			int numTriangles = mesh->numTriangles(*vIter);
			bool isBoundary = mesh->isBoundary(*vIter);
			if (isBoundary) {
				continue;
			}
			smoother.triangleInteriorSmooth(*vIter, true);
		}
	}
	//int i = 0;
	//while (i < epoch) {
	//	reportIter(i++, "doSmooth");
	//	for (CTMesh::VertexIter vIter(mesh); !vIter.end(); vIter++) {
	//		int numQuad = mesh->numQuad(*vIter);
	//		int numTriangles = mesh->numTriangles(*vIter);
	//		bool isBoundary = mesh->isBoundary(*vIter);
	//		if (isBoundary) {
	//			continue;
	//		}
	//		if (numQuad>0 && numTriangles>0 || mesh->isFront(*vIter)) {
	//			// get ife & ofe
	//			HalfedgeHandle ife = NULL, ofe = NULL;
	//			HalfedgeHandle he = mesh->vertexHalfedge(*vIter);
	//			if (mesh->isFront(*vIter)) {
	//				do {
	//					if (ife==NULL && mesh->isFront(he)) {
	//						ife = he;
	//					}
	//					else if (mesh->isFront(mesh->halfedgeSym(he))) {
	//						ofe = mesh->halfedgeSym(he);
	//					}
	//				} while (he = mesh->halfedgePrev(mesh->halfedgeSym(he)),
	//					he != mesh->vertexHalfedge(*vIter));
	//			}
	//			else {
	//				do {
	//					if (!mesh->isQuad(mesh->halfedgeFace(he))
	//						&& mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(he)))) {
	//						ife = he;
	//					}
	//					else if (mesh->isQuad(mesh->halfedgeFace(he))
	//						&& !mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(he)))) {
	//						ofe = mesh->halfedgeSym(he);
	//					}
	//				} while (he = mesh->halfedgePrev(mesh->halfedgeSym(he)),
	//					he != mesh->vertexHalfedge(*vIter));
	//			}
	//			
	//			if (!ife || !ofe) {
	//				mesh->highlight(*vIter);
	//				cout << (*vIter)->getId() << endl;
	//				mesh->updateDebug();
	//				assert(false);
	//			}
	//			smoother.boundaryEdgeSmooth(ife, ofe);
	//		}
	//	}
	//}
	//i = 0;
	//while (i < epoch) {
	//	reportIter(i++, "doSmooth");
	//	for (CTMesh::VertexIter vIter(mesh); !vIter.end(); vIter++) {
	//		int numQuad = mesh->numQuad(*vIter);
	//		int numTriangles = mesh->numTriangles(*vIter);
	//		bool isBoundary = mesh->isBoundary(*vIter);
	//		assert(!easySmooth);
	//		if (isBoundary) {
	//			continue;
	//		}
	//		if (numQuad > 0 && numTriangles > 0 || mesh->isFront(*vIter)) {
	//			continue;
	//		}
	//		else if (numQuad > 0 && numTriangles == 0) {
	//			smoother.quadriInteriorSmooth(*vIter);
	//		}
	//		else if (numQuad == 0 && numTriangles > 0) {
	//			smoother.triangleInteriorSmooth(*vIter, true);
	//		}
	//		else {
	//			assert(false);
	//		}
	//	}
	//}

	return 0;
}

int QMorph::generateCorner(HalfedgeHandle lfe, HalfedgeHandle rfe) {
	mesh->setSide(mesh->getPrevFe(lfe), lfe, NULL);
	mesh->setSide(lfe, rfe, NULL);
	mesh->setSide(rfe, mesh->getNextFe(rfe), NULL);
	if (mesh->isQuad(mesh->halfedgeFace(mesh->getNextFe(rfe)))) {
		mesh->buildQuad(lfe, rfe, mesh->halfedgeSym(mesh->halfedgePrev(mesh->getNextFe(rfe))));
		mesh->setTopEdge(rfe, mesh->halfedgePrev(lfe));
		mesh->setSide(mesh->getPrevFe(lfe), lfe, mesh->halfedgeSym(mesh->getTopEdge(rfe)));
	}
	else if (mesh->isQuad(mesh->halfedgeFace(mesh->getPrevFe(lfe)))) {
		mesh->buildQuad(mesh->halfedgeSym(mesh->halfedgeNext(mesh->getPrevFe(lfe))), lfe, rfe);
		mesh->setTopEdge(lfe, mesh->halfedgeNext(rfe));
		mesh->setSide(rfe, mesh->getNextFe(rfe), mesh->getTopEdge(lfe));
	}
	else {
		if (frontEdgeSideDefine(mesh->getPrevFe(lfe), lfe)) { //fail to define side edge
			return 1; //reclassify corner must be called again
		}
		mesh->buildQuad(mesh->getLeftSide(lfe), lfe, rfe);
		mesh->setSide(mesh->getPrevFe(lfe), lfe, mesh->halfedgeSym(mesh->halfedgePrev(lfe)));
		mesh->setSide(rfe, mesh->getNextFe(rfe), mesh->halfedgeNext(rfe));
	}
	mesh->setNeedTopEdge(lfe, false);
	mesh->setNeedTopEdge(rfe, false);
	return 0;
}

// This function deals with corner case for frontEdges
int QMorph::doCornerGenerate() {
	int i = 0;
	bool init = true;
	HalfedgeHandle iter = getFrontEdgeGroup();
	do {
		reportIter(i++, "doCornerGenerate");
		HalfedgeHandle lfe = iter;
		HalfedgeHandle rfe = mesh->getNextFe(lfe);
		if (init && mesh->getClass(lfe) / 2 == 1) {
			init = false;
			continue;
		}
		if (!mesh->getNeedTopEdge(lfe)) {
			continue;
		}
		if (mesh->isQuad(mesh->halfedgeFace(rfe))) {
			continue;
		}
		if (mesh->getClass(lfe) % 2 == 1) {
			if (mesh->getClass(rfe) % 2 == 1) { // form a ``|_|`` shape
				HalfedgeHandle
					lfePrev = mesh->getPrevFe(lfe),
					rfeNext = mesh->getNextFe(rfe),
					nnRfe = mesh->getNextFe(rfeNext);
				if (mesh->isQuad(mesh->halfedgeFace(mesh->getPrevFe(lfe))) || mesh->isQuad(mesh->halfedgeFace(mesh->getNextFe(rfe)))) {
					if (globalIter == 115) {
						mesh->highlight({ lfe,rfe });
						mesh->updateDebug();
					}
					generateCorner(lfe, rfe);
					return 1;
				}
				mesh->setNeedTopEdge(lfe, false);
				mesh->setNeedTopEdge(rfe, false);
				mesh->setNeedTopEdge(rfeNext, false);
				mesh->setSide(lfePrev, lfe, NULL);
				mesh->setSide(rfeNext, nnRfe, NULL);
				mesh->setFront(rfeNext, false);
				mesh->setFront(rfe, false);
				mesh->setFront(lfe, false);
				mesh->buildQuad(lfe, rfe, rfeNext);
				HalfedgeHandle newFe = mesh->halfedgeSym(mesh->halfedgeNext(rfeNext));
				mesh->setFront(newFe, true);
				mesh->setPrevFe(nnRfe, newFe);
				mesh->setNextFe(lfePrev, newFe);
				updateHeadFrontEdgeGroup(newFe);
				
				return 1;
			}
			else { // form a ``|_.. shape
				generateCorner(lfe, rfe);
				return 1;
			}
		}

	} while (iter = mesh->getNextFe(iter), iter != getFrontEdgeGroup());
	return 0;
}

void QMorph::updateFeClassification(HalfedgeHandle feList) {
	if (!feList) {
		feList = getFrontEdgeGroup();
	}
	// This assertion asures a circle stores in the list.
	mesh->topology_assert(mesh->isFront(feList), { feList });
	HalfedgeHandle fe = feList;
	do {
		mesh->setNextFe(fe, mesh->getNextFe(fe));
		fe = mesh->getNextFe(fe);
	} while (fe != feList);
}

bool QMorph::proceedNextFeLoop(bool reclasssify = true)
{
	HalfedgeHandle newFhe = NULL;
	HalfedgeHandle prevNewFhe = NULL;
	HalfedgeHandle newFheHead = NULL;
	if (mesh->frontEdgeSize(getFrontEdgeGroup()) == 4) {
		return false;
	}
	HalfedgeHandle fhe = getFrontEdgeGroup();
	int i = 0;
	do
	{
		reportIter(i++, "updateCurFront");
		if (mesh->getNextFe(fhe) == mesh->halfedgeSym(fhe)) {
			mesh->setFront(fhe, false);
			mesh->setFront(mesh->getNextFe(fhe), false);
			fhe = mesh->getNextFe(fhe);
		}
		if (!mesh->getNeedTopEdge(fhe)) 
		{
			if (mesh->getNextFe(fhe) == mesh->halfedgeNext(fhe)) {
				continue;
			}
			if (!mesh->isBoundary(mesh->halfedgeSym(mesh->halfedgeNext(fhe)))
				&&!mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(mesh->halfedgeNext(fhe))))) {
				HalfedgeHandle lfe = NULL, rfe = NULL;
				for (CTMesh::VertexIHalfedgeIter viIter(mesh, mesh->halfedgeTarget(fhe)); !viIter.end(); viIter++) {
					if (!mesh->isQuad(mesh->halfedgeFace(*viIter)) && mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(*viIter)))) {
						lfe = *viIter;
					}
					if (mesh->isQuad(mesh->halfedgeFace(*viIter)) && !mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(*viIter)))) {
						rfe = mesh->halfedgeSym(*viIter);
					}
				}
				assert(lfe && rfe);
				mesh->setFront(lfe, true);
				mesh->setFront(rfe, true);
				mesh->setNextFe(prevNewFhe, lfe);
				mesh->setNextFe(lfe, rfe);
				prevNewFhe = rfe;
			}
			continue;
		}

		newFhe = mesh->halfedgeSym(mesh->getTopEdge(fhe));

		
		if (mesh->numQuad(mesh->halfedgeEdge(newFhe)) == 1)
		{
			if (!newFheHead) {
				newFheHead = newFhe;
			}
			// Build the new front edge list
			// Mark all the front elements.
			mesh->setFront(newFhe, true);
			if (prevNewFhe) {
				mesh->setPrevFe(newFhe, prevNewFhe);
				mesh->topology_assert(mesh->halfedgeTarget(prevNewFhe) == mesh->halfedgeSource(newFhe), { prevNewFhe, newFhe });
			}
			if (newFhe) {
				prevNewFhe = newFhe;
			}
		}
		else if (mesh->numQuad(mesh->halfedgeEdge(newFhe)) == 2) {
			continue;
		}
		else {
			cerr << "error updateCurFe" << endl;
			mesh->highlight(newFhe);
			mesh->updateDebug();
			assert(false);
		}
	} while (mesh->setFront(fhe, false), fhe = mesh->getNextFe(fhe), fhe != getFrontEdgeGroup());
	if (prevNewFhe) {
		mesh->setNextFe(prevNewFhe, newFheHead);
		updateHeadFrontEdgeGroup(newFheHead);
	}
	else {
		popFrontEdgeGroup();
	}
	return true;
}

// Only perform merge seam for now
int QMorph::doSeam()
{
	HalfedgeHandle feIter = getFrontEdgeGroup();
	int i = 0;
	do {
		reportIter(i++, "doSeam");
		mesh->topology_assert(mesh->getNextFe(feIter), { feIter });
		if (mesh->isBoundary(mesh->halfedgeSym(feIter))
			|| mesh->isQuad(mesh->halfedgeFace(feIter))
			|| mesh->isQuad(mesh->halfedgeFace(mesh->getNextFe(feIter)))) {
			continue;
		}
		if (mesh->angle(feIter, mesh->getNextFe(feIter)) < seamEpsilon) {
			if (i == 64&&globalIter==27) {
				mesh->highlight({ feIter,mesh->getNextFe(feIter) });
				mesh->updateDebug();
			}
			// v1p--prevFe--v1---v2n---nextNextFe--v2nn
			//              | nextFe
			//           feIter /
			//              |  /
			//              | /
			//              v2
			
			HalfedgeHandle nextFe = mesh->getNextFe(feIter);
			HalfedgeHandle nextNextFe = mesh->getNextFe(mesh->getNextFe(feIter));
			HalfedgeHandle prevFe = mesh->getPrevFe(feIter);
			VertexHandle v1p = mesh->halfedgeSource(prevFe);
			VertexHandle v1 = mesh->halfedgeTarget(prevFe);
			VertexHandle v2 = mesh->halfedgeTarget(feIter);
			VertexHandle v2n = mesh->halfedgeTarget(nextFe);
			VertexHandle v2nn = mesh->halfedgeTarget(nextNextFe);
			mesh->setSide(prevFe, feIter, NULL);
			mesh->setSide(feIter, nextFe, NULL);
			mesh->setSide(nextFe, nextNextFe, NULL);
			mesh->setFront(feIter, false);
			mesh->setFront(nextFe, false);
			typedef HalfedgeHandle HalfedgeAttribute; // TODO: encapsulation
			// save attributes
			//HalfedgeAttribute
				//revFeAttribute = new CToolHalfedge(),
				//nextNextFeAttribute = new CToolHalfedge();

			//prevFe->attributeCopyTo(prevFeAttribute);
			//nextNextFe->attributeCopyTo(nextNextFeAttribute);
			mesh->edgeRecovery(v1, v2n);
			
			// TODO: might delete edge with special attributes
			mesh->clearFace({ feIter,nextFe, mesh->vertexHalfedge(v2n, v1)});
			if (i == 64 && globalIter == 27) {
				mesh->highlight({ feIter,mesh->getNextFe(feIter) });
				mesh->updateDebug();
			}
			v1 = mesh->mergeEdge(v1, v2n);
			// v1p--prevFe-> v1 --nextNextFe-> v2nn
			//               |
			//               |
			//               v2
			prevFe = mesh->vertexHalfedge(v1p, v1);
			nextNextFe = mesh->vertexHalfedge(v1, v2nn);
			//prevFeAttribute->attributeCopyTo(prevFe);
			//nextNextFeAttribute->attributeCopyTo(nextNextFe);
			//delete prevFeAttribute;
			//delete feIterAttribute;
			//delete nextFeAttribute;
			//delete nextNextFeAttribute;
			
			mesh->setNextFe(prevFe, nextNextFe);
			mesh->setNextFe(mesh->getPrevFe(prevFe), prevFe);
			mesh->setNextFe(nextNextFe, mesh->getNextFe(nextNextFe));
			feIter = prevFe;
			updateHeadFrontEdgeGroup(feIter);
			return -1;
		}
	} while (feIter = mesh->getNextFe(feIter), feIter != getFrontEdgeGroup());
	return 0;
}

// result including fe1 & fe2 itself, means if getNextFe(fe1)==fe2, return 2
int QMorph::countFeToFe(HalfedgeHandle fe1, HalfedgeHandle fe2) {
	int count = 1;
	HalfedgeHandle feIter = fe1;
	while (feIter != fe2) {
		count++;
		feIter = mesh->getNextFe(feIter);
	}
	return count;
}

// This function maintains 
int QMorph::seperateFrontLoop(HalfedgeHandle cutPos) {
	mesh->topology_assert(mesh->isFront(mesh->halfedgeTarget(cutPos)) && mesh->isFront(mesh->halfedgeSource(cutPos)), { cutPos });
	mesh->topology_assert(!mesh->isFront(cutPos) && !mesh->isFront(mesh->halfedgeSym(cutPos)), { cutPos });
	VertexHandle vertex1 = mesh->halfedgeTarget(cutPos);
	VertexHandle vertex2 = mesh->halfedgeSource(cutPos);
	HalfedgeHandle fe1, fe2, fe3, fe4;
	fe3 = mesh->halfedgeNext(cutPos);
	while (!mesh->isFront(fe3)) {
		fe3 = mesh->halfedgeNext(mesh->halfedgeSym(fe3));
	}
	fe2 = mesh->getPrevFe(fe3);

	fe1 = mesh->halfedgeNext(mesh->halfedgeSym(cutPos));
	while (!mesh->isFront(fe1)) {
		fe1 = mesh->halfedgeNext(mesh->halfedgeSym(fe1));
	}
	fe4 = mesh->getPrevFe(fe1);
	// <--fe1-(v2)<-fe4---
	//         ^|
	//         |(cutPos)
	//         |v
	//---fe2->(v1)--fe3-->


	if (mesh->halfedgeTarget(fe3) == mesh->halfedgeSource(fe4)
		|| mesh->halfedgeTarget(fe1) == mesh->halfedgeSource(fe2)) {
		mesh->swapEdge(mesh->halfedgeEdge(cutPos));
		return 0;
	}

	HalfedgeHandle fe_iter = fe3;
	int count_fe1 = 2, count_fe2 = 2;
	bool splitFe = false;
	while (fe_iter != fe2) {
		fe_iter = mesh->getNextFe(fe_iter);
		count_fe1++;
		if (fe_iter == fe4) {
			splitFe = true;
			break;
		}
	}
	if (count_fe1 < 4) {
		return -1;
	}
	fe_iter = fe1;
	while (fe_iter != fe4) {
		fe_iter = mesh->getNextFe(fe_iter);
		count_fe2++;
		if (fe_iter == fe2) {
			assert(splitFe);
			break;
		}
	}
	if (count_fe2 < 4) {
		return -1;
	}
	removeFrontEdgeGroup(fe1);
	removeFrontEdgeGroup(fe2);
	mesh->setSide(fe4, fe1, NULL);
	mesh->setSide(fe2, fe3, NULL);
	// ensure the connectivity (bidirection linked list)
	if (min(count_fe1, count_fe2) % 2 == 1) {
		VertexHandle mid = mesh->splitEdge(cutPos, (mesh->getPoint(vertex1) + mesh->getPoint(vertex2)) / 2.0);
		mesh->setFront(mesh->vertexHalfedge(vertex2, mid), true);
		mesh->setFront(mesh->vertexHalfedge(vertex1, mid), true);
		mesh->setFront(mesh->vertexHalfedge(mid, vertex2), true);
		mesh->setFront(mesh->vertexHalfedge(mid, vertex1), true);
		mesh->setNextFe(fe4, mesh->vertexHalfedge(vertex2, mid));
		mesh->setNextFe(mesh->vertexHalfedge(vertex2, mid), mesh->vertexHalfedge(mid, vertex1));
		mesh->setNextFe(mesh->vertexHalfedge(mid, vertex1), fe3);
		mesh->setNextFe(fe2, mesh->vertexHalfedge(vertex1, mid));
		mesh->setNextFe(mesh->vertexHalfedge(vertex1, mid), mesh->vertexHalfedge(mid, vertex2));
		mesh->setNextFe(mesh->vertexHalfedge(mid, vertex2), fe1);
	}
	else if (true || count_fe1 % 2 == 0 && count_fe2 % 2 == 0) {
		mesh->setFront(mesh->vertexHalfedge(vertex2, vertex1), true);
		mesh->setFront(mesh->vertexHalfedge(vertex1, vertex2), true);
		mesh->setNextFe(fe4, mesh->vertexHalfedge(vertex2, vertex1));
		mesh->setNextFe(mesh->vertexHalfedge(vertex2, vertex1), fe3);
		mesh->setNextFe(fe2, mesh->vertexHalfedge(vertex1, vertex2));
		mesh->setNextFe(mesh->vertexHalfedge(vertex1, vertex2), fe1);
	}
	else {
		return -1; //TODO: suspend process of current frontedge group
	}
	// Then update getFrontEdgeGroup() & frontEdgesGroup
	if (splitFe) {
		pushTailFrontEdgeGroup(fe1);
	}
	pushTailFrontEdgeGroup(fe3);
	return 0;
}

int QMorph::doQMorphProcess() {
	this->initFrontEdgeGroup();
	while (doSmooth(), globalIter++, getFrontEdgeGroup())
	{
		if (frontEdgeGroupSize(getFrontEdgeGroup()) == 4) {
			HalfedgeHandle he = getFrontEdgeGroup();
			mesh->setSide(he, mesh->getNextFe(he), NULL);
			mesh->setSide(mesh->getNextFe(he), mesh->getNextFe(mesh->getNextFe(he)), NULL);
			mesh->setSide(mesh->getNextFe(mesh->getNextFe(he)), mesh->getPrevFe(he), NULL);
			mesh->setSide(mesh->getPrevFe(he), he, NULL);
			mesh->buildQuad(mesh->getPrevFe(mesh->getPrevFe(he)), mesh->getPrevFe(he), he, mesh->getNextFe(he));
			mesh->setFront(mesh->getPrevFe(mesh->getPrevFe(he)), false);
			mesh->setFront(mesh->getPrevFe(he), false);
			mesh->setFront(mesh->getNextFe(he), false);
			mesh->setFront(he, false);
			popFrontEdgeGroup();
			continue;
		}
		if (globalIter == 27) {
			highlightAllFes();
		}
		if (doSeam() != 0)
		{
			continue;
		}
		updateFeClassification();
		if (doCornerGenerate()) {
			continue;
		}
		if (doSideDefine() == -1) { //fail to sideDefine because frontEdges are splited
			continue;
		}
		
		doEdgeRecovery();
		proceedNextFeLoop();
		if (getFrontEdgeGroup()) {
			switchFrontEdgeGroup();
		}
	}
	return 0;
}
