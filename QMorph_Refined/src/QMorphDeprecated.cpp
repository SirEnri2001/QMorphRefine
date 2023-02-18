#include "QMorph.h"
#ifdef COMPILE_DEPRECATED
void QMorph::flipFix()
{
	HalfedgeHandle fhe = getFrontEdgeGroup();
	do
	{
		VertexHandle box[4];
		box[0] = mesh->halfedgeSource(fhe);
		box[1] = mesh->halfedgeTarget(fhe);
		box[2] = mesh->halfedgeTarget(mesh->getRightSide(fhe)) == box[1] ?
			mesh->halfedgeSource(mesh->getRightSide(fhe)) :
			mesh->halfedgeTarget(mesh->getRightSide(fhe));
		box[3] = mesh->halfedgeTarget(mesh->getTopEdge(fhe)) == box[2] ?
			mesh->halfedgeSource(mesh->getTopEdge(fhe)) :
			mesh->halfedgeTarget(mesh->getTopEdge(fhe));

		fhe = mesh->getNextFe(fhe);
	} while (fhe == getFrontEdgeGroup());
}


void QMorph::seam(HalfedgeHandle ek, HalfedgeHandle el)
{
	//return;
	cout << "\n\n Seam \n\n";
	// Initialise vertex Nj Nk Nl Nt 
	assert(mesh->getNextFe(ek) == el);
	if ((mesh->calc_edge_length(ek) / mesh->calc_edge_length(el)) > 2.5)
		return splitSeam1(ek, el);
	if ((mesh->calc_edge_length(el) / mesh->calc_edge_length(ek)) > 2.5)
		return splitSeam2(ek, el);
	HalfedgeHandle ej = mesh->getPrevFe(ek);
	HalfedgeHandle em = mesh->getNextFe(el);
	HalfedgeHandle en = mesh->getNextFe(em);
	VertexHandle nm = mesh->halfedgeTarget(em);
	VertexHandle nj, nk, nl, nt;
	nj = mesh->halfedgeSource(ek);
	nk = mesh->halfedgeTarget(ek);
	nl = mesh->halfedgeTarget(el);
	HalfedgeHandle ejl = mesh->edgeRecovery(nj, nl);
	if (mesh->halfedgeTarget(ejl) != nl)
		ejl = mesh->halfedgeSym(ejl);
	nt = mesh->halfedgeTarget(mesh->halfedgeNext(ejl));
	Point vjl = mesh->getPoint(nl) - mesh->getPoint(nj);
	Point vjk = mesh->getPoint(nk) - mesh->getPoint(nj);
	Point vjt = mesh->getPoint(nt) - mesh->getPoint(nj);
	if ((vjl.cross(vjk)).dot(vjl.cross(vjt)) > 0)
		nt = mesh->halfedgeTarget(mesh->halfedgeNext(mesh->halfedgeSym(ejl)));

	// Clear the content of the quad
	HalfedgeHandle quad[4] = {
		ek, el,
		mesh->sourceTargetHalfedge(nl,nt),
		mesh->sourceTargetHalfedge(nt,nj) };
	//mesh->clearQuad(quad);

	// Get all the vertices which connects with Nl but not Nj
	list<VertexHandle> verts;
	for (CTMesh::VertexVertexIter lviter(mesh, nl); !lviter.end(); lviter++)
		verts.push_back(*lviter);

	for (CTMesh::VertexVertexIter lviter(mesh, nl); !lviter.end(); lviter++)
	{
		for (CTMesh::VertexVertexIter jviter(mesh, nj); !jviter.end(); jviter++)
			if (*lviter == *jviter)
				verts.remove(*jviter);
	}

	// Preparations before further clean up;
	Point pos = mesh->getPoint(nj) + mesh->getPoint(nl);
	pos /= 2;
	if (mesh->halfedgeSource(ej) == mesh->halfedgeTarget(em))
		return seamLastQuad(ek, el);

	mesh->setFront(ek, false);

	// Delete Nl and all adjcent edges to make sure there's no overlapping triangles.
	mesh->deleteVertexMergeFace(nl);

	// Move Nj to the middle point
	mesh->setPoint(nj, pos);

	// Connect Nj with all the verts, making Nj the final point.
	for (VertexHandle vert : verts)
		mesh->splitFace(nj, vert);

	// Post process of newly rebuilt elements.
	HalfedgeHandle ejm = mesh->sourceTargetHalfedge(nj, nm);
	mesh->setFront(ejm, true);
	mesh->setPrevFe(ejm, ej);
	mesh->setNextFe(ejm, en);

	// Update
	updateHeadFrontEdgeGroup(ej);

	//checkFrontAssert();
}

void QMorph::splitSeam1(HalfedgeHandle ek, HalfedgeHandle el)
{
	cout << "\n\n Split Seam 1\n\n";
	HalfedgeHandle ej, em;
	ej = mesh->getPrevFe(ek);
	em = mesh->getNextFe(el);
	VertexHandle vj, vk, vl;
	vj = mesh->halfedgeTarget(ej);
	vk = mesh->halfedgeTarget(ek);
	vl = mesh->halfedgeTarget(el);
	VertexHandle vq = NULL, vt = NULL;
	FaceHandle oldFace = mesh->halfedgeFace(ek);
	if (!mesh->isQuad(oldFace))
		oldFace = mesh->halfedgeFace(mesh->halfedgeSym(ek));
	for (CTMesh::VertexVertexIter viter(mesh, vk); !viter.end(); viter++)
		for (CTMesh::FaceVertexIter fiter(oldFace); !fiter.end(); fiter++)
			if (*viter == *fiter && *viter != vj)
			{
				vq = *viter;
				break;
			}
	assert(vq);

	oldFace = mesh->halfedgeFace(ek);
	if (mesh->isQuad(oldFace))
		oldFace = mesh->halfedgeFace(mesh->halfedgeSym(ek));
	for (CTMesh::VertexVertexIter viter(mesh, vk); !viter.end(); viter++)
		for (CTMesh::FaceVertexIter fiter(oldFace); !fiter.end(); fiter++)
			if (*viter == *fiter && *viter != vj)
			{
				vt = *viter;
				break;
			}
	assert(vt);

	VertexHandle vs = mesh->addVertexOnEdge(mesh->halfedgeEdge(ek));
	mesh->splitFace(vs, vt);
	mesh->splitFace(vs, vq);
	HalfedgeHandle es = mesh->sourceTargetHalfedge(vj, vs);
	if (mesh->halfedgeTarget(es) != vs)
		es = mesh->halfedgeSym(es);
	/*EdgeHandle* eq_t = mesh->splitFace(vs, vq);
	HalfedgeHandle eq = mesh->edgeHalfedge(eq_t, 0);
	if (mesh->halfedgeTarget(eq) != vq)
		eq = mesh->halfedgeSym(eq);*/
	HalfedgeHandle esl = mesh->edgeRecovery(vs, vl);
	if (mesh->halfedgeTarget(esl) != vl)
		esl = mesh->halfedgeSym(esl);
	mesh->setFront(es, true);
	mesh->setPrevFe(es, ej);
	mesh->setNextFe(es, esl);
	mesh->setFront(el, false);
	mesh->setNextFe(esl, em);
	mesh->setFront(esl, true);

	HalfedgeHandle quad[4] = {
		mesh->sourceTargetHalfedge(vs, vq),
		mesh->sourceTargetHalfedge(vq, vk),
		mesh->sourceTargetHalfedge(vk, vl),
		mesh->sourceTargetHalfedge(vl, vs)

	};
	//mesh->clearQuad(quad);

	updateHeadFrontEdgeGroup(ej);
	//checkFrontAssert();
}

void QMorph::splitSeam2(HalfedgeHandle ek, HalfedgeHandle el)
{
	cout << "\n\n Split Seam 2 \n\n";
	HalfedgeHandle ej, em;
	ej = mesh->getPrevFe(ek);
	em = mesh->getNextFe(el);
	VertexHandle vj, vk, vl;
	vj = mesh->halfedgeTarget(ej);
	vk = mesh->halfedgeTarget(ek);
	vl = mesh->halfedgeTarget(el);
	VertexHandle vq = NULL, vt = NULL;
	FaceHandle oldFace = mesh->halfedgeFace(el);
	if (!mesh->isQuad(oldFace))
		oldFace = mesh->halfedgeFace(mesh->halfedgeSym(el));
	for (CTMesh::VertexVertexIter viter(mesh, vk); !viter.end(); viter++)
		for (CTMesh::FaceVertexIter fiter(oldFace); !fiter.end(); fiter++)
			if (*viter == *fiter && *viter != vl)
			{
				vq = *viter;
				break;
			}
	assert(vq);

	oldFace = mesh->halfedgeFace(el);
	if (mesh->isQuad(oldFace))
		oldFace = mesh->halfedgeFace(mesh->halfedgeSym(el));
	for (CTMesh::VertexVertexIter viter(mesh, vk); !viter.end(); viter++)
		for (CTMesh::FaceVertexIter fiter(oldFace); !fiter.end(); fiter++)
			if (*viter == *fiter && *viter != vl)
			{
				vt = *viter;
				break;
			}
	assert(vt);

	VertexHandle vs = mesh->addVertexOnEdge(mesh->halfedgeEdge(el));
	mesh->splitFace(vs, vt);
	mesh->splitFace(vs, vq);
	HalfedgeHandle esl = mesh->sourceTargetHalfedge(vs, vl);
	HalfedgeHandle ejs = mesh->edgeRecovery(vj, vs);
	if (mesh->halfedgeTarget(ejs) != vs)
		ejs = mesh->halfedgeSym(ejs);
	mesh->setFront(esl, true);
	mesh->setPrevFe(esl, ejs);
	mesh->setNextFe(esl, em);
	mesh->setPrevFe(ejs, ej);
	mesh->setFront(ejs, true);
	mesh->setFront(ek, true);

	HalfedgeHandle quad[4] = {
		mesh->sourceTargetHalfedge(vj, vk),
		mesh->sourceTargetHalfedge(vk, vq),
		mesh->sourceTargetHalfedge(vq, vs),
		mesh->sourceTargetHalfedge(vs, vj)

	};
	//mesh->clearQuad(quad);

	updateHeadFrontEdgeGroup(ej);
	//checkFrontAssert();
}

void QMorph::seamLastQuad(HalfedgeHandle ek, HalfedgeHandle el)
{
	// Initialise vertex Nj Nk Nl Nt 
	VertexHandle nj, nk, nl, nm;
	nj = mesh->halfedgeSource(ek);
	nk = mesh->halfedgeTarget(ek);
	nl = mesh->halfedgeTarget(el);
	HalfedgeHandle ej = mesh->getPrevFe(ek);
	HalfedgeHandle em = mesh->getNextFe(el);
	nm = mesh->halfedgeTarget(em);


	// Get all the vertices which connects with Nl but not Nj
	list<VertexHandle> verts;
	for (CTMesh::VertexVertexIter lviter(mesh, nl); !lviter.end(); lviter++)
		verts.push_back(*lviter);

	for (CTMesh::VertexVertexIter lviter(mesh, nl); !lviter.end(); lviter++)
	{
		for (CTMesh::VertexVertexIter jviter(mesh, nj); !jviter.end(); jviter++)
			if (*lviter == *jviter)
				verts.remove(*lviter);
	}
	// Preparations before further clean up;
	Point pos = mesh->getPoint(nj) + mesh->getPoint(nl);
	pos /= 2;
	mesh->setFront(ek, false);
	mesh->setFront(ej, false);
	mesh->setFront(el, false);
	mesh->setFront(em, false);
	//meter* tmpMeter = averLengthes.top();
	//averLengthes.pop();
	//delete tmpMeter;

	// Delete Nl and all adjcent edges to make sure there's no overlapping triangles.
	mesh->deleteVertexMergeFace(nl);

	// Move Nj to the middle point
	mesh->setPoint(nj, pos);

	// Connect Nj with all the verts, making Nj the final point.
	for (VertexHandle vert : verts)
		mesh->splitFace(nj, vert);

}
bool QMorph::needSeam(HalfedgeHandle ek, HalfedgeHandle el)
{
	assert(mesh->getNextFe(ek) == el);
	double epsilon[2] = { 15.0, 45.0 };
	int nq = mesh->numQuad(mesh->halfedgeTarget(ek));
	double angle = mesh->angle(ek, el);
	return((nq >= 5 && angle < epsilon[0]) || (nq < 5 && angle < epsilon[1]));
}

HalfedgeHandle QMorph::getWellCover(HalfedgeHandle wellBottom) {
	assert(mesh->isFront(wellBottom));
	CTMesh::VertexIHalfedgeIter heIter(mesh, mesh->halfedgeSource(mesh->getPrevFe(wellBottom)));
	HalfedgeHandle wellCover = NULL;
	for (; !heIter.end(); heIter++) {
		if (mesh->halfedgeSource(*heIter) == mesh->halfedgeTarget(mesh->getNextFe(wellBottom))) {
			wellCover = *heIter;
			break;
		}
	}
	return wellCover;
}

// must updateAverlengthAndFeGroup after calling
HalfedgeHandle QMorph::fillDeepWell(HalfedgeHandle wellBottom) { //return new frontside as well cover
	HalfedgeHandle leftside = wellBottom;
	HalfedgeHandle leftCover;
	leftCover = getWellCover(leftside);
	while (leftCover)
	{
		HalfedgeHandle hes[]{ leftside,mesh->getNextFe(leftside),leftCover,mesh->getPrevFe(leftside) };
		//mesh->clearQuad(hes);
		mesh->setNextFe(mesh->getPrevFe(mesh->getPrevFe(leftside)), mesh->halfedgeSym(leftCover));
		mesh->setNextFe(mesh->halfedgeSym(leftCover), mesh->getNextFe(mesh->getNextFe(leftside)));
		mesh->setFront(mesh->halfedgeSym(leftCover), true);

		// Recover the attributes of old frontEdge elements back to default
		mesh->setFront(leftside, false);
		leftside = mesh->halfedgeSym(leftCover);
		leftCover = getWellCover(leftside);
	}


	//checkFrontAssert(); 
	return leftside;
}

//Algorithm 1. Edge recovery
void QMorph::frontEdgeRecovery(HalfedgeHandle frontHe) {
	if (mesh->getLeftSide(frontHe) && mesh->getRightSide(frontHe)) {
		;
	}
	else {
		mesh->highlight(frontHe);
		mesh->updateDebug();
		assert(false);
	}

	// left and right side & topology check
	assert(mesh->getLeftSide(frontHe) && mesh->getRightSide(frontHe));
	assert(mesh->halfedgeSource(frontHe) == mesh->halfedgeTarget(mesh->getLeftSide(frontHe)));
	assert(mesh->halfedgeTarget(frontHe) == mesh->halfedgeSource(mesh->getRightSide(frontHe)));

	VertexHandle Nc = mesh->halfedgeTarget(mesh->getRightSide(frontHe));
	VertexHandle Nd = mesh->halfedgeSource(mesh->getLeftSide(frontHe));
	assert(Nd != Nc);
	if (mesh->sourceTargetHalfedge(Nc, Nd)) {
		mesh->setTopEdge(frontHe, mesh->sourceTargetHalfedge(Nc, Nd));
		return;
	}
	mesh->edgeRecovery(Nc, Nd);

	//Point normal = (mesh->getPoint(Nc) - mesh->getPoint(mesh->halfedgeSource(frontHe))).cross(mesh->getPoint(Nd) - mesh->getPoint(Nc));
	//list<HalfedgeHandle>* newLambdaSet = new list<HalfedgeHandle>();
	//HalfedgeHandle Ei;
	//do {
	//	if (newLambdaSet->size() > 0) {
	//		delete lambdaSet;
	//		lambdaSet = newLambdaSet;
	//		newLambdaSet = new list<HalfedgeHandle>();
	//	}

	//	for (list<HalfedgeHandle>::iterator iterEi = lambdaSet->begin();
	//		iterEi != lambdaSet->end();
	//		iterEi++)
	//	{
	//		Ei = *iterEi;
	//		// get quadraliteral's points
	//		//     Pa
	//		//   /    \
	//		// Pb--Ei->Pc
	//		//   \    /
	//		//     Pd
	//		VertexHandle Pa = mesh->halfedgeTarget(mesh->halfedgeNext(Ei));
	//		VertexHandle Pb = mesh->halfedgeSource(Ei);
	//		VertexHandle Pc = mesh->halfedgeTarget(Ei);
	//		VertexHandle Pd = mesh->halfedgeTarget(mesh->halfedgeNext(mesh->halfedgeSym(Ei)));
	//		Point normal = (mesh->getPoint(Pb)-mesh->getPoint(Pa)).cross(mesh->getPoint(Pc)-mesh->getPoint(Pa));
	//		if (
	//			((mesh->getPoint(Pa)-mesh->getPoint(Pc)).cross(mesh->getPoint(Pd)-mesh->getPoint(Pc))).dot(normal) 
	//			* 
	//			((mesh->getPoint(Pd)-mesh->getPoint(Pb)).cross(mesh->getPoint(Pa)-mesh->getPoint(Pb))).dot(normal)<0.0) { 
	//			//if this is a boomerang»ØÐýïÚ
	//			newLambdaSet->push_back(Ei);
	//			continue;
	//		}
	//		else {
	//			assert(!mesh->isBoundary(Ei));
	//			EdgeHandle Ej = mesh->swapEdge(mesh->halfedgeEdge(Ei));
	//			if (mesh->edgeVertex1(Ej) == Nc || mesh->edgeVertex2(Ej) == Nc
	//				|| mesh->edgeVertex1(Ej) == Nd || mesh->edgeVertex2(Ej) == Nd) {
	//				continue;
	//			}
	//			double* km = solveKMEquation(mesh->getPoint(mesh->edgeVertex1(Ej)) - mesh->getPoint(Nc),
	//				mesh->getPoint(mesh->edgeVertex2(Ej)) - mesh->getPoint(Nc), mesh->getPoint(Nd) - mesh->getPoint(Nc));
	//			if (km[0] < 1.0 && km[1]>0.0 && km[1] < 1.0) {
	//				newLambdaSet->push_back(mesh->edgeHalfedge(Ej, 0));
	//			}
	//			delete[] km;
	//		}
	//	}
	//} while (newLambdaSet->size()>0);
	if (mesh->sourceTargetHalfedge(Nc, Nd)) {
		mesh->setTopEdge(frontHe, mesh->sourceTargetHalfedge(Nc, Nd));
		return;
	}
}


int QMorph::doClearQuard() {
	int i = 0;
	HalfedgeHandle iter = getFrontEdgeGroup();
	do {
		//reportIter(i++, "doClearQuad");
		HalfedgeHandle fe = iter;
		if (!mesh->getNeedTopEdge(fe)) {
			continue;
		}
		if (mesh->getTopEdge(fe) == NULL) {
			mesh->highlight(fe);
			mesh->updateDebug();
		}

	} while (iter = mesh->getNextFe(iter), iter != getFrontEdgeGroup());
	return 0;
}

int QMorph::frontEdgeSideDefine(int i, HalfedgeHandle lfe, HalfedgeHandle rfe) {
	//assert(mesh->halfedgeTarget(lfe) == mesh->halfedgeSource(rfe));
	//HalfedgeHandle minAngleHe = lfe;
	//VertexHandle pivotVertex = mesh->halfedgeTarget(lfe);
	//Point pivot = mesh->getPoint(pivotVertex);
	//Point bisector = mesh->bisector(lfe, rfe);
	//VertexInFrontHeIterator pivotIter(mesh, pivotVertex);
	//double minAngle = 360.0;

	//for (; !pivotIter.end(); pivotIter++) {
	//	if (mesh->isBoundary(mesh->halfedgeSym(*pivotIter))) {
	//		continue;
	//	}
	//	double angle = mesh->angle(bisector + pivot, mesh->halfedgeSym(*pivotIter));
	//	if (angle < minAngle) {
	//		minAngle = angle;
	//		minAngleHe = *pivotIter;
	//	}
	//}
	//if (minAngle < constEpsilon) { // satisfy first condition
	//	if (mesh->halfedgeSource(mesh->getPrevFe(lfe)) == mesh->halfedgeSource(minAngleHe)
	//		|| mesh->halfedgeTarget(mesh->getNextFe(rfe)) == mesh->halfedgeSource(minAngleHe)) {
	//		// prevent a size 3 front edge group
	//		;
	//	}
	//	else if (mesh->numQuad(mesh->halfedgeSource(minAngleHe)) > 1 || mesh->halfedgeSource(minAngleHe)->isSide) {
	//		return sideDefineQuadFallback(lfe, rfe, minAngleHe);
	//	}
	//	else if (mesh->isFront(mesh->halfedgeSource(minAngleHe))) {
	//		if (seperateFrontLoop(minAngleHe) == 0) {
	//			return -1;
	//		}
	//	}
	//	else
	//	{
	//		mesh->setSide(lfe, rfe, mesh->halfedgeSym(minAngleHe));
	//		mesh->halfedgeSource(minAngleHe)->isSide = true;
	//		return 0;
	//	}
	//}

	//minAngle = 360.0;
	//// traverse all quadrilateral surround the vertex
	////(mAV)<--.
	////   |   / \
	////   |  /   \
	////   | /     \
	////   v--minAH>(pV)
	//VertexHandle minAngleVertex = mesh->halfedgeSource(lfe);
	//for (pivotIter.reset(); !pivotIter.end(); pivotIter++)
	//{
	//	if (mesh->isFront(mesh->halfedgePrev(*pivotIter))) {
	//		continue;
	//	}
	//	VertexHandle curVertex =
	//		mesh->halfedgeTarget(mesh->halfedgeNext(
	//			mesh->halfedgeSym(mesh->halfedgePrev(*pivotIter))));
	//	double angle = mesh->angle(bisector + pivot, pivotVertex, mesh->getPoint(curVertex));
	//	if (angle < minAngle) {
	//		minAngle = angle;
	//		minAngleHe = *pivotIter;
	//		minAngleVertex = curVertex;
	//	}
	//}
	//HalfedgeHandle newHe = mesh->halfedgeNext(minAngleHe);
	//if (minAngle < constEpsilon
	//	&& 2.0 * eDist(mesh->getPoint(minAngleVertex), mesh->getPoint(mesh->halfedgeTarget(lfe))) <
	//	sqrt(3) * (mesh->calc_edge_length(lfe) + mesh->calc_edge_length(rfe))) {  //safisty second condition
	//	if (minAngleVertex == mesh->halfedgeSource(mesh->getPrevFe(lfe)) ||
	//		minAngleVertex == mesh->halfedgeTarget(mesh->getNextFe(rfe))) {
	//		// prevent a size 3 front edge group
	//		;
	//	}
	//	else if (mesh->getLeftSide(lfe) && mesh->halfedgeSource(mesh->getLeftSide(lfe))
	//		== mesh->halfedgeSource(mesh->halfedgePrev(mesh->halfedgeSym(mesh->halfedgePrev(minAngleHe))))) {
	//		mesh->swapEdge(mesh->halfedgeEdge(mesh->halfedgePrev(minAngleHe)));
	//		return sideDefineQuadFallback(lfe, rfe, mesh->sourceTargetHalfedge(minAngleVertex, pivotVertex));
	//	}
	//	else if (mesh->isFront(minAngleVertex)) {
	//		HalfedgeHandle he = mesh->edgeHalfedge(mesh->swapEdge(mesh->halfedgeEdge(mesh->halfedgePrev(minAngleHe))), 0);
	//		if (seperateFrontLoop(he) == 0) {
	//			return -1;
	//		}
	//	}
	//	else if (!mesh->halfedgeTarget(mesh->halfedgeNext(mesh->halfedgeSym(mesh->halfedgePrev(minAngleHe))))->isSide) {
	//		mesh->swapEdge(mesh->halfedgeEdge(mesh->halfedgePrev(minAngleHe)));
	//		newHe = mesh->halfedgeNext(minAngleHe);
	//		mesh->setSide(lfe, rfe, newHe);
	//		mesh->halfedgeTarget(newHe)->isSide = true;
	//		return 0;
	//	}
	//}

	//HalfedgeHandle he2 = minAngleHe;
	//for (pivotIter.reset(); !pivotIter.end(); pivotIter++) {
	//	if ((bisector.cross(mesh->getPoint(mesh->halfedgeSource(*pivotIter)) - pivot))
	//		.dot(bisector.cross(mesh->getPoint(mesh->halfedgeTarget(mesh->halfedgeNext(*pivotIter))) - pivot)) < 0) {
	//		he2 = *pivotIter;
	//	}
	//}
	////if (mesh->isBoundary(he2)||he2->isFront) {
	////	continue;
	////}
	//HalfedgeHandle he1 = mesh->halfedgeNext(he2);
	//// 2 halfedges in a patch: --he2-->.--he1-->

	//if (mesh->isQuad(mesh->halfedgeFace(mesh->halfedgeSym(mesh->halfedgePrev(he2))))) {
	//	return sideDefineQuadFallback(lfe, rfe, he2);
	//}

	//if (mesh->isFront(mesh->halfedgeTarget(he2)) && mesh->isFront(mesh->halfedgeSource(he2)) && he2 != lfe) {
	//	HalfedgeHandle tempHe = he2;
	//	if (seperateFrontLoop(tempHe) == 0) {
	//		return -1;
	//	}
	//}
	//// solve the 2x2 linear system
	//Point v1 = mesh->getPoint(mesh->halfedgeSource(he2)) - mesh->getPoint(mesh->halfedgeTarget(he2));
	//Point v2 = mesh->getPoint(mesh->halfedgeTarget(he1)) - mesh->getPoint(mesh->halfedgeTarget(he2));
	//double* km = solveKMEquation(v1, v2, bisector);

	//Point normal = v2.cross(v1);
	////Point projection = bisector - normal * (bisector * normal);
	//Point p = bisector * (km[0] + EPSILON * 10.0) + pivot;
	//VertexHandle spliter = mesh->splitEdge(mesh->halfedgePrev(he2), p);
	//newHe = mesh->sourceTargetHalfedge(pivotVertex, spliter);
	//assert(newHe);
	//mesh->setSide(lfe, rfe, newHe);
	return 0;
}

//downSide.sourceVertex should connect a quad vertex
int QMorph::sideDefineQuadFallback(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle downSide) {
	HalfedgeHandle upSideHe = mesh->halfedgeSym(downSide);
	assert(!mesh->isQuad(mesh->halfedgeFace(upSideHe)));
	assert(!mesh->isQuad(mesh->halfedgeFace(downSide)));
	mesh->setSide(lfe, rfe, NULL);
	VertexHandle v = mesh->splitEdge(downSide,
		(mesh->getPoint(mesh->halfedgeSource(downSide)) + mesh->getPoint(mesh->halfedgeTarget(downSide))) / 2);
	upSideHe = mesh->sourceTargetHalfedge(mesh->halfedgeTarget(lfe), v);
	mesh->setSide(lfe, rfe, upSideHe);
	mesh->halfedgeTarget(upSideHe)->isSide = true;
	return 0;
}

#endif