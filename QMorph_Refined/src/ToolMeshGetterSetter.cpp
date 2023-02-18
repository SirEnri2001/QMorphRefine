#include"ToolMesh.h"

void CToolMesh::setSideEdge(HalfedgeHandle side, HalfedgeHandle fe) {
	if (fe) {
		side->isSideEdge = true;
		side->feReference = fe;
	}
	else {
		side->isSideEdge = false;
		side->feReference = NULL;
	}
}

void CToolMesh::setSide(HalfedgeHandle lfe, HalfedgeHandle rfe, HalfedgeHandle upSide) {
	if (upSide) {
		setRightSide(lfe, upSide);
		setLeftSide(rfe, halfedgeSym(upSide));
	}
	else {
		setRightSide(lfe, NULL);
		setLeftSide(rfe, NULL);
	}
}

bool CToolMesh::isSideEdge(HalfedgeHandle he) {
	return he->isSideEdge;
}

HalfedgeHandle CToolMesh::getFeReference(HalfedgeHandle he) {
	return he->feReference;
}

void CToolMesh::setPoint(VertexHandle v, const Point& p) {
	if (p[0] != p[0]) {
		cout << "ERR: cannot set NaN to a vertex";
		assert(false);
	}
	v->point() = p;
}

const Point CToolMesh::getPoint(VertexHandle v) const {
	return v->point();
}

bool CToolMesh::isFront(HalfedgeHandle he) {
	return he->isFront;
}
bool CToolMesh::isFront(VertexHandle v) {
	return v->frontNum > 0;
}

int CToolMesh::getClass(HalfedgeHandle he) {
	return he->classNum;
}
void CToolMesh::setClass(HalfedgeHandle he, int cls) {
	he->classNum = cls;
}
HalfedgeHandle CToolMesh::getPrevFe(HalfedgeHandle he) {
	return he->prevFe;
}

//setPrevFe
void CToolMesh::setPrevFe(HalfedgeHandle fe, HalfedgeHandle prev) {
	assert(isFront(fe));
	assert(isFront(prev));
	fe->prevFe = prev;
	prev->nextFe = fe;
	if (angle(prev, fe) < constAngle) {
		setClass(fe, getClass(fe) | 2);
		setClass(prev, getClass(prev) | 1);
	}
	else {
		setClass(fe, getClass(fe) & 1);
		setClass(prev, getClass(prev) & 2);
	}
}

//getNextFe
HalfedgeHandle CToolMesh::getNextFe(HalfedgeHandle he) {
	return he->nextFe;
}

//setNextFe
void CToolMesh::setNextFe(HalfedgeHandle fe, HalfedgeHandle next) {
	assert(isFront(fe));
	assert(isFront(next));
	next->prevFe = fe;
	fe->nextFe = next;
	if (angle(fe, next) < constAngle) {
		setClass(fe, getClass(fe) | 1);
		setClass(next, getClass(next) | 2);
	}
	else {
		setClass(fe, getClass(fe) & 2);
		setClass(next, getClass(next) & 1);
	}
}

HalfedgeHandle CToolMesh::getLeftSide(HalfedgeHandle he) {
	return he->leftSide;
}
HalfedgeHandle CToolMesh::getRightSide(HalfedgeHandle he) {
	return he->rightSide;
}
void CToolMesh::setLeftSide(HalfedgeHandle he, HalfedgeHandle left) {
	if (getLeftSide(he)) {
		halfedgeSource(getLeftSide(he))->isSide = false;
		setSideEdge(getLeftSide(he), NULL);
	}
	he->leftSide = left;
	if (left) {
		setSideEdge(left, he);
		halfedgeTarget(left)->isSide = true;
	}
}

void CToolMesh::setRightSide(HalfedgeHandle he, HalfedgeHandle right) {
	if (getRightSide(he)) {
		halfedgeTarget(getRightSide(he))->isSide = false;
		setSideEdge(getRightSide(he), NULL);
	}
	he->rightSide = right;
	if (right) {
		setSideEdge(right, he);
		halfedgeTarget(right)->isSide = true;
	}
}

//get & set topEdge
HalfedgeHandle CToolMesh::getTopEdge(HalfedgeHandle he) {
	return he->topEdge;
}
void CToolMesh::setTopEdge(HalfedgeHandle he, HalfedgeHandle top) {
	he->topEdge = top;
}

//get & set needTopEdge
bool CToolMesh::getNeedTopEdge(HalfedgeHandle he) {
	return he->needTopEdge;
}
void CToolMesh::setNeedTopEdge(HalfedgeHandle he, bool need) {
	he->needTopEdge = need;
}

void CToolMesh::setFront(HalfedgeHandle fe, bool val)
{
	if (val) {
		if (isFront(fe)) {
			return;
		}
		setClass(fe, 0);
		fe->isFront = true;
		halfedgeSource(fe)->frontNum++;
		halfedgeTarget(fe)->frontNum++;
	}
	else {
		if (!isFront(fe)) {
			return;
		}
		setClass(fe, -1);

		fe->isFront = false;
		halfedgeSource(fe)->frontNum--;
		halfedgeTarget(fe)->frontNum--;
		setLeftSide(fe, NULL);
		setRightSide(fe, NULL);
	}

}
