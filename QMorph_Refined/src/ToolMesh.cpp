#include<map>
#include<vector>
#include<queue>
#include<list>

#include "ToolMesh.h"
bool startDebug = false;

VertexHandle CToolMesh::edgeVertex1(EdgeHandle edge) {
	return halfedgeTarget(edgeHalfedge(edge, 0));
}

VertexHandle CToolMesh::edgeVertex2(EdgeHandle edge) {
	return halfedgeSource(edgeHalfedge(edge, 0));
}

CToolMesh::CToolMesh() {
}



int CToolMesh::frontEdgeSize(HalfedgeHandle fe) {
	int size = 0;
	HalfedgeHandle he = fe;
	do {
		size++;
		he = getNextFe(he);
	} while (he != fe);
	return size;
}

void CToolMesh::splitFace(VertexHandle v1, VertexHandle v2) {
	topology_assert(!vertexEdge(v1, v2), { v1,v2 });
	// find the face that contains v1 and v2
	FaceHandle face = NULL;
	for (VertexFaceIter vf_it(this, v1); !vf_it.end(); vf_it++) {
		for (FaceVertexIter fv_it(*vf_it); !fv_it.end(); fv_it++) {
			if (*fv_it == v2) {
				face = *vf_it;
				break;
			}
		}
	}
	assert(face);
	HalfedgeHandle he = halfedge_handle(face);
	while (halfedgeTarget(he) != v1)
		he = halfedgeNext(he);
	deleteFace(face);
	std::vector<VertexHandle> tmp_face_vhandles;
	tmp_face_vhandles.clear();
	do {
		tmp_face_vhandles.push_back(halfedgeTarget(he));
	} while (he = halfedgeNext(he), halfedgeTarget(he) != v2);
	tmp_face_vhandles.push_back(v2);
	he = halfedgeNext(he);
	createFace(tmp_face_vhandles);
	tmp_face_vhandles.clear();
	do {
		tmp_face_vhandles.push_back(halfedgeTarget(he));
	} while (he = halfedgeNext(he), halfedgeTarget(he) != v1);
	tmp_face_vhandles.push_back(v1);
	tmp_face_vhandles.push_back(v2);
	createFace(tmp_face_vhandles);
}

int CToolMesh::deleteVertexMergeFace(VertexHandle tar)
{
	assert(m_map_vert.find(tar->getId()) != m_map_vert.end());
	if (!vertexHalfedge(tar)) {
		deleteVertex(tar);
		return 0;
	}
	std::list<EdgeHandle> edges;
	VertexEdgeIter ve_iter(this, tar);
	while (!ve_iter.end()) {
		edges.push_back(*ve_iter);
		ve_iter++;
	}
	while (edges.size() > 0) {
		if (deleteEdgeMergeFace(edges.front()) == 1) {
		}
		edges.pop_front();
	}
	deleteVertex(tar);
	return 0;
}

// point connecting edge1 and edge2 must not connect other edges
void CToolMesh::disconnect(EdgeHandle edge1, EdgeHandle edge2) {
	HalfedgeHandle he = edgeHalfedge(edge1, 0);
	if (halfedgeTarget(he) != edgeVertex1(edge2) && halfedgeTarget(he) != edgeVertex2(edge2)) {
		he = halfedgeSym(he);
	}
	topology_assert(halfedgeTarget(he) == edgeVertex1(edge2) || halfedgeTarget(he) == edgeVertex2(edge2));
	HalfedgeHandle he2 = halfedgeSym(halfedgeNext(he));
	setNextHalfedge(he, halfedgeSym(he));
	setNextHalfedge(he2, halfedgeSym(he2));
}

void CToolMesh::disconnect(EdgeHandle edge) {
	// cannot disconnect a edge that a vertex relies on
	topology_assert(vertexHalfedge(edgeVertex1(edge))==NULL || halfedgeEdge(vertexHalfedge(edgeVertex1(edge))) != edge, {edge});
	topology_assert(vertexHalfedge(edgeVertex2(edge)) == NULL || halfedgeEdge(vertexHalfedge(edgeVertex2(edge))) != edge, {edge});

	// here are conditions that make sure the two sym halfedges of one suspended edge are not connected
	// satisfy condition:
	// <--.<--
	//    ^|
	// he0||he1
	//    |V
	// then connect the two halfedges above
	if (halfedgeNext(edgeHalfedge(edge, 0)) != edgeHalfedge(edge, 1)) {
		setNextHalfedge(halfedgePrev(edgeHalfedge(edge, 1)), halfedgeNext(edgeHalfedge(edge, 0)));
		setNextHalfedge(edgeHalfedge(edge, 0), edgeHalfedge(edge, 1));
	}

	// satisfy condition:
	//    ^|
	// he0||he1
	//    |V
	// -->.-->
	// then connect the two halfedges below
	if (halfedgeNext(edgeHalfedge(edge, 1)) != edgeHalfedge(edge, 0)) {
		setNextHalfedge(halfedgePrev(edgeHalfedge(edge, 0)), halfedgeNext(edgeHalfedge(edge, 1)));
		setNextHalfedge(edgeHalfedge(edge, 1), edgeHalfedge(edge, 0));
	}
}

bool CToolMesh::isDisconnected(EdgeHandle edge) {
	return (halfedgeNext(edgeHalfedge(edge, 0)) == edgeHalfedge(edge, 1)
		|| halfedgeNext(edgeHalfedge(edge, 1)) == edgeHalfedge(edge, 0));
	//return edge->disconnected;
}


int CToolMesh::deleteEdgeMergeFace(EdgeHandle tar)
{

	// <--he1-(va)<-he4---
	//         ^|
	//      hea||heb
	//         |v
	//---he2->(vb)--he3-->
	VertexHandle va = edgeVertex1(tar);
	VertexHandle vb = edgeVertex2(tar);
	HalfedgeHandle hea = vertexHalfedge(vb, va);
	HalfedgeHandle heb = vertexHalfedge(va, vb);
	HalfedgeHandle he3 = halfedgeNext(heb);
	HalfedgeHandle he2 = halfedgePrev(hea);
	HalfedgeHandle he4 = halfedgePrev(heb);
	HalfedgeHandle he1 = halfedgeNext(hea);
	if (isDisconnected(tar)) {
		//         ^|
		//      hea||heb
		//         |v

		if (halfedgeNext(edgeHalfedge(tar, 0)) == edgeHalfedge(tar, 1)
			&& halfedgeNext(edgeHalfedge(tar, 1)) == edgeHalfedge(tar, 0)) {
			deleteFace(halfedgeFace(hea));
			setFace(edgeHalfedge(tar, 0), NULL);
			setFace(edgeHalfedge(tar, 1), NULL);
			unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 0)), edgeHalfedge(tar, 0));
			unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 1)), edgeHalfedge(tar, 1));
			disconnect(tar);
			topology_assert(!isSideEdge(edgeHalfedge(tar, 0))
				&& !isSideEdge(edgeHalfedge(tar, 1)), { edgeHalfedge(tar, 0) });
			deleteEdge(tar);
			return 0;
		}

		//         ^|
		//      hea||heb
		//         |v
		//---he2->(vb)--he3-->
	
		if (halfedgeNext(edgeHalfedge(tar, 0)) == edgeHalfedge(tar, 1)) {
			hea = edgeHalfedge(tar, 0);
			heb = edgeHalfedge(tar, 1);
		}
		else {
			hea = edgeHalfedge(tar, 1);
			heb = edgeHalfedge(tar, 0);
		}
		vb = halfedgeTarget(heb);
		va = halfedgeTarget(hea);
		he2 = halfedgePrev(hea);
		he3 = halfedgeNext(heb);
		HalfedgeHandle heIter = he3;
		std::vector<VertexHandle> tmp_face_vhandles;
		do {
			tmp_face_vhandles.push_back(halfedgeTarget(heIter));
		} while (heIter = halfedgeNext(heIter), heIter != he2);
		tmp_face_vhandles.push_back(vb);
		deleteFace(halfedgeFace(hea));
		setFace(edgeHalfedge(tar, 0), NULL);
		setFace(edgeHalfedge(tar, 1), NULL);
		unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 0)), edgeHalfedge(tar, 0));
		unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 1)), edgeHalfedge(tar, 1));
		disconnect(tar);
		topology_assert(!isSideEdge(edgeHalfedge(tar, 0))
			&& !isSideEdge(edgeHalfedge(tar, 1)), { edgeHalfedge(tar, 0) });
		deleteEdge(tar);
		createFace(tmp_face_vhandles);
		return 0;
	}
	if (halfedgeFace(hea) == halfedgeFace(heb)) {
		// Ring-shape face
		//    --           --
		//   /  \         /  \
		//  |    |--tar--|    |
		//   \  /         \  /
		//    --           --
		//          ||
		//          ||
		//          VV
		//        ------
		//      /  ----  \
		//     |  |    |  |
		//     |  |    |  |
		//      \  ----  /
		//        ------
		std::vector<CPoint> cf = halfedgeFace(hea)->crossFieldDirection;
		deleteFace(halfedgeFace(hea));
		setFace(edgeHalfedge(tar, 0), NULL);
		setFace(edgeHalfedge(tar, 1), NULL);
		unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 0)), edgeHalfedge(tar, 0));
		unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 1)), edgeHalfedge(tar, 1));
		disconnect(tar);
		topology_assert(!isSideEdge(edgeHalfedge(tar, 0))
			&& !isSideEdge(edgeHalfedge(tar, 1)), { edgeHalfedge(tar, 0) });
		deleteEdge(tar);
		std::vector<VertexHandle> tmp_face_vhandles;
		HalfedgeHandle heIter = he3;
		do {
			tmp_face_vhandles.push_back(halfedgeTarget(heIter));
		} while(heIter = halfedgeNext(heIter), heIter != he3);
		createFace(tmp_face_vhandles)->crossFieldDirection = cf;
		heIter = he1;
		tmp_face_vhandles.clear();
		do {
			tmp_face_vhandles.push_back(halfedgeTarget(heIter));
		} while (heIter = halfedgeNext(heIter), heIter != he4);
		createFace(tmp_face_vhandles)->crossFieldDirection = cf;
		return 0;
	}
	int cfCount = 0;
	CPoint cf(0, 0, 0);
	if (halfedgeFace(edgeHalfedge(tar, 0))) {
		cf = halfedgeFace(edgeHalfedge(tar, 0))->crossFieldDirection[0];
		deleteFace(halfedgeFace(edgeHalfedge(tar, 0)));
		cfCount++;
	}
	if (halfedgeFace(edgeHalfedge(tar, 1))) {
		deleteFace(halfedgeFace(edgeHalfedge(tar, 1)));
		cfCount++;
	}
	topology_assert(!isSideEdge(edgeHalfedge(tar, 0)), { tar, edgeHalfedge(tar, 0)->feReference });
	topology_assert(!isSideEdge(edgeHalfedge(tar, 1)), { tar, edgeHalfedge(tar, 1)->feReference });
	setFace(edgeHalfedge(tar, 0), NULL);
	setFace(edgeHalfedge(tar, 1), NULL);
	unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 0)), edgeHalfedge(tar, 0));
	unsetHalfedge(halfedgeVertex(edgeHalfedge(tar, 1)), edgeHalfedge(tar, 1));
	disconnect(tar);
	deleteEdge(tar);
	std::vector<VertexHandle> tmp_face_vhandles;
	HalfedgeHandle heIter = he1;
	do {
		tmp_face_vhandles.push_back(halfedgeTarget(heIter));
	} while (heIter = halfedgeNext(heIter), heIter != he2);
	tmp_face_vhandles.push_back(vb);
	heIter = he3;
	do {
		tmp_face_vhandles.push_back(halfedgeTarget(heIter));
	} while (heIter = halfedgeNext(heIter), heIter != he4);
	tmp_face_vhandles.push_back(va);
	FaceHandle face = createFace(tmp_face_vhandles);
	if (cfCount == 1) {
		face->crossFieldDirection = 
	}
	return 0;
}


VertexHandle CToolMesh::addVertexOnEdge(EdgeHandle oldEdge)
{
	HalfedgeHandle he1 = halfedge_handle(oldEdge, 0);
	HalfedgeHandle he2 = halfedgeSym(he1);

	VertexHandle va = halfedgeSource(he1);
	VertexHandle vb = halfedgeTarget(he1);

	Point newPoint = (getPoint(va) + getPoint(vb)) / 2;
	return splitEdge(edgeHalfedge(oldEdge, 0), newPoint);
}

void CToolMesh::clearFace(vector<HalfedgeHandle> heVector) {
	HalfedgeHandle he1 = NULL, he2;
	vector<CToolVertex*> vVector;
	he2 = heVector.front();
	for (HalfedgeHandle he : heVector) {
		vVector.push_back(halfedgeSource(he));
		if (he1 == NULL) {
			he1 = he2;
			continue;
		}
		he2 = he;
		topology_assert(halfedgeTarget(he1) == halfedgeSource(he2));
		he1 = he2;
	}
	topology_assert(halfedgeTarget(he2) == halfedgeSource(heVector.front()));
	for (int i = 0; i < heVector.size(); i++) {
		//           v2
		//     ^     ^
		//      \   he2
		//    curHe  |
		//        \  |
		//         \ |
		// v0--he1-->v1
		// 
		//  he1==heVector[i]
		std::queue<CToolVertex*> q;
		HalfedgeHandle he1 = heVector[i];
		HalfedgeHandle he2 = heVector[(i + 1) % heVector.size()];
		VertexHandle v0 = halfedgeSource(he1);
		VertexHandle v1 = halfedgeTarget(he1);
		VertexHandle v2 = halfedgeTarget(he2);
		HalfedgeHandle curHe = halfedgeNext(he1);
		while (curHe != he2)
		{
			auto vViter = find(vVector.begin(), vVector.end(), halfedgeTarget(curHe));
			if (vViter != vVector.end()) {
				curHe = halfedgeNext(halfedgeSym(curHe));
				deleteEdgeMergeFace(vertexEdge(v1, *vViter));
				continue;
			}
			if (!halfedgeTarget(curHe)->markDelete) {
				q.push(halfedgeTarget(curHe));
				halfedgeTarget(curHe)->markDelete = true;
			}
			curHe = halfedgeNext(halfedgeSym(curHe));
		}
		while (!q.empty()) {
			//HalfedgeHandle he = vertexHalfedge(q.front());
			//assert(q.front() == halfedgeTarget(vertexHalfedge(q.front())));
			if (vertexHalfedge(q.front())) {
				for (VertexVertexIter vv_it(this, q.front()); !vv_it.end(); ++vv_it) {
					if (find(vVector.begin(), vVector.end(), *vv_it) == vVector.end()) {
						if (!(*vv_it)->markDelete) {
							q.push(*vv_it);
							(*vv_it)->markDelete = true;
						}
					}
				}
			}
			deleteVertexMergeFace(q.front());
			q.pop();
		}
	}
}

void CToolMesh::clearFace(initializer_list<HalfedgeHandle> heList)
{
	clearFace(vector<HalfedgeHandle>(heList));
}

int CToolMesh::numQuad(EdgeHandle edge)
{
	assert(edge != NULL);
	int num = 0;
	if (edgeFace1(edge) != NULL && isQuad(edgeFace1(edge)))
		num++;
	if (edgeFace2(edge) != NULL && isQuad(edgeFace2(edge)))
		num++;
	return num;
}

int CToolMesh::numTriangles(VertexHandle vert) {
	assert(vert != NULL);
	int num = 0;
	for (VertexFaceIter vfiter(this, vert); !vfiter.end(); ++vfiter) {
		if (!isQuad(*vfiter))
			num++;
	}
	return num;
}

int CToolMesh::numQuad(VertexHandle vert)
{
	assert(vert != NULL);
	int num = 0;
	for (VertexFaceIter vfiter(this, vert); !vfiter.end(); ++vfiter) {
		if (isQuad(*vfiter))
			num++;
	}
	return num;
}


bool CToolMesh::isQuad(FaceHandle face)
{
	assert(face);
	int vertNum = 0;
	for (FaceVertexIter fviter(face); !fviter.end(); ++fviter) {
		vertNum++;
	}
	return vertNum == 4;
}

void CToolMesh::unsetHalfedge(VertexHandle v, HalfedgeHandle he) {
	topology_assert(halfedgeVertex(he) == v);
	if (vertexHalfedge(v) != he) {
		return;
	}
	if (halfedgeNext(he) == halfedgeSym(he)) {
		setHalfedge(v, NULL);
		return;
	}
	else {
		setHalfedge(v, halfedgeSym(halfedgeNext(he)));
		return;
	}
}

CPoint CToolMesh::edgeCrossField(EdgeHandle edge, int id) {
	if (isBoundary(edge)) {
		return edgeFace1(edge)->crossFieldDirection[id];
	}
	HalfedgeHandle he1 = edgeHalfedge(edge, 0);
	CPoint crossFieldVec1 = halfedgeFace(he1)->crossFieldDirection[id];
	CPoint crossFieldVec2 = halfedgeFace(halfedgeSym(he1))->crossFieldDirection[(id + he1->crossFieldMatching)%4];
	CPoint edgeCrossFieldVec = crossFieldVec1 + crossFieldVec2;
	edgeCrossFieldVec = edgeCrossFieldVec / edgeCrossFieldVec.norm();
	return edgeCrossFieldVec;
}

CPoint CToolMesh::vertexCrossField(VertexHandle vertex, int id) {
	if (vertex->isSingular) {
		return CPoint(0,0,0);
	}
	int rotationIndex = id;
	CPoint vertexCrossFieldVec(0, 0, 0);
	if (isBoundary(vertex)) {
		for (VertexIHalfedgeIter vhIter(this, vertex); !vhIter.end(); vhIter++) {
			if (isBoundary(*vhIter)) {
				int rotationIndex = 0;
				HalfedgeHandle he = halfedgeSym(*vhIter);
				do {
					if (isBoundary(halfedgeEdge(halfedgePrev(he)))) {
						break;
					}
					rotationIndex += halfedgePrev(he)->crossFieldMatching;
				} while (he = halfedgeSym(halfedgePrev(he)), !isBoundary(he));
				vertexCrossFieldVec += halfedgeFace(halfedgeSym(*vhIter))->crossFieldDirection[id];
				vertexCrossFieldVec += halfedgeFace(halfedgeSym(halfedgeNext(*vhIter)))->crossFieldDirection[(id+rotationIndex)%4];
				vertexCrossFieldVec /= vertexCrossFieldVec.norm();
				return vertexCrossFieldVec;
			}
		}
	}
	for (VertexIHalfedgeIter vihIter(this, vertex); !vihIter.end(); vihIter++) {
		vertexCrossFieldVec += halfedgeFace(*vihIter)->crossFieldDirection[rotationIndex % 4];
		rotationIndex += halfedgeNext(*vihIter)->crossFieldMatching;
	}
	vertexCrossFieldVec /= vertexCrossFieldVec.norm();
	return vertexCrossFieldVec;
}

CPoint CToolMesh::nearestCrossField(VertexHandle vertex, CPoint localDirection, int& index) {
	CPoint nearest(0, 0, 0);
	double minAngle = 360;
	for (int i = 0; i < 4; i++) {
		CPoint cf = vertexCrossField(vertex, i);
		double angle = this->angle(cf+getPoint(vertex), vertex, localDirection+getPoint(vertex));
		if (angle < minAngle) {
			nearest = cf;
			index = i;
		}
	}
	return nearest;
}

int CToolMesh::getMatchingIndex(CPoint cp1, CPoint cp2) {
	cp1 /= cp1.norm();
	cp2 /= cp2.norm();
	if ((cp1 + cp2).norm() < FLT_EPSILON) {
		return 2;
	}
	if ((cp1 - cp2).norm() < FLT_EPSILON) {
		return 0;
	}
	double angle = acos_limited(cp1.dot(cp2));
	if (angle < 45) {
		return 0;
	}
	if (angle < 135) {
		return 1;
	}
	return 2;
}

void CToolMesh::alignToCrossField(EdgeHandle edge, VertexHandle pivotVertex) {
	CPoint edgeVector = pivotVertex == edgeVertex1(edge) ? getPoint(edgeVertex2(edge)) - getPoint(edgeVertex1(edge)) : getPoint(edgeVertex1(edge)) - getPoint(edgeVertex2(edge));
	int index;
	CPoint cfDirection = nearestCrossField(pivotVertex, edgeVector, index);
	CPoint newEdgeVector = cfDirection * edgeVector.dot(cfDirection);
	if (pivotVertex == edgeVertex1(edge)) {
		setPoint(edgeVertex2(edge), newEdgeVector + getPoint(pivotVertex));
	}
	else {
		setPoint(edgeVertex1(edge), newEdgeVector + getPoint(pivotVertex));
	}
}