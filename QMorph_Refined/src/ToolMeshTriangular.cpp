// This file defines functions that constrains in a triangular mesh components
#include "ToolMesh.h"

//             v1
//            /  \
//           /    \
//          /      \
//         /        \
//--bhe1->va-oldEdge->vb--bhe2->
//         \        /
//          \      /
//           \    /
//            \  /
//             v2
// Only allowed in triangular mesh
VertexHandle CToolMesh::splitEdge(HalfedgeHandle oldHe, Point& pos)
{
	if (halfedgeFace(oldHe)) {
		topology_assert(!isQuad(halfedgeFace(oldHe)), { oldHe });
	}
	if (halfedgeFace(halfedgeSym(oldHe))) {
		topology_assert(!isQuad(halfedgeFace(halfedgeSym(oldHe))), { oldHe });
	}
	EdgeHandle oldEdge = halfedgeEdge(oldHe);
	VertexHandle newVertex = createVertex(pos);
	VertexHandle va = halfedgeSource(oldHe);
	VertexHandle vb = halfedgeTarget(oldHe);
	VertexHandle v1 = halfedgeTarget(halfedgeNext(oldHe));
	VertexHandle v2 = halfedgeTarget(halfedgeNext(halfedgeSym(oldHe)));
	bool isBoundary0 = isBoundary(oldHe);
	bool isBoundary1 = isBoundary(halfedgeSym(oldHe));
	HalfedgeHandle bhe1 = NULL, bhe2 = NULL;

	if (isBoundary0) {
		bhe1 = halfedgePrev(oldHe);
		bhe2 = halfedgeNext(oldHe);
	}
	if (isBoundary1) {
		bhe1 = halfedgeSym(halfedgeNext(halfedgeSym(oldHe)));
		bhe2 = halfedgeSym(halfedgePrev(halfedgeSym(oldHe)));
	}

	if (!isBoundary0) {
		deleteFace(face_handle(oldHe));
	}
	if (!isBoundary1) {
		deleteFace(face_handle(halfedgeSym(oldHe)));
	}
	setFace(oldHe, NULL);
	setFace(halfedgeSym(oldHe), NULL);
	unsetHalfedge(halfedgeVertex(oldHe), oldHe);
	unsetHalfedge(halfedgeVertex(halfedgeSym(oldHe)), halfedgeSym(oldHe));
	disconnect(oldEdge);
	deleteEdge(oldEdge);
	createEdge(va, newVertex);
	createEdge(newVertex, vb);
	if (!isBoundary0) {
		createFace(newVertex, vb, v1);
		createFace(va, newVertex, v1);
	}
	if (!isBoundary1) {
		createFace(newVertex, va, v2);
		createFace(vb, newVertex, v2);
	}

	if (isBoundary0) {
		setNextHalfedge(bhe1, vertexHalfedge(va, newVertex));
		setNextHalfedge(vertexHalfedge(va, newVertex), vertexHalfedge(newVertex, vb));
		setNextHalfedge(vertexHalfedge(newVertex, vb), bhe2);
	}

	if (isBoundary1) {
		setPrevHalfedge(halfedgeSym(bhe1), vertexHalfedge(newVertex, va));
		setPrevHalfedge(vertexHalfedge(newVertex, va), vertexHalfedge(vb, newVertex));
		setPrevHalfedge(vertexHalfedge(vb, newVertex), halfedgeSym(bhe2));
	}
	return newVertex;
}



EdgeHandle CToolMesh::swapEdge(EdgeHandle oldEdge)
{
	bool debug = false;
	topology_assert(!isBoundary(oldEdge), { edgeHalfedge(oldEdge, 0) });
	HalfedgeHandle he1 = halfedge_handle(oldEdge, 0);
	HalfedgeHandle he2 = halfedge_handle(oldEdge, 1);

	VertexHandle va = halfedgeSource(he1);
	VertexHandle vb = halfedgeTarget(he1);
	VertexHandle v1 = halfedgeTarget(halfedgeNext(he1));
	VertexHandle v2 = halfedgeTarget(halfedgeNext(he2));
	deleteFace(edgeFace1(oldEdge));
	deleteFace(edgeFace2(oldEdge));
	setFace(edgeHalfedge(oldEdge, 0), NULL);
	setFace(edgeHalfedge(oldEdge, 1), NULL);
	unsetHalfedge(halfedgeVertex(edgeHalfedge(oldEdge, 0)), edgeHalfedge(oldEdge, 0));
	unsetHalfedge(halfedgeVertex(edgeHalfedge(oldEdge, 1)), edgeHalfedge(oldEdge, 1));
	disconnect(oldEdge);
	deleteEdge(oldEdge);
	createEdge(v1, v2);
	createFace(v1, va, v2);
	FaceHandle face = createFace(v2, vb, v1);
	he1 = halfedge_handle(face);
	if (halfedgeSource(he1) == v2)
		he1 = halfedgePrev(he1);
	if (halfedgeTarget(he1) == v1)
		he1 = halfedgeNext(he1);
	return halfedgeEdge(he1);
}

// Warning: Lose attribute of halfedges&edges connected to the edge
VertexHandle CToolMesh::mergeEdge(VertexHandle va, VertexHandle vb) {
	//     v1
	//    /  \   /
	//   /    \ /
	//  va----vb---
	//   \    / \
	//    \  /   \
	//     v2
	HalfedgeHandle he1 = sourceTargetHalfedge(va, vb);
	HalfedgeHandle he2 = halfedgeSym(he1);
	VertexHandle v1 = halfedgeTarget(halfedgeNext(he1));
	VertexHandle v2 = halfedgeTarget(halfedgeNext(he2));
	list<VertexHandle> vbConnectedVertices;
	map<int, HalfedgeHandle> outwardHeAttributesMap;
	map<int, HalfedgeHandle> inwardHeAttributesMap;
	for (VertexIHalfedgeIter vinIter(this, vb); !vinIter.end(); ++vinIter) {
		if (halfedgeSource(*vinIter) != va && halfedgeSource(*vinIter) != v1 && halfedgeSource(*vinIter) != v2) {
			HalfedgeHandle outHeAttr = new CToolHalfedge();
			HalfedgeHandle inHeAttr = new CToolHalfedge();
			halfedgeSym(*vinIter)->attributeCopyTo(outHeAttr);
			(*vinIter)->attributeCopyTo(inHeAttr);
			outwardHeAttributesMap.insert(std::pair<int, HalfedgeHandle>((halfedgeSource(*vinIter))->getId(), outHeAttr));
			inwardHeAttributesMap.insert(std::pair<int, HalfedgeHandle>((halfedgeSource(*vinIter))->getId(), inHeAttr));
			vbConnectedVertices.push_back(halfedgeSource(*vinIter));
		}
	}
	deleteVertexMergeFace(vb);
	for (list<VertexHandle>::iterator iter = vbConnectedVertices.begin(); iter != vbConnectedVertices.end(); ++iter) {
		splitFace(va, *iter);
		HalfedgeHandle inwardHe = sourceTargetHalfedge(*iter, va);
		assert(inwardHe);
		outwardHeAttributesMap[(*iter)->getId()]->attributeCopyTo(halfedgeSym(inwardHe));
		inwardHeAttributesMap[(*iter)->getId()]->attributeCopyTo(inwardHe);
		delete outwardHeAttributesMap[(*iter)->getId()];
		delete inwardHeAttributesMap[(*iter)->getId()];
	}
	return va;
}


typedef struct FaceIterTreeNode
{
	struct FaceIterTreeNode* parent;
	struct FaceIterTreeNode* child[3];
	HalfedgeHandle parentConnectedHe;
	FaceHandle face;
	int childNum;
}FaceIterTreeNode;

void deleteTree(FaceIterTreeNode* root) {
	int childIdx = 0;
	while (childIdx < root->childNum) {
		deleteTree(root->child[childIdx]);
		childIdx++;
	}
	delete root;
}
// Nc & Nd must not connected
// Topology version of calculateRambdaSet
list<HalfedgeHandle>* CToolMesh::calculateRambdaSet(VertexHandle Nc, VertexHandle Nd) {
	topology_assert(!vertexEdge(Nc, Nd));
	list<FaceIterTreeNode*> faceIterTrees, roots;
	list<CToolFace*> traversedFace;

	// Init the face-face tree
	for (CTMesh::VertexFaceIter vf_iter(this, Nc); !vf_iter.end(); vf_iter++)
	{
		if (isQuad(*vf_iter)) {
			continue;
		}
		FaceIterTreeNode* root = new FaceIterTreeNode;
		root->parent = NULL;
		root->parentConnectedHe = NULL;
		//root->childNum = 0;
		root->face = *vf_iter;
		faceIterTrees.push_back(root);
		roots.push_back(root);
		traversedFace.push_back(*vf_iter);
	}
	assert(faceIterTrees.size() > 0); // find at least one triangular face around Nc
	FaceIterTreeNode* targetTree = NULL;
	// BFS traverse f-f tree until find the target vertex Nd
	while (targetTree == NULL) {
		FaceIterTreeNode* curTree = faceIterTrees.front();
		faceIterTrees.pop_front();
		int childIdx = 0;
		for (CTMesh::FaceHalfedgeIter fhIter(curTree->face); !fhIter.end(); fhIter++) {
			FaceHandle attachedFace = halfedgeFace(halfedgeSym(*fhIter));
			if (attachedFace && !isQuad(attachedFace)
				&& find(traversedFace.begin(), traversedFace.end(), attachedFace) == traversedFace.end()
				&& !isFront(*fhIter)
				&& !isSideEdge(*fhIter)
				&& !isSideEdge(halfedgeSym(*fhIter))
				)
			{
				traversedFace.push_back(attachedFace);
				FaceIterTreeNode* newNode = new FaceIterTreeNode;
				newNode->face = attachedFace;
				newNode->parentConnectedHe = halfedgeSym(*fhIter);
				newNode->parent = curTree;
				curTree->child[childIdx++] = newNode;
				curTree->childNum++;
				if (halfedgeTarget(halfedgeNext(halfedgeSym(*fhIter))) == Nd) {
					targetTree = newNode;
					break;
				}
				else {
					faceIterTrees.push_back(newNode);
				}
			}
		}
	}
	list<HalfedgeHandle>* rambdaSet = new list<HalfedgeHandle>();
	while (targetTree->parent) {
		rambdaSet->push_back(targetTree->parentConnectedHe);
		targetTree = targetTree->parent;
	}
	for (auto iter = roots.begin(); iter != roots.end(); iter++) {
		deleteTree(*iter);
	}
	return rambdaSet;
}

// Algorithm 2. Formation of Rambda(S) at page 10
// Point & edge position version calculateRambdaSet
// 早就看你不顺眼了，10个bug，9个出在你身上
// Deprecated
list<HalfedgeHandle>* CToolMesh::calculateRambdaSet(int i, VertexHandle Nc, VertexHandle Nd) {
	return calculateRambdaSet(Nc, Nd);
	// front edge must have the left and right side
	//Point Vs = getPoint(Nd) - getPoint(Nc);
	//Point normalNc = (getPoint(halfedgeTarget(halfedgeNext(halfedge_handle(Nc))))
	//	- getPoint(Nc)).cross(getPoint(halfedgeSource(halfedge_handle(Nc))) - getPoint(Nc));

	//CTMesh::VertexOHalfedgeIter EkIter(this, Nc);
	//HalfedgeHandle Ek = *EkIter;
	//bool findFirstCrossed = false;
	//for (; !EkIter.end(); EkIter++) {
	//	if (isBoundary(*EkIter)) {
	//		continue;
	//	}
	//	Ek = *EkIter; // Point from Nc
	//	HalfedgeHandle Ek1 = halfedgePrev(Ek); // Point at Nc
	//	Point Vk = getPoint(halfedgeTarget(Ek)) - getPoint(halfedgeSource(Ek));
	//	Point Vk1 = getPoint(halfedgeSource(Ek1)) - getPoint(halfedgeTarget(Ek1));
	//	double* km = solveKMEquation(Vk, Vk1, Vs);

	//	if (km[1] < 1.0 && km[1]>0.0 && km[0] > 0.0) {
	//		delete[] km;
	//		findFirstCrossed = true;
	//		break;
	//	}
	//	delete[] km;

	//}
	//assert(findFirstCrossed);
	//HalfedgeHandle Ei = halfedgeNext(Ek);
	//HalfedgeHandle Ei_ = halfedgeSym(Ei);
	//list<HalfedgeHandle>* lambdaSet = new list<HalfedgeHandle>();
	//lambdaSet->push_back(Ei);
	//// 8. WHILE not done
	//while (true) {
	//	VertexHandle Ni = halfedgeTarget(halfedgeNext(Ei_));
	//	if (Nd == halfedgeSource(Ei_) || Nd == halfedgeTarget(Ei_) || Nd == Ni) {
	//		break;
	//	}
	//	Point Vi = (getPoint(Ni) - getPoint(Nc)).cross(normalNc);
	//	if (Vs.dot(Vi) < 0) {
	//		Ei = halfedgePrev(Ei_);
	//	}
	//	else {
	//		Ei = halfedgeNext(Ei_);
	//	}
	//	Ei_ = halfedgeSym(Ei);
	//	if (getLeftSide(Ei) || getRightSide(Ei)) {
	//		delete lambdaSet;
	//		return NULL;
	//	}
	//	else {
	//		lambdaSet->push_back(Ei);
	//	}

	//}
	//return lambdaSet;
}

HalfedgeHandle CToolMesh::edgeRecovery(VertexHandle Nc, VertexHandle Nd) {
	assert(Nd != Nc);
	while (sourceTargetHalfedge(Nc, Nd) == NULL) {
		auto lambdaSet = calculateRambdaSet(Nc, Nd);
		assert(lambdaSet);
		while (lambdaSet->size() > 0) {
			if (vertexEdge(
				halfedgeTarget(halfedgeNext(lambdaSet->front())),
				halfedgeTarget(halfedgeNext(halfedgeSym(lambdaSet->front())))
			)) {
				assert(lambdaSet->size() > 1);
				lambdaSet->push_back(lambdaSet->front());
				lambdaSet->pop_front();
			}
			swapEdge(halfedgeEdge(lambdaSet->front()));
			lambdaSet->pop_front();
		}
	}
	assert(sourceTargetHalfedge(Nc, Nd));
	return sourceTargetHalfedge(Nc, Nd);
}
