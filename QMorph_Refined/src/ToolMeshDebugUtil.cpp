#include "ToolMesh.h"


void CToolMesh::highlight(VertexHandle vert) {
#ifdef _DEBUG
	Point p = getPoint(vert);
	debug.HighlightVertex(new double[3] { p[0], p[1], p[2]});
#endif
}

void CToolMesh::highlight(CPoint p) {
#ifdef _DEBUG
	debug.HighlightVertex(new double[3] { p[0], p[1], p[2]});
#endif
}

void CToolMesh::highlight(EdgeHandle edge) {
#ifdef _DEBUG
	Point p1, p2;
	p1 = getPoint(edgeVertex1(edge));
	p2 = getPoint(edgeVertex2(edge));
	debug.HighlightEdge(new double[3] {p1[0], p1[1], p1[2]}, new double[3] {p2[0], p2[1], p2[2]});
#endif
}

void CToolMesh::highlight(CPoint p1,CPoint p2) {
#ifdef _DEBUG
	debug.HighlightEdge(new double[3] {p1[0], p1[1], p1[2]}, new double[3] {p2[0], p2[1], p2[2]});
#endif
}

void CToolMesh::highlightCrossField() {
#ifdef _DEBUG
	for (CTMesh::FaceIter fiter(this); !fiter.end(); fiter++) {
		CPoint center(0, 0, 0);
		double length = 0.0;
		int pointCount = 0;
		for (CTMesh::FaceVertexIter fvIter(*fiter); !fvIter.end(); fvIter++) {
			center += getPoint(*fvIter);
			pointCount++;
		}
		for (CTMesh::FaceEdgeIter feIter(*fiter); !feIter.end(); feIter++) {
			length += this->length(edgeHalfedge(*feIter,0));
		}
		center /= pointCount;
		length /= pointCount*3;
		highlight(center, (*fiter)->crossFieldDirection[0]* length + center);
		highlight(center, (*fiter)->crossFieldDirection[1] * length + center);
		highlight(center, (*fiter)->crossFieldDirection[2] * length + center);
		highlight(center, (*fiter)->crossFieldDirection[3] * length + center);
	}
#endif
}

void CToolMesh::highlightEdgeCrossField() {
#ifdef _DEBUG
	for (CTMesh::EdgeIter eiter(this); !eiter.end(); eiter++) {
		CPoint center=(getPoint(edgeVertex1(*eiter))+ getPoint(edgeVertex2(*eiter)))/2;
		double length = this->length(edgeHalfedge(*eiter, 0));
		highlight(center, this->edgeCrossField(*eiter, 0) * length /3 + center);
		highlight(center, this->edgeCrossField(*eiter, 1) * length / 3 + center);
		highlight(center, this->edgeCrossField(*eiter, 2) * length / 3 + center);
		highlight(center, this->edgeCrossField(*eiter, 3) * length / 3 + center);
	}
#endif
}

void CToolMesh::highlightVertexCrossField() {
#ifdef _DEBUG
	for (CTMesh::VertexIter viter(this); !viter.end(); viter++) {
		double length = this->length(vertexHalfedge(*viter));
		CPoint center = getPoint(*viter);
		highlight(center, this->vertexCrossField(*viter, 0) * length / 3 + center);
		highlight(center, this->vertexCrossField(*viter, 1) * length / 3 + center);
		highlight(center, this->vertexCrossField(*viter, 2) * length / 3 + center);
		highlight(center, this->vertexCrossField(*viter, 3) * length / 3 + center);
	}
#endif
}

void CToolMesh::highlightSingularCrossField() {
#ifdef _DEBUG
	for (VertexIter vIter(this); !vIter.end(); vIter++) {
		if ((*vIter)->isSingular) {
			highlight(*vIter);
		}
	}
#endif
}

void CToolMesh::highlight(HalfedgeHandle edge) {
	highlight(halfedgeEdge(edge));
}

void CToolMesh::topology_assert(bool expr, initializer_list<Component*> componentList) {
	if (!expr) {
		cout << "ERROR: Topology assert failed" << endl;
		highlight(componentList);
		updateDebug();
		assert(false);
	}
}

void CToolMesh::highlight(initializer_list<Component*> componentList) {
	for (Component* component : componentList) {
		if (typeid(*component->getPointer()) == typeid(CToolEdge)) {
			highlight((CToolEdge*)component);
		}
		if (typeid(*component->getPointer()) == typeid(CToolHalfedge)) {
			highlight((CToolHalfedge*)component);
		}
		if (typeid(*component->getPointer()) == typeid(CToolVertex)) {
			highlight((CToolVertex*)component);
		}
	}
}

void CToolMesh::updateDebug() {
#ifdef _DEBUG
	debug.SetModel(this->write_obj_dump(), ".obj");
	//write_obj("debug.obj");
	debug.Update();
#endif
}