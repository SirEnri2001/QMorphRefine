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