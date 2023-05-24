#include "ToolMesh.h"

// calculate the bisector of angle between he1 & he1_next. result based on local coordination.
// he1 and he1_next must be connected and continous. 
Point CToolMesh::bisector(HalfedgeHandle he1, HalfedgeHandle he1_next) {
	CPoint joint;
	//assert(he1 == nextBoundaryHalfedge(he1_next) || he1_next == nextBoundaryHalfedge(he1));
	topology_assert(halfedgeTarget(he1) == halfedgeSource(he1_next), { he1, he1_next });
	//--he1-->--he1_next-->
	joint = getPoint(halfedgeTarget(he1));
	//<--v1--.--v2-->
	Point cp1 = getPoint(halfedgeSource(he1)) - joint;
	Point cp2 = getPoint(halfedgeTarget(he1_next)) - joint;
	cp1 = cp1 / cp1.norm();
	cp2 = cp2 / cp2.norm();
	Point cp3 = cp1 + cp2;
	CPoint c;
	Point localNormal = normalVertex(halfedgeSource(he1));
	localNormal = localNormal/localNormal.norm();
	if ((cp2.cross(cp1)).norm() < 1e-4 && cp2.dot(cp1) < 0.0) {
		Point bisector = cp1.cross(localNormal);
		bisector = bisector / bisector.norm();
		return bisector;
	}
	if (cp2.cross(cp1).dot(localNormal) < 0) {
		cp3 = -cp3;
	}
	return cp3 / cp3.norm();
}

Point CToolMesh::normalVertex(VertexHandle vertex) {
	Point vsum(0.0, 0.0, 0.0);
	for (CTMesh::VertexFaceIter vfiter(this, vertex); !vfiter.end(); vfiter++) {
		Point a, b, c;
		FaceHandle patch = *vfiter;
		HalfedgeHandle he = halfedge_handle(patch);
		a = getPoint(halfedgeTarget(he));
		b = getPoint(halfedgeTarget(halfedgeNext(he)));
		c = getPoint(halfedgeTarget(halfedgeNext(halfedgeNext(he))));
		vsum += (c - b).cross(a - b);
	}
	return vsum / vsum.norm();
}

double CToolMesh::angle(Point cp1, VertexHandle pivot, Point cp2) {
	Point v1 = cp1 - getPoint(pivot);
	Point v2 = cp2 - getPoint(pivot);
	return acos_limited(v1.dot(v2) / (v1.norm() * v2.norm())) / PI * 180;

}
double CToolMesh::angle(HalfedgeHandle bhe, HalfedgeHandle bhe_next) {
	Point BA = getPoint(halfedgeSource(bhe)) - getPoint(halfedgeTarget(bhe));
	Point BC = getPoint(halfedgeTarget(bhe_next))
		- getPoint(halfedgeSource(bhe_next));
	Point normal = (getPoint(halfedgeTarget(halfedgeNext(bhe)))
		- getPoint(halfedgeTarget(bhe)))
		.cross(getPoint(halfedgeSource(bhe)) - getPoint(halfedgeTarget(bhe)));
	Point p = getPoint(halfedgeTarget(bhe));
	double res;
	if (normal.dot(BC.cross(BA)) < 0.0) {
		res = 360.0 - acos_limited(BC.dot(BA) / (BC.norm() * BA.norm())) / PI * 180;
	}
	else {
		res = acos_limited(BA.dot(BC) / (BA.norm() * BC.norm())) / PI * 180;
	}
	if (res != res) {
		assert(false);
	}
	return res;
}
double CToolMesh::angle(Point cpoint, HalfedgeHandle he) {
	Point cp1 = getPoint(halfedgeTarget(he)) - getPoint(halfedgeSource(he));
	Point cp2 = cpoint - getPoint(halfedgeSource(he));
	/*if (normalOfVertex(mesh, halfedgeSource(he)) * (cp2 ^ cp1) < 0.0) {
		return 360.0 - acos(cp1 * cp2 / (cp1.norm() * cp2.norm())) / PI * 180;
	}*/
	return acos_limited(cp2.dot(cp1) / (cp1.norm() * cp2.norm())) / PI * 180;

}

double CToolMesh::getWeight(EdgeHandle he) {
	return 1.0;
}


double CToolMesh::length(HalfedgeHandle he) {
	return (getPoint(halfedgeTarget(he)) - getPoint(halfedgeSource(he))).norm();
}
