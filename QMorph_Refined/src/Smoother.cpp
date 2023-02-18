#include "Smoother.h"
#include"QMorph.h"
#include"util.h"

void Smoother::setMesh(CTMesh* mesh) {
	this->mesh = mesh;
}

Point Smoother::getDelC(VertexHandle Ni, HalfedgeHandle ife, HalfedgeHandle ofe) {
	double lD;
	HalfedgeHandle lhe = mesh->halfedgePrev(mesh->halfedgePrev(mesh->halfedgeSym(ife)));
	HalfedgeHandle rhe = mesh->halfedgeNext(mesh->halfedgeNext(mesh->halfedgeSym(ofe)));
	Point vd1 = mesh->getPoint(mesh->halfedgeSource(lhe)) - mesh->getPoint(mesh->halfedgeTarget(lhe));
	Point vd2 = mesh->getPoint(mesh->halfedgeTarget(rhe)) - mesh->getPoint(mesh->halfedgeSource(rhe));
	double angle = mesh->angle(lhe, rhe);
	VertexHandle Nj = mesh->halfedgeSource(mesh->halfedgePrev(mesh->halfedgeSym(ife)));
	Point Pi = mesh->getPoint(Ni) - mesh->getPoint(Nj);
	//Point Pi = Ni->getPoint() - Nj->getPoint();
	lD = (vd1.norm() + vd2.norm()) / 2.0 / sin(angle / 180 * PI / 2); // Equation 2.1 & 2.2
	//lA = (delA + Pi).norm();
	//Point delB = -Pi + (delA + Pi) * lD / lA;
	Point normal = (mesh->getPoint(mesh->halfedgeSource(ife)) - mesh->getPoint(Ni)).cross(mesh->getPoint(Nj) - mesh->getPoint(Ni));
	Point Pb1 = bisectorOfCpoints(mesh->getPoint(mesh->halfedgeSource(ife)) - mesh->getPoint(Nj),
		mesh->getPoint(mesh->halfedgeTarget(ofe)) - mesh->getPoint(Nj), normal);
	Point Pb2 = bisectorOfCpoints(Pb1, mesh->getPoint(Ni) - mesh->getPoint(Nj), (mesh->getPoint(Ni) - mesh->getPoint(Nj)).cross(Pb1));
	Pb2 = Pb2 / Pb2.norm();
	double* km = solveKMEquation(mesh->getPoint(mesh->halfedgeSource(ife)) - mesh->getPoint(Nj),
		mesh->getPoint(mesh->halfedgeTarget(ofe)) - mesh->getPoint(Nj), Pb2);
	double lQ = km[0];
	if (lD > lQ) {
		Pb2 = Pb2 * (lD + lQ) / 2;
	}
	else {
		Pb2 = Pb2 * lD;
	}
	delete[] km;
	Point delC = Pb2 - Pi;
	return delC;
}

// p.28 Paving: A New Approach ...
// bV has to be on the frontEdge which is not defined initially
void Smoother::boundaryEdgeSmooth(HalfedgeHandle ofe) {
	HalfedgeHandle ife = mesh->getPrevFe(ofe); //In & Out front edge
	boundaryEdgeSmooth(ife, ofe);
}

void Smoother::boundaryEdgeSmooth(HalfedgeHandle ife, HalfedgeHandle ofe) {
	VertexHandle bV = mesh->halfedgeSource(ofe);
	// Calculating Vector 2.12
	Point viSum(0, 0, 0);
	HalfedgeHandle quardHe = ife;
	int count = 0;

	while (mesh->halfedgeSym(quardHe) != ofe) {
		quardHe = mesh->halfedgeSym(quardHe);
		Point p1 = mesh->getPoint(mesh->halfedgeTarget(quardHe));
		Point p2 = mesh->getPoint(mesh->halfedgeSource(mesh->halfedgePrev(quardHe)));
		Point p3 = mesh->getPoint(mesh->halfedgeTarget(mesh->halfedgeNext(quardHe)));

		viSum = viSum + p1 + p2 - p3; // Vmj+Vml-Vmk
		if (viSum[0] != viSum[0]) {
			assert(false);
		}
		quardHe = mesh->halfedgePrev(quardHe);

		count++;
	}
	viSum = viSum / count;
	Point delA = viSum - mesh->getPoint(bV);
	if (count != 2) {
		mesh->setPoint(bV, mesh->getPoint(bV) + delA);
		return;
	}
	double lD, lA;
	HalfedgeHandle lhe = mesh->halfedgePrev(mesh->halfedgePrev(mesh->halfedgeSym(ife)));
	HalfedgeHandle rhe = mesh->halfedgeNext(mesh->halfedgeNext(mesh->halfedgeSym(ofe)));
	Point vd1 = mesh->getPoint(mesh->halfedgeSource(lhe)) - mesh->getPoint(mesh->halfedgeTarget(lhe));
	Point vd2 = mesh->getPoint(mesh->halfedgeTarget(rhe)) - mesh->getPoint(mesh->halfedgeSource(rhe));
	double angle = mesh->angle(lhe, rhe);
	VertexHandle Nj = mesh->halfedgeSource(mesh->halfedgePrev(mesh->halfedgeSym(ife)));
	Point Pi = mesh->getPoint(bV) - mesh->getPoint(Nj);
	lD = (vd1.norm() + vd2.norm()) / 2.0 / sin(angle / 180 * PI / 2); // Equation 2.1 & 2.2
	lA = (delA + Pi).norm();
	Point delB = -Pi + (delA + Pi) * lD / lA;
	Point normal = (mesh->getPoint(mesh->halfedgeSource(ife)) - mesh->getPoint(bV)).cross(mesh->getPoint(Nj) - mesh->getPoint(bV));
	if (normal[0] != normal[0]) {// nan condition

		assert(false);
	}
	Point Pb1 = bisectorOfCpoints(mesh->getPoint(mesh->halfedgeSource(ife)) - mesh->getPoint(Nj),
		mesh->getPoint(mesh->halfedgeTarget(ofe)) - mesh->getPoint(Nj), normal);
	Point Pb2 = bisectorOfCpoints(Pb1, Pi, Pi.cross(Pb1));
	double* km = solveKMEquation(mesh->getPoint(mesh->halfedgeSource(ife)) - mesh->getPoint(Nj),
		mesh->getPoint(mesh->halfedgeTarget(ofe)) - mesh->getPoint(Nj), Pb2);
	double lQ = Pb2.norm() * km[0];
	if (lD > lQ) {
		Pb2 = Pb2 / Pb2.norm() * (lD + lQ) / 2;
	}
	else {
		Pb2 = Pb2 / Pb2.norm() * lD;
	}
	Point delC = Pb2 - Pi;
	Point delI = (delB + delC) / 2;
	mesh->setPoint(bV, mesh->getPoint(bV) + delI);
	// move bV to the midpoint of ife & ofe slightly
	//Point midPoint = (mesh->getPoint(mesh->halfedgeSource(ife)) + mesh->getPoint(mesh->halfedgeTarget(ofe))) / 2;
	//mesh->setPoint(bV, (mesh->getPoint(bV) + delI + midPoint*0.3) / 1.3);
	delete[] km;
	return;

}


double getTriangularWeight(CTMesh* mesh, EdgeHandle edge)
{
	return -1.0 / exp((mesh->getPoint(mesh->edgeVertex1(edge)) - mesh->getPoint(mesh->edgeVertex2(edge))).norm());
	/*
	double res = 0.0;

	HalfedgeHandle half_edge = mesh->edgeHalfedge(edge, 0);
	VertexHandle v1 = mesh->halfedgeSource(half_edge);
	VertexHandle v2 = mesh->halfedgeTarget(half_edge);

	//求出三角形的第三个点
	VertexHandle v3 = mesh->halfedgeTarget(mesh->halfedgeNext(half_edge));
	res += (v1->getPoint() - v3->getPoint()) * (v2->getPoint() - v3->getPoint())
		/ ((v1->getPoint() - v3->getPoint()) ^ (v2->getPoint() - v3->getPoint())).norm();

	//如果是边界点，则权重只有一边的三角形
	if (mesh->isBoundary(edge))
	{
		return res;
	}

	//指向另外的三角形
	half_edge = mesh->halfedgeSym(half_edge);
	VertexHandle v4 = half_edge->he_next()->vertex();
	res += (v1->getPoint() - v4->getPoint()) * (v2->getPoint() - v4->getPoint())
		/ ((v1->getPoint() - v4->getPoint()) ^ (v2->getPoint() - v4->getPoint())).norm();
	return res;*/
}

Point convertLaplacianOrigin(Point origin);

void Smoother::triangleInteriorSmooth(VertexHandle vertex, bool legacyWeight = false) {

	// average point position
	CPoint averagePos(0.0, 0.0, 0.0);
	int count = 0;
	for (CTMesh::VertexVertexIter vviter(mesh, vertex); !vviter.end(); vviter++) {
		averagePos = averagePos + (*vviter)->point();
		count++;
	}
	averagePos = averagePos / count;
	mesh->setPoint(vertex, averagePos);
	
	/*double weightSum = 0.0;
	Point sum(0, 0, 0);
	for (CTMesh::VertexOHalfedgeIter outIter(mesh, vertex); !outIter.end(); outIter++) {
		double weight;
		if (legacyWeight) {
			weight = mesh->getWeight(mesh->halfedgeEdge(*outIter));
		}
		else {
			weight = getTriangularWeight(mesh, mesh->halfedgeEdge(*outIter));
		}
		sum = sum + (convertLaplacianOrigin(mesh->getPoint(mesh->halfedgeTarget(*outIter)) - mesh->getPoint(vertex))
			* weight);
		weightSum += weight;
	}
	Point del = sum / weightSum;
	mesh->setPoint(vertex, mesh->getPoint(vertex) + del);*/
}

void Smoother::quadriInteriorSmooth(VertexHandle vertex) {
	CTMesh::VertexOHalfedgeIter outIter(mesh, vertex);
	double divisor = 0.0;
	Point sum(0, 0, 0);
	for (; !outIter.end(); outIter++) {
		Point Cj(0, 0, 0);
		if (mesh->isBoundary(mesh->halfedgeTarget(*outIter))) {
			Cj = mesh->getPoint(mesh->halfedgeTarget(*outIter)) - mesh->getPoint(vertex) +
				getDelC(vertex, mesh->halfedgeSym(mesh->halfedgeNext(mesh->halfedgeSym(*outIter))),
					mesh->halfedgeSym(mesh->halfedgePrev(*outIter)));
		}
		else {
			Cj = mesh->getPoint(mesh->halfedgeTarget(*outIter)) - mesh->getPoint(vertex);
		}
		sum = sum + Cj * Cj.norm();
		divisor += Cj.norm();
	}
	Point del = sum / divisor;
	mesh->setPoint(vertex, mesh->getPoint(vertex) + del);
	return;
}

int Smoother::doBoundarySmooth(HalfedgeHandle bhe, int epoch) {
	HalfedgeHandle he = bhe;
	HalfedgeHandle he_start = he;
	int i = 0;
	while (i < epoch) {
		do {
			reportIter(i++, "doBoundarySmooth");
			if (!mesh->isBoundary(mesh->halfedgeSym(he))) {
				boundaryEdgeSmooth(he);
			}
			he = mesh->getNextFe(he);
		} while (he != he_start);
	}
	return 0;
}

void Smoother::doTriangleSmooth(int epoch) {
	while (epoch > 0) {
		CTMesh::VertexIter vIter(mesh);
		for (; !vIter.end(); vIter++) {
			if (!mesh->isBoundary(*vIter)
				&& mesh->numQuad(*vIter) > 0
				&& !mesh->isFront(*vIter)) {
				;
			}
			else if (!mesh->isBoundary(*vIter) && !mesh->isFront(*vIter)) {
				triangleInteriorSmooth(*vIter, true);
			}
		}
		epoch--;
	}

}