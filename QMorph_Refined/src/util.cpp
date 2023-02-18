#include"util.h"


double eDist(Point cp1, Point cp2) {
	return sqrt((cp1[0] - cp2[0]) * (cp1[0] - cp2[0]) +
		(cp1[1] - cp2[1]) * (cp1[1] - cp2[1]) +
		(cp1[2] - cp2[2]) * (cp1[2] - cp2[2]));
}

void reportIter(int i, string name, bool direct) {
	if (i == 0) {
		cout << endl << name<<endl;
	}
	if (i % 50 == 0 && i != 0) {
		//cout << endl << i << " ... ";
	}
	else if (direct||i % 10 == 0) {
		//cout << i << " ... ";
	}
}
// solve a linear equation that using AC&AD to represent AB
// k*AB = m*AC+(1-m)*AD
double* solveKMEquation(Point AC, Point AD, Point AB) {
	//Point C & D mustn't be the same pos.
	assert(eDist(AC, AD) > EPSILON);
	Point normal = AD .cross(AC);
	normal = normal / normal.norm();
	//projection on the patch
	Point projection = AB - normal * (AB * normal);
	AB = projection;
	double cNorm = AC.norm();
	AD = AD / cNorm;
	AC = AC / cNorm;
	bool AB_normalized = false;
	if (AB.norm() > 5 || AB.norm() < 0.2) {
		AB = AB / cNorm;
		AB_normalized = true;
	}
	// solve the 2x2 linear system
	int ind1 = 1, ind2 = 2;
	if (abs((AD[ind1] - AC[ind1]) * AB[ind2] - (AD[ind2] - AC[ind2]) * AB[ind1]) < EPSILON) {
		ind1 = 0;
		ind2 = 1;
	}
	if (abs((AD[ind1] - AC[ind1]) * AB[ind2] - (AD[ind2] - AC[ind2]) * AB[ind1]) < EPSILON) {
		ind1 = 0;
		ind2 = 2;
	}
	if (abs((AD[ind1] - AC[ind1]) * AB[ind2] - (AD[ind2] - AC[ind2]) * AB[ind1]) < EPSILON) {
		assert(false);
	}
	//coefficientMat << projection[ind1], AC[ind1] - AD[ind1], projection[ind2], AC[ind2] - AD[ind2];
	//Eigen::Vector2d b(AC[ind1], AC[ind2]);
	//Eigen::Vector2d x = coefficientMat.fullPivLu().solve(b);
	//x[0] = x[0] / AC.norm();
	double l = ((AD[ind1] - AC[ind1]) * AB[ind2] - (AD[ind2] - AC[ind2]) * AB[ind1]);
	assert(abs(l) > EPSILON);
	double k = (AD[ind1] * AC[ind2] - AC[ind1] * AD[ind2])
		/ ((AD[ind1] - AC[ind1]) * AB[ind2] - (AD[ind2] - AC[ind2]) * AB[ind1]);
	double m = (AD[ind1] * AB[ind2] - AB[ind1] * AD[ind2])
		/ ((AD[ind1] - AC[ind1]) * AB[ind2] - (AD[ind2] - AC[ind2]) * AB[ind1]);
	if (AB_normalized) {
		return new double[2] {k, m};
	}
	else {
		return new double[2] {k* cNorm, m};

	}
}

Point convertLaplacianOrigin(Point origin) {
	return origin;
}

Point bisectorOfCpoints(Point left, Point right, Point normal) {
	left = left / left.norm();
	right = right / right.norm();
	normal = normal / normal.norm();
	Point sharpBisector = left + right;
	sharpBisector = sharpBisector / sharpBisector.norm();
	if ((right.cross(left)).dot(normal) > EPSILON) {
		return sharpBisector;
	}
	else if ((right .cross(left)).dot(normal) < -EPSILON) {
		return -sharpBisector;
	}
	else {
		return (right.cross(normal)) / (right .cross(normal)).norm();
	}
}