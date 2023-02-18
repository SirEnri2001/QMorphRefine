#pragma once

#ifndef _UTIL
#define _UTIL
#include <initializer_list>
#include "ToolMesh.h"
#define EPSILON FLT_EPSILON

double eDist(Point cp1, Point cp2);
void reportIter(int i, string name = "", bool direct = false);
double* solveKMEquation(Point AC, Point AD, Point AB);
Point convertLaplacianOrigin(Point origin);

Point bisectorOfCpoints(Point left, Point right, Point normal);

#endif