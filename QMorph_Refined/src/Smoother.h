#pragma once
#include "ToolMesh.h"


class Smoother
{
public:
	void setMesh(CTMesh* mesh);
	Point getDelC(VertexHandle Ni, HalfedgeHandle ife, HalfedgeHandle ofe);
	void boundaryEdgeSmooth(HalfedgeHandle ofe);
	void Smoother::boundaryEdgeSmooth(HalfedgeHandle ife, HalfedgeHandle ofe);
	void triangleInteriorSmooth(VertexHandle, bool); // should eliminate overlap triangle around the vertex
	void quadriInteriorSmooth(VertexHandle);
	void doTriangleSmooth(int epoch = 3);
	int doBoundarySmooth(HalfedgeHandle bhe, int epoch = 1);
  	CTMesh* mesh;
};

