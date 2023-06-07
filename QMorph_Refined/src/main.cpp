#include <iostream>
#include <vector>
#include "QMorph.h"

int main(int argc, char* argv[]) {
	CTMesh mesh;
	// get filename
	if (argc < 2) {
		std::cout << "Please input filename" << std::endl;
		return -1;
	}
	// get extension
	std::string ext = argv[1];
	std::string name = ext.substr(0, ext.find_last_of("."));
	ext = ext.substr(ext.find_last_of(".") + 1);
	if (ext == "obj") {
		mesh.read_obj(argv[1]);
	}
	else if (ext == "off") {
		mesh.read_off(argv[1]);
	}
	else if (ext == "m") {
		mesh.read_m(argv[1]);
	}
	else {
		std::cout << "Unsupported file format" << std::endl;
		return -1;
	}
	mesh.calculateCrossField();
	mesh.highlightCrossField();
	mesh.highlightSingularCrossField();
	mesh.updateDebug();
	//QMorph qmorph(&mesh);
	//qmorph.doQMorphProcess();
	//int totalCount = 0;
	//int irregularCount = 0;
	//for (CTMesh::VertexIter vIter(&mesh); !vIter.end(); vIter++) {
	//	if (mesh.isBoundary(*vIter)) {
	//		continue;
	//	}
	//	int count = 0;
	//	for (CTMesh::VertexEdgeIter vfIter(&mesh,*vIter); !vfIter.end(); vfIter++) {
	//		count++;
	//	}
	//	if (count != 4) {
	//		irregularCount++;
	//	}
	//	totalCount++;
	//}
	//string newSuffix = "_qmorph.obj";
	//mesh.write_obj((name + newSuffix).c_str());
	return 0;
	
}