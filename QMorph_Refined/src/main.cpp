#include <iostream>
#include <vector>
#include "QMorph.h"

// ..\data\ccc.m: deleteVertexMergeFace at point(id==870)

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
	QMorph qmorph(&mesh);
	qmorph.doQMorphProcess();
	string newSuffix = "_qmorph.obj";
	mesh.write_obj((name + newSuffix).c_str());
	return 0;
	
}