#include "CrossField.h"

typedef enum { CONSTRAINTS, HARD_PRESCRIPTION, SOFT_PRESCRIPTION } ViewingModes;
ViewingModes viewingMode = CONSTRAINTS;

bool alterRoSyWeight = false;

int CrossField::main() {
	if (ct_mesh == nullptr) {
		assert(false);
	}
	// Load mesh
	this->ct_mesh->write_obj_set_map("temp.obj");
	directional::readOBJ("temp.obj", mesh);
	ftb.init(mesh);
	pvFieldHard.init(ftb, directional::fieldTypeEnum::POLYVECTOR_FIELD, N);

	//discovering and constraining sharp edges
	std::vector<int> constFaceslist;
	std::vector<Eigen::Vector3d> constVectorslist;
	for (int i = 0; i < mesh.EF.rows(); i++) {
		if (mesh.EF(i, 1) == -1) {
			constFaceslist.push_back(mesh.EF(i, 0));
			constVectorslist.push_back((mesh.V.row(mesh.EV(i, 0)) - mesh.V.row(mesh.EV(i, 1))).normalized());
		}
		if (mesh.EF(i, 0) == -1) {
			constFaceslist.push_back(mesh.EF(i, 1));
			constVectorslist.push_back((mesh.V.row(mesh.EV(i, 1)) - mesh.V.row(mesh.EV(i, 0))).normalized());
		}
	}

	constFaces.resize(constFaceslist.size());
	constVectors.resize(constVectorslist.size(), 3);
	for (int i = 0; i < constFaces.size(); i++) {
		constFaces(i) = constFaceslist[i];
		constVectors.row(i) = constVectorslist[i];
	}

	//generating the viewing fields
	Eigen::MatrixXd rawFieldConstraints = Eigen::MatrixXd::Zero(mesh.F.rows(), N * 3);
	Eigen::VectorXi posInFace = Eigen::VectorXi::Zero(mesh.F.rows());
	for (int i = 0; i < constFaces.size(); i++) {
		rawFieldConstraints.block(constFaces(i), 3 * posInFace(constFaces(i)), 1, 3) = constVectors.row(i);
		posInFace(constFaces(i))++;
	}

	//Just to show the other direction if N is even, since we are by default constraining it
	if (N % 2 == 0)
		rawFieldConstraints.middleCols(rawFieldConstraints.cols() / 2, rawFieldConstraints.cols() / 2) = -rawFieldConstraints.middleCols(0, rawFieldConstraints.cols() / 2);


	constraintsField.init(ftb, directional::fieldTypeEnum::RAW_FIELD, N);
	constraintsField.set_extrinsic_field(rawFieldConstraints);

	smoothWeight = 1.0;
	roSyWeight = 1.0;
	alignWeights = Eigen::VectorXd::Constant(constFaces.size(), 1.0);
	//ghost mesh only for constraints
	directional::polyvector_field(ftb, constFaces, constVectors, smoothWeight, roSyWeight, Eigen::VectorXd::Constant(constFaces.size(), -1), N, pvFieldHard);
	directional::polyvector_to_raw(pvFieldHard, rawFieldHard, true);
	directional::principal_matching(rawFieldHard);
	
	//write cross field attributes to CTMesh
	for (int faceIndex = 0; faceIndex < mesh.F.rows();faceIndex++) {
		std::vector<CPoint> crossFieldVector;
		for (int j = 0; j < 4; j++) {
			crossFieldVector.push_back(CPoint(
				rawFieldHard.extField(faceIndex, 0+3*j),
				rawFieldHard.extField(faceIndex, 1+3*j),
				rawFieldHard.extField(faceIndex, 2+3*j)
			));
		}

		for (int j = 0; j < 3; j++) {
			int ajacentEdgeIndex = mesh.FE(faceIndex, j);
			bool isFromFaceToAjacent = mesh.EF(ajacentEdgeIndex, 0) == faceIndex;
			int ajacentFaceIndex = isFromFaceToAjacent ? mesh.EF(ajacentEdgeIndex, 1) : mesh.EF(ajacentEdgeIndex, 0);
			if (ajacentFaceIndex == -1) {
				continue;
			}
			int matchingNumber = isFromFaceToAjacent ? rawFieldHard.matching(ajacentEdgeIndex): N - rawFieldHard.matching(ajacentEdgeIndex);
			FaceHandle curFace = ct_mesh->objIdVertexMap[faceIndex];
			FaceHandle adjFace = ct_mesh->objIdVertexMap[ajacentFaceIndex];
			for (CTMesh::FaceHalfedgeIter fhIter(curFace); !fhIter.end();fhIter++) {
				if (ct_mesh->halfedgeFace(ct_mesh->halfedgeSym(*fhIter))) {
					(*fhIter)->crossFieldMatching = matchingNumber;
				}
			}
		}
		ct_mesh->objIdVertexMap[faceIndex]->crossFieldDirection = crossFieldVector;
	}


	return 0;
}