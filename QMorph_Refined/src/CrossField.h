#pragma once
#include "ToolMesh.h"
#include <Eigen/Core>
#include <directional/readOFF.h>
#include <directional/readOBJ.h>
#include <directional/polyvector_to_raw.h>
#include <directional/polyvector_field.h>
#include <directional/principal_matching.h>
#include <directional/write_raw_field.h>
#include <directional/IntrinsicFaceTangentBundle.h>
#include <directional/CartesianField.h>
class CrossField
{
public:
    void setMesh(CTMesh* mesh) {
        this->ct_mesh = mesh;
    }
    CTMesh* ct_mesh;
    int main();

    Eigen::VectorXi constFaces;
    directional::TriMesh mesh;
    directional::IntrinsicFaceTangentBundle ftb;
    directional::CartesianField pvFieldHard, rawFieldHard, constraintsField;
    Eigen::MatrixXd constVectors;
    Eigen::VectorXd alignWeights;

    double smoothWeight, roSyWeight;

    int N = 4;
};
