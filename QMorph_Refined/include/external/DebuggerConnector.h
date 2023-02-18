#pragma once
#ifndef _DEBUGGER_CONNECTOR
#define _DEBUGGER_CONNECTOR
#include <Windows.h>
#include <process.h>
#include <exception>
#undef min
#undef max
#include <iostream>
#include <fstream>
#include <sstream>
#include "json.hpp"

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>

using namespace std;
#define BUF_SIZE 10485760
using json = nlohmann::json;

typedef OpenMesh::PolyMesh_ArrayKernelT<> Mesh;

void inTryBlock(HANDLE write_sem, HANDLE read_sem, LPVOID lpBase, json vertices, json edges, json mesh, bool meshUpdated);

class DebuggerConnector
{
public:
    DebuggerConnector();
    void HighlightVertex(double* v); // array v with 3 elements
    void HighlightEdge(double* v1, double* v2); // array v1, v2 with 3 elements each
    void SetModel(std::string fileDumped, std::string inputExtension); // .ply on sharedmem
    void SetModel(Mesh mesh); // .ply on sharedmem
    void Update();
    string getDumpString();
private:
    json vertices;
    json edges;
    string mesh;
    HANDLE read_sem, write_sem, hMapFile;
    LPVOID lpBase;
    void init();
    bool meshUpdated;
};
#endif