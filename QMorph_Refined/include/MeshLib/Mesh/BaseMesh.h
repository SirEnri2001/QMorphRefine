/*!
*      \file BaseMesh.h
*      \brief Base Mesh Class for all types of Mesh Classes
*
*		This is the fundamental class for meshes
*	   \author David Gu
*		\modified SirEnri 16/2/2023
*      \date 10/07/2010
*
*/

#ifndef _MESHLIB_BASE_MESH_H_
#define _MESHLIB_BASE_MESH_H_

#define MAX_LINE 2048

#include <math.h>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <list>
#include <vector>
#include <map>

#include "../Geometry/Point.h"
#include "../Geometry/Point2.h"
#include "../Parser/StrUtil.h"

#include "Edge.h"
#include "Vertex.h"
#include "HalfEdge.h"
#include "Face.h"

namespace MeshLib {

	/*!
	* \brief CBaseMesh, base class for all types of mesh classes
	*
	*  This is the fundamental class for meshes. It includes a list of vertices,
	*  a list of edges, a list of faces. All the geometric objects are connected by pointers,
	*  vertex, edge, face are connected by halfedges. The mesh class has file IO functionalities,
	*  supporting .obj, .m and .off file formats. It offers Euler operators, each geometric primative
	*  can access its neighbors freely.
	*
	* \tparam CVertex   vertex   class, derived from MeshLib::CVertex   class
	* \tparam CEdge     edge     class, derived from MeshLib::CEdge     class
	* \tparam CFace     face     class, derived from MeshLib::CFace     class
	* \tparam CHalfEdge halfedge class, derived from MeshLib::CHalfEdge class
	*/
	template<typename CVertex = CVertex, typename CEdge = CEdge, typename CFace = CFace, typename CHalfEdge = CHalfEdge>
	class CBaseMesh
	{
	protected:
		int globalVid = 0;
		int globalFid = 0;


		// setter
		void setHalfedge(CEdge* e, int id, CHalfEdge* he) {
			e->halfedge(id) = he;
		}

		void setSourceVertex(CHalfEdge* he, CVertex* v) {
			he->source() = v;
		}

		void setVertex(CHalfEdge* he, CVertex* v) {
			he->vertex() = v;
		}

		void setHalfedge(CVertex* v, CHalfEdge* he) {
			v->halfedge() = he;
		}

		// make he & sym the sym hes, and make sym attached to edge of he
		void setHalfedge(CHalfEdge* he, CHalfEdge* sym) {
			he->he_sym() = sym;
			sym->he_sym() = he;
			sym->edge() = he->edge();
		}

		void setHalfedge(CFace* f, CHalfEdge* he) {
			f->halfedge() = he;
		}

		void setPrevHalfedge(CHalfEdge* he, CHalfEdge* prev) {
			he->he_prev() = prev;
			prev->he_next() = he;
		}

		void setNextHalfedge(CHalfEdge* he, CHalfEdge* next) {
			he->he_next() = next;
			next->he_prev() = he;
		}

		void setNextHalfedgeNull(CHalfEdge* he) {
			he->he_next() = NULL;
		}
		void setPrevHalfedgeNull(CHalfEdge* he) {
			he->he_prev() = NULL;
		}
		void setFace(CHalfEdge* he, CFace* f) {
			he->face() = f;
		}

		void setEdge(CHalfEdge* he, CEdge* e) {
			he->edge() = e;
		}
	public:
		// pointer to Vertices, Halfedges, Edges, Face and Solid
		typedef CVertex* tVertex;
		typedef CHalfEdge* tHalfEdge;
		typedef CEdge* tEdge;
		typedef CFace* tFace;

		std::string write_obj_dump() {
			std::stringstream _os;

			int vid = 1;
			for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
			{
				tVertex v = *viter;
				v->fakeId = vid++;
			}

			for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
			{
				tVertex v = *viter;

				_os << "v";

				for (int i = 0; i < 3; i++)
				{
					_os << " " << v->point()[i];
				}
				_os << std::endl;
			}

			for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
			{
				tVertex v = *viter;

				_os << "vt";

				for (int i = 0; i < 2; i++)
				{
					_os << " " << v->uv()[i];
				}
				_os << std::endl;
			}

			for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
			{
				tVertex v = *viter;

				_os << "vn";

				for (int i = 0; i < 3; i++)
				{
					_os << " " << v->normal()[i];
				}
				_os << std::endl;
			}


			for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
			{
				tFace f = *fiter;

				_os << "f";

				tHalfEdge he = faceHalfedge(f);

				do {
					int vid = he->target()->fakeId;
					_os << " " << vid << "/" << vid << "/" << vid;
					he = halfedgeNext(he);
				} while (he != f->halfedge());
				_os << std::endl;
			}
			return _os.str();
		};

		//constructor and destructor
		/*!
		CBaseMesh constructor.
		*/
		CBaseMesh() {
			m_with_normal = false;
			m_with_texture = false;
		};
		/*!
		CBasemesh destructor
		*/
		~CBaseMesh();

		//copy operator
		/*!
		Copy operator
		*/
		void copy(CBaseMesh& mesh);

		//file io
		/*!
		Read an .obj file.
		\param filename the input .obj file name
		*/
		void read_obj(const char* filename);
		/*!
		Write an .obj file.
		\param output the output .obj file name
		*/
		void write_obj(const char* output);

		/*!
		Read an .m file.
		\param input the input obj file name
		*/
		void read_m(const char* input);
		/*!
		Write an .m file.
		\param output the output .m file name
		*/
		void write_m(const char* output);

		/*!
		Read an .off file
		\param input the input .off filename
		*/
		void read_off(const char* input);
		/*!
		Write an .off file.
		\param output the output .off file name
		*/
		void write_off(const char* output);

		//number of vertices, faces, edges
		/*! number of vertices */
		int  numVertices();
		/*! number of edges */
		int  numEdges();
		/*! number of faces */
		int  numFaces();

		//is boundary
		/*! whether a vertex is on the boundary
		\param v the pointer to the vertex
		*/
		bool    isBoundary(tVertex  v);
		/*! whether an edge is on the boundary
		\param e the pointer to the edge
		*/
		bool    isBoundary(tEdge    e);
		/*! whether a halfedge is on the boundary
		\param he the pointer to the halfedge
		*/
		bool    isBoundary(tHalfEdge  he);

		//acess vertex - id
		/*!
		Access a vertex by its id
		\param id the vertex id
		\return the vertex, whose ID equals to id. NULL, if there is no such a vertex.
		*/
		tVertex idVertex(int id);
		/*!
		The vertex id
		\param v the input vertex
		\return the vertex id.
		*/
		int     vertexId(tVertex  v);

		//access face - id
		/*!
		Access a face by its id
		\param id the face id
		\return the face, whose ID equals to id. NULL, if there is no such a face.
		*/
		tFace   idFace(int id);
		/*!
		The face id
		\param f the input face
		\return the face id.
		*/
		int     faceId(tFace  f);

		//access edge - edge key, vertex
		/*!
		Access an edge by its two end vertices
		\param v0 one vertex of the edge
		\param v1 the other vertex of the edge
		\return the edge connecting both v0 and v1, NULL if no such edge exists.
		*/
		tEdge   vertexEdge(tVertex v0, tVertex v1);

		//access halfedge - halfedge key, vertex
		/*!
		Access a halfedge by its two end vertices
		\param v0 one vertex of the halfedge
		\param v1 the other vertex of the halfedge
		\return the halfedge connecting both v0 and v1, NULL if no such edge exists.
		*/

		tHalfEdge   vertexHalfedge(tVertex v0, tVertex v1);
		/*!
		Access a halfedge by its target vertex, and attaching face.
		\param v target vertex
		\param f attaching face
		\return halfedge, whose target is v, attaching face is f. NULL if no such an halfedge exists.
		*/
		tHalfEdge   corner(tVertex v, tFace f);

		//halfedge->face
		/*!
		The face a halfedge attaching to.
		\param he the input halfedge
		\return the face he attaches
		*/
		tFace   halfedgeFace(tHalfEdge he);
		//halfedge->vertex
		/*!
		The target vertex of a halfedge.
		\param he the input halfedge.
		\return the target vertex of he.
		*/
		tVertex halfedgeVertex(tHalfEdge he);
		//halfedge->vertex
		/*!
		The target vertex of a halfedge.
		\param he the input halfedge.
		\return the target vertex of he.
		*/
		tVertex halfedgeTarget(tHalfEdge he);
		//halfedge->vertex
		/*!
		The source vertex of a halfedge.
		\param he the input halfedge.
		\return the source vertex of he.
		*/
		tVertex halfedgeSource(tHalfEdge he);

		//halfedge->next
		/*!
		The next halfedge of a halfedge.
		\param he the input halfedge.
		\return the next halfedge of he.
		*/

		tHalfEdge   halfedgeNext(tHalfEdge he);
		//halfedge->prev
		/*!
		The previous halfedge of a halfedge.
		\param he the input halfedge.
		\return the next halfedge of he.
		*/
		tHalfEdge   halfedgePrev(tHalfEdge he);
		//halfedge->sym
		/*!
		The dual halfedge of a halfedge.
		\param he the input halfedge.
		\return the dual halfedge of he.
		*/
		tHalfEdge   halfedgeSym(tHalfEdge he);
		//halfedge->edge
		/*!
		The edge of a halfedge.
		\param he the input halfedge.
		\return the edge of he.
		*/

		tEdge       halfedgeEdge(tHalfEdge he);
		//v->halfedge
		/*!
		The halfedge targeting at a vertex.
		\param v the input vertex.
		\return the halfedge targeting at v, which is the most ccw in halfedge of v.
		*/
		tHalfEdge   vertexHalfedge(tVertex v);
		//v->edges
		/*!
		The edge list attaching to the vertex v, such that v is the first vertex of the edge
		\param v the input vertex.
		\return the reference to the edge list
		*/
		std::list<tEdge>& vertexEdges(tVertex v);

		//edge->vertex
		/*!
		The first vertex of an edge.
		\param e the input edge.
		\return the first vertex of e.
		*/
		tVertex edgeVertex1(tEdge  e);
		/*!
		The second vertex of an edge.
		\param e the input edge.
		\return the second vertex of e.
		*/
		tVertex edgeVertex2(tEdge  e);

		//edge->face
		/*!
		The first face attaching to an edge.
		\param e the input edge.
		\return the first face attaching to e.
		*/
		tFace edgeFace1(tEdge  e);
		/*!
		The second face attaching to an edge.
		\param e the input edge.
		\return the second face attaching to e.
		*/
		tFace edgeFace2(tEdge  e);

		//edge->halfedge
		/*!
		The halfedge attaching to an edge.
		\param e the input edge.
		\param id the index of the halfedge, either 0 or 1
		\return the halfedge[i] attaching to edge e.
		*/

		tHalfEdge edgeHalfedge(tEdge  e, int id);

		//face->halfedge
		/*!
		The first halfedge attaching to a face f.
		\param f the input face.
		\return the first halfedge attaching to f.
		*/

		tHalfEdge faceHalfedge(tFace f);

		////Euler operations
		///*!
		//The most Clw Out HalfEdge of a vertex
		//\param v the input vertex.
		//\return the most Clw Out HalfEdge of v.
		//*/
		//tHalfEdge vertexMostClwOutHalfEdge(tVertex  v);
		///*!
		//The next Ccw Out HalfEdge
		//\param he the input halfedge .
		//\return the next Ccw Out HalfEdge, sharing the same source of he.
		//*/

		tHalfEdge vertexNextCcwOutHalfEdge(tHalfEdge  he);

		///*!
		//The most Ccw Out HalfEdge of a vertex
		//\param v the input vertex.
		//\return the most Ccw Out HalfEdge of v.
		//*/
		//tHalfEdge vertexMostCcwOutHalfEdge(tVertex  v);
		///*!
		//The next Clw Out HalfEdge
		//\param he the input halfedge .
		//\return the next Clw Out HalfEdge, sharing the same source of he.
		//*/
		tHalfEdge vertexNextClwOutHalfEdge(tHalfEdge  he);

		///*!
		//The most Clw In HalfEdge of a vertex
		//\param v the input vertex.
		//\return the most Clw In HalfEdge of v.
		//*/
		//tHalfEdge vertexMostClwInHalfEdge(tVertex  v);
		///*!
		//The next Ccw In HalfEdge
		//\param he the input halfedge .
		//\return the next Ccw In HalfEdge, sharing the same target of he.
		//*/
		tHalfEdge vertexNextCcwInHalfEdge(tHalfEdge  he);

		///*!
		//The most Ccw In HalfEdge of a vertex
		//\param v the input vertex.
		//\return the most Ccw In HalfEdge of v.
		//*/
		//tHalfEdge vertexMostCcwInHalfEdge(tVertex  v);
		///*!
		//The next Clw In HalfEdge
		//\param he the input halfedge .
		//\return the next Clw In HalfEdge, sharing the same target of he.
		//*/
		tHalfEdge vertexNextClwInHalfEdge(tHalfEdge  he);

		/*!
		The next Ccw HalfEdge of a halfedge in a face
		\param he the input halfedge.
		\return the next Ccw HalfEdge of he in a face.
		*/
		tHalfEdge faceNextCcwHalfEdge(tHalfEdge  he);
		/*!
		The next Clw HalfEdge of a halfedge in a face
		\param he the input halfedge.
		\return the next Clw HalfEdge of he in a face.
		*/
		tHalfEdge faceNextClwHalfEdge(tHalfEdge  he);


		/*!
		Edge length
		\param e the input edge
		\return the length of the edge e
		*/
		double edgeLength(tEdge e);

		/*!
		List of the edges of the mesh.
		*/
		std::list<tEdge>& edges() { return m_edges; };
		/*!
		List of the faces of the mesh.
		*/
		std::list<tFace>& faces() { return m_faces; };
		/*!
		List of the vertices of the mesh.
		*/
		std::list<tVertex>& vertices() { return m_verts; };
		/*
			bool with_uv() { return m_with_texture; };
			bool with_normal() { return m_with_normal; };
		*/
	protected:

		/*! list of edges */
		std::list<tEdge>                          m_edges;
		/*! list of vertices */
		std::list<tVertex>                        m_verts;
		/*! list of faces */
		std::list<tFace>							m_faces;

		//maps

		/*! map between vetex and its id*/
		std::map<int, tVertex>                    m_map_vert;
		/*! map between face and its id*/
		std::map<int, tFace>						m_map_face;
		/*! Create a vertex
		\param id Vertex id
		\return pointer to the new vertex
		*/
		tVertex   createVertex(int id = -1, CPoint pos = CPoint(0,0,0)) {
			globalVid = id < globalVid ? globalVid + 1 : id + 1;
			if (id == -1) {
				id = globalVid;
			}
			CVertex* v = new CVertex();
			assert(v != NULL);
			v->setId(id);
			v->point() = pos;
			if (m_map_vert.find(id) != m_map_vert.end()) {
				cerr << "Duplicate id!" << endl;
				exit(-1);
			}
			m_verts.push_back(v);
			m_map_vert.insert(std::pair<int, CVertex*>(v->getId(), v));
			return v;
		}

		tVertex createVertex(CPoint pos = CPoint(0, 0, 0)) {
			return createVertex(-1, pos);
		}
		/*! Create an edge
		\param v1 end vertex of the edge
		\param v2 end vertex of the edge
		\return pointer to the new edge
		*/
		tEdge     createEdge(tVertex v1, tVertex v2);
		/*! Create a face
		\param v an array of vertices
		\param id face id
		\return pointer to the new face
		*/
		tFace     createFace(tVertex  v[], int size = 3, int id = -1); //create a triangle
		/*! Create a face
		\param v an array of vertices
		\param id face id
		\return pointer to the new face
		*/
		tFace     createFace(std::vector<tVertex>& v, int id = -1); //create a triangle

		tFace createFace(tVertex v1, tVertex v2, tVertex v3) {
			tVertex v[3]{ v1, v2, v3 };
			return createFace(v, 3);
		}

		tFace createFace(tVertex v1, tVertex v2, tVertex v3, tVertex v4) {
			tVertex v[4] = { v1, v2, v3, v4 };
			return createFace(v, 4);
		}

		/*! delete one face
		\param pFace the face to be deleted
		*/
		void      deleteFace(tFace  pFace);
		void deleteEdge(tEdge edge);
		void deleteVertex(tVertex pVertex) {
			if (pVertex->halfedge()||pVertex->edges().size()>0) {
				cerr << "Cannot delete vertex with edges, must delete connect edges and halfedges first" << endl;
				assert(false);
			}
			m_verts.remove(pVertex);
			m_map_vert.erase(vertexId(pVertex));
			delete pVertex;
		}
		/*! whether the vertex is with texture coordinates */
		bool      m_with_texture;
		/*! whether the mesh is with normal */
		bool      m_with_normal;

		/*! label boundary vertices, edges, faces */
		void labelBoundary(void);

	};
}

#include "iterators.h"
using namespace MeshLib;

/*!
The first vertex of an edge.
\param e the input edge.
\return the first vertex of e.
*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CVertex* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::edgeVertex1(tEdge   e)
{
	assert(e->halfedge(0) != NULL);
	return (CVertex*)e->halfedge(0)->source();
};

/*!
The second vertex of an edge.
\param e the input edge.
\return the first vertex of e.
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CVertex* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::edgeVertex2(tEdge   e)
{
	assert(e->halfedge(0) != NULL);
	return (CVertex*)e->halfedge(0)->target();
};

/*!
The first face attaching to an edge.
\param e the input edge.
\return the first face attaching to e.
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CFace* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::edgeFace1(tEdge   e)
{
	assert(e->halfedge(0) != NULL);
	return (CFace*)e->halfedge(0)->face();
};

//access he->f
/*!
The halfedge attaching to an edge.
\param e the input edge.
\param id the index of the halfedge, either 0 or 1
\return the halfedge[i] attaching to edge e.
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::edgeHalfedge(tEdge   e, int id)
{
	return (CHalfEdge*)e->halfedge(id);
};

//access e->f
/*!
The second face attaching to an edge.
\param e the input edge.
\return the first face attaching to e.
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CFace* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::edgeFace2(tEdge   e)
{
	assert(e->halfedge(1) != NULL);
	return (CFace*)e->halfedge(1)->face();
};

//access he->f
/*!
	The face a halfedge attaching to.
	\param he the input halfedge
	\return the face he attaches
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CFace* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgeFace(tHalfEdge   he)
{
	return (CFace*)he->face();
};

//access f->he
	/*!
	The first halfedge attaching to a face f.
	\param f the input face.
	\return the first halfedge attaching to f.
	*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::faceHalfedge(tFace   f)
{
	return (CHalfEdge*)f->halfedge();
};


//access he->next
/*!
	The next halfedge of a halfedge.
	\param he the input halfedge.
	\return the next halfedge of he.
	*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgeNext(tHalfEdge   he)
{
	return (CHalfEdge*)he->he_next();
};

//access he->prev
/*!
	The previous halfedge of a halfedge.
	\param he the input halfedge.
	\return the next halfedge of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgePrev(tHalfEdge  he)
{
	return (CHalfEdge*)he->he_prev();
};

//access he->sym
/*!
	The dual halfedge of a halfedge.
	\param he the input halfedge.
	\return the dual halfedge of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgeSym(tHalfEdge   he)
{
	return (CHalfEdge*)he->he_sym();
};

//access he->edge
/*!
	The edge of a halfedge.
	\param he the input halfedge.
	\return the edge of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgeEdge(tHalfEdge  he)
{
	return (CEdge*)he->edge();
};

//access he->v
/*!
	The target vertex of a halfedge.
	\param he the input halfedge.
	\return the target vertex of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CVertex* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgeVertex(tHalfEdge  he)
{
	return (CVertex*)he->vertex();
};

//access he->v
/*!
	The target vertex of a halfedge.
	\param he the input halfedge.
	\return the target vertex of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CVertex* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgeTarget(tHalfEdge   he)
{
	return (CVertex*)he->vertex();
};

//access he->v
/*!
	The source vertex of a halfedge.
	\param he the input halfedge.
	\return the source vertex of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CVertex* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::halfedgeSource(tHalfEdge   he)
{
	return (CVertex*)he->he_sym()->vertex();
};
/*! whether a vertex is on the boundary
	\param v the pointer to the vertex
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline bool CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::isBoundary(tVertex   v)
{
	tHalfEdge he = vertexHalfedge(v);
	tHalfEdge he1 = he;
	do
	{
		if (halfedgeFace(he1) == NULL)
			return true;
		he1 = halfedgeSym(halfedgeNext(he1));
	} while (he != he1);
	return false;
};
/*! whether an edge is on the boundary
	\param e the pointer to the edge
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline bool CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::isBoundary(tEdge   e)
{
	if (halfedgeFace(edgeHalfedge(e, 0)) == NULL
		|| halfedgeFace(edgeHalfedge(e, 1)) == NULL)
		return true;
	return false;
};
/*! whether a halfedge is on the boundary
	\param he the pointer to the halfedge
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline bool CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::isBoundary(tHalfEdge   he)
{
	if (halfedgeFace(he) == NULL) return true;
	return false;
};

/*! Number of vertices of the mesh
*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline int CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::numVertices()
{
	return (int)m_verts.size();
};

/*! Number of edges of the mesh
*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline int CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::numEdges()
{
	return (int)m_edges.size();
};

/*! Number of faces of the mesh
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline int CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::numFaces()
{
	return (int)m_faces.size();
};

/*!
	Access a halfedge by its target vertex, and attaching face.
	\param v target vertex
	\param f attaching face
	\return halfedge, whose target is v, attaching face is f. NULL if no such an halfedge exists.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::corner(tVertex  v, tFace  f)
{
	CHalfEdge* he = faceHalfedge(f);
	do {
		if (he->vertex() == v)
			return (CHalfEdge*)he;
		he = faceNextCcwHalfEdge(he);
	} while (he != faceHalfedge(f));
	return NULL;
};
/*!
	The next Ccw Out HalfEdge
	\param he the input halfedge .
	\return the next Ccw Out HalfEdge, sharing the same source of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexNextCcwOutHalfEdge(tHalfEdge  he)
{
	return (CHalfEdge*)he->ccw_rotate_about_source();
};
/*!
	The next Clw Out HalfEdge
	\param he the input halfedge .
	\return the next Clw Out HalfEdge, sharing the same source of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexNextClwOutHalfEdge(tHalfEdge   he)
{
	assert(he->he_sym() != NULL);
	return (CHalfEdge*)he->clw_rotate_about_source();
};

/*!
The next Ccw In HalfEdge
\param he the input halfedge .
\return the next Ccw In HalfEdge, sharing the same target of he.
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexNextCcwInHalfEdge(tHalfEdge   he)
{
	assert(he->he_sym() != NULL);
	return (CHalfEdge*)he->ccw_rotate_about_target();
};
/*!
	The next Clw In HalfEdge
	\param he the input halfedge .
	\return the next Clw In HalfEdge, sharing the same target of he.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexNextClwInHalfEdge(tHalfEdge   he)
{
	return (CHalfEdge*)he->clw_rotate_about_target();
};
/*!
	The next Clw HalfEdge of a halfedge in a face
	\param he the input halfedge.
	\return the next Clw HalfEdge of he in a face.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::faceNextClwHalfEdge(tHalfEdge   he)
{
	return (CHalfEdge*)he->he_prev();
};
/*!
	The next Ccw HalfEdge of a halfedge in a face
	\param he the input halfedge.
	\return the next Ccw HalfEdge of he in a face.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::faceNextCcwHalfEdge(tHalfEdge   he)
{
	return (CHalfEdge*)he->he_next();
};
/*!
 CBaseMesh destructor
 */
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::~CBaseMesh()
{
	//remove vertices

	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		CVertex* pV = *viter;
		delete pV;
	}
	m_verts.clear();

	//remove faces

	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		CFace* pF = *fiter;

		tHalfEdge he = faceHalfedge(pF);

		std::list<CHalfEdge*> hes;
		do {
			he = halfedgeNext(he);
			hes.push_back(he);
		} while (he != pF->halfedge());

		for (std::list<CHalfEdge*>::iterator hiter = hes.begin(); hiter != hes.end(); hiter++)
		{
			CHalfEdge* pH = *hiter;
			delete pH;
		}
		hes.clear();

		delete pF;
	}
	m_faces.clear();

	//remove edges
	for (std::list<CEdge*>::iterator eiter = m_edges.begin(); eiter != m_edges.end(); eiter++)
	{
		CEdge* pE = *eiter;
		delete pE;
	}

	m_edges.clear();

	//clear all the maps
	m_map_vert.clear();
	m_map_face.clear();
	//m_map_edge.clear();
};

/*!
	Edge length
*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
double CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::edgeLength(tEdge  e)
{
	CVertex* v1 = edgeVertex1(e);
	CVertex* v2 = edgeVertex2(e);

	return (v1->point() - v2->point()).norm();
}

/*!
Read an .obj file.
\param filename the filename .obj file name
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::read_obj(const char* filename)
{

	std::fstream f(filename, std::fstream::in);
	if (f.fail()) return;

	char cmd[1024];

	int  vid = 1;
	int  fid = 1;

	bool with_uv = false;
	bool with_normal = false;

	std::vector<CPoint2> uvs;
	std::vector<CPoint> normals;


	while (f.getline(cmd, 1024))
	{
		std::string line(cmd);
		line = strutil::trim(line);

		strutil::Tokenizer stokenizer(line, " \t\r\n");

		stokenizer.nextToken();
		std::string token = stokenizer.getToken();

		if (token == "v")
		{
			CPoint p;
			for (int i = 0; i < 3; i++)
			{
				stokenizer.nextToken();
				token = stokenizer.getToken();
				p[i] = strutil::parseString<float>(token);
			}

			CVertex* v = createVertex(vid++, p);
			continue;
		}


		if (token == "vt")
		{
			with_uv = true;
			CPoint2 uv;
			for (int i = 0; i < 2; i++)
			{
				stokenizer.nextToken();
				token = stokenizer.getToken();
				uv[i] = strutil::parseString<float>(token);
			}
			uvs.push_back(uv);
			continue;
		}


		if (token == "vn")
		{
			with_normal = true;

			CPoint n;
			for (int i = 0; i < 3; i++)
			{
				stokenizer.nextToken();
				token = stokenizer.getToken();
				n[i] = strutil::parseString<float>(token);
			}
			normals.push_back(n);
			continue;
		}




		if (token == "f")
		{
			CVertex* v[3];
			for (int i = 0; i < 3; i++)
			{
				stokenizer.nextToken();
				token = stokenizer.getToken();


				strutil::Tokenizer tokenizer(token, " /\t\r\n");

				int ids[3];
				int k = 0;
				while (tokenizer.nextToken())
				{
					std::string token = tokenizer.getToken();
					ids[k] = strutil::parseString<int>(token);
					k++;
				}


				v[i] = m_map_vert[ids[0]];
				if (with_uv)
					v[i]->uv() = uvs[ids[1] - 1];
				if (with_normal)
					v[i]->normal() = normals[ids[2] - 1];
			}
			createFace(v);
		}
	}

	f.close();

	labelBoundary();
}

//access id->v
/*!
Access a vertex by its id
\param id the vertex id
\return the vertex, whose ID equals to id. NULL, if there is no such a vertex.
*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CVertex* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::idVertex(int id)
{
	return m_map_vert[id];
};

//access v->id
/*!
	The vertex id
	\param v the input vertex
	\return the vertex id.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline int CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexId(tVertex   v)
{
	return v->getId();
};

//access id->f
/*!
	Access a face by its id
	\param id the face id
	\return the face, whose ID equals to id. NULL, if there is no such a face.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CFace* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::idFace(int id)
{
	return m_map_face[id];
};

//acess f->id
/*!
	The face id
	\param f the input face
	\return the face id.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline int CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::faceId(tFace   f)
{
	return f->id();
};

/*! Create an edge
\param v1 end vertex of the edge
\param v2 end vertex of the edge
\return pointer to the new edge
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::createEdge(tVertex  v1, tVertex  v2)
{
	tVertex pV = (v1->getId() < v2->getId()) ? v1 : v2;
	std::list<CEdge*>& ledges = (std::list<CEdge*> &) pV->edges();
	if (vertexEdge(v1, v2)) {
		return vertexEdge(v1, v2);
	}
	// create a new edge
	CEdge* e = new CEdge;
	CHalfEdge* he1 = new CHalfEdge;
	CHalfEdge* he2 = new CHalfEdge;
	setVertex(he1, v2);
	setVertex(he2, v1);
	//setSourceVertex(he1, v1);
	//setSourceVertex(he2, v2);
	setHalfedge(v2, he1);
	setHalfedge(v1, he2);
	setHalfedge(e, 0, he1);
	setHalfedge(e, 1, he2);
	setEdge(he1, e);
	setEdge(he2, e);

	m_edges.push_back(e);
	ledges.push_back(e);
	return e;
};



//access vertex->edge
/*!
Access an edge by its two end vertices
\param v0 one vertex of the edge
\param v1 the other vertex of the edge
\return the edge connecting both v0 and v1, NULL if no such edge exists.
*/
//use the edge list associated with each vertex to locate the edge

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexEdge(tVertex  v0, tVertex  v1)
{
	CVertex* pV = (v0->getId() < v1->getId()) ? v0 : v1;
	std::list<CEdge*>& ledges = vertexEdges(pV);

	for (std::list<CEdge*>::iterator eiter = ledges.begin(); eiter != ledges.end(); eiter++)
	{
		CEdge* pE = *eiter;
		CHalfEdge* pH = edgeHalfedge(pE, 0);
		if (pH->source() == v0 && pH->target() == v1) return pE;
		if (pH->source() == v1 && pH->target() == v0) return pE;
	}
	return NULL;
};

/*!
Access a halfedge by its two end vertices
\param v0 one vertex of the halfedge
\param v1 the other vertex of the halfedge
\return the halfedge connecting both v0 and v1, NULL if no such edge exists.
*/

//access vertex->halfedge
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexHalfedge(tVertex  v0, tVertex  v1)
{
	CEdge* e = vertexEdge(v0, v1);
	if (e == NULL) {
		return NULL;
	}
	CHalfEdge* he = (CHalfEdge*)e->halfedge(0);
	if (he->vertex() == v1) return he;
	he = (CHalfEdge*)e->halfedge(1);
	assert(he->vertex() == v1);
	return he;
};


/*!
Access the edge list of a vertex, {e} such that e->vertex1() == v
\param v vertex
\return the list of adjacent edges
*/

//access vertex->edges
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline std::list<CEdge*>& CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexEdges(tVertex  v0)
{
	return (std::list<CEdge*> &)v0->edges();
};

//access vertex->halfedge
/*!
	The halfedge targeting at a vertex.
	\param v the input vertex.
	\return the halfedge targeting at v, which is the most ccw in halfedge of v.
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
inline CHalfEdge* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::vertexHalfedge(tVertex  v)
{
	return (CHalfEdge*)v->halfedge();
};

/*!
	Read an .m file.
	\param input the input obj file name
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::read_m(const char* input)
{
	std::fstream is(input, std::fstream::in);

	if (is.fail())
	{
		fprintf(stderr, "Error in opening file %s\n", input);
		return;
	}

	char buffer[MAX_LINE];
	int id;

	while (is.getline(buffer, MAX_LINE))
	{

		std::string line(buffer);
		line = strutil::trim(line);

		strutil::Tokenizer stokenizer(line, " \r\n");

		stokenizer.nextToken();
		std::string token = stokenizer.getToken();

		if (token == "Vertex")
		{
			stokenizer.nextToken();
			token = stokenizer.getToken();
			id = strutil::parseString<int>(token);

			CPoint p;
			for (int i = 0; i < 3; i++)
			{
				stokenizer.nextToken();
				token = stokenizer.getToken();
				p[i] = strutil::parseString<float>(token);
			}

			tVertex v = createVertex(id, p);
			if (!stokenizer.nextToken("\t\r\n")) continue;
			token = stokenizer.getToken();

			int sp = (int)token.find("{");
			int ep = (int)token.find("}");

			if (sp >= 0 && ep >= 0)
			{
				v->string() = token.substr(sp + 1, ep - sp - 1);
			}
			continue;
		}


		if (token == "Face")
		{
			stokenizer.nextToken();
			token = stokenizer.getToken();
			id = strutil::parseString<int>(token);

			std::vector<CVertex*> v;
			while (stokenizer.nextToken())
			{
				token = stokenizer.getToken();
				if (strutil::startsWith(token, "{")) break;
				int vid = strutil::parseString<int>(token);
				v.push_back(idVertex(vid));
			}

			tFace f = createFace(v, id);

			if (strutil::startsWith(token, "{"))
			{
				f->string() = strutil::trim(token, "{}");
			}
			continue;
		}

		//read in edge attributes
		if (token == "Edge")
		{
			stokenizer.nextToken();
			token = stokenizer.getToken();
			int id0 = strutil::parseString<int>(token);

			stokenizer.nextToken();
			token = stokenizer.getToken();
			int id1 = strutil::parseString<int>(token);


			CVertex* v0 = idVertex(id0);
			CVertex* v1 = idVertex(id1);

			tEdge edge = vertexEdge(v0, v1);

			if (!stokenizer.nextToken("\t\r\n")) continue;
			token = stokenizer.getToken();

			int sp = (int)token.find("{");
			int ep = (int)token.find("}");

			if (sp >= 0 && ep >= 0)
			{
				edge->string() = token.substr(sp + 1, ep - sp - 1);
			}
			continue;
		}

		//read in edge attributes
		if (token == "Corner")
		{
			stokenizer.nextToken();
			token = stokenizer.getToken();
			int vid = strutil::parseString<int>(token);

			stokenizer.nextToken();
			token = stokenizer.getToken();
			int fid = strutil::parseString<int>(token);


			CVertex* v = idVertex(vid);
			CFace* f = idFace(fid);
			tHalfEdge he = corner(v, f);


			if (!stokenizer.nextToken("\t\r\n")) continue;
			token = stokenizer.getToken();

			int sp = (int)token.find("{");
			int ep = (int)token.find("}");

			if (sp >= 0 && ep >= 0)
			{
				he->string() = token.substr(sp + 1, ep - sp - 1);
			}
			continue;
		}
	}

	labelBoundary();
	
	//read in the traits

	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); ++viter)
	{
		CVertex* v = *viter;
		v->_from_string();
	}

	for (std::list<CEdge*>::iterator eiter = m_edges.begin(); eiter != m_edges.end(); ++eiter)
	{
		CEdge* e = *eiter;
		e->_from_string();
	}

	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); ++fiter)
	{
		CFace* f = *fiter;
		f->_from_string();
	}

	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		CFace* pF = *fiter;

		CHalfEdge* pH = faceHalfedge(pF);
		do {
			pH->_from_string();
			pH = faceNextCcwHalfEdge(pH);
		} while (pH != faceHalfedge(pF));
	}

};

/*!
	Write an .m file.
	\param output the output .m file name
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::write_m(const char* output)
{
	//write traits to string
	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		CVertex* pV = *viter;
		pV->_to_string();
	}

	for (std::list<CEdge*>::iterator eiter = m_edges.begin(); eiter != m_edges.end(); eiter++)
	{
		CEdge* pE = *eiter;
		pE->_to_string();
	}

	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		CFace* pF = *fiter;
		pF->_to_string();
	}

	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		CFace* pF = *fiter;
		CHalfEdge* pH = faceHalfedge(pF);
		do {
			pH->_to_string();
			pH = faceNextCcwHalfEdge(pH);
		} while (pH != faceHalfedge(pF));
	}


	std::fstream _os(output, std::fstream::out);
	if (_os.fail())
	{
		fprintf(stderr, "Error is opening file %s\n", output);
		return;
	}


	//remove vertices
	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		tVertex v = *viter;

		_os << "Vertex " << v->getId();

		for (int i = 0; i < 3; i++)
		{
			_os << " " << v->point()[i];
		}
		if (v->string().size() > 0)
		{
			_os << " " << "{" << v->string() << "}";
		}
		_os << std::endl;
	}

	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		tFace f = *fiter;

		_os << "Face " << f->id();
		tHalfEdge he = faceHalfedge(f);
		do {
			_os << " " << he->target()->getId();
			he = halfedgeNext(he);
		} while (he != f->halfedge());

		if (f->string().size() > 0)
		{
			_os << " " << "{" << f->string() << "}";
		}
		_os << std::endl;
	}

	for (std::list<CEdge*>::iterator eiter = m_edges.begin(); eiter != m_edges.end(); eiter++)
	{
		tEdge e = *eiter;
		if (e->string().size() > 0)
		{
			_os << "Edge " << edgeVertex1(e)->getId() << " " << edgeVertex2(e)->getId() << " ";
			_os << "{" << e->string() << "}" << std::endl;
		}
	}

	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		tFace f = *fiter;

		tHalfEdge he = faceHalfedge(f);

		do {
			if (he->string().size() > 0)
			{
				_os << "Corner " << he->vertex()->getId() << " " << f->id() << " ";
				_os << "{" << he->string() << "}" << std::endl;
			}
			he = halfedgeNext(he);
		} while (he != f->halfedge());

	}

	_os.close();
};


//assume the mesh is with uv coordinates and normal vector for each vertex
/*!
	Write an .obj file.
	\param output the output .obj file name
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::write_obj(const char* output)
{
	std::fstream _os(output, std::fstream::out);
	if (_os.fail())
	{
		fprintf(stderr, "Error is opening file %s\n", output);
		return;
	}

	int vid = 1;
	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		tVertex v = *viter;
		v->fakeId = vid++;
	}

	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		tVertex v = *viter;

		_os << "v";

		for (int i = 0; i < 3; i++)
		{
			_os << " " << v->point()[i];
		}
		_os << std::endl;
	}

	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		tVertex v = *viter;

		_os << "vt";

		for (int i = 0; i < 2; i++)
		{
			_os << " " << v->uv()[i];
		}
		_os << std::endl;
	}

	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		tVertex v = *viter;

		_os << "vn";

		for (int i = 0; i < 3; i++)
		{
			_os << " " << v->normal()[i];
		}
		_os << std::endl;
	}


	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		tFace f = *fiter;

		_os << "f";

		tHalfEdge he = faceHalfedge(f);

		do {
			int vid = he->target()->fakeId;
			_os << " " << vid << "/" << vid << "/" << vid;
			he = halfedgeNext(he);
		} while (he != f->halfedge());
		_os << std::endl;
	}

	_os.close();
}

/*!
	Write an .off file.
	\param output the output .off file name
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::write_off(const char* output)
{
	std::fstream _os(output, std::fstream::out);
	if (_os.fail())
	{
		fprintf(stderr, "Error is opening file %s\n", output);
		return;
	}

	_os << "OFF" << std::endl;
	_os << m_verts.size() << " " << m_faces.size() << " " << m_edges.size() << std::endl;


	int vid = 0;
	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		tVertex v = *viter;
		v->fakeId = vid++;
	}

	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		tVertex v = *viter;
		_os << v->point()[0] << " " << v->point()[1] << " " << v->point()[2] << std::endl;
		//_os << v->normal()[0] << " " << v->normal()[1]<< " " << v->normal()[2]<< std::endl;
	}


	for (std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		tFace f = *fiter;

		_os << "3";

		tHalfEdge he = faceHalfedge(f);

		do {
			int vid = he->target()->fakeId;
			_os << " " << vid;
			he = halfedgeNext(he);
		} while (he != f->halfedge());
		_os << std::endl;
	}

	_os.close();
};


//template pointer converting to base class pointer is OK (BasePointer) = (TemplatePointer)
//(TemplatePointer)=(BasePointer) is incorrect
/*! delete one face
\param pFace the face to be deleted
*/

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::deleteFace(tFace  pFace)
{
	assert(pFace);
	std::map<int, tFace>::iterator fiter = m_map_face.find(pFace->id());
	m_faces.remove(pFace);
	m_map_face.erase(fiter);
	CHalfEdge* he = faceHalfedge(pFace);
	CHalfEdge* he1 = he;
	do {
		he1->face() = NULL;
		he1 = halfedgeNext(he1);
	} while (he1 != he);

	delete pFace;
};

/*!
	Read an .off file
	\param input the input .off filename
	*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::read_off(const char* input)
{
	std::fstream is(input, std::fstream::in);

	if (is.fail())
	{
		fprintf(stderr, "Error is opening file %s\n", input);
		return;
	}

	char buffer[MAX_LINE];

	//read in the first line "OFF"

	while (is.getline(buffer, MAX_LINE))
	{
		std::string line(buffer);

		strutil::Tokenizer stokenizer(line, " \r\n");

		stokenizer.nextToken();
		std::string token = stokenizer.getToken();
		if (token == "OFF" || token == "NOFF") break;
	}

	int nVertices, nFaces, nEdges;

	//read in Vertex Number, Face Number, Edge Number

	is.getline(buffer, MAX_LINE);
	std::string line(buffer);

	strutil::Tokenizer stokenizer(line, " \r\n");

	stokenizer.nextToken();
	std::string token = stokenizer.getToken();
	nVertices = strutil::parseString<int>(token);

	stokenizer.nextToken();
	token = stokenizer.getToken();
	nFaces = strutil::parseString<int>(token);

	stokenizer.nextToken();
	token = stokenizer.getToken();
	nEdges = strutil::parseString<int>(token);

	//printf("V %d F %d E %d\n" , nVertices, nFaces, nEdges);


	for (int id = 0; id < nVertices; id++)
	{
		is.getline(buffer, MAX_LINE);
		std::string line(buffer);

		strutil::Tokenizer stokenizer(line, " \r\n");
		CPoint p;
		for (int j = 0; j < 3; j++)
		{
			stokenizer.nextToken();
			std::string token = stokenizer.getToken();
			p[j] = strutil::parseString<float>(token);
		}

		CVertex* v = createVertex(id+1, p);
	}


	for (int id = 0; id < nFaces; id++)
	{

		is.getline(buffer, MAX_LINE);
		std::string line(buffer);

		strutil::Tokenizer stokenizer(line, " \r\n");
		stokenizer.nextToken();
		std::string token = stokenizer.getToken();

		int n = strutil::parseString<int>(token);
		assert(n == 3);

		CVertex* v[3];
		for (int j = 0; j < 3; j++)
		{
			stokenizer.nextToken();
			std::string token = stokenizer.getToken();
			int vid = strutil::parseString<int>(token);
			v[j] = idVertex(vid + 1);
		}

		createFace(v);

	}

	is.close();

	labelBoundary();

};


/*!
	Label boundary edges, vertices
*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::labelBoundary(void)
{
	for (auto edge : m_edges)
	{
		if (halfedgeFace(edgeHalfedge(edge, 0)) == NULL && halfedgeFace(edgeHalfedge(edge, 1))==NULL) {
			cerr << "WARNING : Singular edge detected" << endl;
		}
		
		CHalfEdge* pH = edgeHalfedge(edge, 0);
		
		while (halfedgeNext(pH) == NULL) {
			CHalfEdge* he = halfedgeSym(pH);
			do {
				he = halfedgeSym(halfedgePrev(he));
			} while (halfedgeFace(he) != NULL);
			setNextHalfedge(pH, he);
			pH = he;
		}

		pH = edgeHalfedge(edge, 1);
		
		while (halfedgeNext(pH) == NULL) {
			CHalfEdge* he = halfedgeSym(pH);
			do {
				he = halfedgeSym(halfedgePrev(he));
			} while (halfedgeFace(he) != NULL);
			setNextHalfedge(pH, he);
			pH = he;
		}
	}

	std::list<CVertex*> dangling_verts;
	//Label boundary edges
	for (std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); ++viter)
	{
		tVertex     v = *viter;
		if (v->halfedge() != NULL) continue;
		dangling_verts.push_back(v);
	}

	for (std::list<CVertex*>::iterator viter = dangling_verts.begin(); viter != dangling_verts.end(); ++viter)
	{
		tVertex v = *viter;
		m_verts.remove(v);
		delete v;
		v = NULL;
	}

	// TODO remove deformed edges
};

/*! Create a face
	\param v an array of vertices
	\param id face id
	\return pointer to the new face
*/
template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CFace* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::createFace(tVertex  vs[], int size, int id)
{
	std::vector<CVertex*> vertices(vs, vs + size);
	return createFace(vertices, id);
};

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
CFace* CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::createFace(std::vector<tVertex>& v, int id)
{
	globalFid = id < globalFid ? globalFid +1: id + 1;
	if (id == -1) {
		id = globalFid;
	}
	CFace* f = new CFace();
	assert(f != NULL);
	f->id() = id;
	m_faces.push_back(f);
	m_map_face.insert(std::pair<int, tFace>(f->id(), f));
	//create halfedges
	std::vector<tHalfEdge> hes;

	for (size_t i = 0; i < v.size(); i++) {
		createEdge(v[i], v[(i + 1) % v.size()]);
		tHalfEdge pH = vertexHalfedge(v[i], v[(i + 1) % v.size()]);
		assert(halfedgeFace(pH) == NULL);
		setFace(pH, f);
		setHalfedge(f, pH);
		//setHalfedge(v[i], pH);
		hes.push_back(pH);
	}

	//linking to each other
	for (size_t i = 0; i < hes.size(); i++)
	{
		hes[i]->he_next() = hes[(i + 1) % hes.size()];
		hes[i]->he_prev() = hes[(i + hes.size() - 1) % hes.size()];
	}

	return f;
};

// <--.<---
//   ^|
//   ||
// oldEdge
//   ||
//   |v
// -->.--->

template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
void CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>::deleteEdge(tEdge edge) {
	if (halfedgeNext(edgeHalfedge(edge, 0)) != edgeHalfedge(edge, 1) || halfedgeNext(edgeHalfedge(edge, 1)) != edgeHalfedge(edge, 0)) {
		cout << "Edge is still connected! Must disconnect the edge first" << endl;
		assert(false);
	}
	if (halfedgeFace(edgeHalfedge(edge, 0)) || halfedgeFace(edgeHalfedge(edge, 1))) {
		cout << "Cannot delete edge with face! Must delete face first" << endl;
		assert(false);
	}
	if (vertexHalfedge(edgeVertex1(edge)) == NULL || halfedgeEdge(vertexHalfedge(edgeVertex1(edge))) != edge) {
		;
	}
	else {
		cout << "vertexHalfedge check failed! Must update vertex attribute first" << endl;
		assert(false);
	}
	if (vertexHalfedge(edgeVertex2(edge)) == NULL || halfedgeEdge(vertexHalfedge(edgeVertex2(edge))) != edge) {
		;
	}
	else {
		cout << "vertexHalfedge check failed! Must update vertex attribute first" << endl;
		assert(false);
	}
	tVertex v1 = edgeVertex1(edge);
	tVertex v2 = edgeVertex2(edge);
	tVertex pV = (v1->getId() < v2->getId()) ? v1 : v2;
	std::list<CEdge*>&ledges = (std::list<CEdge*> &) pV->edges();
	auto it = std::find(ledges.begin(), ledges.end(), edge);
	if (it != ledges.end()) {
		ledges.erase(it);
	}
	else {
		assert(false);
	}
	m_edges.remove(edge);
	delete edgeHalfedge(edge, 0);
	delete edgeHalfedge(edge, 1);
	delete edge;
	return;
}
#endif //_MESHLIB_BASE_MESH_H_ defined
