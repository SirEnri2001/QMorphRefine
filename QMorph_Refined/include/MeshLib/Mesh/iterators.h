/*!
*      \file Iterators.h
*      \brief Iterators for accessing geometric objects on a mesh
*	   \author David Gu
*      \date 10/07/2010
*
*/

#ifndef  _ITERATORS_H_
#define  _ITERATORS_H_

#include "BaseMesh.h"

namespace MeshLib {

	//v->in halfedge
	/*!
		\brief VertexInHalfedgeIterator, transverse all the incoming halfedges of a vertex ccwly.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class VertexInHalfedgeIterator
	{
	public:
		/*!
		VertexInHalfedgeIteartor constructor
		\param pMesh pointer to the current mesh
		\param v     pointer to the current vertex
		*/
		VertexInHalfedgeIterator(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh, CVertex* v) {
			m_pMesh = pMesh;
			m_vertex = v;
			m_halfedge = m_pMesh->vertexHalfedge(v);
			m_halfedge_end = m_pMesh->halfedgeSym(m_halfedge);
		}
		void reset() {
			m_halfedge = m_pMesh->vertexHalfedge(m_vertex);
		}
		/*!
		VertexOutHalfedgeIterator destructor
		*/
		~VertexInHalfedgeIterator() {};
		/*!
		prefix ++ operator, goes to the next ccw vertex out halfedge
		*/
		void next() {
			assert(m_halfedge != NULL);
			if (m_pMesh->halfedgeNext(m_halfedge) == m_halfedge_end) {
				m_halfedge = NULL;
			}
			else
				m_halfedge = m_pMesh->halfedgeSym(m_pMesh->halfedgeNext(m_halfedge));
		}

		void operator++() //prefix
		{
			next();
		};

		/*!
			postfix ++ operator, goes to the next ccw vertex out halfedge
		*/
		void operator++(int) //postfix
		{
			next();
		};

		/*!
			The current halfedge the iterator pointing to.
		*/

		virtual CHalfEdge* value() { return m_halfedge; };
		/*!
		   whether all the out halfedges have been visited.
		*/
		bool end() { return m_halfedge == NULL; };
		/*!
			The current halfedge the iterator pointing to.
		*/
		virtual CHalfEdge* operator*() { return value(); };

	private:
		/*!
			Current mesh.
		*/
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
		/*!
			Current vertex.
		*/
		CVertex* m_vertex;
		/*!
			Current halfedge.
		*/
		CHalfEdge* m_halfedge_end;
		CHalfEdge* m_halfedge;
	};
	//v->out halfedge
	/*!
		\brief VertexOutHalfedgeIterator, transverse all the outgoing halfedges of a vertex ccwly.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class VertexOutHalfedgeIterator
	{
	public:
		/*!
		VertexOutHalfedgeIteartor constructor
		\param pMesh pointer to the current mesh
		\param v     pointer to the current vertex
		*/
		VertexOutHalfedgeIterator<CVertex, CEdge, CFace, CHalfEdge>(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh, CVertex* v) : vih_iter(pMesh, v)
		{
			m_pMesh = pMesh;
		};
		/*!
		VertexOutHalfedgeIterator destructor
		*/
		~VertexOutHalfedgeIterator<CVertex, CEdge, CFace, CHalfEdge>() {};

		CHalfEdge* value() { return m_pMesh->halfedgeSym(*vih_iter); };
		/*!
			The current halfedge the iterator pointing to.
		*/
		CHalfEdge* operator*() { return value(); };

		void reset() {
			vih_iter.reset();
		}

		bool end() {
			return vih_iter.end();
		};

		void next() {
			vih_iter.next();
		}

		void operator++() //prefix
		{
			next();
		};

		void operator++(int) //postfix
		{
			next();
		};
	private:
		VertexInHalfedgeIterator<CVertex, CEdge, CFace, CHalfEdge> vih_iter;
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
	};



	//v -> v

	/*!
		\brief VertexVertexIterator, transverse all the neighboring vertices of a vertex ccwly.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class VertexVertexIterator
	{
	public:
		/*!
			VertexVertexIterator constructor
			\param v the current vertex
		*/
		VertexVertexIterator<CVertex, CEdge, CFace, CHalfEdge>(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh, CVertex* v) :
			vih_iter(pMesh, v)
		{
			m_pMesh = pMesh;
		};

		/*!
			VertexVertexIterator destructor
		*/
		~VertexVertexIterator() {};
		/*!
			The neighboring vertex, pointed by the current iterator
		*/

		CVertex* value()
		{
			return m_pMesh->halfedgeSource(*vih_iter);
		};

		CVertex* operator*() { return value(); };

		bool end() {
			return vih_iter.end();
		};

		void next() {
			vih_iter.next();
		}

		void operator++() //prefix
		{
			next();
		};

		void operator++(int) //postfix
		{
			next();
		};
	private:
		VertexInHalfedgeIterator<CVertex, CEdge, CFace, CHalfEdge> vih_iter;
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
	};


	//v -> edge
	/*!
		\brief VertexEdgeIterator, transverse all the neighboring edges of a vertex ccwly.
	*/

	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class VertexEdgeIterator
	{
	public:
		/*!
			VertexEdgeIterator constructor
			\param v the current vertex
		*/
		VertexEdgeIterator<CVertex, CEdge, CFace, CHalfEdge>(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh, CVertex* v) :
			vih_iter(pMesh, v)
		{
			m_pMesh = pMesh;
		};

		CEdge* value()
		{
			return m_pMesh->halfedgeEdge(*vih_iter);
		};

		CEdge* operator*() { return value(); };

		bool end() {
			return vih_iter.end();
		};

		void next() {
			vih_iter.next();
		}

		void operator++() //prefix
		{
			next();
		};

		void operator++(int) //postfix
		{
			next();
		};
	private:
		VertexInHalfedgeIterator<CVertex, CEdge, CFace, CHalfEdge> vih_iter;
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
	};



	// v->face
	/*!
		\brief VertexFaceIterator, transverse all the neighboring faces of a vertex ccwly.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class VertexFaceIterator
	{
	public:
		/*!
			VertexFaceIterator constructor
			\param v the current vertex
		*/
		VertexFaceIterator(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh, CVertex*& v) :
			vih_iter(pMesh, v)
		{
			m_pMesh = pMesh;
			if (m_pMesh->halfedgeFace(*vih_iter) == NULL) {
				next();
			}
		};
		void next() {
			do {
				vih_iter.next();
			} while (!vih_iter.end() && !m_pMesh->halfedgeFace(*vih_iter));
		}

		CFace* value() {
			return m_pMesh->halfedgeFace(*vih_iter);
		}

		CFace* operator*() { return value(); };

		bool end() {
			return vih_iter.end();
		};

		void operator++() //prefix
		{
			next();
		};

		void operator++(int) //postfix
		{
			next();
		};
	private:
		VertexInHalfedgeIterator<CVertex, CEdge, CFace, CHalfEdge> vih_iter;
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
	};

	// f -> halfedge
	/*!
		\brief FaceHalfedgeIterator, transverse all the halfedges of a face CCWly.
	*/

	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class FaceHalfedgeIterator
	{
	public:
		/*!
			FaceHalfedgeIterator constructor
			\param f the current face
		*/
		FaceHalfedgeIterator(CFace* f)
		{
			m_face = f;
			m_halfedge = (CHalfEdge*)f->halfedge();
		};
		/*!
			FaceHalfedgeIterator destructor
		*/
		~FaceHalfedgeIterator() {};
		/*!
			VertexVertexIterator prefix operator ++, goes to the next halfedge CCWly
		*/
		void operator++() //prefix
		{
			assert(m_halfedge != NULL);
			m_halfedge = (CHalfEdge*)m_halfedge->he_next();

			if (m_halfedge == m_face->halfedge())
			{
				m_halfedge = NULL;
				return;
			};
		}

		/*!
			VertexVertexIterator prefix operator ++, goes to the next halfedge CCWly
		*/
		void operator++(int) //postfix
		{
			assert(m_halfedge != NULL);
			m_halfedge = (CHalfEdge*)m_halfedge->he_next();

			if (m_halfedge == m_face->halfedge())
			{
				m_halfedge = NULL;
				return;
			};
		}

		/*!
			The halfedge, pointed by the current iterator
		*/
		CHalfEdge* value() { return m_halfedge; };
		/*!
			The halfedge, pointed by the current iterator
		*/
		CHalfEdge* operator*() { return value(); };

		/*!
			Indicate whether all the halfedges have been accessed.
		*/
		bool end() { return m_halfedge == NULL; };

	private:
		/*!
			current face
		*/
		CFace* m_face;
		/*!
			current halfedge
		*/
		CHalfEdge* m_halfedge;
	};


	// f -> edge
	/*!
		\brief FaceEdgeIterator, transverse all the edges of a face CCWly.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class FaceEdgeIterator
	{
	public:
		/*!
			FaceEdgeIterator constructor
			\param f the current face
		*/
		FaceEdgeIterator(CFace* f)
		{
			m_face = f;
			m_halfedge = (CHalfEdge*)f->halfedge();
		};

		/*!
			FaceEdgeIterator destructor
		*/
		~FaceEdgeIterator() {};
		/*!
			FaceEdgeIterator prefix operator ++, goes to the next edge CCWly
		*/
		void operator++()	//prefix
		{
			assert(m_halfedge != NULL);
			m_halfedge = (CHalfEdge*)m_halfedge->he_next();

			if (m_halfedge == (CHalfEdge*)m_face->halfedge())
			{
				m_halfedge = NULL;
				return;
			};
		}

		/*!
			FaceEdgeIterator prefix operator ++, goes to the next edge CCWly
		*/
		void operator++(int)	//postfix
		{
			assert(m_halfedge != NULL);
			m_halfedge = (CHalfEdge*)m_halfedge->he_next();

			if (m_halfedge == m_face->halfedge())
			{
				m_halfedge = NULL;
				return;
			};
		}
		/*!
			The edge, pointed by the current iterator
		*/
		CEdge* value() { return (CEdge*)m_halfedge->edge(); };
		/*!
			The edge, pointed by the current iterator
		*/
		CEdge* operator*() { return value(); };
		/*!
			Indicate whether all the edges have been transversed.
		*/
		bool end() { return m_halfedge == NULL; };

	private:
		/*! Current face. */
		CFace* m_face;
		/*! Current halfedge. */
		CHalfEdge* m_halfedge;
	};


	// f -> vertex
	/*!
		\brief FaceVertexIterator, transverse all the vertices of a face CCWly.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class FaceVertexIterator
	{
	public:
		/*!
			FaceVertexIterator constructor
			\param f the current face
		*/
		FaceVertexIterator(CFace* f)
		{
			m_face = f;
			m_halfedge = (CHalfEdge*)f->halfedge();
		};
		/*!
			FaceVertexIterator destructor
		*/

		~FaceVertexIterator() {};
		/*!
			FaceVertexIterator prefix operator ++, goes to the next vertex CCWly
		*/
		void operator++() //prefix
		{
			assert(m_halfedge != NULL);
			m_halfedge = (CHalfEdge*)m_halfedge->he_next();

			if (m_halfedge == (CHalfEdge*)m_face->halfedge())
			{
				m_halfedge = NULL;
				return;
			};
		}

		/*!
			FaceVertexIterator prefix operator ++, goes to the next vertex CCWly
		*/
		void operator++(int) //postfix
		{
			assert(m_halfedge != NULL);
			m_halfedge = (CHalfEdge*)m_halfedge->he_next();

			if (m_halfedge == (CHalfEdge*)m_face->halfedge())
			{
				m_halfedge = NULL;
				return;
			};
		}
		/*!
			The vertex, pointed by the current iterator
		*/
		CVertex* value() { return (CVertex*)m_halfedge->target(); };
		/*!
			The vertex, pointed by the current iterator
		*/
		CVertex* operator*() { return value(); };
		/*!
			Indicate whether all the vertices have been accessed.
		*/
		bool end() { return m_halfedge == NULL; };

	private:
		/*!	Current face.
		*/
		CFace* m_face;
		/*!	Current halfedge.
		*/
		CHalfEdge* m_halfedge;
	};


	// mesh->v
	/*!
		\brief MeshVertexIterator, transverse all the vertices in the mesh.
	*/

	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class MeshVertexIterator
	{
	public:
		/*!
		MeshVertexIterator constructor,
		\param pMesh the current mesh
		*/
		MeshVertexIterator(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh)
		{
			m_pMesh = pMesh;
			m_iter = m_pMesh->vertices().begin();
		}
		/*!
		The vertex, pointed by the current iterator
		*/
		CVertex* value() { 
			return *m_iter; 
		};
		/*!
		The vertex, pointed by the current iterator
		*/

		CVertex* operator*() { return value(); };
		/*!
			MeshVertexIterator prefix operator ++, goes to the next vertex
		*/
		void operator++() { ++m_iter; }; //prefix
		/*!
			MeshVertexIterator prefix operator ++, goes to the next vertex
		*/
		void operator++(int) { ++m_iter; }; //postfix
		/*!
			Indicate whether all the vertices have been accessed.
		*/
		bool end() { return m_iter == m_pMesh->vertices().end(); }

	private:
		/*!
			Current mesh.
		*/
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
		/*!
		Current vertex list iterator.
		*/
		typename std::list<CVertex*>::iterator m_iter;
	};

	// mesh->f
	/*!
		\brief MeshFaceIterator, transverse all the faces in the mesh.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class MeshFaceIterator
	{
	public:
		/*!
		MeshFaceIterator constructor,
		\param pMesh the current mesh
		*/
		MeshFaceIterator(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh)
		{
			m_pMesh = pMesh;
			m_iter = pMesh->faces().begin();
		}
		/*!
		The face, pointed by the current iterator
		*/
		CFace* value() { return *m_iter; };
		/*!
		The face, pointed by the current iterator
		*/
		CFace* operator*() { return value(); };

		/*!
			MeshFaceIterator prefix operator ++, goes to the next vertex
		*/
		void operator++() { ++m_iter; }; //prefix
		/*!
			MeshFaceIterator postfix operator ++, goes to the next vertex
		*/
		void operator++(int) { ++m_iter; }; //postfix
		/*!
			Indicate whether all the faces have been accessed.
		*/
		bool end() { return m_iter == m_pMesh->faces().end(); }

	private:
		/*! Current mesh.
		*/
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
		/*! Current face list iterator.
		*/
		typename std::list<CFace*>::iterator  m_iter;
	};

	//Mesh->e
	/*!
		\brief MeshEdgeIterator, transverse all the edges in the mesh.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class MeshEdgeIterator
	{
	public:
		/*!
		MeshEdgeIterator constructor,
		\param pMesh the current mesh
		*/
		MeshEdgeIterator(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh)
		{
			m_pMesh = pMesh;
			m_iter = m_pMesh->edges().begin();
		}
		/*!
		The edge, pointed by the current iterator
		*/
		CEdge* value() { return *m_iter; };
		/*!
		The edge, pointed by the current iterator
		*/
		CEdge* operator*() { return value(); };
		/*!
			MeshEdgeIterator prefix operator ++, goes to the next edge
		*/
		void operator++() { ++m_iter; }; //prefix
		/*!
			MeshEdgeIterator postfix operator ++, goes to the next edge
		*/
		void operator++(int) { m_iter++; }; //postfix
		/*!
			Indicate whether all the edges have been accessed.
		*/
		bool end() { return m_iter == m_pMesh->edges().end(); }


	private:
		/*!
		current mesh
		*/
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
		/*!
		current edge list iterator
		*/
		typename std::list<CEdge*>::iterator m_iter;
	};

	// Mesh->he
	/*!
		\brief MeshHalfEdgeIterator, transverse all the halfedges in the mesh.
	*/
	template<typename CVertex, typename CEdge, typename CFace, typename CHalfEdge>
	class MeshHalfEdgeIterator
	{
	public:
		/*!
		MeshHalfedgeIterator constructor,
		\param pMesh the current mesh
		*/
		MeshHalfEdgeIterator(CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* pMesh)
		{
			m_pMesh = pMesh;
			m_iter = m_pMesh->edges().begin();
			m_id = 0;
		}
		/*!
		The halfedge, pointed by the current iterator
		*/
		CHalfEdge* value() { CEdge* e = *m_iter; return (CHalfEdge*)e->halfedge(m_id); };
		/*!
		The halfedge, pointed by the current iterator
		*/
		CHalfEdge* operator*() { return value(); };
		/*!
			MeshVertexIterator prefix operator ++, goes to the next halfedge
		*/
		void operator++() //prefix
		{
			++m_id;

			switch (m_id)
			{
			case 1:
			{
				CEdge* e = *m_iter;
				if (e->halfedge(m_id) == NULL)
				{
					m_id = 0;
					++m_iter;
				}
			}
			break;
			case 2:
				m_id = 0;
				++m_iter;
				break;
			}
		};
		/*!
			MeshVertexIterator postfix operator ++, goes to the next vertex
		*/
		void operator++(int) //postfix
		{
			++m_id;

			switch (m_id)
			{
			case 1:
			{
				CEdge* e = *m_iter;
				if (e->halfedge(m_id) == NULL)
				{
					m_id = 0;
					++m_iter;
				}
			}
			break;
			case 2:
				m_id = 0;
				++m_iter;
				break;
			}
		};
		/*!
		Indicate whether all the halfedges have been accessed
		*/
		bool end() { return m_iter == m_pMesh->edges().end(); }


	private:
		/*!
			Current halfedge
		*/
		CHalfEdge* m_he;
		/*!
			Current mesh
		*/
		CBaseMesh<CVertex, CEdge, CFace, CHalfEdge>* m_pMesh;
		/*!
			Current edge list iterator
		*/
		typename std::list<CEdge*>::iterator m_iter;
		int  m_id;
	};


} //Iterators

#endif
