/*!
*      \file Vertex.h
*      \brief Base class of vertex
*	   \author David Gu
*      \date 10/07/2010
*
*/

#ifndef  _MESHLIB_VERTEX_H_
#define  _MESHLIB_VERTEX_H_

#include <stdlib.h>
#include <string>
#include <list>
#include "../Geometry/Point.h"
#include "../Geometry/Point2.h"
#include "HalfEdge.h"

namespace MeshLib{

  class CHalfEdge;

  /*!
  \brief CVertex class, which is the base class of all kinds of vertex classes
  */

  class CVertex
  {
  public:
	  /*!
	  CVertex constructor
	  */
      CVertex(){
		  m_halfedge = NULL; 
		  m_id = -1;
	  };
	  /*!
	  CVertex destructor 
	  */
    ~CVertex(){};

	/*! The point of the vertex
	*/
    CPoint & point()    { return m_point;};
	/*! The normal of the vertex
	*/
    CPoint & normal()   { return m_normal; };
	/*! The texutre coordinates of the vertex
	*/
	CPoint2 & uv()       { return m_uv; };

	/*! One incoming halfedge of the vertex .
	*/
    CHalfEdge * & halfedge() {
		return m_halfedge; 
	};
	/*! the string of the vertex. 
	*/
	std::string & string() { return m_string;};
	/*! Vertex id. 
	*/
  protected:
    int  & id() { return m_id; };
  public:
	  int fakeId = -1;
	  int getId() {
		  return m_id;
	  }
	  void setId(int id) {
		  m_id = id;
	  }
    /*! Convert vertex traits to string. 
	*/
	void _to_string()   {};
	/*! Read traits from the string. 
	*/
	void _from_string() {};

	/*!	Adjacent edges, temporarily used for loading the mesh
	 */
	std::list<CEdge*> & edges() { return m_edges; };

  protected:

    /*! Vertex ID. 
	*/
    int    m_id ;
    /*! Vertex position point. 
	*/
    CPoint m_point;
	/*! Normal at the vertex. 
	*/
    CPoint m_normal;
	/*! Texture coordinates of the vertex. 
	*/
	CPoint2 m_uv;
	
	// in boundary halfedge if vertex is on boundary
    CHalfEdge *     m_halfedge;
	/*! Indicating if the vertex is on the boundary. 
	*/
	/*! The string of the vertex, which stores the traits information. 
	*/
	std::string     m_string;

	/*! List of adjacent edges, such that current vertex is the end vertex of the edge with smaller id
	 */
	std::list<CEdge*> m_edges;

  }; //class CVertex

}//name space MeshLib

#endif //_MESHLIB_VERTEX_H_defined