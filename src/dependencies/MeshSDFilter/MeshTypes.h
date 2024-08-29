// BSD 3-Clause License
//
// Copyright (c) 2017, Bailin Deng
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef MESHTYPES_H
#define MESHTYPES_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "EigenTypes.h"

#include <OpenMesh/Core/Geometry/VectorT.hh>


/// Default traits for triangle mesh
struct TriTraits : public OpenMesh::DefaultTraits
{
  /// Use double precision points
  typedef OpenMesh::Vec3d Point;
  /// Use double precision Normals
  typedef OpenMesh::Vec3d Normal;

  /// Use RGBA Color
  typedef OpenMesh::Vec4f Color;

};

/// Simple name for triangle mesh
typedef OpenMesh::TriMesh_ArrayKernelT<TriTraits>  TriMesh;


namespace SDFilter
{

// Collect all point positions into a matrix.
// Argument points must be have been initialized with the correct number of columns.
template<typename MeshT>
void get_vertex_points(const MeshT &mesh, Matrix3X &points)
{
    points.resize(3, mesh.n_vertices());
    for(typename MeshT::ConstVertexIter cv_it = mesh.vertices_begin(); cv_it != mesh.vertices_end(); ++ cv_it)
    {
        points.col(cv_it->idx()) = to_eigen_vec3d(mesh.point(*cv_it));
    }
}


template<typename MeshT>
void set_vertex_points(MeshT &mesh, const Matrix3X &pos)
{
    assert(static_cast<int>(mesh.n_vertices()) * 3 == static_cast<int>(pos.size()));

    for(typename MeshT::ConstVertexIter cv_it = mesh.vertices_begin();
        cv_it != mesh.vertices_end(); ++ cv_it)
    {
        Eigen::Vector3d pt = pos.col(cv_it->idx());
        mesh.set_point(*cv_it, from_eigen_vec3d<typename MeshT::Point>(pt));
    }
}


template<typename MeshT>
void get_vertex_points(const MeshT &mesh, std::vector<double> &points)
{
    points.assign(3 * mesh.n_vertices(), 0.0);
    for(typename MeshT::ConstVertexIter cv_it = mesh.vertices_begin(); cv_it != mesh.vertices_end(); ++ cv_it)
    {
    	typename MeshT::Point pt = mesh.point(*cv_it);
    	int v_idx = cv_it->idx();

    	assert(v_idx >=0 && v_idx < static_cast<int>(mesh.n_vertices()));

    	for(int i = 0; i < 3; ++ i){
    		points[3 * v_idx + i] = pt[i];
    	}
    }
}

void get_face_vertex_indices(const TriMesh &mesh, Matrix3Xi &face_vertex_indices)
{
	face_vertex_indices.resize(3, mesh.n_faces());

	for(TriMesh::ConstFaceIter cf_it = mesh.faces_begin(); cf_it != mesh.faces_end(); ++ cf_it)
	{
		int i = 0;

		for(TriMesh::ConstFaceVertexIter cfv_it = mesh.cfv_iter(*cf_it); cfv_it.is_valid(); ++ cfv_it)
		{
			face_vertex_indices(i++, cf_it->idx()) = cfv_it->idx();
		}
	}
}


template<typename MeshT>
void set_vertex_points(MeshT &mesh, const std::vector<double> &pos)
{
    assert(mesh.n_vertices() * 3 == pos.size());

    for(typename MeshT::ConstVertexIter cv_it = mesh.vertices_begin();
        cv_it != mesh.vertices_end(); ++ cv_it)
    {
        int addr = cv_it->idx() * 3;
        mesh.set_point(*cv_it, typename MeshT::Point(pos[addr], pos[addr+1], pos[addr+2]));
    }
}

// Write a mesh to an ASCII file with high accuracy
template<typename MeshT>
inline bool write_mesh_high_accuracy(const MeshT &mesh, const std::string &filename)
{
	return OpenMesh::IO::write_mesh(mesh, filename, OpenMesh::IO::Options::Default, 16);
}

template<typename MeshT>
inline Eigen::Vector3d bbox_dimension(const MeshT &mesh)
{
	Matrix3X vtx_pos;
	get_vertex_points(mesh, vtx_pos);

	return vtx_pos.rowwise().maxCoeff() - vtx_pos.rowwise().minCoeff();
}


template<typename MeshT>
inline double bbox_diag_length(const MeshT &mesh)
{
	return bbox_dimension(mesh).norm();
}

template<typename MeshT>
inline Eigen::Vector3d mesh_center(const MeshT &mesh)
{
	Matrix3X vtx_pos;
	get_vertex_points(mesh, vtx_pos);

	return vtx_pos.rowwise().mean();
}


template<typename MeshT>
inline double average_neighbor_face_centroid_dist(const MeshT &mesh)
{
	Matrix3X centroid_pos(3, mesh.n_faces());
	for(typename MeshT::ConstFaceIter cf_it = mesh.faces_begin(); cf_it != mesh.faces_end(); ++ cf_it)
	{
		centroid_pos.col(cf_it->idx()) = to_eigen_vec3d(mesh.calc_face_centroid(*cf_it));
	}

	double centroid_dist = 0;
	int n = 0;
	for(typename MeshT::ConstEdgeIter ce_it = mesh.edges_begin(); ce_it != mesh.edges_end(); ++ ce_it)
	{
		if(!mesh.is_boundary(*ce_it))
		{
			int f1 = mesh.face_handle(mesh.halfedge_handle(*ce_it, 0)).idx(),
				f2 = mesh.face_handle(mesh.halfedge_handle(*ce_it, 1)).idx();

			centroid_dist += (centroid_pos.col(f1) - centroid_pos.col(f2)).norm();
			n++;
		}
	}

	return centroid_dist / n;

}

template<typename MeshT>
inline double average_edge_length(const MeshT &mesh)
{
	if(static_cast<int>(mesh.n_edges()) == 0){
		return 0.0;
	}

	double length = 0;

	for(typename MeshT::ConstEdgeIter ce_it = mesh.edges_begin(); ce_it != mesh.edges_end(); ++ ce_it)
	{
		length += mesh.calc_edge_length(*ce_it);
	}

	return length / mesh.n_edges();
}


// Center the mesh at the origin, and normalize its scale
// Return the orginal mesh center, and the orginal scale
template<typename MeshT>
inline void normalize_mesh(MeshT &mesh, Eigen::Vector3d &original_center, double &original_scale)
{
	original_center = mesh_center(mesh);
	original_scale = bbox_diag_length(mesh);

	Matrix3X vtx_pos;
	get_vertex_points(mesh, vtx_pos);

	vtx_pos.colwise() -= original_center;
	vtx_pos *= (1.0 / original_scale);

	set_vertex_points(mesh, vtx_pos);
}

// Scale and move a normalized mesh to according to its original scale and center
template<typename MeshT>
inline void restore_mesh(MeshT &mesh, const Eigen::Vector3d &original_center, double original_scale)
{
	Matrix3X vtx_pos;
	get_vertex_points(mesh, vtx_pos);

	Eigen::Vector3d center = vtx_pos.rowwise().mean();
	vtx_pos.colwise() -= center;

	vtx_pos *= original_scale;
	vtx_pos.colwise() += original_center;

	set_vertex_points(mesh, vtx_pos);
}

}

#endif // MESHTYPES_H
