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


#ifndef MESHNORMALFILTER_H_
#define MESHNORMALFILTER_H_

#include "MeshTypes.h"
#include "SDFilter.h"
#include <cmath>
#include <algorithm>

namespace SDFilter
{

class MeshFilterParameters : public Parameters
{
public:
	MeshFilterParameters()
	:mesh_update_method(ITERATIVE_UPDATE), mesh_update_closeness_weight(0.001), mesh_update_iter(20)
	{
		// Use an threshold value corresponding to eps_angle_degree degrees change of normal vectors between two iterations
		double eps_angle_degree = 0.2;
		avg_disp_eps = 2 * std::sin(eps_angle_degree * 0.5 * M_PI / 180);
	}

	virtual ~MeshFilterParameters(){}

	// Methods for mesh vertex update according to filtered normals
	enum MeshUpdateMethod
	{
		ITERATIVE_UPDATE,	// ShapeUp styled iterative solver
		POISSON_UPDATE,	// Poisson-based update from [Want et al. 2015]
	};

	MeshUpdateMethod mesh_update_method;

	double mesh_update_closeness_weight;	// Weight for the closeness term in mesh update

	int mesh_update_iter;	// Number of mesh update iterations

	virtual bool valid_parameters() const
	{
		if(!Parameters::valid_parameters()){
			return false;
		}

		if(mesh_update_iter <= 0){
			std::cerr << "Error: MeshUpdateIterations must be positive" << std::endl;
			return false;
		}

		if(mesh_update_closeness_weight < 0){
			std::cerr << "Error: MeshUpdateClosenessWeight must be positive" << std::endl;
			return false;
		}

		return true;
	}

protected:
	virtual bool load_option(const OptionInterpreter &opt)
	{
		return Parameters::load_option(opt) ||
				opt.load("MeshUpdateClosenessWeight", mesh_update_closeness_weight) ||
				opt.load("MeshUpdateIterations", mesh_update_iter);
	}

	virtual void output_options()
	{
		Parameters::output_options();
		std::cout << "Mesh update closeness weight: " << mesh_update_closeness_weight << std::endl;
		std::cout << "Mesh update iterations: " << mesh_update_iter << std::endl;
	}
};

class MeshNormalFilter : public SDFilter
{
public:

	MeshNormalFilter(const TriMesh &mesh)
	:mesh_(mesh), print_error_evaluation_(false), linear_solver_(Parameters::LDLT), system_matrix_factorized_(false){}

	virtual ~MeshNormalFilter() {}

	// Pass param by value, to allow changing the value of eta
	bool filter(MeshFilterParameters param, TriMesh &output_mesh)
	{
		assert(param.valid_parameters());

		Timer timer;
		Timer::EventID mesh_flter_begin_time = timer.get_time();

		// Rescale the eta parameter, according to average distance between neighboring face centroids
		param.eta *= average_neighbor_face_centroid_dist(mesh_);

		if(!SDFilter::filter(param)){
			std::cerr << "Error in performing SD filter" << std::endl;
			return false;
		}


		Timer::EventID update_begin_time = timer.get_time();

		// Normalize the filtered face normals
		Matrix3X target_normals = signals_.block(0, 0, 3, signals_.cols());
		target_normals.colwise().normalize();

		if(param.mesh_update_method == MeshFilterParameters::ITERATIVE_UPDATE){
			if(!iterative_mesh_update(param, target_normals, output_mesh)){
				std::cerr << "Error in iteative mesh update" << std::endl;
				return false;
			}
		}
		else{
			if(!Poisson_mesh_update(target_normals, output_mesh)){
				std::cerr << "Error in Poisson mesh update" << std::endl;
				return false;
			}
		}

		Timer::EventID update_end_time = timer.get_time();

		if(print_timing_){
			std::cout << "Mesh udpate timing: " << timer.elapsed_time(update_begin_time, update_end_time) << " secs" << std::endl;
			std::cout << "Mesh filter total timing: " << timer.elapsed_time(mesh_flter_begin_time, update_end_time) << " secs" << std::endl;
		}

		if(print_error_evaluation_)
		{
			std::cout << std::endl;
			show_normalized_mesh_displacement_norm(output_mesh);
			show_normal_error_statistics(output_mesh, target_normals, 2, 10);
		}

		return true;
	}

protected:

	TriMesh mesh_;

	bool print_error_evaluation_;	// The printing of mesh update error

	bool get_neighborhood(const Parameters &param, Eigen::Matrix2Xi &neighbor_pairs, Eigen::VectorXd &neighbor_dist)
	{
		// Neighborhood radius set to three times of eta
		double radius = 3.0 * param.eta;
		int n_faces = mesh_.n_faces();
		int n_vtx = mesh_.n_vertices();

		Matrix3X face_centroids(3, n_faces);
		std::vector< std::vector<int> > upper_neighbor_lists(n_faces), lower_neighbor_lists(n_faces);	// Lists of neighbor faces with indices larger/smaller than the current face

		OMP_PARALLEL
		{
			// Pre-compute face centroids
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				face_centroids.col(i) = to_eigen_vec3d(mesh_.calc_face_centroid(TriMesh::FaceHandle(i)));
			}

			// Store neighbor lists for each face
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				std::queue<int> face_queue;
				std::vector<bool> face_visited(n_faces, false), vtx_visited(n_vtx, false);
				face_visited[i] = true;
				face_queue.push(i);

				Eigen::Vector3d center_face_centroid = face_centroids.col(i);

				while(!face_queue.empty())
				{
					TriMesh::FaceHandle fh(face_queue.front());
					face_queue.pop();

					// Enumerate the three vertices of the current face, and check their neighboring faces
					for(TriMesh::ConstFaceVertexIter cfv_it = mesh_.cfv_iter(fh); cfv_it.is_valid(); ++ cfv_it)
					{
						int current_vtx = cfv_it->idx();

						if(!vtx_visited[current_vtx])
						{
							vtx_visited[current_vtx] = true;

							for(TriMesh::ConstVertexFaceIter cvf_it = mesh_.cvf_iter(*cfv_it); cvf_it.is_valid(); ++ cvf_it)
							{
								int current_face = cvf_it->idx();

								if(!face_visited[current_face])
								{
									face_visited[current_face] = true;

									double dist = (face_centroids.col(current_face) - center_face_centroid).norm();

									if(dist < radius)
									{
										face_queue.push(current_face);

										if(current_face < i){
											// Make sure each neigbor list only stores indices smaller than its face index
											lower_neighbor_lists[i].push_back(current_face);
										}
										else if(current_face > i){
											upper_neighbor_lists[i].push_back(current_face);
										}
									}
								}
							}
						}
					}
				}
			}

			// Sort the neighbor lists
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				std::sort(upper_neighbor_lists[i].begin(), upper_neighbor_lists[i].end());
				std::sort(lower_neighbor_lists[i].begin(), lower_neighbor_lists[i].end());
			}
		}

		// For each upper neighbor list, check the lower neighbor list of the candidate neighbor to make sure the neighboring relationship is symmetric

		Eigen::VectorXi lower_neighbor_front_idx(n_faces);	// Indices for keeping track of elements that have been checked in each lower neighbor list
		lower_neighbor_front_idx.setZero();
		VectorXIdx segment_start_addr(n_faces);	// Starting address for the segment of each face within the neighbor pair array
		Eigen::Index n_neighbor_pairs = 0;

		for(int i = 0; i < n_faces; ++ i)
		{
			std::vector<int> &current_upper_list = upper_neighbor_lists[i];
			int j = 0;

			while(j < static_cast<int>(current_upper_list.size()))
			{
				// Try to locate the current face in the lower list of the current upper neighbor.
				// If it can be found, then the neighboring relationship is symmetric
				int neighbor_idx = current_upper_list[j];
				std::vector<int> &lower_list = lower_neighbor_lists[neighbor_idx];

				int lower_list_size = lower_list.size();
				int current_lower_list_idx = lower_neighbor_front_idx(neighbor_idx);

				while(current_lower_list_idx < lower_list_size && lower_list[current_lower_list_idx] < i){
					current_lower_list_idx ++;
				}

				lower_neighbor_front_idx(neighbor_idx) = current_lower_list_idx;

				// If the current upper neighbor is not symmetric, remove it from the upper neighbor list.
				// Otherwise, move on to check the next upper neighbor.
				if(current_lower_list_idx < lower_list_size && lower_list[current_lower_list_idx] == i){
					j++;
				}
				else{
					current_upper_list.erase(current_upper_list.begin() + j);
				}
			}

			segment_start_addr(i) = n_neighbor_pairs;
			n_neighbor_pairs += current_upper_list.size();
		}

		neighbor_pairs.resize(2, n_neighbor_pairs);
		neighbor_dist.resize(n_neighbor_pairs);

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				std::vector<int> &upper_neighbors = upper_neighbor_lists[i];

				if(!upper_neighbors.empty()){
					Eigen::Index start_col = segment_start_addr(i);
					Eigen::Index n_cols = upper_neighbors.size();
					neighbor_pairs.block(0, start_col, 1, n_cols).setConstant(i);
					neighbor_pairs.block(1, start_col, 1, n_cols) = Eigen::Map<Eigen::VectorXi>(upper_neighbors.data(), upper_neighbors.size()).transpose();
				}
			}

			OMP_FOR
			for(Eigen::Index i = 0; i < n_neighbor_pairs; ++ i)
			{
				int idx1 = neighbor_pairs(0, i), idx2 = neighbor_pairs(1, i);
				neighbor_dist(i) = (face_centroids.col(idx1) - face_centroids.col(idx2)).norm();
			}
		}

		//std::cout << "n_neighbor_pairs:" << n_neighbor_pairs << std::endl;
		//std::cout << "Average neighborhood size: " << 2.0 * n_neighbor_pairs / n_faces << std::endl;

		return n_neighbor_pairs > Eigen::Index(0);
	}


	void get_initial_data(Eigen::MatrixXd &guidance, Eigen::MatrixXd &init_signals, Eigen::VectorXd &area_weights)
	{
		init_signals.resize(3, mesh_.n_faces());

		for(TriMesh::ConstFaceIter cf_it = mesh_.faces_begin(); cf_it != mesh_.faces_end(); ++ cf_it)
		{
			Eigen::Vector3d f_normal = to_eigen_vec3d(mesh_.calc_face_normal(*cf_it)).normalized();
			init_signals.col(cf_it->idx()) = f_normal;
		}

		guidance = init_signals;

		get_face_area_weights(mesh_, area_weights);
	}


	void reset_mesh_update_system()
	{
		system_matrix_factorized_ = false;
	}


	void set_mesh(const TriMesh &mesh, bool invalidate_update_system)
	{
		mesh_ = mesh;

		if(invalidate_update_system){
			reset_mesh_update_system();
		}
	}



private:

	// Pre-computed information for mesh update problem
	//	\min_X  ||AX - B||^2 + w ||X - X0||^2,
	// where X are new mesh vertex positions, A is a mean-centering matrix, B are the target mean-centered positions,
	// X0 are initial vertex positions, and w > 0 is a closeness weight.
	// This amounts to solving a linear system
	//	(A^T A + w I) X = A^T B + w X0.
	// We precompute matrix A^T, and pre-factorize A^T A + w I
	LinearSolver linear_solver_;	// Linear system solver for mesh update
	SparseMatrixXd At_;		// Transpose of part of the linear least squares matrix that corresponds to mean centering of face vertices
	bool system_matrix_factorized_;	// Whether the matrix




	// Set up and pre-factorize the linear system for iterative mesh update
	bool setup_mesh_udpate_system(const Matrix3Xi &face_vtx_idx, double w_closeness)
	{
		if(system_matrix_factorized_)
		{
			return true;
		}

		int n_faces = mesh_.n_faces();
		int n_vtx = mesh_.n_vertices();
		std::vector<Triplet> A_triplets(9 * n_faces);
		std::vector<Triplet> I_triplets(n_vtx);

		// Matrix for mean centering of three vertices
		Eigen::Matrix3d mean_centering_mat;
		get_mean_centering_matrix(mean_centering_mat);

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				Eigen::Vector3i vtx_idx = face_vtx_idx.col(i);

				int triplet_addr = 9 * i;
				int row_idx = 3 * i;
				for(int j = 0; j < 3; ++ j)
				{
					for(int k = 0; k < 3; ++ k){
						A_triplets[triplet_addr++] = Triplet(row_idx, vtx_idx(k), mean_centering_mat(j, k));
					}

					row_idx++;
				}
			}

			OMP_FOR
			for(int i = 0; i < n_vtx; ++ i)
			{
				I_triplets[i] = Triplet(i, i, w_closeness);
			}
		}


		SparseMatrixXd A(3 * n_faces, n_vtx);
		A.setFromTriplets(A_triplets.begin(), A_triplets.end());
		At_ = A.transpose();
		At_.makeCompressed();

		SparseMatrixXd wI(n_vtx, n_vtx);
		wI.setFromTriplets(I_triplets.begin(), I_triplets.end());
		SparseMatrixXd M = At_ * A  + wI;

		linear_solver_.reset_pattern();
		if(!linear_solver_.compute(M)){
			std::cerr << "Error: failed to pre-factorize mesh update system" << std::endl;
			return false;
		}

		system_matrix_factorized_ = true;
		return true;
	}


	void get_face_area_weights(const TriMesh &mesh, Eigen::VectorXd &face_area_weights) const
	{
		face_area_weights.resize(mesh.n_faces());

		for(TriMesh::ConstFaceIter cf_it = mesh.faces_begin(); cf_it != mesh.faces_end(); ++ cf_it)
		{
			face_area_weights(cf_it->idx()) = mesh.calc_sector_area(mesh.halfedge_handle(*cf_it));
		}

		face_area_weights /= face_area_weights.mean();
	}


	bool iterative_mesh_update(const MeshFilterParameters &param, const Matrix3X &target_normals, TriMesh &output_mesh)
	{
		// Rescale closeness weight using the ratio between face number and vertex number, and take its square root
		double w_closeness = param.mesh_update_closeness_weight * double(mesh_.n_faces()) / mesh_.n_vertices();

		output_mesh = mesh_;

		Matrix3Xi face_vtx_idx;
		get_face_vertex_indices(output_mesh, face_vtx_idx);

		if(!setup_mesh_udpate_system(face_vtx_idx, w_closeness)){
			return false;
		}

		std::cout << "Starting iterative mesh update......" << std::endl;

		Matrix3X vtx_pos;
		get_vertex_points(output_mesh, vtx_pos);

		int n_faces = output_mesh.n_faces();
		Eigen::Matrix3Xd target_plane_local_frames(3, 2 * n_faces);	// Local frame for the target plane of each face
		std::vector<bool> local_frame_initialized(n_faces, false);

		Eigen::MatrixX3d wX0 = vtx_pos.transpose() * w_closeness;	// Part of the linear system right-hand-side that corresponds to initial vertex positions
		Eigen::MatrixX3d B(3 * n_faces, 3);	// Per-face target position of the new vertices

		int n_vtx = output_mesh.n_vertices();
		Eigen::MatrixX3d rhs(n_vtx, 3), sol(n_vtx, 3);

		for(int iter = 0; iter < param.mesh_update_iter; ++ iter)
		{
			OMP_PARALLEL
			{
				OMP_FOR
				for(int i = 0; i < n_faces; ++ i)
				{
					Eigen::Vector3d current_normal = to_eigen_vec3d(output_mesh.calc_face_normal(TriMesh::FaceHandle(i)));
					Eigen::Vector3d target_normal = target_normals.col(i);

					Eigen::Matrix3d face_vtx_pos;
					get_mean_centered_face_vtx_pos(vtx_pos, face_vtx_idx.col(i), face_vtx_pos);

					Eigen::Matrix3Xd target_pos;

					// If the current normal is not pointing away from the target normal, simply project the points onto the target plane
					if(current_normal.dot(target_normal) >= 0){
						target_pos = face_vtx_pos - target_normal * (target_normal.transpose() * face_vtx_pos);
					}
					else{
						// Otherwise, project the points onto a line in the target plane
						typedef Eigen::Matrix<double, 3, 2> Matrix32d;
						Matrix32d current_local_frame;
						if(local_frame_initialized[i]){
							current_local_frame = target_plane_local_frames.block(0, 2*i, 3, 2);
						}
						else{
							Eigen::JacobiSVD<Eigen::Vector3d, Eigen::FullPivHouseholderQRPreconditioner> jSVD_normal(target_normal, Eigen::ComputeFullU);
							current_local_frame = jSVD_normal.matrixU().block(0, 1, 3, 2);
							target_plane_local_frames.block(0, 2*i, 3, 2) = current_local_frame;
							local_frame_initialized[i] = true;
						}

						Matrix32d local_coord = face_vtx_pos.transpose() * current_local_frame;
						Eigen::JacobiSVD<Matrix32d> jSVD_coord(local_coord, Eigen::ComputeFullV);
						Eigen::Vector2d fitting_line_direction = jSVD_coord.matrixV().col(0);
						Eigen::Vector3d line_direction_3d = current_local_frame * fitting_line_direction;
						target_pos = line_direction_3d * (line_direction_3d.transpose() * face_vtx_pos);
					}

					B.block(3 * i, 0, 3, 3) = target_pos.transpose();
				}
			}

			// Solver linear system
			rhs = At_ * B + wX0;
			if(!linear_solver_.solve(rhs, sol)){
				std::cerr << "Error: failed to solve mesh update system" << std::endl;
				return false;
			}

			vtx_pos = sol.transpose();
			set_vertex_points(output_mesh, vtx_pos);
		}

		return true;
	}


	bool Poisson_mesh_update(const Matrix3X &target_normals, TriMesh &output_mesh)
	{
		output_mesh = mesh_;

		Matrix3Xi face_vtx_idx;
		get_face_vertex_indices(output_mesh, face_vtx_idx);

		std::cout << "Starting Poisson mesh update......" << std::endl;

		Matrix3X vtx_pos;
		get_vertex_points(output_mesh, vtx_pos);

		int n_faces = output_mesh.n_faces();
		int n_vtx = output_mesh.n_vertices();
		Eigen::VectorXd face_area_weights;
		get_face_area_weights(output_mesh, face_area_weights);

		// Compute the initial centroid
		Eigen::Vector3d initial_centroid = compute_centroid(face_vtx_idx, face_area_weights, vtx_pos);

		// Set up the linear least squares system
		SparseMatrixXd A(3*n_faces + 1, n_vtx);
		Eigen::MatrixX3d B(3*n_faces + 1, 3);
		std::vector<Triplet> A_triplets(9 * n_faces + 1);

		// Set the target position of the first vertex at the origin
		A_triplets.back() = Triplet(3*n_faces, 0, 1.0);
		B.row(3*n_faces).setZero();

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				// Compute rotation from current face to the target plane
				Eigen::Vector3d init_normal = to_eigen_vec3d(output_mesh.calc_face_normal(TriMesh::FaceHandle(i)));
				Eigen::Vector3d target_normal = target_normals.col(i);
				Eigen::Quaternion<double> rot = Eigen::Quaternion<double>::FromTwoVectors(init_normal, target_normal);

				Eigen::Vector3i vtx_idx = face_vtx_idx.col(i);
				Eigen::Matrix3d face_vtx_pos;
				for(int j = 0; j < 3; ++ j){
					face_vtx_pos.col(j) = vtx_pos.col(vtx_idx(j));
				}

				double area_weight = std::sqrt(face_area_weights(i));

				for(int j = 0; j < 3; ++ j)
				{
					// Search for coefficients such that
					// [v1 - (a * v2 + (1 - a) * v3)] * (v2 - v3) = 0
					// ==> a = [(v1 - v3) * (v2 - v3)] / ||v2 - v3||^2
					// Then the gradient coefficients become (1, -a, a - 1)

					int i1 = j, i2 = (j+1)%3, i3 = (j+2)%3;

					Eigen::Vector3d v1 = face_vtx_pos.col(i1),
							v2 = face_vtx_pos.col(i2),
							v3 = face_vtx_pos.col(i3);

					double a = 0.5;
					if((v2 - v3).norm() > 1e-12){
						a = (v1 - v3).dot(v2 - v3) / (v2 - v3).squaredNorm();
					}

					// Compute gradient coefficient w.r.t vertex positions
					Eigen::Vector3d current_grad_coef;
					current_grad_coef(i1) = 1.0;
					current_grad_coef(i2) = -a;
					current_grad_coef(i3) = a - 1.0;
					current_grad_coef *= area_weight;

					// Fill gradient coefficients into matrix
					int triplet_addr = i*9 + j*3;
					int row_idx = 3*i + j;
					A_triplets[triplet_addr++] = Triplet(row_idx, vtx_idx(i1), current_grad_coef(i1));
					A_triplets[triplet_addr++] = Triplet(row_idx, vtx_idx(i2), current_grad_coef(i2));
					A_triplets[triplet_addr] = Triplet(row_idx, vtx_idx(i3), current_grad_coef(i3));

					// Put the target gradient on the right-hand-side
					Eigen::Vector3d target_grad = rot * (face_vtx_pos * current_grad_coef);
					B.row(row_idx) = target_grad.transpose();
				}
			}
		}

		// Solver linear system
		A.setFromTriplets(A_triplets.begin(), A_triplets.end());
		SparseMatrixXd At = A.transpose();
		SparseMatrixXd M = At * A;
		Eigen::MatrixX3d rhs = At * B;
		Eigen::MatrixX3d sol(n_vtx, 3);

		linear_solver_.reset_pattern();
		if(!(linear_solver_.compute(M) && linear_solver_.solve(rhs, sol))){
			std::cerr << "Error: failed to solve linear system for Poisson mesh update" << std::endl;
			return false;
		}

		// Align the new mesh with the intial mesh
		vtx_pos = sol.transpose();
		Eigen::Vector3d new_centroid = compute_centroid(face_vtx_idx, area_weights_, vtx_pos);
		vtx_pos.colwise() += initial_centroid - new_centroid;
		set_vertex_points(output_mesh, vtx_pos);


		return true;
	}



	// Generate the matrix for mean-centering of the vertices of a triangle
	void get_mean_centering_matrix(Eigen::Matrix3d &mat)
	{
		mat = Eigen::Matrix3d::Identity() - Eigen::Matrix3d::Constant(1.0/3);
	}

	void get_mean_centered_face_vtx_pos(const Eigen::Matrix3Xd &vtx_pos, const Eigen::Vector3i &face_vtx, Eigen::Matrix3d &face_vtx_pos)
	{
		for(int i = 0; i < 3; ++ i){
			face_vtx_pos.col(i) = vtx_pos.col(face_vtx(i));
		}

		Eigen::Vector3d mean_pt = face_vtx_pos.rowwise().mean();
		face_vtx_pos.colwise() -= mean_pt;
	}

	// Compute the centroid of a mesh given its vertex positions and face areas
	Eigen::Vector3d compute_centroid(const Eigen::Matrix3Xi &face_vtx_idx, const Eigen::VectorXd &face_areas, const Eigen::Matrix3Xd &vtx_pos)
	{
		int n_faces = face_vtx_idx.cols();
		Eigen::Matrix3Xd face_centroids(3, n_faces);

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_faces; ++ i)
			{
				Eigen::Vector3d c = Eigen::Vector3d::Zero();
				Eigen::Vector3i face_vtx = face_vtx_idx.col(i);

				for(int j = 0; j < 3; ++ j){
					c += vtx_pos.col(face_vtx(j));
				}

				face_centroids.col(i) = c / 3.0;
			}
		}

		return (face_centroids * face_areas) / face_areas.sum();
	}


	////////// Methods for evaluating the quality of the updated mesh /////////////

	// Compute the L2 norm between the initial mesh and filtered mesh
	void show_normalized_mesh_displacement_norm(const TriMesh &filtered_mesh)
	{
		Eigen::Matrix3Xd init_vtx_pos, new_vtx_pos;
		get_vertex_points(mesh_, init_vtx_pos);
		get_vertex_points(filtered_mesh, new_vtx_pos);
		Eigen::VectorXd vtx_disp_sqr_norm = (init_vtx_pos - new_vtx_pos).colwise().squaredNorm();

		// Computer normalized vertex area weights from the original mesh
		Eigen::VectorXd face_area_weights;
		get_face_area_weights(mesh_, face_area_weights);
		Eigen::Matrix3Xi face_vtx_indices;
		get_face_vertex_indices(mesh_, face_vtx_indices);
		int n_faces = mesh_.n_faces();

		Eigen::VectorXd vtx_area(mesh_.n_vertices());
		vtx_area.setZero();
		for(int i = 0; i < n_faces; ++ i){
			for(int j = 0; j < 3; ++ j){
				vtx_area(face_vtx_indices(j, i)) += face_area_weights(i);
			}
		}
		vtx_area /= vtx_area.sum();

		std::cout << "Normalized mesh displacement norm: " <<
				std::sqrt(vtx_area.dot(vtx_disp_sqr_norm)) / average_edge_length(mesh_) << std::endl;
	}


	void show_error_statistics(const Eigen::VectorXd &err_values, double bin_size, int n_bins)
	{
		int n_elems = err_values.size();

		Eigen::VectorXi error_bin_idx(n_elems);
		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < n_elems; ++ i){
				error_bin_idx(i) = std::min(n_bins, static_cast<int>(std::floor(err_values(i) / bin_size)));
			}
		}

		Eigen::VectorXd bin_count(n_bins + 1);
		bin_count.setZero();

		for(int i = 0; i < n_elems; ++ i)
		{
			bin_count( error_bin_idx(i) ) += 1;
		}

		bin_count /= bin_count.sum();

		for(int i = 0; i < n_bins; ++ i)
		{
			double lower_val = bin_size * i;
			double upper_val = bin_size * (i+1);
			std::cout << lower_val << " to " << upper_val << ": " << bin_count(i) * 100 << "%" << std::endl;
		}

		std::cout << "Over " << bin_size * n_bins << ": " << bin_count(n_bins) * 100 << "%" << std::endl;
	}

	// Show statistics of the deviation between the new normals and target normals (in degrees)
	void show_normal_error_statistics(const TriMesh &mesh, const Matrix3X &target_normals, int bin_size_in_degrees, int n_bins)
	{
		// Compute the normal deviation angle, and the number of flipped normals
		int n_faces = mesh.n_faces();
		Eigen::VectorXd face_normal_error_angle(n_faces);

		for(int i = 0; i < n_faces; ++ i)
		{
			Eigen::Vector3d normal = to_eigen_vec3d(mesh.calc_face_normal(TriMesh::FaceHandle(i)));
			double error_angle_cos = std::max(-1.0, std::min(1.0, normal.dot(target_normals.col(i))));
			face_normal_error_angle(i) = std::acos(error_angle_cos);
		}

		face_normal_error_angle *= (180 / M_PI);

		std::cout << "Statistics of deviation between new normals and target normals:" << std::endl;
		std::cout << "===============================================================" << std::endl;
		show_error_statistics(face_normal_error_angle, bin_size_in_degrees, n_bins);
	}
};

}



#endif /* MESHNORMALFILTER_H_ */
