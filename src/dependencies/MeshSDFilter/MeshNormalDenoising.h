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


#ifndef MESHNORMALDENOISING_H_
#define MESHNORMALDENOISING_H_

#include "MeshNormalFilter.h"
#include <algorithm>


namespace SDFilter
{

class MeshDenoisingParameters: public MeshFilterParameters
{
public:
	MeshDenoisingParameters()
	:outer_iterations(5){}

	virtual ~MeshDenoisingParameters(){}

	int outer_iterations;	// Number of outer iterations where the SD filter is applied

	virtual bool valid_parameters() const
	{
		if(!MeshFilterParameters::valid_parameters()){
			return false;
		}

		if(outer_iterations <= 0){
			std::cerr << "Error: outer_iterations must be positive" << std::endl;
			return false;
		}

		return true;
	}

protected:

	virtual bool load_option(const OptionInterpreter &opt)
	{
		return MeshFilterParameters::load_option(opt) ||
				opt.load("OuterIterations", outer_iterations);
	}

	virtual void output_options()
	{
		Parameters::output_options();
		std::cout << "Mesh denoising outer iterations: " << outer_iterations << std::endl;
	}
};


class MeshNormalDenoising : public MeshNormalFilter
{
public:
	MeshNormalDenoising(const TriMesh &mesh)
	:MeshNormalFilter(mesh)
	{
		print_progress_ = false;
		print_diagnostic_info_ = false;
		print_timing_ = false;
		print_error_evaluation_ = false;
	}


	bool denoise(const MeshDenoisingParameters &param, TriMesh &output_mesh)
	{
		assert(param.valid_parameters());

		std::cout << "Denoising started" << std::endl;

		Timer timer;
		Timer::EventID denoise_begin_time = timer.get_time();

		for(int i = 0; i < param.outer_iterations; ++ i)
		{
			std::cout << "Outer iteration " << (i+1) << "..." << std::endl;

			if(!filter(param, output_mesh)){
				std::cerr << "Unable to perform normal filter. Denoising aborted." << std::endl;
				return false;
			}

			// Use the filtered mesh as the input mesh for the next outer iteration
			set_mesh(output_mesh, false);
		}

		Timer::EventID denoise_end_time = timer.get_time();
		std::cout << "Denoising completed, timing: " <<
				timer.elapsed_time(denoise_begin_time, denoise_end_time) << std::endl;

		return true;
	}


protected:

	virtual void get_initial_data(Eigen::MatrixXd &guidance_normals, Eigen::MatrixXd &init_normals, Eigen::VectorXd &area_weights)
	{
		// Call the base class method to fill in initial data
		MeshNormalFilter::get_initial_data(guidance_normals, init_normals, area_weights);

		// Update the guidance to patch-based normals

		int face_count = mesh_.n_faces(), edge_count = mesh_.n_edges(), vtx_count = mesh_.n_vertices();

		std::vector<double> edge_saliency(edge_count, 0); // Pre-computed edge saliency, defined as difference between adjacent normals

		std::vector< std::vector<int> > adj_faces_per_vtx(vtx_count);
		std::vector< std::vector<int> > adj_nonboundary_edges_per_vtx(vtx_count);
		std::vector< std::vector<int> > neighborhood_faces_per_face(face_count);

		std::vector<double> patch_normal_consistency(face_count, 0);
		Eigen::Matrix3Xd patch_avg_normal(3, face_count);

		double epsilon = 1e-9;

		OMP_PARALLEL
		{
			OMP_FOR
			for(int i = 0; i < edge_count; ++ i)
			{
				TriMesh::EdgeHandle eh(i);

				if(!mesh_.is_boundary(eh))
				{
					int f1 = mesh_.face_handle(mesh_.halfedge_handle(eh, 0)).idx();
					int f2 = mesh_.face_handle(mesh_.halfedge_handle(eh, 1)).idx();
					edge_saliency[i] = (init_normals.col(f1) - init_normals.col(f2)).norm();
				}
			}

			OMP_FOR
			for(int i = 0; i < vtx_count; ++ i)
			{
				// Collect neighboring faces and non-boundary edges for each vertex

				TriMesh::VertexHandle vh(i);

				for(TriMesh::ConstVertexFaceIter cvf_it = mesh_.cvf_iter(vh); cvf_it.is_valid(); ++ cvf_it){
					adj_faces_per_vtx[i].push_back(cvf_it->idx());
				}

				for(TriMesh::ConstVertexEdgeIter cve_it = mesh_.cve_iter(vh); cve_it.is_valid(); ++ cve_it){
					if(!mesh_.is_boundary(*cve_it)){
						adj_nonboundary_edges_per_vtx[i].push_back(cve_it->idx());
					}
				}
			}

			OMP_FOR
			for(int i = 0; i < face_count; ++ i)
			{
				// Each candidate patch is associated with a face at its center.
				// We collect all the faces and non-boundary edges within such a patch

				TriMesh::FaceHandle fh(i);
				std::vector<int> faces_in_patch, edges_in_patch;

				for(TriMesh::ConstFaceVertexIter cfv_it = mesh_.cfv_iter(fh); cfv_it.is_valid(); ++ cfv_it){
					int vtx_idx = cfv_it->idx();
					faces_in_patch.insert(faces_in_patch.end(), adj_faces_per_vtx[vtx_idx].begin(), adj_faces_per_vtx[vtx_idx].end());
					edges_in_patch.insert(edges_in_patch.end(), adj_nonboundary_edges_per_vtx[vtx_idx].begin(), adj_nonboundary_edges_per_vtx[vtx_idx].end());
				}

				// Sort and remove duplicates
				std::sort(faces_in_patch.begin(), faces_in_patch.end());
				faces_in_patch.erase(std::unique(faces_in_patch.begin(), faces_in_patch.end()), faces_in_patch.end());
				neighborhood_faces_per_face[i] = faces_in_patch;

				std::sort(edges_in_patch.begin(), edges_in_patch.end());
				edges_in_patch.erase(std::unique(edges_in_patch.begin(), edges_in_patch.end()), edges_in_patch.end());

				// Collect face normals and edge saliency values from the patch, and compute patch normal consistency value and average normal
				int n_faces_in_patch = faces_in_patch.size();
				int n_edges_in_patch = edges_in_patch.size();

				Eigen::Matrix3Xd face_normals(3, n_faces_in_patch);
				Eigen::VectorXd face_area(n_faces_in_patch);
				Eigen::VectorXd edge_saliency_values(n_edges_in_patch);

				for(int k = 0; k < n_edges_in_patch; ++ k){
					edge_saliency_values(k) = edge_saliency[ edges_in_patch[k] ];
				}

				if(n_edges_in_patch > 0){
					patch_normal_consistency[i] = edge_saliency_values.maxCoeff() / (epsilon + edge_saliency_values.sum());
				}

				for(int k = 0; k < n_faces_in_patch; ++ k)
				{
					int f_k = faces_in_patch[k];
					face_normals.col(k) = init_normals.col(f_k);
					face_area(k) = area_weights(f_k);
				}

				// Find the max normal difference within a patch
				double max_normal_diff = 0;
				for(int k = 0; k < n_faces_in_patch - 1; ++ k)
				{
					Eigen::Matrix3Xd N = face_normals.block(0, k+1, 3, n_faces_in_patch-k-1);
					N.colwise() -= face_normals.col(k);
					double max_diff = N.colwise().norm().maxCoeff();
					max_normal_diff = std::max(max_normal_diff, max_diff);
				}

				patch_normal_consistency[i] *= max_normal_diff;
				patch_avg_normal.col(i) = (face_normals * face_area).normalized();
			}

			OMP_FOR
			for(int i = 0; i < face_count; ++ i)
			{
				// For each face, select the patch with the most consistent normals to construct its guidance
				std::vector<int> &neighborhood_faces = neighborhood_faces_per_face[i];
				assert(!neighborhood_faces.empty());

				Eigen::VectorXd patch_scores(neighborhood_faces.size());
				for(int k= 0; k < static_cast<int>(neighborhood_faces.size()); ++ k){
					patch_scores(k) = patch_normal_consistency[neighborhood_faces[k]];
				}

				int best_score_idx = -1;
				patch_scores.minCoeff(&best_score_idx);
				guidance_normals.col(i) = patch_avg_normal.col( neighborhood_faces[best_score_idx] );
			}
		}
	}
};

}



#endif /* MESHNORMALDENOISING_H_ */
