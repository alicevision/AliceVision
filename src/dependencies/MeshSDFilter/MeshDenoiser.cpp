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


#include "MeshTypes.h"
#include "MeshNormalDenoising.h"
#include <iostream>
#include <fstream>
#include <vector>



int main(int argc, char **argv)
{
	if(argc != 4)
	{
		std::cout << "Usage:	MeshDenoiser  OPTION_FILE  INPUT_MESH  OUTPUT_MESH" << std::endl;
		return 1;
	}

	TriMesh mesh;
    if(!OpenMesh::IO::read_mesh(mesh, argv[2]))
    {
    	std::cerr << "Error: unable to read input mesh from the file " << argv[2] << std::endl;
    	return 1;
    }

	#ifdef USE_OPENMP
    Eigen::initParallel();
	#endif

    // Load option file
    SDFilter::MeshDenoisingParameters param;
    if(!param.load(argv[1])){
    	std::cerr << "Error: unable to load option file " << argv[1] << std::endl;
    	return 1;
    }
    if(!param.valid_parameters()){
    	std::cerr << "Invalid filter options. Aborting..." << std::endl;
    	return 1;
    }
    param.output();


    // Normalize the input mesh
    Eigen::Vector3d original_center;
    double original_scale;
    SDFilter::normalize_mesh(mesh, original_center, original_scale);

    // Filter the normals and construct the output mesh
    SDFilter::MeshNormalDenoising denoiser(mesh);
    TriMesh output_mesh;
    denoiser.denoise(param, output_mesh);

    SDFilter::restore_mesh(output_mesh, original_center, original_scale);

    // Save output mesh
	if(!SDFilter::write_mesh_high_accuracy(output_mesh, argv[3])){
		std::cerr << "Error: unable to save the result mesh to file " << argv[3] << std::endl;
		return 1;
	}

	return 0;
}





