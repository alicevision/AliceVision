#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>


namespace aliceVision {
namespace inverseRendering {

using Eigen::MatrixXf;
using namespace aliceVision::image;

struct AugmentedNormals {
	MatrixXf nx; 
	MatrixXf ny; 
	MatrixXf nz;
	MatrixXf ambiant; 
	MatrixXf nx_ny; 
	MatrixXf nx_nz;
	MatrixXf ny_nz; 
	MatrixXf nx2_ny2; 
	MatrixXf nz2;
};

void getAugmentedNormals(AugmentedNormals& agmNormals, const Image<RGBfColor>& normals);

}
}
