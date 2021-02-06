#ifndef DATAIO_HPP_INCLUDED
#define DATAIO_HPP_INCLUDED
#include <string>
#include <vector>

void loadLightIntensities(const std::string& intFileName, const std::vector<int>& usedPictures, std::vector<std::array<float, 3>>& intList);

void loadLightDirections(const std::string& dirFileName, const std::vector<int>& usedPictures, Eigen::MatrixXf& lightMat);

void loadMask(std::string const& maskName, aliceVision::image::Image<float>& mask);

void getIndMask(aliceVision::image::Image<float> const& mask, std::vector<int>& indexes);

void intensityScaling(std::array<float, 3> const& intensities, aliceVision::image::Image<aliceVision::image::RGBfColor> imageToScale);

void reshapeImage(const aliceVision::image::Image<aliceVision::image::RGBfColor>& imageIn, Eigen::MatrixXf& imageOut);

#endif // DATAIO_HPP_INCLUDED
