#ifndef DATAIO_HPP_INCLUDED
#define DATAIO_HPP_INCLUDED
#include <string>
#include <vector>

void loadLightIntensities(const std::string& intFileName, const std::vector<int>& usedPictures, std::vector<std::array<float, 3>>& intList);

void loadLightDirections(const std::string& dirFileName, const std::vector<int>& usedPictures, Eigen::MatrixXf& lightMat);

void loadMask(std::string const& maskName, aliceVision::image::Image<float>& mask);
#endif // DATAIO_HPP_INCLUDED
