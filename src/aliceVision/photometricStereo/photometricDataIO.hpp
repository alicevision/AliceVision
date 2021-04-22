#ifndef DATAIO_HPP_INCLUDED
#define DATAIO_HPP_INCLUDED
#include <string>
#include <vector>

void loadLightIntensities(const std::string& intFileName, const std::vector<int>& usedPictures, std::vector<std::array<float, 3>>& intList);

void loadLightDirections(const std::string& dirFileName, const std::vector<int>& usedPictures, const Eigen::MatrixXf& convertionMatrix, Eigen::MatrixXf& lightMat);

void loadMask(std::string const& maskName, aliceVision::image::Image<float>& mask);

void getIndMask(aliceVision::image::Image<float> const& mask, std::vector<int>& indexes);

void intensityScaling(std::array<float, 3> const& intensities, aliceVision::image::Image<aliceVision::image::RGBfColor>& imageToScale);

void image2PsMatrix(const aliceVision::image::Image<aliceVision::image::RGBfColor>& imageIn, Eigen::MatrixXf& imageOut);

void applyMask(const Eigen::MatrixXf& inputMatrix, const std::vector<int>& maskIndexes, Eigen::MatrixXf& maskedMatrix);

void normals2picture(const Eigen::MatrixXf& normalsMatrix, aliceVision::image::Image<aliceVision::image::RGBfColor>& normalsIm);

void convertNormalMap2png(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normalsIm, aliceVision::image::Image<aliceVision::image::RGBColor>& normalsImPNG);

void readMatrix(const std::string& fileName, Eigen::MatrixXf& matrix);

#endif // DATAIO_HPP_INCLUDED
