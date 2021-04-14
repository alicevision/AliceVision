#ifndef NORMALINTEGRATION_HPP_INCLUDED
#define NORMALINTEGRATION_HPP_INCLUDED
#include <string>
#include <vector>

void normal2PQ(const aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, Eigen::MatrixXf& p, Eigen::MatrixXf& q);

void getDivergenceField(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void setBoundaryConditions(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void normalIntegration(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<float>& solution);

#endif // NORMALINTEGRATION_HPP_INCLUDED
