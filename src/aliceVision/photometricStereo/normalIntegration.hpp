#ifndef NORMALINTEGRATION_HPP_INCLUDED
#define NORMALINTEGRATION_HPP_INCLUDED
#include <string>
#include <vector>

void normal2PQ(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, Eigen::MatrixXd& p, Eigen::MatrixXd& q, bool perspective, Eigen::Matrix3f K);

void getDivergenceField(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void setBoundaryConditions(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void normalIntegration(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<float>& solution, bool perspective, Eigen::Matrix3f K);

#endif // NORMALINTEGRATION_HPP_INCLUDED
