// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>

namespace aliceVision {
namespace sfm {

class CostProjectionParallaxSimple : public ceres::CostFunction {
public:
  CostProjectionParallaxSimple(const sfmData::Observation& measured, const std::shared_ptr<camera::IntrinsicBase> & intrinsics) : _measured(measured), _intrinsics(intrinsics)
  {
    set_num_residuals(3);

    mutable_parameter_block_sizes()->push_back(16);
    mutable_parameter_block_sizes()->push_back(16);
    mutable_parameter_block_sizes()->push_back(16);
    mutable_parameter_block_sizes()->push_back(intrinsics->getParams().size());    
    mutable_parameter_block_sizes()->push_back(5);    
  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override
  {
    const double * parameter_pose = parameters[0];
    const double * parameter_primary = parameters[1];
    const double * parameter_secondary = parameters[2];
    const double * parameter_intrinsics = parameters[3];
    const double * parameter_landmark = parameters[4];

    const Eigen::Map<const SE3::Matrix> pose_T_world(parameter_pose);
    const Eigen::Map<const SE3::Matrix> primary_T_world(parameter_primary);
    const Eigen::Map<const SE3::Matrix> secondary_T_world(parameter_secondary);

    const Eigen::Map<const Eigen::Vector<double, 5>> pt(parameter_landmark);

    const geometry::Pose3 posecur(pose_T_world);
    const geometry::Pose3 pose1(primary_T_world);
    const geometry::Pose3 pose2(secondary_T_world);

    Eigen::Matrix3d pose_R_world = pose_T_world.block<3, 3>(0, 0);
    Eigen::Matrix3d primary_R_world = primary_T_world.block<3, 3>(0, 0);
    Eigen::Matrix3d secondary_R_world = secondary_T_world.block<3, 3>(0, 0);
        
    Eigen::Vector3d pose_t_world = pose_T_world.block<3, 1>(0, 3);
    Eigen::Vector3d primary_t_world = primary_T_world.block<3, 1>(0, 3);
    Eigen::Vector3d secondary_t_world = secondary_T_world.block<3, 1>(0, 3);

    Eigen::Vector3d world_t_pose = - pose_R_world.transpose() * pose_t_world;
    Eigen::Vector3d world_t_primary = - primary_R_world.transpose() * primary_t_world;
    Eigen::Vector3d world_t_secondary = - secondary_R_world.transpose() * secondary_t_world;

    Vec3 primary_n = pt.block<3, 1>(0, 0);
    double costh = pt(3);
    double sinth = pt(4);

    Vec3 world_n = primary_R_world.transpose() * primary_n;

    Vec3 a = world_t_primary - world_t_secondary;
    Vec3 b = world_t_primary - world_t_pose;

    Vec3 p3 = SO3::skew(a) * world_n;
    double p3norm = p3.norm();
    double p4 = a.dot(world_n);

    Vec3 h1 = p3norm * world_n;
    Vec3 h2 = b - p4 * world_n;
    Vec3 cur_n_unnorm = costh * h1 + sinth * h2;
    
    
    double normcur = cur_n_unnorm.norm();
    double invnormcur = 1.0 / normcur;
    Vec3 cur_n = cur_n_unnorm * invnormcur;

    Vec3 mes_n = _intrinsics->toUnitSphere(_intrinsics->ima2cam(_measured.x));
    Vec3 world_mes_n = pose_R_world.transpose() * mes_n;


    residuals[0] = cur_n(0) - world_mes_n(0);
    residuals[1] = cur_n(1) - world_mes_n(1);
    residuals[2] = cur_n(2) - world_mes_n(2);

    if (jacobians == nullptr)
    {
        return true;
    }

    //derivative of  cur_n wrt cur_n_unnorm
    Eigen::Matrix<double, 1, 3> d_normcur_d_cur_n_unnorm;
    d_normcur_d_cur_n_unnorm(0, 0) =  cur_n_unnorm(0) / normcur;
    d_normcur_d_cur_n_unnorm(0, 1) =  cur_n_unnorm(1) / normcur;
    d_normcur_d_cur_n_unnorm(0, 2) =  cur_n_unnorm(2) / normcur;
    double d_invnormcur_d_normcur = -1.0 / (normcur * normcur);
    Eigen::Matrix<double, 3, 3> d_cur_n_d_cur_n_unnorm = Eigen::Matrix3d::Identity() * invnormcur + cur_n_unnorm * (d_invnormcur_d_normcur * d_normcur_d_cur_n_unnorm);

    //Derivative wrt h1,h2
    Eigen::Matrix<double, 3, 3> d_cur_n_unnorm_d_h1 = costh * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> d_cur_n_unnorm_d_h2 = sinth * Eigen::Matrix3d::Identity();

    //Derivative of p3 wrt its norm
    Eigen::Matrix<double, 1, 3> d_p3norm_d_p3;
    d_p3norm_d_p3(0, 0) =  p3(0) / p3norm;
    d_p3norm_d_p3(0, 1) =  p3(1) / p3norm;
    d_p3norm_d_p3(0, 2) =  p3(2) / p3norm;
    
    Eigen::Matrix<double, 3, 9> d_world_mes_n_d_pose_R_world = getJacobian_AtB_wrt_A<3,3,1>(pose_R_world, mes_n);

    Eigen::Matrix<double, 3, 9> d_world_t_pose_d_pose_R_world = - getJacobian_AtB_wrt_A<3,3,1>(pose_R_world, pose_t_world);
    Eigen::Matrix<double, 3, 3> d_world_t_pose_d_pose_t_world = - getJacobian_AB_wrt_B<3,3,1>(pose_R_world.transpose(), pose_t_world);
    Eigen::Matrix<double, 3, 9> d_world_t_primary_d_primary_R_world = - getJacobian_AtB_wrt_A<3,3,1>(primary_R_world, primary_t_world);
    Eigen::Matrix<double, 3, 3> d_world_t_primary_d_primary_t_world = - getJacobian_AB_wrt_B<3,3,1>(primary_R_world.transpose(), primary_t_world);
    Eigen::Matrix<double, 3, 9> d_world_t_secondary_d_secondary_R_world = - getJacobian_AtB_wrt_A<3,3,1>(secondary_R_world, secondary_t_world);
    Eigen::Matrix<double, 3, 3> d_world_t_secondary_d_secondary_t_world = - getJacobian_AB_wrt_B<3,3,1>(secondary_R_world.transpose(), secondary_t_world);

    Eigen::Matrix<double, 3, 3> d_a_d_world_t_primary = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> d_b_d_world_t_primary = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> d_a_d_world_t_secondary = - Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> d_b_d_world_t_pose = - Eigen::Matrix3d::Identity();

    //[a]xb = -[b]xa
    Eigen::Matrix<double, 3, 3> d_p3_d_a = - SO3::skew(world_n);

    //double p4 = a.dot(world_n);
    Eigen::Matrix<double, 1, 3> d_p4_d_a = world_n;

    //Vec3 h1 = p3norm * world_n;
    Eigen::Matrix<double, 3, 1> d_h1_d_p3norm = world_n;

    Eigen::Matrix<double, 3, 3> d_h2_d_b = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> d_h2_d_p4 = - world_n;
    
    //Wrt pose
    if (jacobians[0] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 16, Eigen::RowMajor>> J(jacobians[0]);

        J.fill(0);

        Eigen::Matrix<double, 3, 9> Jpose_R_world = (d_cur_n_d_cur_n_unnorm * d_cur_n_unnorm_d_h2 * d_h2_d_b * d_b_d_world_t_pose * d_world_t_pose_d_pose_R_world) - d_world_mes_n_d_pose_R_world;
        Eigen::Matrix<double, 3, 3> Jpose_t_world = (d_cur_n_d_cur_n_unnorm * d_cur_n_unnorm_d_h2 * d_h2_d_b * d_b_d_world_t_pose * d_world_t_pose_d_pose_t_world);

        J.block<3, 3>(0, 0) = Jpose_R_world.block<3, 3>(0, 0);
        J.block<3, 3>(0, 4) = Jpose_R_world.block<3, 3>(0, 3);
        J.block<3, 3>(0, 8) = Jpose_R_world.block<3, 3>(0, 6);
        J.block<3, 3>(0, 12) = Jpose_t_world;

        J = J * getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), pose_T_world);
    }

    //Wrt primary
    if (jacobians[1] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 16, Eigen::RowMajor>> J(jacobians[1]);

        J.fill(0);

        Eigen::Matrix<double, 3, 9> Jprimary_R_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_R_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_R_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_b * d_b_d_world_t_primary * d_world_t_primary_d_primary_R_world)
            );
         
        Eigen::Matrix<double, 3, 3> Jprimary_t_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_t_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_t_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_b * d_b_d_world_t_primary * d_world_t_primary_d_primary_t_world)
            );
         

        J.block<3, 3>(0, 0) = Jprimary_R_world.block<3, 3>(0, 0);
        J.block<3, 3>(0, 4) = Jprimary_R_world.block<3, 3>(0, 3);
        J.block<3, 3>(0, 8) = Jprimary_R_world.block<3, 3>(0, 6);
        J.block<3, 3>(0, 12) = Jprimary_t_world;

        J = J * getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), primary_T_world);
    }

    //Wrt secondary
    if (jacobians[2] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 16, Eigen::RowMajor>> J(jacobians[2]);

        J.fill(0);

        Eigen::Matrix<double, 3, 9> Jsecondary_R_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a)  + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a) 
            )
            * d_a_d_world_t_secondary * d_world_t_secondary_d_secondary_R_world; 

        Eigen::Matrix<double, 3, 3> Jsecondary_t_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a)  + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a) 
            )
            * d_a_d_world_t_secondary * d_world_t_secondary_d_secondary_t_world;


        J.block<3, 3>(0, 0) = Jsecondary_R_world.block<3, 3>(0, 0);
        J.block<3, 3>(0, 4) = Jsecondary_R_world.block<3, 3>(0, 3);
        J.block<3, 3>(0, 8) = Jsecondary_R_world.block<3, 3>(0, 6);
        J.block<3, 3>(0, 12) = Jsecondary_t_world;

        J = J * getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), secondary_T_world);
    }

    if (jacobians[4] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 5, Eigen::RowMajor>> J(jacobians[4]);

        J.fill(0);

        
        Eigen::Matrix<double, 3, 3> d_p3_d_world_n = SO3::skew(a);
        Eigen::Matrix<double, 1, 3> d_p4_d_world_n = a.transpose();

        Eigen::Matrix<double, 3, 1> d_cur_n_unnorm_d_p4 = - sinth * world_n;

        Eigen::Matrix<double, 3, 2> d_cur_n_unnorm_d_rot;
        d_cur_n_unnorm_d_rot.col(0) = h1;
        d_cur_n_unnorm_d_rot.col(1) = h2;

        Eigen::Matrix<double, 2, 1> d_rot_d_angle;
        d_rot_d_angle(0, 0) = -sinth;
        d_rot_d_angle(1, 0) = costh;

        Eigen::Matrix<double, 3, 3> d_h1_d_world_n = p3norm * Eigen::Matrix3d::Identity() +  world_n * (d_p3norm_d_p3 * d_p3_d_world_n);
        Eigen::Matrix<double, 3, 3> d_h2_d_world_n = - (p4 * Eigen::Matrix3d::Identity() + world_n * d_p4_d_world_n);

    
        J.block<3, 2>(0, 3) = d_cur_n_d_cur_n_unnorm * d_cur_n_unnorm_d_rot;
        J.block<3, 3>(0, 0) = d_cur_n_d_cur_n_unnorm * (d_cur_n_unnorm_d_h1 * d_h1_d_world_n + d_cur_n_unnorm_d_h2 * d_h2_d_world_n);
    }

    return true;
  }

private:
  const sfmData::Observation & _measured;
  const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
};


class CostProjectionParallaxSecondarySimple : public ceres::CostFunction {
public:
  CostProjectionParallaxSecondarySimple(const sfmData::Observation& measured, const std::shared_ptr<camera::IntrinsicBase> & intrinsics) : _measured(measured), _intrinsics(intrinsics)
  {
    set_num_residuals(3);

    mutable_parameter_block_sizes()->push_back(16);
    mutable_parameter_block_sizes()->push_back(16);
    mutable_parameter_block_sizes()->push_back(intrinsics->getParams().size());    
    mutable_parameter_block_sizes()->push_back(5);    
  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override
  {
    const double * parameter_pose = parameters[0];
    const double * parameter_primary = parameters[1];
    const double * parameter_intrinsics = parameters[2];
    const double * parameter_landmark = parameters[3];

    const Eigen::Map<const SE3::Matrix> pose_T_world(parameter_pose);
    const Eigen::Map<const SE3::Matrix> primary_T_world(parameter_primary);

    const Eigen::Map<const Eigen::Vector<double, 5>> pt(parameter_landmark);

    const geometry::Pose3 posecur(pose_T_world);
    const geometry::Pose3 pose1(primary_T_world);

    Eigen::Matrix3d pose_R_world = pose_T_world.block<3, 3>(0, 0);
    Eigen::Matrix3d primary_R_world = primary_T_world.block<3, 3>(0, 0);
        
    Eigen::Vector3d pose_t_world = pose_T_world.block<3, 1>(0, 3);
    Eigen::Vector3d primary_t_world = primary_T_world.block<3, 1>(0, 3);

    Eigen::Vector3d world_t_pose = - pose_R_world.transpose() * pose_t_world;
    Eigen::Vector3d world_t_primary = - primary_R_world.transpose() * primary_t_world;

    Vec3 primary_n = pt.block<3, 1>(0, 0);
    double costh = pt(3);
    double sinth = pt(4);

    Vec3 world_n = primary_R_world.transpose() * primary_n;

    Vec3 a = world_t_primary - world_t_pose;

    Vec3 p3 = SO3::skew(a) * world_n;
    double p3norm = p3.norm();
    double p4 = a.dot(world_n);

    Vec3 h1 = p3norm * world_n;
    Vec3 h2 = a - p4 * world_n;
    Vec3 cur_n_unnorm = costh * h1 + sinth * h2;
    
    
    double normcur = cur_n_unnorm.norm();
    double invnormcur = 1.0 / normcur;
    Vec3 cur_n = cur_n_unnorm * invnormcur;

    Vec3 mes_n = _intrinsics->toUnitSphere(_intrinsics->ima2cam(_measured.x));
    Vec3 world_mes_n = pose_R_world.transpose() * mes_n;


    residuals[0] = cur_n(0) - world_mes_n(0);
    residuals[1] = cur_n(1) - world_mes_n(1);
    residuals[2] = cur_n(2) - world_mes_n(2);

    if (jacobians == nullptr)
    {
        return true;
    }

    //derivative of  cur_n wrt cur_n_unnorm
    Eigen::Matrix<double, 1, 3> d_normcur_d_cur_n_unnorm;
    d_normcur_d_cur_n_unnorm(0, 0) =  cur_n_unnorm(0) / normcur;
    d_normcur_d_cur_n_unnorm(0, 1) =  cur_n_unnorm(1) / normcur;
    d_normcur_d_cur_n_unnorm(0, 2) =  cur_n_unnorm(2) / normcur;
    double d_invnormcur_d_normcur = -1.0 / (normcur * normcur);
    Eigen::Matrix<double, 3, 3> d_cur_n_d_cur_n_unnorm = Eigen::Matrix3d::Identity() * invnormcur + cur_n_unnorm * (d_invnormcur_d_normcur * d_normcur_d_cur_n_unnorm);

    //Derivative wrt h1,h2
    Eigen::Matrix<double, 3, 3> d_cur_n_unnorm_d_h1 = costh * Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> d_cur_n_unnorm_d_h2 = sinth * Eigen::Matrix3d::Identity();

    //Derivative of p3 wrt its norm
    Eigen::Matrix<double, 1, 3> d_p3norm_d_p3;
    d_p3norm_d_p3(0, 0) =  p3(0) / p3norm;
    d_p3norm_d_p3(0, 1) =  p3(1) / p3norm;
    d_p3norm_d_p3(0, 2) =  p3(2) / p3norm;
    
    Eigen::Matrix<double, 3, 9> d_world_mes_n_d_pose_R_world = getJacobian_AtB_wrt_A<3,3,1>(pose_R_world, mes_n);

    Eigen::Matrix<double, 3, 9> d_world_t_pose_d_pose_R_world = - getJacobian_AtB_wrt_A<3,3,1>(pose_R_world, pose_t_world);
    Eigen::Matrix<double, 3, 3> d_world_t_pose_d_pose_t_world = - getJacobian_AB_wrt_B<3,3,1>(pose_R_world.transpose(), pose_t_world);
    Eigen::Matrix<double, 3, 9> d_world_t_primary_d_primary_R_world = - getJacobian_AtB_wrt_A<3,3,1>(primary_R_world, primary_t_world);
    Eigen::Matrix<double, 3, 3> d_world_t_primary_d_primary_t_world = - getJacobian_AB_wrt_B<3,3,1>(primary_R_world.transpose(), primary_t_world);

    Eigen::Matrix<double, 3, 3> d_a_d_world_t_primary = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 3> d_a_d_world_t_pose = - Eigen::Matrix3d::Identity();

    //[a]xb = -[b]xa
    Eigen::Matrix<double, 3, 3> d_p3_d_a = - SO3::skew(world_n);

    //double p4 = a.dot(world_n);
    Eigen::Matrix<double, 1, 3> d_p4_d_a = world_n;

    //Vec3 h1 = p3norm * world_n;
    Eigen::Matrix<double, 3, 1> d_h1_d_p3norm = world_n;

    Eigen::Matrix<double, 3, 3> d_h2_d_a = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> d_h2_d_p4 = - world_n;
    
    //Wrt pose
    if (jacobians[0] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 16, Eigen::RowMajor>> J(jacobians[0]);

        J.fill(0);

        Eigen::Matrix<double, 3, 9> Jpose_R_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a * d_a_d_world_t_pose * d_world_t_pose_d_pose_R_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a * d_a_d_world_t_pose * d_world_t_pose_d_pose_R_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_a * d_a_d_world_t_pose * d_world_t_pose_d_pose_R_world)
            ) - d_world_mes_n_d_pose_R_world;
         
        Eigen::Matrix<double, 3, 3> Jpose_t_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a * d_a_d_world_t_pose * d_world_t_pose_d_pose_t_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a * d_a_d_world_t_pose * d_world_t_pose_d_pose_t_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_a * d_a_d_world_t_pose * d_world_t_pose_d_pose_t_world)
            );

        J.block<3, 3>(0, 0) = Jpose_R_world.block<3, 3>(0, 0);
        J.block<3, 3>(0, 4) = Jpose_R_world.block<3, 3>(0, 3);
        J.block<3, 3>(0, 8) = Jpose_R_world.block<3, 3>(0, 6);
        J.block<3, 3>(0, 12) = Jpose_t_world;

        J = J * getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), pose_T_world);
    }

    //Wrt primary
    if (jacobians[1] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 16, Eigen::RowMajor>> J(jacobians[1]);

        J.fill(0);

        Eigen::Matrix<double, 3, 9> Jprimary_R_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_R_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_R_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_R_world)
            );
         
        Eigen::Matrix<double, 3, 3> Jprimary_t_world = d_cur_n_d_cur_n_unnorm * 
            (
                (d_cur_n_unnorm_d_h1 * d_h1_d_p3norm * d_p3norm_d_p3 * d_p3_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_t_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_p4 * d_p4_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_t_world) + 
                (d_cur_n_unnorm_d_h2 * d_h2_d_a * d_a_d_world_t_primary * d_world_t_primary_d_primary_t_world)
            );
         

        J.block<3, 3>(0, 0) = Jprimary_R_world.block<3, 3>(0, 0);
        J.block<3, 3>(0, 4) = Jprimary_R_world.block<3, 3>(0, 3);
        J.block<3, 3>(0, 8) = Jprimary_R_world.block<3, 3>(0, 6);
        J.block<3, 3>(0, 12) = Jprimary_t_world;

        J = J * getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), primary_T_world);
    }

    if (jacobians[3] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 5, Eigen::RowMajor>> J(jacobians[3]);

        J.fill(0);

        
        Eigen::Matrix<double, 3, 3> d_p3_d_world_n = SO3::skew(a);
        Eigen::Matrix<double, 1, 3> d_p4_d_world_n = a.transpose();

        Eigen::Matrix<double, 3, 1> d_cur_n_unnorm_d_p4 = - sinth * world_n;

        Eigen::Matrix<double, 3, 2> d_cur_n_unnorm_d_rot;
        d_cur_n_unnorm_d_rot.col(0) = h1;
        d_cur_n_unnorm_d_rot.col(1) = h2;

        Eigen::Matrix<double, 2, 1> d_rot_d_angle;
        d_rot_d_angle(0, 0) = -sinth;
        d_rot_d_angle(1, 0) = costh;

        Eigen::Matrix<double, 3, 3> d_h1_d_world_n = p3norm * Eigen::Matrix3d::Identity() +  world_n * (d_p3norm_d_p3 * d_p3_d_world_n);
        Eigen::Matrix<double, 3, 3> d_h2_d_world_n = - (p4 * Eigen::Matrix3d::Identity() + world_n * d_p4_d_world_n);

    
        J.block<3, 2>(0, 3) = d_cur_n_d_cur_n_unnorm * d_cur_n_unnorm_d_rot;
        J.block<3, 3>(0, 0) = d_cur_n_d_cur_n_unnorm * (d_cur_n_unnorm_d_h1 * d_h1_d_world_n + d_cur_n_unnorm_d_h2 * d_h2_d_world_n);
    }

    return true;
  }

private:
  const sfmData::Observation & _measured;
  const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
};

class CostProjectionParallaxPrimarySimple : public ceres::CostFunction {
public:
  CostProjectionParallaxPrimarySimple(const sfmData::Observation& measured, const std::shared_ptr<camera::IntrinsicBase> & intrinsics) : _measured(measured), _intrinsics(intrinsics)
  {
    set_num_residuals(3);

    mutable_parameter_block_sizes()->push_back(5);    
  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override
  {
    const double * parameter_landmark = parameters[0];

    const Eigen::Map<const Eigen::Vector<double, 5>> pt(parameter_landmark);

    Vec3 primary_n = pt.block<3, 1>(0, 0);
    Vec3 mes_n = _intrinsics->toUnitSphere(_intrinsics->ima2cam(_measured.x));


    residuals[0] = primary_n(0) - mes_n(0);
    residuals[1] = primary_n(1) - mes_n(1);
    residuals[2] = primary_n(2) - mes_n(2);

    if (jacobians == nullptr)
    {
        return true;
    }

    if (jacobians[0] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 5, Eigen::RowMajor>> J(jacobians[0]);

        J.fill(0);
        J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    }

    return true;
  }

private:
  const sfmData::Observation & _measured;
  const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
};

}
}