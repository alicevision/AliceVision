#pragma once

#include <aliceVision/camera/camera.hpp>
#include <ceres/rotation.h>

namespace aliceVision {
namespace sfm {

/**
 * @brief Ceres functor for a prior on the relative rotation between two views.
 *
 *  The residual is the angle-axis of (two_R_one_estimated * two_R_one_prior^T), in degrees.
 *  Only the rotation part of the pose blocks is used.
 *
 *  Data parameter blocks are the following <3,6,6>
 *  - 3 => dimension of the residuals,
 *  - 6 => the pose block of the first view
 *  - 6 => the pose block of the second view
 */
struct RotationPriorErrorFunctor
{
    /**
     * @param[in] two_R_one prior on the rotation from the first view frame to the second view frame
     */
    explicit RotationPriorErrorFunctor(const Eigen::Matrix3d& two_R_one)
      : m_two_R_one(two_R_one)
    {}

    /**
     * @param[in] cam_R1 pose block of the first view: 3 for rotation (angle axis), 3 for center
     * @param[in] cam_R2 pose block of the second view: 3 for rotation (angle axis), 3 for center
     * @param[out] out_residuals rotation error as angle axis, in degrees
     */
    template<typename T>
    bool operator()(const T* const cam_R1, const T* const cam_R2, T* out_residuals) const
    {
        Eigen::Matrix<T, 3, 3> oneRo, twoRo, twoRone, R_error;

        ceres::AngleAxisToRotationMatrix(cam_R1, oneRo.data());
        ceres::AngleAxisToRotationMatrix(cam_R2, twoRo.data());
        twoRone = twoRo * oneRo.transpose();
        R_error = twoRone * m_two_R_one.transpose();

        ceres::RotationMatrixToAngleAxis(R_error.data(), out_residuals);

        out_residuals[0] *= 180.0 / M_PI;
        out_residuals[1] *= 180.0 / M_PI;
        out_residuals[2] *= 180.0 / M_PI;

        return true;
    }

    Eigen::Matrix3d m_two_R_one;
};

}  // namespace sfm
}  // namespace aliceVision
