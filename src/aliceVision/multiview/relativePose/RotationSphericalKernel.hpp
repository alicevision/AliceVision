// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


#include <aliceVision/camera/camera.hpp>
#include <aliceVision/multiview/relativePose/Rotation3PSolver.hpp>

#include <Eigen/Dense>

namespace aliceVision {
namespace multiview {
namespace relativePose {

class RotationSphericalKernel
{
public:
    using ModelT = robustEstimation::Mat3Model;

public:
    RotationSphericalKernel(const camera::IntrinsicBase & camera1, 
                            const camera::IntrinsicBase & camera2, 
                            const std::vector<Eigen::Vector2d> & observations1, 
                            const std::vector<Eigen::Vector2d> & observations2) 
    : _camera1(camera1), 
      _camera2(camera2)
    {
        for (const auto & pt : observations1)
        {
            _liftedObservations1.push_back(camera1.toUnitSphere(camera1.removeDistortion(camera1.ima2cam(pt))));
        }

        for (const auto & pt : observations2)
        {
            _liftedObservations2.push_back(camera2.toUnitSphere(camera2.removeDistortion(camera2.ima2cam(pt))));
        }
    }

    /**
     * @brief Return the minimum number of required samples for the solver
     * @return minimum number of required samples
     */
    std::size_t getMinimumNbRequiredSamples() const
    {
        return 3;
    }

    /**
     * @brief Return the minimum number of required samples for the solver Ls
     * @return minimum number of required samples
     */
    std::size_t getMinimumNbRequiredSamplesLS() const
    {
        return 3;
    }

    /**
     * @brief Return the maximum number of models for the solver
     * @return maximum number of models
     */
    std::size_t getMaximumNbModels() const
    {
        return 1;
    }

    /**
     * @brief The number of elements in the data.
     * @return the number of elements in the data.
     */
    std::size_t nbSamples() const
    {
        return _liftedObservations1.size();
    }

    /**
     * @brief Get logalpha0, Alpha0 is used to make the error adaptive to the image size
     * @return logalpha0
     */
    double logalpha0() const
    {
        double aerr = _camera2.pixelProbability() * 2.0; 
        return log10(aerr);
    }

    double errorVectorDimension() const
    {
        return 1.0;
    }

    /**
     * @brief This function is called to estimate the model from the minimum number
     * of sample \p minSample (i.e. minimal problem solver).
     * @param[in] samples A vector containing the indices of the data to be used for
     * the minimal estimation.
     * @param[out] models The model(s) estimated by the minimal solver.
     */
    void fit(const std::vector<std::size_t>& samples, std::vector<robustEstimation::Mat3Model>& models) const
    {
        Mat x1(3, 3);
        Mat x2(3, 3);

        for (int pos = 0; pos < 3; pos++)
        {
            size_t id = samples[pos];
            x1.col(pos) = _liftedObservations1[id];
            x2.col(pos) = _liftedObservations2[id];
        }

        _solver.solve(x1, x2, models);
    }


    /**
     * @brief Function that computes the estimation error for a given model and all the elements.
     * @param[in] model The model to consider.
     * @param[out] vec_errors The vector containing all the estimation errors for every element.
     */
    void errors(const robustEstimation::Mat3Model & model, std::vector<double>& errors) const 
    {
        Eigen::Matrix3d R = model.getMatrix();

        for (int idx = 0; idx < _liftedObservations1.size(); idx++)
        {
            const Vec3 x1 = _liftedObservations1[idx];
            const Vec3 x2 = _liftedObservations2[idx];
            const Vec3 x = R * x1;

            errors[idx] = std::acos(x2.dot(x));
        }
    }

private:
    const camera::IntrinsicBase & _camera1;
    const camera::IntrinsicBase & _camera2; 
    
    std::vector<Eigen::Vector3d> _liftedObservations1;
    std::vector<Eigen::Vector3d> _liftedObservations2;
    multiview::relativePose::Rotation3PSolver _solver;
};

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
