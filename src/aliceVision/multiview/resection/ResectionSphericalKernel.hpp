// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/robustEstimation/PointFittingKernel.hpp>
#include <aliceVision/multiview/resection/Resection6PSolver.hpp>
#include <aliceVision/multiview/resection/ProjectionDistanceError.hpp>
#include <aliceVision/multiview/resection/P3PSolver.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

class ResectionSphericalKernel
{
public:
    using ModelT = Eigen::Matrix4d;

public:
    ResectionSphericalKernel(std::shared_ptr<camera::IntrinsicBase> & camera, 
                    const std::vector<Eigen::Vector3d> & structure, 
                    const std::vector<Eigen::Vector2d> & observations) 
    : _camera(camera), 
    _structure(structure), 
    _observations(observations)
    {
        for (const auto & pt : _observations)
        {
            _liftedObservations.push_back(_camera->toUnitSphere(_camera->removeDistortion(_camera->ima2cam(pt))));
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
        return 4;
    }

    /**
     * @brief The number of elements in the data.
     * @return the number of elements in the data.
     */
    std::size_t nbSamples() const
    {
        return _structure.size();
    }

    /**
     * @brief Get logalpha0, Alpha0 is used to make the error adaptive to the image size
     * @return logalpha0
     */
    double logalpha0() const
    {
        return log10(M_PI / (_camera->w() * _camera->h()));
    }

    double errorVectorDimension() const
    {
        return 2.0;
    }

    /**
     * @brief This function is called to estimate the model from the minimum number
     * of sample \p minSample (i.e. minimal problem solver).
     * @param[in] samples A vector containing the indices of the data to be used for
     * the minimal estimation.
     * @param[out] models The model(s) estimated by the minimal solver.
     */
    void fit(const std::vector<std::size_t>& samples, std::vector<Eigen::Matrix4d>& models) const
    {
        Mat3 X;
        Mat3 x;

        for (int pos = 0; pos < 3; pos++)
        {
            size_t id = samples[pos];
            X.col(pos) = _structure[id];
            x.col(pos) = _liftedObservations[id];
        }

        _solver.solve(x, X, models);
    }


    /**
     * @brief Function that computes the estimation error for a given model and all the elements.
     * @param[in] model The model to consider.
     * @param[out] vec_errors The vector containing all the estimation errors for every element.
     */
    void errors(const Eigen::Matrix4d & model, std::vector<double>& errors) const 
    {
        for (int idx = 0; idx < _structure.size(); idx++)
        {
            const Vec4 X = _structure[idx].homogeneous();
            const Vec2 x = _observations[idx];

            const Vec2 residual = _camera->residual(geometry::Pose3(model), X, x);

            errors[idx] = residual.norm();
        }
    }

private:
    std::shared_ptr<camera::IntrinsicBase> _camera; 
    std::vector<Eigen::Vector3d> _structure;
    std::vector<Eigen::Vector2d> _observations;
    std::vector<Eigen::Vector3d> _liftedObservations;
    multiview::resection::P3PSolver _solver;
};

}  // namespace resection
}  // namespace multiview
}  // namespace aliceVision
