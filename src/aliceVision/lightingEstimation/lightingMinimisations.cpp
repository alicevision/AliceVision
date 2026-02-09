// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include "lightingMinimisations.hpp"

#include <aliceVision/system/Logger.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ceres/ceres.h>

namespace aliceVision {
namespace lightingEstimation {

// coarse directionnal estimation
struct CoarseDirectionnalEstimation {

	/*
	 * Variable:
	 *  - s: 3D directionnal light source
	 * Data:
	 *  - n: normal
	 *  - i intensity
	 * 
	 * Function:
	 *   - intensity estimation: ie = {s . n}+
	 *   - absolute difference between ie and i
	 */

	Eigen::MatrixX3f normals;
	Eigen::VectorXf pixelsIntensity;

	// constructor
	CoarseDirectionnalEstimation(const Eigen::MatrixX3f& normals_, const Eigen::VectorXf& pixelsIntensity_)
		: normals(normals_), pixelsIntensity(pixelsIntensity_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* x = parameters[0];

		// getting light direction
		Eigen::Matrix<T, 3, 1> vecLightDir;
		vecLightDir << x[0], x[1], x[2];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelsIntensityCeres = pixelsIntensity.cast<T>();

		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = (normalsCeres * vecLightDir).cwiseMax(T(0));
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorVec = pixelsIntensityCeres - pixelIntensityEstimated;
		// absolute residual
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorSq = errorVec.array().square();

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residualVec(residual, errorSq.rows());
		residualVec = errorSq;

        return true;
    }
};

void coarseDirectionnalLightEstimation(const Eigen::MatrixX3f& normals, const Eigen::VectorXf& pixelsIntensity, double epsilon, Eigen::Vector3f &lightingDirection)
{
    std::vector<double> x{lightingDirection[0], lightingDirection[1], lightingDirection[2]};
    double* params[] = { x.data() };
    int nb_pix = normals.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<CoarseDirectionnalEstimation>(
                new CoarseDirectionnalEstimation(normals, pixelsIntensity));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    problem.AddResidualBlock(dynamic_cost, loss, params, 1);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_DEBUG(summary.BriefReport());
    lightingDirection[0] = x[0];
    lightingDirection[1] = x[1];
    lightingDirection[2] = x[2];
}



// coarse punctual estimation
struct CoarsePunctualEstimation {

	/*
	 * Variable:
	 *  - d: distance from light source to scene center
	 * Data:
	 *  - x: 3D point
	 *  - n: normal
	 *  - i intensity
	 *  - phi: intensity of light source
     *  - t: lighting direction
     *  - c: scene center
	 * 
	 * Function:
	 *   - directionnal lighting at each point: s = phi . d^2. (c + d.t - x) / (c + d.t - x)^3
	 *   - intensity estimation: ie = {s . n}+
	 *   - absolute difference between ie and i
	 */

	Eigen::MatrixX3f points;
	Eigen::MatrixX3f normals;
	Eigen::VectorXf pixelsIntensity;
    Eigen::Vector3f lightingDirection;
	double lightingIntensity;
    Eigen::Vector3f sceneCenter;

	// constructor
	CoarsePunctualEstimation(
        const Eigen::MatrixX3f& points_, 
        const Eigen::MatrixX3f& normals_, 
        const Eigen::VectorXf& pixelsIntensity_, 
        const Eigen::Vector3f& lightingDirection_, 
		double lightingIntensity_,
        const Eigen::Vector3f& sceneCenter_)
		: points(points_), 
		normals(normals_), 
		pixelsIntensity(pixelsIntensity_), 
		lightingDirection(lightingDirection_), 
		lightingIntensity(lightingIntensity_), 
		sceneCenter(sceneCenter_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* x = parameters[0];

        T d = x[0];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> pointsCeres = points.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelsIntensityCeres = pixelsIntensity.cast<T>();
        Eigen::Matrix<T, 3, 1> lightingDirectionCeres = lightingDirection.cast<T>();
        Eigen::Matrix<T, 3, 1> sceneCenterCeres = sceneCenter.cast<T>();
		T phi = T(lightingIntensity);

        // light position
        Eigen::Matrix<T, 3, 1> q = sceneCenterCeres + d * lightingDirectionCeres;
		// directionnal light estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecSourcePoint = (pointsCeres.rowwise() - q.transpose()) * (-1.0);
		// compute norm
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePoint = vecSourcePoint.rowwise().norm();
		// norm to the cube
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointCube = normSourcePoint.cwiseProduct(normSourcePoint).cwiseProduct(normSourcePoint);
		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecLightDir = vecSourcePoint.array().colwise() / normSourcePointCube.array() * phi * d * d;
		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = (vecLightDir.cwiseProduct(normalsCeres)).rowwise().sum().cwiseMax(T(0));
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorVec = pixelsIntensityCeres - pixelIntensityEstimated;
		// squaring residual
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorSq = errorVec.array().square();

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residualVec(residual, errorSq.rows());
		residualVec = errorSq;

        return true;
    }
};

void coarsePunctualLightEstimation(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    const Eigen::Vector3f& sceneCenter,
    const Eigen::Vector3f& lightingDirection,
    double lightingIntensity,
    double epsilon, 
    double &lightingDistance)
{
    std::vector<double> x = {lightingDistance};
    double* params[] = { x.data() };
    int nb_pix = normals.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<CoarsePunctualEstimation>(
                new CoarsePunctualEstimation(points, normals, pixelsIntensity, lightingDirection, lightingIntensity, sceneCenter));

    dynamic_cost->AddParameterBlock(1);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    problem.AddResidualBlock(dynamic_cost, loss, params, 1);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_DEBUG(summary.BriefReport());
    lightingDistance = x[0];
}

// near-light model residual
struct PointSourceModelRefinement {

	/*
	 * Variable:
	 *  - q: 3D position of light source
	 *  - phi: intensity of light source
	 * Data:
	 *  - x: 3D point
	 *  - n: normal
	 *  - i intensity
	 * 
	 * Function:
	 *   - directionnal lighting at each point: s = phi . (q - x) / (q - x)^3
	 *   - intensity estimation: ie = {s . n}+
	 *   - absolute difference between ie and i
	 */

	Eigen::MatrixX3f points;
	Eigen::MatrixX3f normals;
	Eigen::VectorXf pixelsIntensity;

	// constructor
	PointSourceModelRefinement(const Eigen::MatrixX3f& points_, const Eigen::MatrixX3f& normals_, const Eigen::VectorXf& pixelsIntensity_)
		: points(points_), normals(normals_), pixelsIntensity(pixelsIntensity_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* x = parameters[0];

		// getting light position
		Eigen::Matrix<T, 3, 1> q;
		q << x[0], x[1], x[2];
		// getting light intensity
		T phi = x[3];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> pointsCeres = points.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelsIntensityCeres = pixelsIntensity.cast<T>();

		// directionnal light estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecSourcePoint = (pointsCeres.rowwise() - q.transpose()) * (-1.0);
		// compute norm
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePoint = vecSourcePoint.rowwise().norm();
		// norm to the cube
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointCube = normSourcePoint.cwiseProduct(normSourcePoint).cwiseProduct(normSourcePoint);
		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecLightDir = vecSourcePoint.array().colwise() / normSourcePointCube.array() * phi;
		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = (vecLightDir.cwiseProduct(normalsCeres)).rowwise().sum().cwiseMax(T(0));
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorVec = pixelsIntensityCeres - pixelIntensityEstimated;
		// Huber loss
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorSq = errorVec.array().square();

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residualVec(residual, errorSq.rows());
		residualVec = errorSq;

        return true;
    }
};

void pointSourceModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::VectorXf& pixelsIntensity, 
	double epsilon, 
	Eigen::Vector3f &lightingPosition, 
	double &lightingIntensity)
{
    std::vector<double> x{lightingPosition[0], lightingPosition[1], lightingPosition[2], lightingIntensity};
    int nb_pix = points.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<PointSourceModelRefinement>(
                new PointSourceModelRefinement(points, normals, pixelsIntensity));

    dynamic_cost->AddParameterBlock(4);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    double* params[] = { x.data() };
    problem.AddResidualBlock(dynamic_cost, loss, params, 1);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_DEBUG(summary.BriefReport());
    lightingPosition[0] = x[0];
    lightingPosition[1] = x[1];
    lightingPosition[2] = x[2];
    lightingIntensity = x[3];
}

} // namespace lightingEstimation
} // namespace aliceVision