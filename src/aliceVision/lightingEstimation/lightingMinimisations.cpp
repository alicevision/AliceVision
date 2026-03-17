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
#include <Eigen/SVD>
#include <array>
#include <cmath>
#include <limits>
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
	double var;

	// constructor
	CoarseDirectionnalEstimation(const Eigen::MatrixX3f& normals_, const Eigen::VectorXf& pixelsIntensity_, double var_=0.01)
		: normals(normals_), pixelsIntensity(pixelsIntensity_), var(var_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* x = parameters[0];

		T varCeres = T(var);

		// getting light direction
		Eigen::Matrix<T, 3, 1> vecLightDir;
		vecLightDir << x[0], x[1], x[2];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelsIntensityCeres = pixelsIntensity.cast<T>();

		// light intensity
		T lightIntensity = vecLightDir.norm();
		// light direction
		Eigen::Matrix<T, Eigen::Dynamic, 1> vecLightDirNorm = vecLightDir / lightIntensity;

		// dot product between light direction and normal
		Eigen::Matrix<T, Eigen::Dynamic, 1> dotLightNormalIntensityEstimated = normalsCeres * vecLightDirNorm;

		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = (dotLightNormalIntensityEstimated).cwiseMax(T(0)) * lightIntensity;

		// ponderating from distance to 0
		Eigen::Matrix<T, Eigen::Dynamic, 1> ponderation = (dotLightNormalIntensityEstimated.cwiseProduct(dotLightNormalIntensityEstimated) / varCeres * (T(-0.5))).array().exp();
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorVec = ponderation.cwiseProduct(pixelsIntensityCeres - pixelIntensityEstimated);

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residualVec(residual, errorVec.rows());
		residualVec = errorVec;

        return true;
    }
};

void coarseDirectionnalLightEstimation(const Eigen::MatrixX3f& normals, const Eigen::VectorXf& pixelsIntensity, double var, Eigen::Vector3f &lightingDirection)
{
    std::vector<double> x{lightingDirection[0], lightingDirection[1], lightingDirection[2]};
    double* params[] = { x.data() };
    int nb_pix = normals.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<CoarseDirectionnalEstimation>(
                new CoarseDirectionnalEstimation(normals, pixelsIntensity, var));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    problem.AddResidualBlock(dynamic_cost, nullptr, params, 1);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_INFO(summary.BriefReport());
    lightingDirection[0] = x[0];
    lightingDirection[1] = x[1];
    lightingDirection[2] = x[2];
}


// coarse directionnal estimation
struct ColoredDirectionnalEstimation {

	/*
	 * Variable:
	 *  - phi: rgb intensity of light source
	 * Data:
	 *  - n: normal
	 *  - i: rgbintensity
	 *  - s: 3D directionnal light source
	 * 
	 * Function:
	 *   - intensity estimation: ie = {s . n}+ * phi
	 *   - absolute difference between ie and i
	 */

	Eigen::MatrixX3f normals;
	Eigen::MatrixX3f pixelsIntensity;
	Eigen::Vector3f lightingDirection;

	// constructor
	ColoredDirectionnalEstimation(const Eigen::MatrixX3f& normals_, const Eigen::MatrixX3f& pixelsIntensity_, const Eigen::Vector3f& lightingDirection_)
		: normals(normals_), pixelsIntensity(pixelsIntensity_), lightingDirection(lightingDirection_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* x = parameters[0];

		// getting light direction
		Eigen::Matrix<T, 3, 1> phi;
		phi << x[0], x[1], x[2];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelsIntensityCeres = pixelsIntensity.cast<T>();
		Eigen::Matrix<T, 1, 3> lightingDirectionCeres = lightingDirection.cast<T>();

		// dot product between light direction and normal
		Eigen::Matrix<T, Eigen::Dynamic, 1> dotLightNormalIntensityEstimated = normalsCeres * lightingDirectionCeres.transpose();

		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelIntensityEstimated = (dotLightNormalIntensityEstimated).cwiseMax(T(0)) * (phi.transpose());

		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 3> errorVec = pixelsIntensityCeres - pixelIntensityEstimated;

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 3>> residualVec(residual, errorVec.rows(), 3);
		residualVec = errorVec;

        return true;
    }
};

void coloredDirectionnalLightEstimation(const Eigen::MatrixX3f& normals, const Eigen::MatrixX3f& pixelsIntensity, const Eigen::Vector3f &lightingDirection, double epsilon, Eigen::Vector3f &lightingIntensity)
{
    std::vector<double> x{lightingIntensity[0], lightingIntensity[1], lightingIntensity[2]};
    double* params[] = { x.data() };
    int nb_pix = normals.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<ColoredDirectionnalEstimation>(
                new ColoredDirectionnalEstimation(normals, pixelsIntensity, lightingDirection));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix*3));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    problem.AddResidualBlock(dynamic_cost, loss, params, 1);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_INFO(summary.BriefReport());
    lightingIntensity[0] = x[0];
    lightingIntensity[1] = x[1];
    lightingIntensity[2] = x[2];
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
	double var;

	// constructor
	CoarsePunctualEstimation(
        const Eigen::MatrixX3f& points_, 
        const Eigen::MatrixX3f& normals_, 
        const Eigen::VectorXf& pixelsIntensity_, 
        const Eigen::Vector3f& lightingDirection_, 
		double lightingIntensity_,
        const Eigen::Vector3f& sceneCenter_,
		double var_=0.01)
		: points(points_), 
		normals(normals_), 
		pixelsIntensity(pixelsIntensity_), 
		lightingDirection(lightingDirection_), 
		lightingIntensity(lightingIntensity_), 
		sceneCenter(sceneCenter_),
		var(var_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* x = parameters[0];

        T d = x[0];

		T varCeres = T(var);

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
		// norm to the square
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointSquare = normSourcePoint.cwiseProduct(normSourcePoint);

		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecLightDir = vecSourcePoint.array().colwise() / normSourcePoint.array();
		// per point light intensity
		Eigen::Matrix<T, Eigen::Dynamic, 1> vecLightIntensity = phi * d * d / normSourcePointSquare.array();

		// dot product between light direction and normal
		Eigen::Matrix<T, Eigen::Dynamic, 1> dotLightNormalIntensityEstimated = (vecLightDir.cwiseProduct(normalsCeres)).rowwise().sum();

		// per pixel intensity estimation
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = dotLightNormalIntensityEstimated.cwiseMax(T(0)).cwiseProduct(vecLightIntensity);

		// ponderating from distance to 0
		Eigen::Matrix<T, Eigen::Dynamic, 1> ponderation = (dotLightNormalIntensityEstimated.cwiseProduct(dotLightNormalIntensityEstimated) / varCeres * (T(-0.5))).array().exp();
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorVec = ponderation.cwiseProduct(pixelsIntensityCeres - pixelIntensityEstimated);

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residualVec(residual, errorVec.rows());
		residualVec = errorVec;

        return true;
    }
};

void coarsePunctualLightEstimation(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    const Eigen::Vector3f& sceneCenter,
    const Eigen::Vector3f& lightingDirection,
    float lightingIntensity,
    double var, 
    float &lightingDistance)
{
    int nb_pix = normals.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<CoarsePunctualEstimation>(
                new CoarsePunctualEstimation(points, normals, pixelsIntensity, lightingDirection, lightingIntensity, sceneCenter, var));

    dynamic_cost->AddParameterBlock(1);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    std::vector<double> p0 = {lightingDistance};
    double* params[] = { p0.data() };
    problem.AddResidualBlock(dynamic_cost, nullptr, params, 1);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_INFO(summary.BriefReport());
    lightingDistance = p0[0];
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
	double var;

	// constructor
	PointSourceModelRefinement(const Eigen::MatrixX3f& points_, const Eigen::MatrixX3f& normals_, const Eigen::VectorXf& pixelsIntensity_, double var_=0.01)
		: points(points_), normals(normals_), pixelsIntensity(pixelsIntensity_), var(var_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* p0 = parameters[0];
		const T* p1 = parameters[1];

		T varCeres = T(var);

		// getting light position
		Eigen::Matrix<T, 3, 1> q;
		q << p0[0], p0[1], p0[2];
		// getting light intensity
		T phi = p1[0];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> pointsCeres = points.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelsIntensityCeres = pixelsIntensity.cast<T>();

		// directionnal light estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecSourcePoint = (pointsCeres.rowwise() - q.transpose()) * (-1.0);

		// compute norm
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePoint = vecSourcePoint.rowwise().norm();
		// norm to the square
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointSquare = normSourcePoint.cwiseProduct(normSourcePoint);

		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecLightDir = vecSourcePoint.array().colwise() / normSourcePoint.array();
		// per point light intensity
		Eigen::Matrix<T, Eigen::Dynamic, 1> vecLightInt = phi / normSourcePointSquare.array();

		// dot product between light direction and normal
		Eigen::Matrix<T, Eigen::Dynamic, 1> dotLightNormalIntensityEstimated = (vecLightDir.cwiseProduct(normalsCeres)).rowwise().sum();

		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = dotLightNormalIntensityEstimated.cwiseProduct(vecLightInt).cwiseMax(T(0));

		// ponderating from distance to 0
		Eigen::Matrix<T, Eigen::Dynamic, 1> ponderation = (dotLightNormalIntensityEstimated.cwiseProduct(dotLightNormalIntensityEstimated) / varCeres * (T(-0.5))).array().exp();
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 1> errorVec = ponderation.cwiseProduct((pixelsIntensityCeres.array() - pixelIntensityEstimated.array()).matrix());

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residualVec(residual, errorVec.rows());
		residualVec = errorVec;

        return true;
    }
};

void pointSourceModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::VectorXf& pixelsIntensity, 
	double var, 
	Eigen::Vector3f &lightingPosition, 
	float &lightingIntensity)
{
    int nb_pix = points.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<PointSourceModelRefinement>(
                new PointSourceModelRefinement(points, normals, pixelsIntensity, var));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->AddParameterBlock(1);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    std::vector<double> p0{lightingPosition[0], lightingPosition[1], lightingPosition[2]};
    std::vector<double> p1{lightingIntensity};
    double* params[] = { p0.data(), p1.data() };
    problem.AddResidualBlock(dynamic_cost, nullptr, params, 2);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_INFO(summary.BriefReport());
    lightingPosition[0] = p0[0];
    lightingPosition[1] = p0[1];
    lightingPosition[2] = p0[2];
    lightingIntensity = p1[0];
}

// colored near-light model residual
struct ColoredPointSourceModelRefinement {

	/*
	 * Variable:
	 *  - phi: rgb intensity of light source
	 * Data:
	 *  - q: 3D position of light source
	 *  - x: 3D point
	 *  - n: normal
	 *  - i: rgb intensity
	 * 
	 * Function:
	 *   - directionnal lighting at each point: s = (q - x) / (q - x)^3
	 *   - per canal intensity estimation: ie_c = phi_c . {s . n}+
	 *   - absolute difference between ie and i
	 */

	Eigen::MatrixX3f points;
	Eigen::MatrixX3f normals;
	Eigen::MatrixX3f pixelsRGBIntensity;
	Eigen::Vector3f lightPosition;
	double var;

	// constructor
	ColoredPointSourceModelRefinement(const Eigen::MatrixX3f& points_, const Eigen::MatrixX3f& normals_, const Eigen::MatrixX3f& pixelsRGBIntensity_, const Eigen::Vector3f& lightPosition_)
		: points(points_), normals(normals_), pixelsRGBIntensity(pixelsRGBIntensity_), lightPosition(lightPosition_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* p0 = parameters[0];

		// getting light intensity
		Eigen::Matrix<T, 3, 1> phi;
		phi << p0[0], p0[1], p0[2];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> pointsCeres = points.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelsRGBIntensityCeres = pixelsRGBIntensity.cast<T>();
		Eigen::Matrix<T, 3, 1> lightPositionCeres = lightPosition.cast<T>();

		// directionnal light estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecSourcePoint = (pointsCeres.rowwise() - lightPositionCeres.transpose()) * (-1.0);

		// compute norm
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePoint = vecSourcePoint.rowwise().norm();
		// norm to the square
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointSquare = normSourcePoint.cwiseProduct(normSourcePoint);

		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecLightDir = vecSourcePoint.array().colwise() / normSourcePoint.array();

		// dot product between light direction and normal
		Eigen::Matrix<T, Eigen::Dynamic, 1> dotLightNormalIntensityEstimated = vecLightDir.cwiseProduct(normalsCeres).rowwise().sum();

		// rgb intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelRGBIntensityEstimated = ((dotLightNormalIntensityEstimated * phi.transpose()).array().colwise() / normSourcePointSquare.array()).cwiseMax(T(0));

		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 3> errorVec = pixelsRGBIntensityCeres.array() - pixelRGBIntensityEstimated.array();

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 3>> residualVec(residual, errorVec.rows(), 3);
		residualVec = errorVec;

        return true;
    }
};

void coloredPointSourceModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::MatrixX3f& pixelsRGBIntensity, 
	const Eigen::Vector3f &lightingPosition, 
	double epsilon, 
	Eigen::Vector3f &lightingRGBIntensity)
{
    std::vector<double> p0{lightingRGBIntensity[0], lightingRGBIntensity[1], lightingRGBIntensity[2]};
    int nb_pix = points.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<ColoredPointSourceModelRefinement>(
                new ColoredPointSourceModelRefinement(points, normals, pixelsRGBIntensity, lightingPosition));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix*3));

    double* params[] = { p0.data() };
    problem.AddResidualBlock(dynamic_cost, nullptr, params, 1);

    // Options solveur
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.minimizer_type = ceres::LINE_SEARCH;
	options.line_search_direction_type = ceres::LBFGS;
	options.max_lbfgs_rank = 20;
	options.line_search_type = ceres::WOLFE;
	options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

	ALICEVISION_LOG_INFO(summary.BriefReport());
    lightingRGBIntensity[0] = p0[0];
    lightingRGBIntensity[1] = p0[1];
    lightingRGBIntensity[2] = p0[2];
}

} // namespace lightingEstimation
} // namespace aliceVision
