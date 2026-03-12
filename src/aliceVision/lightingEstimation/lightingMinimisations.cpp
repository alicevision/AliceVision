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

namespace {

void logProblemJacobianFromProblem(const std::string& problemName,
                                   ceres::Problem& problem,
                                   const std::vector<double*>& parameterBlocks)
{
    ceres::Problem::EvaluateOptions evalOptions;
    evalOptions.parameter_blocks = parameterBlocks;

    double cost = 0.0;
    std::vector<double> residuals;
    std::vector<double> gradient;
    ceres::CRSMatrix jacobian;
    if (!problem.Evaluate(evalOptions, &cost, &residuals, &gradient, &jacobian))
    {
        ALICEVISION_LOG_WARNING("[" << problemName << "] Could not evaluate Jacobian from Problem.");
        return;
    }

    if (jacobian.num_rows <= 0 || jacobian.num_cols <= 0)
    {
        ALICEVISION_LOG_DEBUG("[" << problemName << "] Empty Jacobian from Problem.");
        return;
    }

    Eigen::MatrixXd jtj = Eigen::MatrixXd::Zero(jacobian.num_cols, jacobian.num_cols);
    for (int r = 0; r < jacobian.num_rows; ++r)
    {
        const int rowStart = jacobian.rows[r];
        const int rowEnd = jacobian.rows[r + 1];
        for (int a = rowStart; a < rowEnd; ++a)
        {
            const int colA = jacobian.cols[a];
            const double valA = jacobian.values[a];
            for (int b = rowStart; b < rowEnd; ++b)
            {
                const int colB = jacobian.cols[b];
                const double valB = jacobian.values[b];
                jtj(colA, colB) += valA * valB;
            }
        }
    }

    std::ostringstream oss;
    oss << "[" << problemName << "] Problem Jacobian shape: "
        << jacobian.num_rows << "x" << jacobian.num_cols
        << ", nnz=" << jacobian.values.size();
    oss << "\n[" << problemName << "] Gradient (" << gradient.size() << "): ";
    for (size_t i = 0; i < gradient.size(); ++i)
    {
        if (i > 0)
            oss << " ";
        oss << gradient[i];
    }
    oss << "\n[" << problemName << "] JtJ (" << jtj.rows() << "x" << jtj.cols() << "):\n" << jtj;
    ALICEVISION_LOG_DEBUG(oss.str());
}

} // namespace

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

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> residualVec(residual, errorVec.rows());
		residualVec = errorVec;

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
    logProblemJacobianFromProblem("CoarseDirectionnalEstimation [before]", problem, {params[0]});

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
    logProblemJacobianFromProblem("CoarseDirectionnalEstimation [after]", problem, {params[0]});

	ALICEVISION_LOG_INFO(summary.BriefReport());
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
    double epsilon, 
    float &lightingDistance)
{
    int nb_pix = normals.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<CoarsePunctualEstimation>(
                new CoarsePunctualEstimation(points, normals, pixelsIntensity, lightingDirection, lightingIntensity, sceneCenter));

    dynamic_cost->AddParameterBlock(1);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    std::vector<double> p0 = {lightingDistance};
    double* params[] = { p0.data() };
    problem.AddResidualBlock(dynamic_cost, loss, params, 1);
    logProblemJacobianFromProblem("CoarsePunctualEstimation [before]", problem, {params[0]});

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
    logProblemJacobianFromProblem("CoarsePunctualEstimation [after]", problem, {params[0]});

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
	PointSourceModelRefinement(const Eigen::MatrixX3f& points_, const Eigen::MatrixX3f& normals_, const Eigen::VectorXf& pixelsIntensity_, double var_=2./255.)
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
		// norm to the cube
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointCube = normSourcePoint.cwiseProduct(normSourcePoint).cwiseProduct(normSourcePoint);
		// norm to the square
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointSquare = normSourcePoint.cwiseProduct(normSourcePoint);
		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecLightDir = vecSourcePoint.array().colwise() / normSourcePoint.array();
		// per point light intensity
		Eigen::Matrix<T, Eigen::Dynamic, 1> vecLightInt = phi / normSourcePointSquare.array();
		// dot product between light direction and normal
		Eigen::Matrix<T, Eigen::Dynamic, 1> dotLightNormalIntensityEstimated = (vecLightDir.cwiseProduct(normalsCeres)).rowwise().sum();
		// ponderating from distance to 0
		Eigen::Matrix<T, Eigen::Dynamic, 1> ponderation = (dotLightNormalIntensityEstimated.cwiseProduct(dotLightNormalIntensityEstimated) / varCeres * (T(-0.5))).array().exp();
		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = (dotLightNormalIntensityEstimated.cwiseProduct(vecLightInt)).cwiseMax(T(0));
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
	double epsilon, 
	Eigen::Vector3f &lightingPosition, 
	float &lightingIntensity)
{
    int nb_pix = points.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<PointSourceModelRefinement>(
                new PointSourceModelRefinement(points, normals, pixelsIntensity));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->AddParameterBlock(1);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    std::vector<double> p0{lightingPosition[0], lightingPosition[1], lightingPosition[2]};
    std::vector<double> p1{lightingIntensity};
    double* params[] = { p0.data(), p1.data() };
    problem.AddResidualBlock(dynamic_cost, loss, params, 2);
    logProblemJacobianFromProblem("PointSourceModelRefinement [before]", problem, {params[0], params[1]});

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
    logProblemJacobianFromProblem("PointSourceModelRefinement [after]", problem, {params[0], params[1]});

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
	 *  - q: 3D position of light source
	 *  - phi: rgb intensity of light source
	 * Data:
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
	double var;

	// constructor
	ColoredPointSourceModelRefinement(const Eigen::MatrixX3f& points_, const Eigen::MatrixX3f& normals_, const Eigen::MatrixX3f& pixelsRGBIntensity_, double var_=2.0/255.)
		: points(points_), normals(normals_), pixelsRGBIntensity(pixelsRGBIntensity_), var(var_)
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
		Eigen::Matrix<T, 3, 1> phi;
		phi << p1[0], p1[1], p1[2];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> pointsCeres = points.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelsRGBIntensityCeres = pixelsRGBIntensity.cast<T>();

		// directionnal light estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecSourcePoint = (pointsCeres.rowwise() - q.transpose()) * (-1.0);
		// compute norm
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePoint = vecSourcePoint.rowwise().norm();
		// norm to the cube
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointCube = normSourcePoint.cwiseProduct(normSourcePoint).cwiseProduct(normSourcePoint);
		// norm to the square
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointSquare = normSourcePoint.cwiseProduct(normSourcePoint);
		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecLightDir = vecSourcePoint.array().colwise() / normSourcePoint.array();
		// dot product between light direction and normal
		Eigen::Matrix<T, Eigen::Dynamic, 1> dotLightNormalIntensityEstimated = (vecLightDir.cwiseProduct(normalsCeres)).rowwise().sum();
		// ponderating from distance to 0
		Eigen::Matrix<T, Eigen::Dynamic, 1> ponderation = (dotLightNormalIntensityEstimated.cwiseProduct(dotLightNormalIntensityEstimated) / varCeres * (T(-0.5))).array().exp();
		// rgb intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelRGBIntensityEstimated = ((dotLightNormalIntensityEstimated * phi.transpose()).array().colwise() / normSourcePointSquare.array()).cwiseMax(T(0));
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 3> errorVec = (pixelsRGBIntensityCeres.array() - pixelRGBIntensityEstimated.array()).colwise() * ponderation.array();

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
	double epsilon, 
	Eigen::Vector3f &lightingPosition, 
	Eigen::Vector3f &lightingRGBIntensity)
{
    std::vector<double> p0{lightingPosition[0], lightingPosition[1], lightingPosition[2]};
    std::vector<double> p1{lightingRGBIntensity[0], lightingRGBIntensity[1], lightingRGBIntensity[2]};
    int nb_pix = points.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<ColoredPointSourceModelRefinement>(
                new ColoredPointSourceModelRefinement(points, normals, pixelsRGBIntensity));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix*3));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    double* params[] = { p0.data(), p1.data() };
    problem.AddResidualBlock(dynamic_cost, loss, params, 2);
    logProblemJacobianFromProblem("ColoredPointSourceModelRefinement [before]", problem, {params[0], params[1]});

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
    logProblemJacobianFromProblem("ColoredPointSourceModelRefinement [after]", problem, {params[0], params[1]});

	ALICEVISION_LOG_INFO(summary.BriefReport());
    lightingPosition[0] = p0[0];
    lightingPosition[1] = p0[1];
    lightingPosition[2] = p0[2];
    lightingRGBIntensity[0] = p1[0];
    lightingRGBIntensity[1] = p1[1];
    lightingRGBIntensity[2] = p1[2];
}

// led model residual
struct LEDModelResidual {

	/*
	 * Variable:
	 *  - q: 3D position of light source
	 *  - d: 3D direction of light source
	 *  - phi: RGB intensities of light source
	 *  - mu: RGB anisotropy parameter
	 * Data:
	 *  - x: 3D point
	 *  - n: normal
	 *  - i: RGB intensity
	 * 
	 * Function:
	 *   - directionnal lighting at each point: sigma = (q - x) / ||q - x||
	 *   - RGB intensity per point: i_c = phi_c * (d . sigma)^mu_c / ||q - x||^2
	 *   - intensity estimation: ie_c = i_c * {s . n}+
	 *   - absolute difference between ie and i
	 */

	Eigen::MatrixX3f points;
	Eigen::MatrixX3f normals;
	Eigen::MatrixX3f pixelsRGBf;

	// constructor
	LEDModelResidual(const Eigen::MatrixX3f& points_, const Eigen::MatrixX3f& normals_, const Eigen::MatrixX3f& pixelsRGBf_)
		: points(points_), normals(normals_), pixelsRGBf(pixelsRGBf_)
	{}

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {
		const T* p0 = parameters[0];
		const T* p1 = parameters[1];
		const T* p2 = parameters[2];
		const T* p3 = parameters[3];

		// getting light position
		Eigen::Matrix<T, 3, 1> q;
		q << p0[0], p0[1], p0[2];
		// getting light direction
		Eigen::Matrix<T, 3, 1> d;
		d << p1[0], p1[1], p1[2];
		// getting light RGB intensities
		Eigen::Matrix<T, 3, 1> phi;
		phi << p2[0], p2[1], p2[2];
		// getting light RGB anisotrophic parameters
		Eigen::Matrix<T, 3, 1> mu;
		mu << p3[0], p3[1], p3[2];

		// data conversion
		Eigen::Matrix<T, Eigen::Dynamic, 3> pointsCeres = points.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> normalsCeres = normals.cast<T>();
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelsRGBfCeres = pixelsRGBf.cast<T>();

		// directionnal light estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecSourcePoint = (pointsCeres.rowwise() - q.transpose()) * (-1.0);
		// compute norm
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePoint = vecSourcePoint.rowwise().norm();
		// norm to the square
		Eigen::Matrix<T, Eigen::Dynamic, 1> normSourcePointSquare = normSourcePoint.cwiseProduct(normSourcePoint);
		// per point light direction
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecSigma = vecSourcePoint.array().colwise() / normSourcePoint.array();
		// per point sigma . d
		Eigen::Matrix<T, Eigen::Dynamic, 1> sigma_dot_d = (vecSigma * d).array().abs();
		// per point anisotropy
		Eigen::Matrix<T, Eigen::Dynamic, 3> anisotropy(pointsCeres.rows(), 3);
		anisotropy(Eigen::placeholders::all, 0) = sigma_dot_d.array().pow(mu(0));
		anisotropy(Eigen::placeholders::all, 1) = sigma_dot_d.array().pow(mu(1));
		anisotropy(Eigen::placeholders::all, 2) = sigma_dot_d.array().pow(mu(2));
		// per point intensity
		Eigen::Matrix<T, Eigen::Dynamic, 3> vecRGBintensity = (anisotropy.array().rowwise() * phi.transpose().array()).colwise() / normSourcePointSquare.array();
		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 1> pixelIntensityEstimated = (vecSigma.cwiseProduct(normalsCeres)).rowwise().sum().cwiseMax(T(0));
		// intensity estimation per point
		Eigen::Matrix<T, Eigen::Dynamic, 3> pixelRGBfEstimated = vecRGBintensity.array().colwise() * pixelIntensityEstimated.array();
		// residual computation
		Eigen::Matrix<T, Eigen::Dynamic, 3> errorVec = pixelsRGBfCeres - pixelRGBfEstimated;

		// set residual
		Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 3>> residualVec(residual, errorVec.rows(), 3);
		residualVec = errorVec;

        return true;
    }
};

void LEDModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::MatrixX3f& pixelsRGBf, 
	double epsilon, 
	Eigen::Vector3f &lightingPosition, 
	Eigen::Vector3f &lightingDirection, 
	Eigen::Vector3f &lightingRGBIntensity,
	Eigen::Vector3f &lightingRGBAnisotropy)
{
    std::vector<double> p0{
      lightingPosition[0],
      lightingPosition[1],
      lightingPosition[2],
    };
    std::vector<double> p1{
      lightingDirection[0],
      lightingDirection[1],
      lightingDirection[2],
    };
    std::vector<double> p2{
      lightingRGBIntensity[0],
      lightingRGBIntensity[1],
      lightingRGBIntensity[2],
    };
    std::vector<double> p3{
      lightingRGBAnisotropy[0],
      lightingRGBAnisotropy[1],
      lightingRGBAnisotropy[2],
    };
    int nb_pix = points.rows();

    ceres::Problem problem;
    auto* dynamic_cost = 
        new ceres::DynamicAutoDiffCostFunction<LEDModelResidual>(
                new LEDModelResidual(points, normals, pixelsRGBf));

    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->AddParameterBlock(3);
    dynamic_cost->SetNumResiduals(static_cast<int>(nb_pix*3));

    ceres::LossFunction* loss = new ceres::HuberLoss(epsilon);

    double* params[] = { p0.data(), p1.data(), p2.data(), p3.data() };
    problem.AddResidualBlock(dynamic_cost, loss, params, 4);
    logProblemJacobianFromProblem("LEDModelResidual [before]", problem, {params[0], params[1], params[2], params[3]});

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
    logProblemJacobianFromProblem("LEDModelResidual [after]", problem, {params[0], params[1], params[2], params[3]});

	ALICEVISION_LOG_INFO(summary.BriefReport());
    lightingPosition[0] = p0[0];
    lightingPosition[1] = p0[1];
    lightingPosition[2] = p0[2];
    lightingDirection[0] = p1[0];
    lightingDirection[1] = p1[1];
    lightingDirection[2] = p1[2];
    lightingRGBIntensity[0] = p2[0];
    lightingRGBIntensity[1] = p2[1];
    lightingRGBIntensity[2] = p2[2];
    lightingRGBAnisotropy[0] = p3[0];
    lightingRGBAnisotropy[1] = p3[1];
    lightingRGBAnisotropy[2] = p3[2];
}

void proportion_in_shadow(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::MatrixX3f& pixelsRGBf, 
	Eigen::Vector3f &lightingPosition)
{
	const int nbPix = points.rows();
	if (nbPix == 0)
	{
		ALICEVISION_LOG_INFO("proportion_in_shadow: 0");
		return;
	}

	const Eigen::RowVector3f lightPos = lightingPosition.transpose();
	const Eigen::MatrixX3f vecSourcePoint = (points.rowwise() - lightPos) * (-1.0f);
	const Eigen::VectorXf invNorm = vecSourcePoint.rowwise().norm().array().max(1e-8f).inverse();
	const Eigen::MatrixX3f vecSigma = vecSourcePoint.array().colwise() * invNorm.array();
	const Eigen::VectorXf pixelIntensityEstimated = (vecSigma.cwiseProduct(normals)).rowwise().sum();
	const float proportion = static_cast<float>((pixelIntensityEstimated.array() < 0.0f).count()) / static_cast<float>(nbPix);

	ALICEVISION_LOG_INFO("proportion_in_shadow: " << proportion);
}


} // namespace lightingEstimation
} // namespace aliceVision
