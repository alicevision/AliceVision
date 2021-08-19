// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/BundleAdjustmentPanoramaCeres.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/sfm/ResidualErrorRotationPriorFunctor.hpp>

#include <boost/filesystem.hpp>
#include <ceres/rotation.h>

#include <aliceVision/camera/Equidistant.hpp>

#include <fstream>

namespace fs = boost::filesystem;

namespace aliceVision {

class CostRotationPrior : public ceres::SizedCostFunction<3, 9, 9> {
public:
  explicit CostRotationPrior(const Eigen::Matrix3d & two_R_one) : _two_R_one(two_R_one) {

  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override {

    const double * parameter_rotation_one = parameters[0];
    const double * parameter_rotation_two = parameters[1];

    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> oneRo(parameter_rotation_one);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> twoRo(parameter_rotation_two);

    Eigen::Matrix3d two_R_one_est = twoRo * oneRo.transpose();
    Eigen::Matrix3d error_R = two_R_one_est * _two_R_one.transpose();
    Eigen::Vector3d error_r = SO3::logm(error_R);

    residuals[0] = error_r(0);
    residuals[1] = error_r(1);
    residuals[2] = error_r(2);

    if (jacobians == nullptr) {
      return true;
    }

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[0]);

      J = SO3::dlogmdr(error_R) * getJacobian_AB_wrt_A<3, 3, 3>(two_R_one_est, _two_R_one.transpose()) * getJacobian_AB_wrt_B<3, 3, 3>(twoRo, oneRo.transpose()) * getJacobian_At_wrt_A<3, 3>() * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), oneRo);
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> J(jacobians[1]);

      J = SO3::dlogmdr(error_R) * getJacobian_AB_wrt_A<3, 3, 3>(two_R_one_est, _two_R_one.transpose()) * getJacobian_AB_wrt_A<3, 3, 3>(twoRo, oneRo.transpose()) * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), twoRo);
    }

    return true;
  }

private:
  Eigen::Matrix3d _two_R_one;
};

class CostEquiDistant : public ceres::SizedCostFunction<2, 9, 9, 7> {
public:
  CostEquiDistant(Vec2 fi, Vec2 fj, std::shared_ptr<camera::EquiDistant> & intrinsic) : _fi(fi), _fj(fj), _intrinsic(intrinsic) {

  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override {

    Vec2 pt_i = _fi;
    Vec2 pt_j = _fj;

    const double * parameter_rotation_i = parameters[0];
    const double * parameter_rotation_j = parameters[1];
    const double * parameter_intrinsics = parameters[2];

    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> iRo(parameter_rotation_i);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jRo(parameter_rotation_j);

    _intrinsic->setScale(parameter_intrinsics[0], parameter_intrinsics[1]);
    _intrinsic->setOffset(parameter_intrinsics[2], parameter_intrinsics[3]);
    _intrinsic->setDistortionParams({parameter_intrinsics[4], parameter_intrinsics[5], parameter_intrinsics[6]});

    Eigen::Matrix3d R = jRo * iRo.transpose();
    geometry::Pose3 T(R, Vec3({0,0,0}));

    Vec2 pt_i_cam = _intrinsic->ima2cam(pt_i);
    Vec2 pt_i_undist = _intrinsic->removeDistortion(pt_i_cam);
    Vec4 pt_i_sphere = _intrinsic->toUnitSphere(pt_i_undist).homogeneous();

    Vec2 pt_j_est = _intrinsic->project(T, pt_i_sphere, true);

    residuals[0] = pt_j_est(0) - pt_j(0);
    residuals[1] = pt_j_est(1) - pt_j(1);

    if (jacobians == nullptr) {
      return true;
    }

    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[0]);

      J = _intrinsic->getDerivativeProjectWrtRotation(T, pt_i_sphere) * getJacobian_AB_wrt_B<3, 3, 3>(jRo, iRo.transpose()) * getJacobian_At_wrt_A<3, 3>() * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), iRo);
    }

    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[1]);

      J = _intrinsic->getDerivativeProjectWrtRotation(T, pt_i_sphere) * getJacobian_AB_wrt_A<3, 3, 3>(jRo, iRo.transpose()) * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), jRo);
    }

    if (jacobians[2] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J(jacobians[2]);

	Eigen::Matrix<double, 4, 3> Jhomogenous = Eigen::Matrix<double, 4, 3>::Identity();

      Eigen::Matrix<double, 2, 2> Jscale = _intrinsic->getDerivativeProjectWrtScale(T, pt_i_sphere) + _intrinsic->getDerivativeProjectWrtPoint(T, pt_i_sphere) * Jhomogenous * _intrinsic->getDerivativetoUnitSphereWrtScale(pt_i_undist);
      Eigen::Matrix<double, 2, 2> Jpp = _intrinsic->getDerivativeProjectWrtPrincipalPoint(T, pt_i_sphere) + _intrinsic->getDerivativeProjectWrtPoint(T, pt_i_sphere) * Jhomogenous * _intrinsic->getDerivativetoUnitSphereWrtPoint(pt_i_undist) * _intrinsic->getDerivativeRemoveDistoWrtPt(pt_i_cam) * _intrinsic->getDerivativeIma2CamWrtPrincipalPoint();
      Eigen::Matrix<double, 2, 3> Jdisto = _intrinsic->getDerivativeProjectWrtDisto(T, pt_i_sphere) + _intrinsic->getDerivativeProjectWrtPoint(T, pt_i_sphere) * Jhomogenous * _intrinsic->getDerivativetoUnitSphereWrtPoint(pt_i_undist) * _intrinsic->getDerivativeRemoveDistoWrtDisto(pt_i_cam);

      J.block<2, 2>(0, 0) = Jscale;
      J.block<2, 2>(0, 2) = Jpp;
      J.block<2, 3>(0, 4) = Jdisto;
    }

    return true;
  }

private:
  Vec2 _fi;
  Vec2 _fj;
  std::shared_ptr<camera::EquiDistant> _intrinsic;
};

class CostPinHole : public ceres::CostFunction {
public:
  CostPinHole(Vec2 fi, Vec2 fj, std::shared_ptr<camera::Pinhole> & intrinsic) : _fi(fi), _fj(fj), _intrinsic(intrinsic) {

    set_num_residuals(2);

    mutable_parameter_block_sizes()->push_back(9);
    mutable_parameter_block_sizes()->push_back(9);
    mutable_parameter_block_sizes()->push_back(intrinsic->getParams().size());    
  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override {

    Vec2 pt_i = _fi;
    Vec2 pt_j = _fj;

    const double * parameter_rotation_i = parameters[0];
    const double * parameter_rotation_j = parameters[1];
    const double * parameter_intrinsics = parameters[2];

    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> iRo(parameter_rotation_i);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jRo(parameter_rotation_j);

    _intrinsic->setScale(parameter_intrinsics[0], parameter_intrinsics[1]);
    _intrinsic->setOffset(parameter_intrinsics[2], parameter_intrinsics[3]);

    std::vector<double> distortion_params;
    size_t params_size = _intrinsic->getParams().size();
    size_t disto_size = _intrinsic->getDistortionParams().size();
    size_t offset = params_size - disto_size;
    for (size_t index = offset; index < params_size; index++) {
      distortion_params.push_back(parameter_intrinsics[index]);
    }
    _intrinsic->setDistortionParams(distortion_params);

    Eigen::Matrix3d R = jRo * iRo.transpose();
    geometry::Pose3 T(R, Vec3({0,0,0}));

    Vec2 pt_i_cam = _intrinsic->ima2cam(pt_i);
    Vec2 pt_i_undist = _intrinsic->removeDistortion(pt_i_cam);
    Vec4 pt_i_sphere = _intrinsic->toUnitSphere(pt_i_undist).homogeneous();

    Vec2 pt_j_est = _intrinsic->project(T, pt_i_sphere, true);

    residuals[0] = pt_j_est(0) - pt_j(0);
    residuals[1] = pt_j_est(1) - pt_j(1);

    if (jacobians == nullptr) {
      return true;
    }

    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[0]);

      J = _intrinsic->getDerivativeProjectWrtRotation(T, pt_i_sphere) * getJacobian_AB_wrt_B<3, 3, 3>(jRo, iRo.transpose()) * getJacobian_At_wrt_A<3, 3>() * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), iRo);
    }

    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 9, Eigen::RowMajor>> J(jacobians[1]);

      J = _intrinsic->getDerivativeProjectWrtRotation(T, pt_i_sphere) * getJacobian_AB_wrt_A<3, 3, 3>(jRo, iRo.transpose()) * getJacobian_AB_wrt_A<3, 3, 3>(Eigen::Matrix3d::Identity(), jRo);
    }

    if (jacobians[2] != nullptr) {
      Eigen::Map<Eigen::Matrix<double,  Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[2], 2, params_size);

      Eigen::Matrix<double, 4, 3> Jhomogenous = Eigen::Matrix<double, 4, 3>::Identity();

      Eigen::Matrix<double, 2, 2> Jscale = _intrinsic->getDerivativeProjectWrtScale(T, pt_i_sphere) + _intrinsic->getDerivativeProjectWrtPoint(T, pt_i_sphere) * Jhomogenous * _intrinsic->getDerivativetoUnitSphereWrtPoint(pt_i_undist) * _intrinsic->getDerivativeRemoveDistoWrtPt(pt_i_cam) * _intrinsic->getDerivativeIma2CamWrtScale(pt_i);
      Eigen::Matrix<double, 2, 2> Jpp = _intrinsic->getDerivativeProjectWrtPrincipalPoint(T, pt_i_sphere) + _intrinsic->getDerivativeProjectWrtPoint(T, pt_i_sphere) * Jhomogenous * _intrinsic->getDerivativetoUnitSphereWrtPoint(pt_i_undist) * _intrinsic->getDerivativeRemoveDistoWrtPt(pt_i_cam) * _intrinsic->getDerivativeIma2CamWrtPrincipalPoint();
      Eigen::Matrix<double, 2, Eigen::Dynamic> Jdisto = _intrinsic->getDerivativeProjectWrtDisto(T, pt_i_sphere) + _intrinsic->getDerivativeProjectWrtPoint(T, pt_i_sphere) * Jhomogenous * _intrinsic->getDerivativetoUnitSphereWrtPoint(pt_i_undist) * _intrinsic->getDerivativeRemoveDistoWrtDisto(pt_i_cam);

      J.block<2, 2>(0, 0) = Jscale;
      J.block<2, 2>(0, 2) = Jpp;
      J.block(0, 4, 2, disto_size) = Jdisto;
    }

    return true;
  }

private:
  Vec2 _fi;
  Vec2 _fj;
  std::shared_ptr<camera::Pinhole> _intrinsic;
};

namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;


void BundleAdjustmentPanoramaCeres::CeresOptions::setDenseBA()
{
  // default configuration use a DENSE representation
  preconditionerType  = ceres::JACOBI;
  linearSolverType = ceres::DENSE_SCHUR;
  sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE; // not used but just to avoid a warning in ceres
  ALICEVISION_LOG_DEBUG("BundleAdjustmentParnorama[Ceres]: DENSE_SCHUR");
}

void BundleAdjustmentPanoramaCeres::CeresOptions::setSparseBA()
{
  preconditionerType = ceres::JACOBI;
  // if Sparse linear solver are available
  // descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustmentParnorama[Ceres]: SPARSE_SCHUR, SUITE_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::CX_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustmentParnorama[Ceres]: SPARSE_SCHUR, CX_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::EIGEN_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustmentParnorama[Ceres]: SPARSE_SCHUR, EIGEN_SPARSE");
  }
  else
  {
    linearSolverType = ceres::DENSE_SCHUR;
    ALICEVISION_LOG_WARNING("BundleAdjustmentParnorama[Ceres]: no sparse BA available, fallback to dense BA.");
  }
}

bool BundleAdjustmentPanoramaCeres::Statistics::exportToFile(const std::string& folder, const std::string& filename) const
{
  std::ofstream os;
  os.open((fs::path(folder) / filename).string(), std::ios::app);

  if(!os.is_open())
  {
    ALICEVISION_LOG_DEBUG("Unable to open the Bundle adjustment statistics file: '" << filename << "'.");
    return false;
  }

  os.seekp(0, std::ios::end); // put the cursor at the end

  if(os.tellp() == std::streampos(0)) // 'tellp' return the cursor's position
  {
    // if the file does't exist: add a header.
    os << "Time/BA(s);RefinedPose;ConstPose;IgnoredPose;"
          "RefinedPts;ConstPts;IgnoredPts;"
          "RefinedK;ConstK;IgnoredK;"
          "ResidualBlocks;SuccessIteration;BadIteration;"
          "InitRMSE;FinalRMSE;"
          "d=-1;d=0;d=1;d=2;d=3;d=4;"
          "d=5;d=6;d=7;d=8;d=9;d=10+;\n";
  }

  std::map<EParameter, std::map<EParameterState, std::size_t>> states = parametersStates;
  std::size_t posesWithDistUpperThanTen = 0;

  for(const auto& it : nbCamerasPerDistance)
    if (it.first >= 10)
      posesWithDistUpperThanTen += it.second;

  os << time << ";"
     << states[EParameter::POSE][EParameterState::REFINED]  << ";"
     << states[EParameter::POSE][EParameterState::CONSTANT] << ";"
     << states[EParameter::POSE][EParameterState::IGNORED]  << ";"
     << states[EParameter::INTRINSIC][EParameterState::REFINED]  << ";"
     << states[EParameter::INTRINSIC][EParameterState::CONSTANT] << ";"
     << states[EParameter::INTRINSIC][EParameterState::IGNORED]  << ";"
     << nbResidualBlocks << ";"
     << nbSuccessfullIterations << ";"
     << nbUnsuccessfullIterations << ";"
     << RMSEinitial << ";"
     << RMSEfinal << ";";

     for(int i = -1; i < 10; ++i)
     {
       auto cdIt = nbCamerasPerDistance.find(i);
       if(cdIt != nbCamerasPerDistance.end())
        os << cdIt->second << ";";
       else
         os << "0;";
     }

     os << posesWithDistUpperThanTen << ";\n";

  os.close();
  return true;
}

void BundleAdjustmentPanoramaCeres::Statistics::show() const
{
  std::map<EParameter, std::map<EParameterState, std::size_t>> states = parametersStates;
  std::stringstream ss;

  if(!nbCamerasPerDistance.empty())
  {
    std::size_t nbCamNotConnected = 0;
    std::size_t nbCamDistEqZero = 0;
    std::size_t nbCamDistEqOne = 0;
    std::size_t nbCamDistUpperOne = 0;

    for(const auto & camdistIt : nbCamerasPerDistance)
    {
      if(camdistIt.first < 0)
        nbCamNotConnected += camdistIt.second;
      else if(camdistIt.first == 0)
        nbCamDistEqZero += camdistIt.second;
      else if(camdistIt.first == 1)
        nbCamDistEqOne += camdistIt.second;
      else if(camdistIt.first > 1)
        nbCamDistUpperOne += camdistIt.second;
    }

    ss << "\t- local strategy enabled: yes\n"
       << "\t- graph-distances distribution:\n"
       << "\t    - not connected: " << nbCamNotConnected << " cameras\n"
       << "\t    - D = 0: " << nbCamDistEqZero << " cameras\n"
       << "\t    - D = 1: " << nbCamDistEqOne << " cameras\n"
       << "\t    - D > 1: " << nbCamDistUpperOne << " cameras\n";
  }
  else
  {
      ss << "\t- local strategy enabled: no\n";
  }

  ALICEVISION_LOG_INFO("Bundle Adjustment Statistics:\n"
                        << ss.str()
                        << "\t- adjustment duration: " << time << " s\n"
                        << "\t- poses:\n"
                        << "\t    - # refined:  " << states[EParameter::POSE][EParameterState::REFINED]  << "\n"
                        << "\t    - # constant: " << states[EParameter::POSE][EParameterState::CONSTANT] << "\n"
                        << "\t    - # ignored:  " << states[EParameter::POSE][EParameterState::IGNORED]  << "\n"
                        << "\t- intrinsics:\n"
                        << "\t    - # refined:  " << states[EParameter::INTRINSIC][EParameterState::REFINED]  << "\n"
                        << "\t    - # constant: " << states[EParameter::INTRINSIC][EParameterState::CONSTANT] << "\n"
                        << "\t    - # ignored:  " << states[EParameter::INTRINSIC][EParameterState::IGNORED]  << "\n"
                        << "\t- # residual blocks: " << nbResidualBlocks << "\n"
                        << "\t- # successful iterations: " << nbSuccessfullIterations   << "\n"
                        << "\t- # unsuccessful iterations: " << nbUnsuccessfullIterations << "\n"
                        << "\t- initial RMSE: " << RMSEinitial << "\n"
                        << "\t- final   RMSE: " << RMSEfinal);
}

void BundleAdjustmentPanoramaCeres::setSolverOptions(ceres::Solver::Options& solverOptions) const
{
  solverOptions.preconditioner_type = _ceresOptions.preconditionerType;
  solverOptions.linear_solver_type = _ceresOptions.linearSolverType;
  solverOptions.sparse_linear_algebra_library_type = _ceresOptions.sparseLinearAlgebraLibraryType;
  solverOptions.minimizer_progress_to_stdout = _ceresOptions.verbose;
  solverOptions.logging_type = ceres::SILENT;
  solverOptions.num_threads = _ceresOptions.nbThreads;
  solverOptions.max_num_iterations = 300;


#if CERES_VERSION_MAJOR < 2
  solverOptions.num_linear_solver_threads = _ceresOptions.nbThreads;
#endif
}

void BundleAdjustmentPanoramaCeres::addExtrinsicsToProblem(const sfmData::SfMData& sfmData, BundleAdjustment::ERefineOptions refineOptions, ceres::Problem& problem)
{
  const bool refineRotation = refineOptions & BundleAdjustment::REFINE_ROTATION;

  const auto addPose = [&](const sfmData::CameraPose& cameraPose, bool isConstant, SO3::Matrix& poseBlock)
  {
    const Mat3& R = cameraPose.getTransform().rotation();
    poseBlock = R;
    double* poseBlockPtr = poseBlock.data();

    /*Define rotation parameterization*/
    problem.AddParameterBlock(poseBlockPtr, 9, new SO3::LocalParameterization);

    // keep the camera extrinsics constants
    if(cameraPose.isLocked() || isConstant || !refineRotation)
    {
      // set the whole parameter block as constant.
      _statistics.addState(EParameter::POSE, EParameterState::CONSTANT);
      problem.SetParameterBlockConstant(poseBlockPtr);
      return;
    }

    _statistics.addState(EParameter::POSE, EParameterState::REFINED);
  };

  // setup poses data
  for(const auto& posePair : sfmData.getPoses())
  {
    const IndexT poseId = posePair.first;
    const sfmData::CameraPose& pose = posePair.second;

    // skip camera pose set as Ignored in the Local strategy
    if(getPoseState(poseId) == EParameterState::IGNORED)
    {
      _statistics.addState(EParameter::POSE, EParameterState::IGNORED);
      continue;
    }

    const bool isConstant = (getPoseState(poseId) == EParameterState::CONSTANT);

    addPose(pose, isConstant, _posesBlocks[poseId]);
  }
}

void BundleAdjustmentPanoramaCeres::addIntrinsicsToProblem(const sfmData::SfMData& sfmData, BundleAdjustment::ERefineOptions refineOptions, ceres::Problem& problem)
{
  const std::size_t minImagesForOpticalCenter = 3;
  
  const bool refineIntrinsicsOpticalCenter = (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
  const bool refineIntrinsicsFocalLength = refineOptions & REFINE_INTRINSICS_FOCAL;
  const bool refineIntrinsicsDistortion = refineOptions & REFINE_INTRINSICS_DISTORTION;
  const bool refineIntrinsics = refineIntrinsicsDistortion || refineIntrinsicsFocalLength || refineIntrinsicsOpticalCenter;

  std::map<IndexT, std::size_t> intrinsicsUsage;

  // count the number of reconstructed views per intrinsic
  for(const auto& viewPair: sfmData.getViews())
  {
    const sfmData::View& view = *(viewPair.second);

    if(intrinsicsUsage.find(view.getIntrinsicId()) == intrinsicsUsage.end()) {
      intrinsicsUsage[view.getIntrinsicId()] = 0;
    }

    if(sfmData.isPoseAndIntrinsicDefined(&view)) {
      ++intrinsicsUsage.at(view.getIntrinsicId());
    }
  }

  for(const auto& intrinsicPair: sfmData.getIntrinsics())
  {
    const IndexT intrinsicId = intrinsicPair.first;
    const auto& intrinsicPtr = intrinsicPair.second;
    const auto usageIt = intrinsicsUsage.find(intrinsicId);
    if(usageIt == intrinsicsUsage.end()) {
      // if the intrinsic is never referenced by any view, skip it
      continue;
    }
    const std::size_t usageCount = usageIt->second;

    // do not refine an intrinsic does not used by any reconstructed view
    if(usageCount == UndefinedIndexT || getIntrinsicState(intrinsicId) == EParameterState::IGNORED)
    {
      _statistics.addState(EParameter::INTRINSIC, EParameterState::IGNORED);
      continue;
    }

    assert(isValid(intrinsicPtr->getType()));

    std::vector<double>& intrinsicBlock = _intrinsicsBlocks[intrinsicId];
    intrinsicBlock = intrinsicPtr->getParams();

    double* intrinsicBlockPtr = intrinsicBlock.data();
    problem.AddParameterBlock(intrinsicBlockPtr, intrinsicBlock.size());

    // keep the camera intrinsic constant
    if(intrinsicPtr->isLocked() || !refineIntrinsics || getIntrinsicState(intrinsicId) == EParameterState::CONSTANT)
    {
      // set the whole parameter block as constant.
      _statistics.addState(EParameter::INTRINSIC, EParameterState::CONSTANT);
      problem.SetParameterBlockConstant(intrinsicBlockPtr);
      continue;
    }

    // constant parameters
    std::vector<int> constantIntrinisc;

    // refine the focal length
    if(refineIntrinsicsFocalLength)
    {
      // we don't have an initial guess, but we assume that we use
      // a converging lens, so the focal length should be positive.
      problem.SetParameterLowerBound(intrinsicBlockPtr, 0, 0.0);
      problem.SetParameterLowerBound(intrinsicBlockPtr, 1, 0.0);
    }
    else
    {
      // set focal length as constant
      constantIntrinisc.push_back(0);
      constantIntrinisc.push_back(1);
    }

    // optical center
    bool optional_center = ((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA) && (usageCount > minImagesForOpticalCenter));
    if((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || optional_center)
    {
      // refine optical center within 10% of the image size.
      assert(intrinsicBlock.size() >= 4);

      const double opticalCenterMinPercent = -0.05;
      const double opticalCenterMaxPercent =  0.05;

      // add bounds to the principal point
      problem.SetParameterLowerBound(intrinsicBlockPtr, 2, opticalCenterMinPercent * intrinsicPtr->w());
      problem.SetParameterUpperBound(intrinsicBlockPtr, 2, opticalCenterMaxPercent * intrinsicPtr->w());
      problem.SetParameterLowerBound(intrinsicBlockPtr, 3, opticalCenterMinPercent * intrinsicPtr->h());
      problem.SetParameterUpperBound(intrinsicBlockPtr, 3, opticalCenterMaxPercent * intrinsicPtr->h());
    }
    else
    {
      // don't refine the optical center
      constantIntrinisc.push_back(2);
      constantIntrinisc.push_back(3);
    }

    // lens distortion
    if(!refineIntrinsicsDistortion) {
      for(std::size_t i = 4; i < intrinsicBlock.size(); ++i) {
        constantIntrinisc.push_back(i);
      }
    }

    if(!constantIntrinisc.empty())
    {
      ceres::SubsetParameterization* subsetParameterization = new ceres::SubsetParameterization(intrinsicBlock.size(), constantIntrinisc);
      problem.SetParameterization(intrinsicBlockPtr, subsetParameterization);
    }

    _statistics.addState(EParameter::INTRINSIC, EParameterState::REFINED);
  }
}


void BundleAdjustmentPanoramaCeres::addConstraints2DToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
  // set a LossFunction to be less penalized by false measurements.
  // note: set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction* lossFunction = new ceres::HuberLoss(Square(8.0)); // TODO: make the LOSS function and the parameter an option

  for (const auto & constraint : sfmData.getConstraints2D()) {
    const sfmData::View& view_1 = sfmData.getView(constraint.ViewFirst);
    const sfmData::View& view_2 = sfmData.getView(constraint.ViewSecond);

    assert(getPoseState(view_1.getPoseId()) != EParameterState::IGNORED);
    assert(getIntrinsicState(view_1.getIntrinsicId()) != EParameterState::IGNORED);
    assert(getPoseState(view_2.getPoseId()) != EParameterState::IGNORED);
    assert(getIntrinsicState(view_2.getIntrinsicId()) != EParameterState::IGNORED);

    /* Get pose */
    double * poseBlockPtr_1 = _posesBlocks.at(view_1.getPoseId()).data();
    double * poseBlockPtr_2 = _posesBlocks.at(view_2.getPoseId()).data();

    /*Get intrinsics */
    double * intrinsicBlockPtr_1 = _intrinsicsBlocks.at(view_1.getIntrinsicId()).data();
    double * intrinsicBlockPtr_2 = _intrinsicsBlocks.at(view_2.getIntrinsicId()).data();

    /* For the moment assume a unique camera */
    assert(intrinsicBlockPtr_1 == intrinsicBlockPtr_2);

    std::shared_ptr<IntrinsicBase> intrinsic = sfmData.getIntrinsicsharedPtr(view_1.getIntrinsicId());
    std::shared_ptr<camera::EquiDistant> equidistant = std::dynamic_pointer_cast<camera::EquiDistant>(intrinsic);
    std::shared_ptr<camera::Pinhole> pinhole = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

    if (equidistant != nullptr)  
    {
      ceres::CostFunction* costFunction = new CostEquiDistant(constraint.ObservationFirst.x, constraint.ObservationSecond.x, equidistant);
      problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2, intrinsicBlockPtr_1);

      /* Symmetry */
      costFunction = new CostEquiDistant(constraint.ObservationSecond.x, constraint.ObservationFirst.x, equidistant);
      problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_2, poseBlockPtr_1, intrinsicBlockPtr_1);
    }
    else if (pinhole != nullptr)  
    {
      ceres::CostFunction* costFunction = new CostPinHole(constraint.ObservationFirst.x, constraint.ObservationSecond.x, pinhole);
      problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2, intrinsicBlockPtr_1);
      /* Symmetry */
      costFunction = new CostPinHole(constraint.ObservationSecond.x, constraint.ObservationFirst.x, pinhole);
      problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_2, poseBlockPtr_1, intrinsicBlockPtr_1);
    }
    else {
      ALICEVISION_LOG_ERROR("Incompatible camera for a 2D constraint");
      return;
    }
  }
}

void BundleAdjustmentPanoramaCeres::addRotationPriorsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
  // set a LossFunction to be less penalized by false measurements.
  // note: set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction* lossFunction = nullptr;

  for (const auto & prior : sfmData.getRotationPriors()) {

    const sfmData::View& view_1 = sfmData.getView(prior.ViewFirst);
    const sfmData::View& view_2 = sfmData.getView(prior.ViewSecond);

    assert(getPoseState(view_1.getPoseId()) != EParameterState::IGNORED);
    assert(getPoseState(view_2.getPoseId()) != EParameterState::IGNORED);

    double * poseBlockPtr_1 = _posesBlocks.at(view_1.getPoseId()).data();
    double * poseBlockPtr_2 = _posesBlocks.at(view_2.getPoseId()).data();

    ceres::CostFunction* costFunction = new CostRotationPrior(prior._second_R_first);
    problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2);
  }
}

void BundleAdjustmentPanoramaCeres::createProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
  // clear previously computed data
  resetProblem();

  // add SfM extrincics to the Ceres problem
  addExtrinsicsToProblem(sfmData, refineOptions, problem);

  // add SfM intrinsics to the Ceres problem
  addIntrinsicsToProblem(sfmData, refineOptions, problem);

  // add 2D constraints to the Ceres problem
  addConstraints2DToProblem(sfmData, refineOptions, problem);

  // add rotation priors to the Ceres problem
  addRotationPriorsToProblem(sfmData, refineOptions, problem);
}

void BundleAdjustmentPanoramaCeres::updateFromSolution(sfmData::SfMData& sfmData, ERefineOptions refineOptions) const
{
  const bool refinePoses = (refineOptions & REFINE_ROTATION) || (refineOptions & REFINE_TRANSLATION);
  const bool refineIntrinsicsOpticalCenter = (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
  const bool refineIntrinsics = (refineOptions & REFINE_INTRINSICS_FOCAL) || (refineOptions & REFINE_INTRINSICS_DISTORTION) || refineIntrinsicsOpticalCenter;
  const bool refineStructure = refineOptions & REFINE_STRUCTURE;

  // update camera poses with refined data
  if(refinePoses)
  {
    // absolute poses
    for(auto& posePair : sfmData.getPoses())
    {
      const IndexT poseId = posePair.first;

      // do not update a camera pose set as Ignored or Constant in the Local strategy
      if(getPoseState(poseId) != EParameterState::REFINED) {
        continue;
      }

      const SO3::Matrix & poseBlock = _posesBlocks.at(poseId);

      // update the pose
      posePair.second.setTransform(poseFromRT(poseBlock, Vec3(0,0,0)));
    }
  }

  // update camera intrinsics with refined data
  if(refineIntrinsics)
  {
    for(const auto& intrinsicBlockPair: _intrinsicsBlocks)
    {
      const IndexT intrinsicId = intrinsicBlockPair.first;

      // do not update a camera pose set as Ignored or Constant in the Local strategy
      if(getIntrinsicState(intrinsicId) != EParameterState::REFINED) {
        continue;
      }

      sfmData.getIntrinsics().at(intrinsicId)->updateFromParams(intrinsicBlockPair.second);
    }
  }
}

bool BundleAdjustmentPanoramaCeres::adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions)
{
  // create problem
  ceres::Problem problem;
  createProblem(sfmData, refineOptions, problem);
  
  // configure a Bundle Adjustment engine and run it
  // make Ceres automatically detect the bundle structure.
  ceres::Solver::Options options;
  setSolverOptions(options);

  // solve BA
  ceres::Solver::Summary summary;  
  ceres::Solve(options, &problem, &summary);

  // print summary
  if(_ceresOptions.summary) {
    ALICEVISION_LOG_INFO(summary.FullReport());
  }

  // solution is not usable
  if(!summary.IsSolutionUsable())
  {
    ALICEVISION_LOG_WARNING("Bundle Adjustment failed, the solution is not usable.");
    return false;
  }

  // update input sfmData with the solution
  updateFromSolution(sfmData, refineOptions);

  // store some statitics from the summary
  _statistics.time = summary.total_time_in_seconds;
  _statistics.nbSuccessfullIterations = summary.num_successful_steps;
  _statistics.nbUnsuccessfullIterations = summary.num_unsuccessful_steps;
  _statistics.nbResidualBlocks = summary.num_residuals;
  _statistics.RMSEinitial = std::sqrt(summary.initial_cost / summary.num_residuals);
  _statistics.RMSEfinal = std::sqrt(summary.final_cost / summary.num_residuals);

  return true;
}



} // namespace sfm
} // namespace aliceVision

