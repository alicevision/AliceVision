// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/BundleAdjustmentSymbolicCeres.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/camera/Equidistant.hpp>
#include <aliceVision/utils/CeresUtils.hpp>

#include <boost/filesystem.hpp>

#include <ceres/rotation.h>

#include <fstream>


namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;

class IntrinsicsSymbolicParameterization : public ceres::LocalParameterization {
 public:
  explicit IntrinsicsSymbolicParameterization(size_t parametersSize, double focalRatio, bool lockFocal, bool lockFocalRatio, bool lockCenter, bool lockDistortion)
  : _globalSize(parametersSize),
    _focalRatio(focalRatio),
    _lockFocal(lockFocal),
    _lockFocalRatio(lockFocalRatio),
    _lockCenter(lockCenter),
    _lockDistortion(lockDistortion)
  {
    _distortionSize = _ambientSize - 4;
    _tangentSize = 0;

    if (!_lockFocal)
    {
      if (_lockFocalRatio)
      {
        _tangentSize += 1;
      }
      else
      {
        _tangentSize += 2;
      }
    }
    else
    {
        if (_lockFocalRatio)
        {
        }
        else
        {
            _localSize += 1;
        }
    }

    if (!_lockCenter)
    {
      _tangentSize += 2;
    }

    if (!_lockDistortion)
    {
      _tangentSize += _distortionSize;
    }
  }

  virtual ~IntrinsicsSymbolicParameterization() = default;


  virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
  {
    for (int i = 0; i < _ambientSize; i++)
    {
      x_plus_delta[i] = x[i];
    }
    
    size_t posDelta = 0;
    if (!_lockFocal)
    {
      if (_lockFocalRatio)
      {
        x_plus_delta[0] = x[0] + delta[posDelta]; 
        x_plus_delta[1] = x[1] + _focalRatio * delta[posDelta];
        ++posDelta;
      }
      else
      {
        x_plus_delta[0] = x[0] + delta[posDelta];
        ++posDelta;
        x_plus_delta[1] = x[1] + delta[posDelta];
        ++posDelta;
      }
    }
    else
    {
        if (_lockFocalRatio)
        {
        }
        else
        {
            x_plus_delta[0] = x[0];
            x_plus_delta[1] = x[1] + delta[posDelta];
            ++posDelta;
        }
    }

    if (!_lockCenter)
    {
      x_plus_delta[2] = x[2] + delta[posDelta]; 
      ++posDelta;

      x_plus_delta[3] = x[3] + delta[posDelta];
      ++posDelta;
    }

    if (!_lockDistortion)
    {
      for (int i = 0; i < _distortionSize; i++)
      {
        x_plus_delta[4 + i] = x[4 + i] + delta[posDelta];
        ++posDelta;
      }
    }

    return true;
  }

  virtual bool ComputeJacobian(const double* x, double* jacobian) const override
  {    
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, AmbientSize(), TangentSize());

    J.fill(0);

    size_t posDelta = 0;
    if (!_lockFocal)
    {
      if (_lockFocalRatio)
      {
        J(0, posDelta) = 1.0;
        J(1, posDelta) = _focalRatio;
        ++posDelta;
      }
      else
      {
        J(0, posDelta) = 1.0;
        ++posDelta;
        J(1, posDelta) = 1.0;
        ++posDelta;
      }
    }
    else
    {
        if (_lockFocalRatio)
        {
        }
        else
        {
            J(0, posDelta) = 0.0;
            J(1, posDelta) = 1.0;
            ++posDelta;
        }
    }

    if (!_lockCenter)
    {
      J(2, posDelta) = 1.0;
      ++posDelta;

      J(3, posDelta) = 1.0;
      ++posDelta;
    }

    if (!_lockDistortion)
    {
      for (int i = 0; i < _distortionSize; i++)
      {
        J(4 + i, posDelta) = 1.0;
        ++posDelta;
      }
    }

    return true;
  }

  bool Minus(const double* y, const double* x, double* delta) const override {
    throw std::invalid_argument("IntrinsicsManifold::Minus() should never be called");
  }

  bool MinusJacobian(const double* x, double* jacobian) const override {
    throw std::invalid_argument("IntrinsicsManifold::MinusJacobian() should never be called");
  }

  int AmbientSize() const override
  {
    return _ambientSize;
  }

  int TangentSize() const override
  { 
    return _tangentSize;
  }

 private:
  size_t _distortionSize;
  size_t _ambientSize;
  size_t _tangentSize;
  double _focalRatio;
  bool _lockFocal;
  bool _lockFocalRatio;
  bool _lockCenter;
  bool _lockDistortion;
};


class CostProjection : public ceres::CostFunction {
public:
  CostProjection(const sfmData::Observation& measured, const std::shared_ptr<camera::IntrinsicBase> & intrinsics, const std::shared_ptr<camera::Undistortion> & undistortion, bool withRig) : _measured(measured), _intrinsics(intrinsics), _undistortion(undistortion), _withRig(withRig)
  {
    set_num_residuals(2);

    mutable_parameter_block_sizes()->push_back(16);
    mutable_parameter_block_sizes()->push_back(16);
    mutable_parameter_block_sizes()->push_back(intrinsics->getParams().size());    
    mutable_parameter_block_sizes()->push_back(3);    

    if (undistortion)
    {
        mutable_parameter_block_sizes()->push_back(2);
        mutable_parameter_block_sizes()->push_back(undistortion->getParameters().size());
    }
  }

  bool Evaluate(double const * const * parameters, double * residuals, double ** jacobians) const override
  {
    const double * parameter_pose = parameters[0];
    const double * parameter_rig = parameters[1];
    const double * parameter_intrinsics = parameters[2];
    const double * parameter_landmark = parameters[3];

    const Eigen::Map<const SE3::Matrix> rTo(parameter_pose);
    const Eigen::Map<const SE3::Matrix> cTr(parameter_rig);
    const Eigen::Map<const Vec3> pt(parameter_landmark);

    /*Update intrinsics object with estimated parameters*/
    size_t params_size = _intrinsics->getParams().size();
    std::vector<double> params;
    for (size_t param_id = 0; param_id < params_size; param_id++) {
      params.push_back(parameter_intrinsics[param_id]);
    }
    _intrinsics->updateFromParams(params);

    if (_undistortion)
    {
        const double* parameter_undistortionOffset = parameters[4];
        const double* parameter_undistortionParams = parameters[5];

        _undistortion->setOffset({ parameter_undistortionOffset[0], parameter_undistortionOffset[1] });

        size_t params_size = _undistortion->getParameters().size();
        std::vector<double> paramsUndist;
        for (size_t param_id = 0; param_id < params_size; param_id++) {
            paramsUndist.push_back(parameter_undistortionParams[param_id]);
        }

        _undistortion->setParameters(paramsUndist);
    }

    const SE3::Matrix T = cTr * rTo;
    const geometry::Pose3 T_pose3(T.block<3, 4>(0, 0));

    const Vec4 pth = pt.homogeneous();
    const Vec2 pt_est = _intrinsics->project(T_pose3, pth, true);
    const double scale = (_measured.scale > 1e-12) ? _measured.scale : 1.0;

    Vec2 measured_undistorted = _measured.x;
    if (_undistortion)
    {
        measured_undistorted = _undistortion->undistort(_measured.x);
    }

    residuals[0] = (pt_est(0) - measured_undistorted(0)) / scale;
    residuals[1] = (pt_est(1) - measured_undistorted(1)) / scale;

    if (jacobians == nullptr) {
      return true;
    }

    Eigen::Matrix2d d_res_d_pt_est = Eigen::Matrix2d::Identity() / scale;

    if (jacobians[0] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor>> J(jacobians[0]);

      J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPose(T_pose3, pth) * getJacobian_AB_wrt_B<4, 4, 4>(cTr, rTo) * getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), rTo);
    }

    if (jacobians[1] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor>> J(jacobians[1]);
      
      J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPose(T_pose3, pth) * getJacobian_AB_wrt_A<4, 4, 4>(cTr, rTo) * getJacobian_AB_wrt_A<4, 4, 4>(Eigen::Matrix4d::Identity(), cTr);
    }

    if (jacobians[2] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[2], 2, params_size);
      
      J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtParams(T_pose3, pth);
    }

    if (jacobians[3] != nullptr) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J(jacobians[3]);


      J = d_res_d_pt_est * _intrinsics->getDerivativeProjectWrtPoint(T_pose3, pth) * Eigen::Matrix<double, 4, 3>::Identity();
    }

    if (_undistortion)
    {
        size_t undistortion_params_size = _undistortion->getParameters().size();

        if (jacobians[4] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> J(jacobians[4]);

            J = -d_res_d_pt_est * _undistortion->getDerivativeUndistortWrtOffset(_measured.x);
            J.fill(0);
        }

        if (jacobians[5] != nullptr) {
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobians[5], 2, undistortion_params_size);

            J = -d_res_d_pt_est * _undistortion->getDerivativeUndistortWrtParameters(_measured.x);
        }
    }

    return true;
  }

private:
  const sfmData::Observation & _measured;
  const std::shared_ptr<camera::IntrinsicBase> _intrinsics;
  std::shared_ptr<camera::Undistortion> _undistortion;
  bool _withRig;
};



void BundleAdjustmentSymbolicCeres::addPose(const sfmData::CameraPose& cameraPose, bool isConstant, SE3::Matrix & poseBlock, ceres::Problem& problem, bool refineTranslation, bool refineRotation, bool constraintPosition)
{
  const Mat3& R = cameraPose.getTransform().rotation();
  const Vec3& t = cameraPose.getTransform().translation();

  poseBlock = SE3::Matrix::Identity();
  poseBlock.block<3,3>(0, 0) = R;
  poseBlock.block<3,1>(0, 3) = t;
  double * poseBlockPtr = poseBlock.data();
  problem.AddParameterBlock(poseBlockPtr, 16);


  // add pose parameter to the all parameters blocks pointers list
  _allParametersBlocks.push_back(poseBlockPtr);

  // keep the camera extrinsics constants
  if(cameraPose.isLocked() || isConstant || (!refineTranslation && !refineRotation))
  {
    // set the whole parameter block as constant.
    _statistics.addState(EParameter::POSE, EParameterState::CONSTANT);

    problem.SetParameterBlockConstant(poseBlockPtr);
    return;
  }

  if (refineRotation && refineTranslation)
  {
#if ALICEVISION_CERES_HAS_MANIFOLD
    problem.SetManifold(poseBlockPtr, new SE3::Manifold(refineRotation, refineTranslation));
#else
    problem.SetParameterization(poseBlockPtr, new utils::ManifoldToParameterizationWrapper(
                                    new SE3::Manifold(refineRotation, refineTranslation)));
#endif
  }
  else
  {
    ALICEVISION_THROW_ERROR("BundleAdjustmentSymbolicCeres: Constant extrinsics not supported at this time");
  }


  _statistics.addState(EParameter::POSE, EParameterState::REFINED);
}

void BundleAdjustmentSymbolicCeres::CeresOptions::setDenseBA()
{
  // default configuration use a DENSE representation
  preconditionerType  = ceres::JACOBI;
  linearSolverType = ceres::DENSE_SCHUR;
  sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE; // not used but just to avoid a warning in ceres
  ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: DENSE_SCHUR");
}

void BundleAdjustmentSymbolicCeres::CeresOptions::setSparseBA()
{
  preconditionerType = ceres::JACOBI;
  // if Sparse linear solver are available
  // descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: SPARSE_SCHUR, SUITE_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::CX_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: SPARSE_SCHUR, CX_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::EIGEN_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustmentSymbolic[Ceres]: SPARSE_SCHUR, EIGEN_SPARSE");
  }
  else
  {
    linearSolverType = ceres::DENSE_SCHUR;
    ALICEVISION_LOG_WARNING("BundleAdjustmentSymbolic[Ceres]: no sparse BA available, fallback to dense BA.");
  }
}

bool BundleAdjustmentSymbolicCeres::Statistics::exportToFile(const std::string& folder, const std::string& filename) const
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
  {
      if (it.first >= 10)
      {
          posesWithDistUpperThanTen += it.second;
      }
  }

  os << time << ";"
     << states[EParameter::POSE][EParameterState::REFINED]  << ";"
     << states[EParameter::POSE][EParameterState::CONSTANT] << ";"
     << states[EParameter::POSE][EParameterState::IGNORED]  << ";"
     << states[EParameter::LANDMARK][EParameterState::REFINED]  << ";"
     << states[EParameter::LANDMARK][EParameterState::CONSTANT] << ";"
     << states[EParameter::LANDMARK][EParameterState::IGNORED]  << ";"
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

void BundleAdjustmentSymbolicCeres::Statistics::show() const
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
      else if(camdistIt.first == 1)
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
                        << "\t- landmarks:\n"
                        << "\t    - # refined:  " << states[EParameter::LANDMARK][EParameterState::REFINED]  << "\n"
                        << "\t    - # constant: " << states[EParameter::LANDMARK][EParameterState::CONSTANT] << "\n"
                        << "\t    - # ignored:  " << states[EParameter::LANDMARK][EParameterState::IGNORED]  << "\n"
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

void BundleAdjustmentSymbolicCeres::setSolverOptions(ceres::Solver::Options& solverOptions) const
{
  solverOptions.preconditioner_type = _ceresOptions.preconditionerType;
  solverOptions.linear_solver_type = _ceresOptions.linearSolverType;
  solverOptions.sparse_linear_algebra_library_type = _ceresOptions.sparseLinearAlgebraLibraryType;
  solverOptions.minimizer_progress_to_stdout = _ceresOptions.verbose;
  solverOptions.logging_type = ceres::SILENT;
  solverOptions.num_threads = 1;// _ceresOptions.nbThreads;
  solverOptions.max_num_iterations = 1000;

#if CERES_VERSION_MAJOR < 2
  solverOptions.num_linear_solver_threads = _ceresOptions.nbThreads;
#endif

  if(_ceresOptions.useParametersOrdering)
  {
    // copy ParameterBlockOrdering
    solverOptions.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering(_linearSolverOrdering));
  }
}

void BundleAdjustmentSymbolicCeres::addExtrinsicsToProblem(const sfmData::SfMData& sfmData, BundleAdjustment::ERefineOptions refineOptions, ceres::Problem& problem)
{
  const bool refineTranslation = refineOptions & BundleAdjustment::REFINE_TRANSLATION;
  const bool refineRotation = refineOptions & BundleAdjustment::REFINE_ROTATION;

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

    addPose(pose, isConstant, _posesBlocks[poseId], problem, refineTranslation, refineRotation, true);
  }

  // setup sub-poses data
  for(const auto& rigPair : sfmData.getRigs())
  {
    const IndexT rigId = rigPair.first;
    const sfmData::Rig& rig = rigPair.second;
    const std::size_t nbSubPoses = rig.getNbSubPoses();

    for(std::size_t subPoseId = 0 ; subPoseId < nbSubPoses; ++subPoseId)
    {
      const sfmData::RigSubPose& rigSubPose = rig.getSubPose(subPoseId);

      if(rigSubPose.status == sfmData::ERigSubPoseStatus::UNINITIALIZED)
        continue;

      const bool isConstant = (rigSubPose.status == sfmData::ERigSubPoseStatus::CONSTANT);

      addPose(sfmData::CameraPose(rigSubPose.pose), isConstant, _rigBlocks[rigId][subPoseId], problem, refineTranslation, refineRotation, false);
    }
  }


  //Add default rig pose
  addPose(sfmData::CameraPose(), true, _rigNull, problem, refineTranslation, refineRotation, false);
}

void BundleAdjustmentSymbolicCeres::addIntrinsicsToProblem(const sfmData::SfMData& sfmData, BundleAdjustment::ERefineOptions refineOptions, ceres::Problem& problem)
{
  const bool refineIntrinsicsOpticalCenter = (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
  const bool refineIntrinsicsFocalLength = refineOptions & REFINE_INTRINSICS_FOCAL;
  const bool refineIntrinsicsDistortion = refineOptions & REFINE_INTRINSICS_DISTORTION;
  const bool refineIntrinsicsUndistortion = refineOptions & REFINE_INTRINSICS_UNDISTORTION;
  const bool refineIntrinsics = refineIntrinsicsDistortion || refineIntrinsicsFocalLength || refineIntrinsicsOpticalCenter;
  const bool fixFocalRatio = true;

  std::map<IndexT, std::size_t> intrinsicsUsage;

  // count the number of reconstructed views per intrinsic
  for(const auto& viewPair: sfmData.getViews())
  {
    const sfmData::View& view = *(viewPair.second);

    if(intrinsicsUsage.find(view.getIntrinsicId()) == intrinsicsUsage.end())
      intrinsicsUsage[view.getIntrinsicId()] = 0;

    if(sfmData.isPoseAndIntrinsicDefined(&view))
      ++intrinsicsUsage.at(view.getIntrinsicId());
  }

  for(const auto& intrinsicPair: sfmData.getIntrinsics())
  {
    const IndexT intrinsicId = intrinsicPair.first;
    const auto& intrinsicPtr = intrinsicPair.second;
    const auto usageIt = intrinsicsUsage.find(intrinsicId);
    if(usageIt == intrinsicsUsage.end())
      // if the intrinsic is never referenced by any view, skip it
      continue;
    const std::size_t usageCount = usageIt->second;

    // do not refine an intrinsic does not used by any reconstructed view
    if(usageCount <= 0 || getIntrinsicState(intrinsicId) == EParameterState::IGNORED)
    {
      _statistics.addState(EParameter::INTRINSIC, EParameterState::IGNORED);
      continue;
    }

    assert(isValid(intrinsicPtr->getType()));

    std::vector<double>& intrinsicBlock = _intrinsicsBlocks[intrinsicId];
    intrinsicBlock = intrinsicPtr->getParams();

    double* intrinsicBlockPtr = intrinsicBlock.data();
    problem.AddParameterBlock(intrinsicBlockPtr, intrinsicBlock.size());

    // add intrinsic parameter to the all parameters blocks pointers list
    _allParametersBlocks.push_back(intrinsicBlockPtr);

    // keep the camera intrinsic constant
    if(intrinsicPtr->isLocked() || !refineIntrinsics || getIntrinsicState(intrinsicId) == EParameterState::CONSTANT)
    {
      // set the whole parameter block as constant.
      _statistics.addState(EParameter::INTRINSIC, EParameterState::CONSTANT);
      problem.SetParameterBlockConstant(intrinsicBlockPtr);
      continue;
    }

    // constant parameters
    bool lockCenter = false;
    bool lockFocal = false;
    bool lockRatio = true;
    bool lockDistortion = false;
    double focalRatio = 1.0;

    // refine the focal length
    if(refineIntrinsicsFocalLength)
    {
      std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicScaleOffset = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsicPtr);
      if(intrinsicScaleOffset->getInitialScale().x() > 0 && intrinsicScaleOffset->getInitialScale().y() > 0)
      {
        // if we have an initial guess, we only authorize a margin around this value.
        assert(intrinsicBlock.size() >= 1);
        const unsigned int maxFocalError = 0.2 * std::max(intrinsicPtr->w(), intrinsicPtr->h()); // TODO : check if rounding is needed
        problem.SetParameterLowerBound(intrinsicBlockPtr, 0, static_cast<double>(intrinsicScaleOffset->getInitialScale().x() - maxFocalError));
        problem.SetParameterUpperBound(intrinsicBlockPtr, 0, static_cast<double>(intrinsicScaleOffset->getInitialScale().x() + maxFocalError));
        problem.SetParameterLowerBound(intrinsicBlockPtr, 1, static_cast<double>(intrinsicScaleOffset->getInitialScale().y() - maxFocalError));
        problem.SetParameterUpperBound(intrinsicBlockPtr, 1, static_cast<double>(intrinsicScaleOffset->getInitialScale().y() + maxFocalError));
      }
      else // no initial guess
      {
        // we don't have an initial guess, but we assume that we use
        // a converging lens, so the focal length should be positive.
        problem.SetParameterLowerBound(intrinsicBlockPtr, 0, 0.0);
        problem.SetParameterLowerBound(intrinsicBlockPtr, 1, 0.0);
      }

      focalRatio = intrinsicBlockPtr[1] / intrinsicBlockPtr[0];

      std::shared_ptr<camera::IntrinsicsScaleOffset> castedcam_iso = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsicPtr);
      if (castedcam_iso)
      {
        lockRatio = castedcam_iso->isRatioLocked();
      }
    }
    else
    {
      // set focal length as constant
      lockFocal = true;
    }

    const bool optional_center = ((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA) && (usageCount > _minNbImagesToRefineOpticalCenter));
    if ((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || optional_center)
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
      lockCenter = true;
    }

    // lens distortion
    if(!refineIntrinsicsDistortion)
    {
      lockDistortion = true;
    }

    IntrinsicsSymbolicParameterization* subsetParameterization = new IntrinsicsSymbolicParameterization(intrinsicBlock.size(), focalRatio, lockFocal, lockRatio, lockCenter, lockDistortion);
    problem.SetParameterization(intrinsicBlockPtr, subsetParameterization);    

    _statistics.addState(EParameter::INTRINSIC, EParameterState::REFINED);
  }
}

void BundleAdjustmentSymbolicCeres::addUndistortionsToProblem(const sfmData::SfMData& sfmData, BundleAdjustment::ERefineOptions refineOptions, ceres::Problem& problem)
{
    const bool refineIntrinsicsUndistortion = refineOptions & REFINE_INTRINSICS_UNDISTORTION;

    /*Sort undistortions by frame id*/
    std::map<IndexT, IndexT> sorted_undistortions;
    for (auto v : sfmData.getViews())
    {
        IndexT uid = v.second->getUndistortionId();
        if (uid == UndefinedIndexT)
        {
            continue;
        }

        
        sorted_undistortions[v.second->getFrameId()] = uid;
    }

    for (auto p : sfmData.getUndistortions())
    {
        std::shared_ptr<camera::Undistortion> undistortionObject = p.second;

        _undistortionOffsetBlocks[p.first] = undistortionObject->getOffset();
        _undistortionParametersBlocks[p.first] = undistortionObject->getParameters();


        Vec2& undistOffset = _undistortionOffsetBlocks[p.first];
        std::vector<double>& undistParameters = _undistortionParametersBlocks[p.first];

        problem.AddParameterBlock(undistOffset.data(), 2);
        problem.AddParameterBlock(undistParameters.data(), undistParameters.size());

        if (!refineIntrinsicsUndistortion)
        {
            problem.SetParameterBlockConstant(undistOffset.data());
            problem.SetParameterBlockConstant(undistParameters.data());
        }

        _allParametersBlocks.push_back(undistOffset.data());
        _allParametersBlocks.push_back(undistParameters.data());
    }

    for (auto it = sorted_undistortions.begin(); it != sorted_undistortions.end(); it++)
    {
        auto next_it = std::next(it);
        if (next_it == sorted_undistortions.end())
        {
            break;
        }

        //std::shared_ptr<camera::Undistortion> undistortion = sfmData.getUndistortions().at(uid);
        Vec2& undistOffset_1 = _undistortionOffsetBlocks[it->second];
        Vec2& undistOffset_2 = _undistortionOffsetBlocks[next_it->second];
        std::vector<double>& undistParameters_1 = _undistortionParametersBlocks[it->second];
        std::vector<double>& undistParameters_2 = _undistortionParametersBlocks[next_it->second];
        
        //problem.AddResidualBlock(new CostUndistortionSimilar(undistParameters_1.size()), nullptr, undistOffset_1.data(), undistOffset_2.data(), undistParameters_1.data(), undistParameters_2.data());

        auto next_next_it = std::next(next_it);
        if (next_next_it == sorted_undistortions.end())
        {
            continue;
        }

        std::vector<double>& undistParameters_3 = _undistortionParametersBlocks[next_next_it->second];
        //problem.AddResidualBlock(new CostUndistortionGradient(undistParameters_1.size()), nullptr, undistParameters_1.data(), undistParameters_2.data(), undistParameters_3.data());
    }
}

void BundleAdjustmentSymbolicCeres::addLandmarksToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
  const bool refineStructure = refineOptions & REFINE_STRUCTURE;

  // set a LossFunction to be less penalized by false measurements.
  // note: set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction* lossFunction = _ceresOptions.lossFunction.get();

  // build the residual blocks corresponding to the track observations
  for(const auto& landmarkPair: sfmData.getLandmarks())
  {
    const IndexT landmarkId = landmarkPair.first;
    const sfmData::Landmark& landmark = landmarkPair.second;

    // do not create a residual block if the landmark
    // have been set as Ignored by the Local BA strategy
    if(getLandmarkState(landmarkId) == EParameterState::IGNORED)
    {
      _statistics.addState(EParameter::LANDMARK, EParameterState::IGNORED);
      continue;
    }

    std::array<double,3>& landmarkBlock = _landmarksBlocks[landmarkId];
    for(std::size_t i = 0; i < 3; ++i)
      landmarkBlock.at(i) = landmark.X(Eigen::Index(i));

    double* landmarkBlockPtr = landmarkBlock.data();

    // add landmark parameter to the all parameters blocks pointers list
    _allParametersBlocks.push_back(landmarkBlockPtr);


    

    // iterate over 2D observation associated to the 3D landmark
    for(const auto& observationPair: landmark.observations)
    {
      const sfmData::View& view = sfmData.getView(observationPair.first);
      const sfmData::Observation& observation = observationPair.second;
      const std::shared_ptr<IntrinsicBase> intrinsic = sfmData.getIntrinsicsharedPtr(view.getIntrinsicId());
      
      std::shared_ptr<camera::Undistortion> undistortion;
      IndexT undistortionId = view.getUndistortionId();
      if (undistortionId != UndefinedIndexT)
      {
          undistortion = sfmData.getUndistortions().at(undistortionId);
      }

      // each residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.

      assert(getPoseState(view.getPoseId()) != EParameterState::IGNORED);
      assert(getIntrinsicState(view.getIntrinsicId()) != EParameterState::IGNORED);

      // needed parameters to create a residual block (K, pose)
      double* poseBlockPtr = _posesBlocks.at(view.getPoseId()).data();
      double* intrinsicBlockPtr = _intrinsicsBlocks.at(view.getIntrinsicId()).data();

      bool withRig = (view.isPartOfRig() && !view.isPoseIndependant());
      double * rigBlockPtr = nullptr;
      if (withRig) {
     
        rigBlockPtr = _rigBlocks.at(view.getRigId()).at(view.getSubPoseId()).data();
      }
      else {
        rigBlockPtr = _rigNull.data();
      }

      // apply a specific parameter ordering:
      if(_ceresOptions.useParametersOrdering)
      {
        _linearSolverOrdering.AddElementToGroup(landmarkBlockPtr, 0);
        _linearSolverOrdering.AddElementToGroup(poseBlockPtr, 1);
        _linearSolverOrdering.AddElementToGroup(intrinsicBlockPtr, 2);
      }

      ceres::CostFunction* costFunction = new CostProjection(observation, intrinsic, undistortion, withRig);

      std::vector<double*> params;
      params.push_back(poseBlockPtr);
      params.push_back(rigBlockPtr);
      params.push_back(intrinsicBlockPtr);
      params.push_back(landmarkBlockPtr);


      if (_undistortionParametersBlocks.find(view.getUndistortionId()) != _undistortionParametersBlocks.end())
      {
          double* undistortionParameterBlockPtr = _undistortionParametersBlocks.at(view.getUndistortionId()).data();
          double* undistortionOffsetBlockPtr = _undistortionOffsetBlocks.at(view.getUndistortionId()).data();
          params.push_back(undistortionOffsetBlockPtr);
          params.push_back(undistortionParameterBlockPtr);
      }
      


      problem.AddResidualBlock(costFunction, lossFunction, params);

      if(!refineStructure || getLandmarkState(landmarkId) == EParameterState::CONSTANT)
      {
        // set the whole landmark parameter block as constant.
        _statistics.addState(EParameter::LANDMARK, EParameterState::CONSTANT);
        problem.SetParameterBlockConstant(landmarkBlockPtr);
      }
      else
      {
        _statistics.addState(EParameter::LANDMARK, EParameterState::REFINED);
      }
    }
  }
}


void BundleAdjustmentSymbolicCeres::createProblem(const sfmData::SfMData& sfmData,
                                          ERefineOptions refineOptions,
                                          ceres::Problem& problem)
{
  // clear previously computed data
  resetProblem();

  // ensure we are not using incompatible options
  // REFINEINTRINSICS_OPTICALCENTER_ALWAYS and REFINEINTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA cannot be used at the same time
  assert(!((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) && (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA)));

  // add SfM extrincics to the Ceres problem
  addExtrinsicsToProblem(sfmData, refineOptions, problem);

  // add SfM intrinsics to the Ceres problem
  addIntrinsicsToProblem(sfmData, refineOptions, problem);

  // add SfM undistortions to the Ceres problem
  addUndistortionsToProblem(sfmData, refineOptions, problem);

  // add SfM landmarks to the Ceres problem
  addLandmarksToProblem(sfmData, refineOptions, problem);
}

void BundleAdjustmentSymbolicCeres::updateFromSolution(sfmData::SfMData& sfmData, ERefineOptions refineOptions) const
{
  const bool refinePoses = (refineOptions & REFINE_ROTATION) || (refineOptions & REFINE_TRANSLATION);
  const bool refineIntrinsicsOpticalCenter = (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
  const bool refineIntrinsics = (refineOptions & REFINE_INTRINSICS_FOCAL) || (refineOptions & REFINE_INTRINSICS_DISTORTION) || refineIntrinsicsOpticalCenter || (refineOptions & REFINE_INTRINSICS_UNDISTORTION);
  const bool refineStructure = refineOptions & REFINE_STRUCTURE;
  const bool refineUndistortion = refineOptions & REFINE_INTRINSICS_UNDISTORTION;

  // update camera poses with refined data
  if(refinePoses)
  {
    // absolute poses
    for(auto& posePair : sfmData.getPoses())
    {
      const IndexT poseId = posePair.first;

      // do not update a camera pose set as Ignored or Constant in the Local strategy
      if(getPoseState(poseId) != EParameterState::REFINED)
        continue;

      const SE3::Matrix & poseBlock = _posesBlocks.at(poseId);

      // update the pose
      posePair.second.setTransform(poseFromRT(poseBlock.block<3, 3>(0, 0), poseBlock.block<3, 1>(0, 3)));
    }

    // rig sub-poses
    for(const auto& rigIt : _rigBlocks)
    {
      sfmData::Rig& rig = sfmData.getRigs().at(rigIt.first);

      for(const auto& subPoseit : rigIt.second)
      {
        sfmData::RigSubPose& subPose = rig.getSubPose(subPoseit.first);
        const SE3::Matrix & subPoseBlock = subPoseit.second;

        // update the sub-pose
        subPose.pose = poseFromRT(subPoseBlock.block<3, 3>(0, 0), subPoseBlock.block<3, 1>(0, 3));
      }
    }
  }

  // update camera intrinsics with refined data
  if (refineIntrinsics)
  {
      for (const auto& intrinsicBlockPair : _intrinsicsBlocks)
      {
          const IndexT intrinsicId = intrinsicBlockPair.first;

          // do not update a camera pose set as Ignored or Constant in the Local strategy
          if (getIntrinsicState(intrinsicId) != EParameterState::REFINED)
          {
              continue;
          }

          sfmData.getIntrinsics().at(intrinsicId)->updateFromParams(intrinsicBlockPair.second);
      }
  }

  if (refineUndistortion)
  {
    for (const auto& undistoBlockPair : _undistortionOffsetBlocks)
    {
        const IndexT undistortionId = undistoBlockPair.first;

        std::shared_ptr<camera::Undistortion> undistortionObject = sfmData.getUndistortions().at(undistortionId);
        if (undistortionObject)
        {
            undistortionObject->setOffset(undistoBlockPair.second);
        }
    }

    for (const auto& undistoBlockPair : _undistortionParametersBlocks)
    {
        const IndexT undistortionId = undistoBlockPair.first;

        std::shared_ptr<camera::Undistortion> undistortionObject = sfmData.getUndistortions().at(undistortionId);
        if (undistortionObject)
        {
            undistortionObject->setParameters(undistoBlockPair.second);
        }
    }
  }

  // update landmarks
  if(refineStructure)
  {
    for(const auto& landmarksBlockPair: _landmarksBlocks)
    {
      const IndexT landmarkId = landmarksBlockPair.first;
      sfmData::Landmark& landmark = sfmData.getLandmarks().at(landmarkId);

      // do not update a camera pose set as Ignored or Constant in the Local strategy
      if(getLandmarkState(landmarkId) != EParameterState::REFINED) {
        continue;
      }

      for(std::size_t i = 0; i < 3; ++i) {
        landmark.X(Eigen::Index(i))= landmarksBlockPair.second.at(i);
      }
    }
  }
}

void BundleAdjustmentSymbolicCeres::createJacobian(const sfmData::SfMData& sfmData,
                                           ERefineOptions refineOptions,
                                           ceres::CRSMatrix& jacobian)
{
  // create problem
  ceres::Problem::Options problemOptions;
  problemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problemOptions);
  createProblem(sfmData, refineOptions, problem);

  // configure Jacobian engine
  double cost = 0.0;
  ceres::Problem::EvaluateOptions evalOpt;
  evalOpt.parameter_blocks = _allParametersBlocks;
  evalOpt.num_threads = 8;
  evalOpt.apply_loss_function = true;

  // create Jacobain
  problem.Evaluate(evalOpt, &cost, NULL, NULL, &jacobian);
}

bool BundleAdjustmentSymbolicCeres::adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions)
{
  // create problem
  ceres::Problem::Options problemOptions;
  problemOptions.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(problemOptions);
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

  //store distance histogram for local strategy
  if(useLocalStrategy())
  {
    _statistics.nbCamerasPerDistance = _localGraph->getDistancesHistogram();
  }

  return true;
}

} // namespace sfm
} // namespace aliceVision

