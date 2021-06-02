// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/ResidualErrorFunctor.hpp>
#include <aliceVision/sfm/ResidualErrorConstraintFunctor.hpp>
#include <aliceVision/sfm/ResidualErrorRotationPriorFunctor.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/camera/Equidistant.hpp>

#include <boost/filesystem.hpp>

#include <ceres/rotation.h>

#include <fstream>



namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;

class IntrinsicsParameterization : public ceres::LocalParameterization {
 public:
  explicit IntrinsicsParameterization(size_t parametersSize, double focalRatio, bool lockFocal, bool lockFocalRatio, bool lockCenter, bool lockDistortion)
  : _globalSize(parametersSize),
    _focalRatio(focalRatio),
    _lockFocal(lockFocal),
    _lockFocalRatio(lockFocalRatio),
    _lockCenter(lockCenter),
    _lockDistortion(lockDistortion)
  {
    _distortionSize = _globalSize - 4;
    _localSize = 0;

    if (!_lockFocal)
    {
      if (_lockFocalRatio)
      {
        _localSize += 1;
      }
      else
      {
        _localSize += 2;
      }
    }

    if (!_lockCenter)
    {
      _localSize += 2;
    }

    if (!_lockDistortion)
    {
      _localSize += _distortionSize;
    }
  }

  virtual ~IntrinsicsParameterization() = default;


  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override
  {
    for (int i = 0; i < _globalSize; i++)
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
        posDelta++;
      }
      else
      {
        x_plus_delta[0] = x[0] + delta[posDelta];
        posDelta++;
        x_plus_delta[1] = x[1] + delta[posDelta];
        posDelta++;
      }
    }

    if (!_lockCenter)
    {
      x_plus_delta[2] = x[2] + delta[posDelta]; 
      posDelta++;

      x_plus_delta[3] = x[3] + delta[posDelta];
      posDelta++;
    }

    if (!_lockDistortion)
    {
      for (int i = 0; i < _distortionSize; i++)
      {
        x_plus_delta[4 + i] = x[4 + i] + delta[posDelta];
        posDelta++;
      }
    }

    return true;
  }

  bool ComputeJacobian(const double* x, double* jacobian) const override
  {    
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(jacobian, GlobalSize(), LocalSize());

    J.fill(0);

    size_t posDelta = 0;
    if (!_lockFocal)
    {
      if (_lockFocalRatio)
      {
        J(0, posDelta) = 1.0;
        J(1, posDelta) = _focalRatio;
        posDelta++;
      }
      else
      {
        J(0, posDelta) = 1.0;
        posDelta++;
        J(1, posDelta) = 1.0;
        posDelta++;
      }
    }

    if (!_lockCenter)
    {
      J(2, posDelta) = 1.0;
      posDelta++;

      J(3, posDelta) = 1.0;
      posDelta++;
    }

    if (!_lockDistortion)
    {
      for (int i = 0; i < _distortionSize; i++)
      {
        J(4 + i, posDelta) = 1.0;
        posDelta++;
      }
    }

    return true;
  }

  int GlobalSize() const override 
  {
    return _globalSize;
  }

  int LocalSize() const override 
  { 
    return _localSize; 
  }

 private:
  size_t _distortionSize;
  size_t _globalSize;
  size_t _localSize;
  double _focalRatio;
  bool _lockFocal;
  bool _lockFocalRatio;
  bool _lockCenter;
  bool _lockDistortion;
};

/**
 * @brief Create the appropriate cost functor according the provided input camera intrinsic model
 * @param[in] intrinsicPtr The intrinsic pointer
 * @param[in] observation The corresponding observation
 * @return cost functor
 */
ceres::CostFunction* createCostFunctionFromIntrinsics(const IntrinsicBase* intrinsicPtr, const sfmData::Observation& observation)
{
  int w = intrinsicPtr->w();
  int h = intrinsicPtr->h();

  switch(intrinsicPtr->getType())
  {
    case EINTRINSIC::PINHOLE_CAMERA:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole, 2, 4, 6, 3>(new ResidualErrorFunctor_Pinhole(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeRadialK1, 2, 5, 6, 3>(new ResidualErrorFunctor_PinholeRadialK1(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeRadialK3, 2, 7, 6, 3>(new ResidualErrorFunctor_PinholeRadialK3(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole3DERadial4, 2, 10, 6, 3>(new ResidualErrorFunctor_Pinhole3DERadial4(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole3DEClassicLD, 2, 9, 6, 3>(new ResidualErrorFunctor_Pinhole3DEClassicLD(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_BROWN:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeBrownT2, 2, 9, 6, 3>(new ResidualErrorFunctor_PinholeBrownT2(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeFisheye, 2, 8, 6, 3>(new ResidualErrorFunctor_PinholeFisheye(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeFisheye1, 2, 5, 6, 3>(new ResidualErrorFunctor_PinholeFisheye1(w, h, observation));
    default:
      throw std::logic_error("Cannot create cost function, unrecognized intrinsic type in BA.");
  }
}

/**
 * @brief Create the appropriate cost functor according the provided input rig camera intrinsic model
 * @param[in] intrinsicPtr The intrinsic pointer
 * @param[in] observation The corresponding observation
 * @return cost functor
 */
ceres::CostFunction* createRigCostFunctionFromIntrinsics(const IntrinsicBase* intrinsicPtr, const sfmData::Observation& observation)
{
  int w = intrinsicPtr->w();
  int h = intrinsicPtr->h();

  switch(intrinsicPtr->getType())
  {
    case EINTRINSIC::PINHOLE_CAMERA:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole, 2, 4, 6, 6, 3>(new ResidualErrorFunctor_Pinhole(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeRadialK1, 2, 5, 6, 6, 3>(new ResidualErrorFunctor_PinholeRadialK1(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeRadialK3, 2, 7, 6, 6, 3>(new ResidualErrorFunctor_PinholeRadialK3(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole3DERadial4, 2, 10, 6, 6, 3>(new ResidualErrorFunctor_Pinhole3DERadial4(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_Pinhole3DEClassicLD, 2, 9, 6, 6, 3>(new ResidualErrorFunctor_Pinhole3DEClassicLD(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_BROWN:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeBrownT2, 2, 9, 6, 6, 3>(new ResidualErrorFunctor_PinholeBrownT2(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeFisheye, 2, 8, 6, 6, 3>(new ResidualErrorFunctor_PinholeFisheye(w, h, observation));
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
      return new ceres::AutoDiffCostFunction<ResidualErrorFunctor_PinholeFisheye1, 2, 5, 6, 6, 3>(new ResidualErrorFunctor_PinholeFisheye1(w, h, observation));
    default:
      throw std::logic_error("Cannot create rig cost function, unrecognized intrinsic type in BA.");
  }
}

/**
 * @brief Create the appropriate cost functor according the provided input camera intrinsic model
 * @param[in] intrinsicPtr The intrinsic pointer
 * @param[in] observation The corresponding observation
 * @return cost functor
 */
ceres::CostFunction* createConstraintsCostFunctionFromIntrinsics(const IntrinsicBase* intrinsicPtr, const Vec2& observation_first, const Vec2& observation_second)
{
  double radius = 0.0;
  const camera::EquiDistant * equi = dynamic_cast<const camera::EquiDistant *>(intrinsicPtr);
  if (equi) {
    radius = equi->getCircleRadius();
  }

  int w = intrinsicPtr->w();
  int h = intrinsicPtr->h();

  switch(intrinsicPtr->getType())
  {
    case EINTRINSIC::PINHOLE_CAMERA:
      return new ceres::AutoDiffCostFunction<ResidualErrorConstraintFunctor_Pinhole, 2, 3, 6, 6>(new ResidualErrorConstraintFunctor_Pinhole(w, h, observation_first.homogeneous(), observation_second.homogeneous()));
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
      return new ceres::AutoDiffCostFunction<ResidualErrorConstraintFunctor_PinholeRadialK1, 2, 4, 6, 6>(new ResidualErrorConstraintFunctor_PinholeRadialK1(w, h, observation_first.homogeneous(), observation_second.homogeneous()));
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
      return new ceres::AutoDiffCostFunction<ResidualErrorConstraintFunctor_PinholeRadialK3, 2, 6, 6, 6>(new ResidualErrorConstraintFunctor_PinholeRadialK3(w, h, observation_first.homogeneous(), observation_second.homogeneous()));
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
      return new ceres::AutoDiffCostFunction<ResidualErrorConstraintFunctor_PinholeFisheye, 2, 7, 6, 6>(new ResidualErrorConstraintFunctor_PinholeFisheye(w, h, observation_first.homogeneous(), observation_second.homogeneous()));
    case EQUIDISTANT_CAMERA:
      return new ceres::AutoDiffCostFunction<ResidualErrorConstraintFunctor_Equidistant, 2, 3, 6, 6>(new ResidualErrorConstraintFunctor_Equidistant(w, h, observation_first.homogeneous(), observation_second.homogeneous(), radius));
    case EQUIDISTANT_CAMERA_RADIAL3:
      return new ceres::AutoDiffCostFunction<ResidualErrorConstraintFunctor_EquidistantRadialK3, 2, 6, 6, 6>(new ResidualErrorConstraintFunctor_EquidistantRadialK3(w, h, observation_first.homogeneous(), observation_second.homogeneous(), radius));
    default:
      throw std::logic_error("Cannot create cost function, unrecognized intrinsic type in BA.");
  } 
}

void BundleAdjustmentCeres::CeresOptions::setDenseBA()
{
  // default configuration use a DENSE representation
  preconditionerType  = ceres::JACOBI;
  linearSolverType = ceres::DENSE_SCHUR;
  sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE; // not used but just to avoid a warning in ceres
  ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: DENSE_SCHUR");
}

void BundleAdjustmentCeres::CeresOptions::setSparseBA()
{
  preconditionerType = ceres::JACOBI;
  // if Sparse linear solver are available
  // descending priority order by efficiency (SUITE_SPARSE > CX_SPARSE > EIGEN_SPARSE)
  if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::SUITE_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: SPARSE_SCHUR, SUITE_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::CX_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::CX_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: SPARSE_SCHUR, CX_SPARSE");
  }
  else if (ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
  {
    sparseLinearAlgebraLibraryType = ceres::EIGEN_SPARSE;
    linearSolverType = ceres::SPARSE_SCHUR;
    ALICEVISION_LOG_DEBUG("BundleAdjustment[Ceres]: SPARSE_SCHUR, EIGEN_SPARSE");
  }
  else
  {
    linearSolverType = ceres::DENSE_SCHUR;
    ALICEVISION_LOG_WARNING("BundleAdjustment[Ceres]: no sparse BA available, fallback to dense BA.");
  }
}

bool BundleAdjustmentCeres::Statistics::exportToFile(const std::string& folder, const std::string& filename) const
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

void BundleAdjustmentCeres::Statistics::show() const
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

void BundleAdjustmentCeres::setSolverOptions(ceres::Solver::Options& solverOptions) const
{
  solverOptions.preconditioner_type = _ceresOptions.preconditionerType;
  solverOptions.linear_solver_type = _ceresOptions.linearSolverType;
  solverOptions.sparse_linear_algebra_library_type = _ceresOptions.sparseLinearAlgebraLibraryType;
  solverOptions.minimizer_progress_to_stdout = _ceresOptions.verbose;
  solverOptions.logging_type = ceres::SILENT;
  solverOptions.num_threads = _ceresOptions.nbThreads;

#if CERES_VERSION_MAJOR < 2
  solverOptions.num_linear_solver_threads = _ceresOptions.nbThreads;
#endif

  if(_ceresOptions.useParametersOrdering)
  {
    // copy ParameterBlockOrdering
    solverOptions.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering(_linearSolverOrdering));
  }
}

void BundleAdjustmentCeres::addExtrinsicsToProblem(const sfmData::SfMData& sfmData, BundleAdjustment::ERefineOptions refineOptions, ceres::Problem& problem)
{
  const bool refineTranslation = refineOptions & BundleAdjustment::REFINE_TRANSLATION;
  const bool refineRotation = refineOptions & BundleAdjustment::REFINE_ROTATION;

  const auto addPose = [&](const sfmData::CameraPose& cameraPose, bool isConstant, std::array<double,6>& poseBlock)
  {
    const Mat3& R = cameraPose.getTransform().rotation();
    const Vec3& t = cameraPose.getTransform().translation();

    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis(static_cast<const double *>(R.data()), angleAxis);
    poseBlock.at(0) = angleAxis[0];
    poseBlock.at(1) = angleAxis[1];
    poseBlock.at(2) = angleAxis[2];
    poseBlock.at(3) = t(0);
    poseBlock.at(4) = t(1);
    poseBlock.at(5) = t(2);

    double* poseBlockPtr = poseBlock.data();
    problem.AddParameterBlock(poseBlockPtr, 6);

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

    // constant parameters
    std::vector<int> constantExtrinsic;

    // don't refine rotations
    if(!refineRotation)
    {
      constantExtrinsic.push_back(0);
      constantExtrinsic.push_back(1);
      constantExtrinsic.push_back(2);
    }

    // don't refine translations
    if(!refineTranslation)
    {
      constantExtrinsic.push_back(3);
      constantExtrinsic.push_back(4);
      constantExtrinsic.push_back(5);
    }

    // subset parametrization
    if(!constantExtrinsic.empty())
    {
      ceres::SubsetParameterization* subsetParameterization = new ceres::SubsetParameterization(6, constantExtrinsic);
      problem.SetParameterization(poseBlockPtr, subsetParameterization);
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

      addPose(sfmData::CameraPose(rigSubPose.pose), isConstant, _rigBlocks[rigId][subPoseId]);
    }
  }
}

void BundleAdjustmentCeres::addIntrinsicsToProblem(const sfmData::SfMData& sfmData, BundleAdjustment::ERefineOptions refineOptions, ceres::Problem& problem)
{
  const bool refineIntrinsicsOpticalCenter = (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) || (refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA);
  const bool refineIntrinsicsFocalLength = refineOptions & REFINE_INTRINSICS_FOCAL;
  const bool refineIntrinsicsDistortion = refineOptions & REFINE_INTRINSICS_DISTORTION;
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
      if(intrinsicScaleOffset->initialScale() > 0)
      {
        // if we have an initial guess, we only authorize a margin around this value.
        assert(intrinsicBlock.size() >= 1);
        const unsigned int maxFocalError = 0.2 * std::max(intrinsicPtr->w(), intrinsicPtr->h()); // TODO : check if rounding is needed
        problem.SetParameterLowerBound(intrinsicBlockPtr, 0, static_cast<double>(intrinsicScaleOffset->initialScale() - maxFocalError));
        problem.SetParameterUpperBound(intrinsicBlockPtr, 0, static_cast<double>(intrinsicScaleOffset->initialScale() + maxFocalError));
        problem.SetParameterLowerBound(intrinsicBlockPtr, 1, static_cast<double>(intrinsicScaleOffset->initialScale() - maxFocalError));
        problem.SetParameterUpperBound(intrinsicBlockPtr, 1, static_cast<double>(intrinsicScaleOffset->initialScale() + maxFocalError));
      }
      else // no initial guess
      {
        // we don't have an initial guess, but we assume that we use
        // a converging lens, so the focal length should be positive.
        problem.SetParameterLowerBound(intrinsicBlockPtr, 0, 0.0);
        problem.SetParameterLowerBound(intrinsicBlockPtr, 1, 0.0);
      }

      focalRatio = intrinsicBlockPtr[1] / intrinsicBlockPtr[0];
    }
    else
    {
      // set focal length as constant
      lockFocal = true;
    }

    // optical center
    if((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS) ||
       ((refineOptions & REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA) && _minNbImagesToRefineOpticalCenter > 0 && usageCount >= _minNbImagesToRefineOpticalCenter))
    {
      // refine optical center within 10% of the image size.
      assert(intrinsicBlock.size() >= 3);

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

    
    IntrinsicsParameterization * subsetParameterization = new IntrinsicsParameterization(intrinsicBlock.size(), focalRatio, lockFocal, lockRatio, lockCenter, lockDistortion);
    problem.SetParameterization(intrinsicBlockPtr, subsetParameterization);

    _statistics.addState(EParameter::INTRINSIC, EParameterState::REFINED);
  }
}

void BundleAdjustmentCeres::addLandmarksToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
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

      // each residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.

      assert(getPoseState(view.getPoseId()) != EParameterState::IGNORED);
      assert(getIntrinsicState(view.getIntrinsicId()) != EParameterState::IGNORED);

      // needed parameters to create a residual block (K, pose)
      double* poseBlockPtr = _posesBlocks.at(view.getPoseId()).data();
      double* intrinsicBlockPtr = _intrinsicsBlocks.at(view.getIntrinsicId()).data();

      // apply a specific parameter ordering:
      if(_ceresOptions.useParametersOrdering)
      {
        _linearSolverOrdering.AddElementToGroup(landmarkBlockPtr, 0);
        _linearSolverOrdering.AddElementToGroup(poseBlockPtr, 1);
        _linearSolverOrdering.AddElementToGroup(intrinsicBlockPtr, 2);
      }

      if(view.isPartOfRig() && !view.isPoseIndependant())
      {
        ceres::CostFunction* costFunction = createRigCostFunctionFromIntrinsics(sfmData.getIntrinsicPtr(view.getIntrinsicId()), observation);

        double* rigBlockPtr = _rigBlocks.at(view.getRigId()).at(view.getSubPoseId()).data();
        _linearSolverOrdering.AddElementToGroup(rigBlockPtr, 1);

        problem.AddResidualBlock(costFunction,
            lossFunction,
            intrinsicBlockPtr,
            poseBlockPtr,
            rigBlockPtr, // subpose of the cameras rig
            landmarkBlockPtr); // do we need to copy 3D point to avoid false motion, if failure ?
      }
      else
      {
        ceres::CostFunction* costFunction = createCostFunctionFromIntrinsics(sfmData.getIntrinsicPtr(view.getIntrinsicId()), observation);

        problem.AddResidualBlock(costFunction,
            lossFunction,
            intrinsicBlockPtr,
            poseBlockPtr,
            landmarkBlockPtr); //do we need to copy 3D point to avoid false motion, if failure ?
      }

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

void BundleAdjustmentCeres::addConstraints2DToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
{
  // set a LossFunction to be less penalized by false measurements.
  // note: set it to NULL if you don't want use a lossFunction.
  ceres::LossFunction* lossFunction = _ceresOptions.lossFunction.get();

  for (const auto & constraint : sfmData.getConstraints2D()) {
    const sfmData::View& view_1 = sfmData.getView(constraint.ViewFirst);
    const sfmData::View& view_2 = sfmData.getView(constraint.ViewSecond);

    assert(getPoseState(view_1.getPoseId()) != EParameterState::IGNORED);
    assert(getIntrinsicState(view_1.getIntrinsicId()) != EParameterState::IGNORED);
    assert(getPoseState(view_2.getPoseId()) != EParameterState::IGNORED);
    assert(getIntrinsicState(view_2.getIntrinsicId()) != EParameterState::IGNORED);

    double * poseBlockPtr_1 = _posesBlocks.at(view_1.getPoseId()).data();
    double * poseBlockPtr_2 = _posesBlocks.at(view_2.getPoseId()).data();

    double * intrinsicBlockPtr_1 = _intrinsicsBlocks.at(view_1.getIntrinsicId()).data();
    double * intrinsicBlockPtr_2 = _intrinsicsBlocks.at(view_2.getIntrinsicId()).data();

    //For the moment assume a unique camera
    assert(intrinsicBlockPtr_1 == intrinsicBlockPtr_2);


    ceres::CostFunction* costFunction = createConstraintsCostFunctionFromIntrinsics(sfmData.getIntrinsicPtr(view_1.getIntrinsicId()), constraint.ObservationFirst.x, constraint.ObservationSecond.x);
    problem.AddResidualBlock(costFunction, lossFunction, intrinsicBlockPtr_1, poseBlockPtr_1, poseBlockPtr_2);
  }
}

void BundleAdjustmentCeres::addRotationPriorsToProblem(const sfmData::SfMData& sfmData, ERefineOptions refineOptions, ceres::Problem& problem)
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


    ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<ResidualErrorRotationPriorFunctor, 3, 6, 6>(new ResidualErrorRotationPriorFunctor(prior._second_R_first));
    problem.AddResidualBlock(costFunction, lossFunction, poseBlockPtr_1, poseBlockPtr_2);
  }
}

void BundleAdjustmentCeres::createProblem(const sfmData::SfMData& sfmData,
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

  // add SfM landmarks to the Ceres problem
  addLandmarksToProblem(sfmData, refineOptions, problem);

  // add 2D constraints to the Ceres problem
  addConstraints2DToProblem(sfmData, refineOptions, problem);

  // add rotation priors to the Ceres problem
  addRotationPriorsToProblem(sfmData, refineOptions, problem);
}

 void BundleAdjustmentCeres::resetProblem()
{
  _statistics = Statistics();

  _allParametersBlocks.clear();
  _posesBlocks.clear();
  _intrinsicsBlocks.clear();
  _landmarksBlocks.clear();
  _rigBlocks.clear();

  _linearSolverOrdering.Clear();
}

void BundleAdjustmentCeres::updateFromSolution(sfmData::SfMData& sfmData, ERefineOptions refineOptions) const
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
      if(getPoseState(poseId) != EParameterState::REFINED)
        continue;

      const std::array<double,6>& poseBlock = _posesBlocks.at(poseId);

      Mat3 R_refined;
      ceres::AngleAxisToRotationMatrix(poseBlock.data(), R_refined.data());
      const Vec3 t_refined(poseBlock.at(3), poseBlock.at(4), poseBlock.at(5));

      // update the pose
      posePair.second.setTransform(poseFromRT(R_refined, t_refined));
    }

    // rig sub-poses
    for(const auto& rigIt : _rigBlocks)
    {
      sfmData::Rig& rig = sfmData.getRigs().at(rigIt.first);

      for(const auto& subPoseit : rigIt.second)
      {
        sfmData::RigSubPose& subPose = rig.getSubPose(subPoseit.first);
        const std::array<double,6>& subPoseBlock = subPoseit.second;

        Mat3 R_refined;
        ceres::AngleAxisToRotationMatrix(subPoseBlock.data(), R_refined.data());
        const Vec3 t_refined(subPoseBlock.at(3), subPoseBlock.at(4), subPoseBlock.at(5));

        // update the sub-pose
        subPose.pose = poseFromRT(R_refined, t_refined);
      }
    }
  }

  // update camera intrinsics with refined data
  if(refineIntrinsics)
  {
    for(const auto& intrinsicBlockPair: _intrinsicsBlocks)
    {
      const IndexT intrinsicId = intrinsicBlockPair.first;

      // do not update a camera pose set as Ignored or Constant in the Local strategy
      if(getIntrinsicState(intrinsicId) != EParameterState::REFINED)
        continue;

      sfmData.getIntrinsics().at(intrinsicId)->updateFromParams(intrinsicBlockPair.second);
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
      if(getLandmarkState(landmarkId) != EParameterState::REFINED)
        continue;

      for(std::size_t i = 0; i < 3; ++i)
        landmark.X(Eigen::Index(i))= landmarksBlockPair.second.at(i);
    }
  }
}

void BundleAdjustmentCeres::createJacobian(const sfmData::SfMData& sfmData,
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

bool BundleAdjustmentCeres::adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions)
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
  if(_ceresOptions.summary)
    ALICEVISION_LOG_INFO(summary.FullReport());

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
    _statistics.nbCamerasPerDistance = _localGraph->getDistancesHistogram();

  return true;
}

} // namespace sfm
} // namespace aliceVision

