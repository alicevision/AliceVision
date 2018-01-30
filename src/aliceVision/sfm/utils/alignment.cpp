// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/geometry/rigidTransformation3D.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

#include <algorithm>

namespace bacc = boost::accumulators;

namespace aliceVision {
namespace sfm {

bool computeSimilarity(const SfMData & sfmDataA,
                       const SfMData & sfmDataB,
                       double * out_S,
                       Mat3 * out_R,
                       Vec3 * out_t)
{
  assert(out_S != nullptr);
  assert(out_R != nullptr);
  assert(out_t != nullptr);
  
  std::vector<IndexT> commonViewIds;
  getCommonViewsWithPoses(sfmDataA, sfmDataB, commonViewIds);
  if(commonViewIds.size() < 2)
  {
    ALICEVISION_LOG_WARNING("Cannot compute similarities. Need at least 2 common views.");
    return false;
  }
  ALICEVISION_LOG_DEBUG("Found " << commonViewIds.size() << " common views.");

  // Move input point in appropriate container
  Mat xA(3, commonViewIds.size());
  Mat xB(3, commonViewIds.size());
  for (size_t i = 0; i  < commonViewIds.size(); ++i)
  {
    IndexT viewId = commonViewIds[i];
    xA.col(i) = sfmDataA.GetPoses().at(sfmDataA.GetViews().at(viewId)->getPoseId()).center();
    xB.col(i) = sfmDataB.GetPoses().at(sfmDataB.GetViews().at(viewId)->getPoseId()).center();
  }

  // Compute rigid transformation p'i = S R pi + t
  double S;
  Vec3 t;
  Mat3 R;
  std::vector<std::size_t> inliers;
  if(!aliceVision::geometry::ACRansac_FindRTS(xA, xB, S, t, R, inliers, true))
    return false;

  ALICEVISION_LOG_DEBUG("There are " << commonViewIds.size() << " common cameras and " << inliers.size() << " were used to compute the similarity transform.");

  *out_S = S;
  *out_R = R;
  *out_t = t;
  return true;
}

void computeNewCoordinateSystemFromCameras(const SfMData& sfmData,
                                           double& out_S,
                                           Mat3& out_R,
                                           Vec3& out_t)
{
  Mat3X vX(3, sfmData.GetLandmarks().size());

  std::size_t i = 0;
  for(const auto& landmark : sfmData.GetLandmarks())
  {
    vX.col(i) = landmark.second.X;
    ++i;
  }

  const std::size_t nbCameras = sfmData.GetPoses().size();
  Mat3X vCamCenter(3,nbCameras);

  // Compute the mean of the point cloud
  Vec3 meanPoints = Vec3::Zero(3,1);
  i=0;
  for(const auto & pose : sfmData.GetPoses())
  {
    vCamCenter.col(i) = pose.second.center();
    meanPoints +=  pose.second.center();
    ++i;
  }

  meanPoints /= nbCameras;

  std::vector<Mat3> vCamRotation; // Camera rotations
  vCamRotation.reserve(nbCameras);
  i=0;
  double rms = 0;
  for(const auto & pose : sfmData.GetPoses())
  {
    Vec3 camCenterMean = vCamCenter.col(i) - meanPoints;
    rms += camCenterMean.transpose() * camCenterMean; // squared dist to the mean of camera centers

    vCamRotation.push_back(pose.second.rotation().transpose()); // Rotation in the world coordinate system
    ++i;
  }
  rms /= nbCameras;
  rms = sqrt(rms);

  // Perform an svd over vX*vXT (var-covar)
  Mat3 dum = vCamCenter * vCamCenter.transpose();
  Eigen::JacobiSVD<Mat3> svd(dum,Eigen::ComputeFullV|Eigen::ComputeFullU);
  Mat3 U = svd.matrixU();

  // Check whether the determinant is positive in order to keep
  // a direct coordinate system
  if(U.determinant() < 0)
  {
    U.col(2) = -U.col(2);
  }

  out_R = U.transpose();

  // TODO: take the median instead of the first camera
  if((out_R * vCamRotation[0])(2,2) > 0)
  {
    out_S = -out_S;
  }

  out_R = Eigen::AngleAxisd(degreeToRadian(90.0),  Vec3(1,0,0)) * out_R;
  out_S = 1.0 / rms;

  out_t = - out_S * out_R * meanPoints;
}

void computeNewCoordinateSystemFromLandmarks(const SfMData& sfmData,
                                    const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
                                    double& out_S,
                                    Mat3& out_R,
                                    Vec3& out_t)
{
    const std::size_t nbLandmarks = sfmData.GetLandmarks().size();

    Mat3X vX(3,nbLandmarks);

    std::size_t i = 0;

    Vec3 meanPoints = Vec3::Zero(3,1);
    std::size_t nbMeanLandmarks = 0;

    bacc::accumulator_set<double, bacc::stats<bacc::tag::min, bacc::tag::max> > accX, accY, accZ;

    for(const auto & landmark : sfmData.GetLandmarks())
    {
      const Vec3& position = landmark.second.X;

      vX.col(i) = position;

      // Compute the mean of the point cloud
      if(imageDescriberTypes.empty() ||
         std::find(imageDescriberTypes.begin(), imageDescriberTypes.end(), landmark.second.descType) != imageDescriberTypes.end())
      {
        meanPoints += position;
        ++nbMeanLandmarks;
      }

      accX(position(0));
      accY(position(1));
      accZ(position(2));
      ++i;
    }

    meanPoints /= nbMeanLandmarks;

    // Center the point cloud in [0;0;0]
    for(int i=0 ; i < nbLandmarks ; ++i)
    {
      vX.col(i) -= meanPoints;
    }

    // Perform an svd over vX*vXT (var-covar)
    Mat3 dum = vX.leftCols(nbMeanLandmarks) * vX.leftCols(nbMeanLandmarks).transpose();
    Eigen::JacobiSVD<Mat3> svd(dum,Eigen::ComputeFullV|Eigen::ComputeFullU);
    Mat3 U = svd.matrixU();

    // Check whether the determinant is negative in order to keep
    // a direct coordinate system
    if(U.determinant() < 0)
    {
      U.col(2) = -U.col(2);
    }

    out_S = 1.0 / std::max({bacc::max(accX) - bacc::min(accX), bacc::max(accY) - bacc::min(accY), bacc::max(accZ) - bacc::min(accZ)});
    out_R = U.transpose();
    out_R = Eigen::AngleAxisd(degreeToRadian(90.0),  Vec3(1,0,0)) * out_R;
    out_t = - out_S * out_R * meanPoints;
}


} // namespace sfm
} // namespace aliceVision
