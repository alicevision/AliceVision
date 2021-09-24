// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/geometry/rigidTransformation3D.hpp>
#include <aliceVision/stl/regex.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <regex>


namespace bacc = boost::accumulators;

namespace aliceVision {
namespace sfm {


std::istream& operator>>(std::istream& in, MarkerWithCoord& marker)
{
    std::string token;
    in >> token;
    std::vector<std::string> markerCoord;
    boost::split(markerCoord, token, boost::algorithm::is_any_of(":="));
    if(markerCoord.size() != 2)
        throw std::invalid_argument("Failed to parse MarkerWithCoord from: " + token);
    marker.id = boost::lexical_cast<int>(markerCoord.front());

    std::vector<std::string> coord;
    boost::split(coord, markerCoord.back(), boost::algorithm::is_any_of(",;_"));
    if (coord.size() != 3)
        throw std::invalid_argument("Failed to parse Marker coordinates from: " + markerCoord.back());

    for (int i = 0; i < 3; ++i)
    {
        marker.coord(i) = boost::lexical_cast<double>(coord[i]);
    }
    return in;
}

std::ostream& operator<<(std::ostream& os, const MarkerWithCoord& marker)
{
    os << marker.id << ":" << marker.coord(0) << "," << marker.coord(1) << "," << marker.coord(2);
    return os;
}

bool computeSimilarityFromCommonViews(const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::vector<std::pair<IndexT, IndexT>>& commonViewIds,
    std::mt19937 &randomNumberGenerator,
    double* out_S,
    Mat3* out_R,
    Vec3* out_t)
{
    assert(out_S != nullptr);
    assert(out_R != nullptr);
    assert(out_t != nullptr);

    std::vector<std::pair<IndexT, IndexT>> reconstructedCommonViewIds;
    for (const auto& c : commonViewIds)
    {
        if (sfmDataA.isPoseAndIntrinsicDefined(c.first) && sfmDataB.isPoseAndIntrinsicDefined(c.second))
        {
            reconstructedCommonViewIds.emplace_back(c);
        }
    }

    if (reconstructedCommonViewIds.empty())
    {
        ALICEVISION_LOG_WARNING("Cannot compute similarities without common view.");
        return false;
    }

    // Move input point in appropriate container
    Mat xA(3, reconstructedCommonViewIds.size());
    Mat xB(3, reconstructedCommonViewIds.size());
    for (std::size_t i = 0; i < reconstructedCommonViewIds.size(); ++i)
    {
        auto viewIdPair = reconstructedCommonViewIds[i];
        xA.col(i) = sfmDataA.getPose(sfmDataA.getView(viewIdPair.first)).getTransform().center();
        xB.col(i) = sfmDataB.getPose(sfmDataB.getView(viewIdPair.second)).getTransform().center();
    }

    if (reconstructedCommonViewIds.size() == 1)
    {
        *out_S = 1.0;
        *out_R = Mat3::Identity();
        *out_t = xB.col(0) - xA.col(0);
        return true;
    }

    // Compute rigid transformation p'i = S R pi + t
    double S;
    Vec3 t;
    Mat3 R;
    std::vector<std::size_t> inliers;

    if (!aliceVision::geometry::ACRansac_FindRTS(xA, xB, randomNumberGenerator, S, t, R, inliers, true))
        return false;

    ALICEVISION_LOG_DEBUG("There are " << reconstructedCommonViewIds.size() << " common cameras and " << inliers.size() << " were used to compute the similarity transform.");

    *out_S = S;
    *out_R = R;
    *out_t = t;

    return true;
}

bool computeSimilarityFromCommonCameras_viewId(const sfmData::SfMData& sfmDataA,
                       const sfmData::SfMData& sfmDataB,
                       std::mt19937 &randomNumberGenerator,
                       double* out_S,
                       Mat3* out_R,
                       Vec3* out_t)
{
  assert(out_S != nullptr);
  assert(out_R != nullptr);
  assert(out_t != nullptr);
  
  std::vector<IndexT> commonViewIds;
  getCommonViewsWithPoses(sfmDataA, sfmDataB, commonViewIds);
  ALICEVISION_LOG_DEBUG("Found " << commonViewIds.size() << " common views.");

  std::vector<std::pair<IndexT, IndexT>> commonViewIds_pairs;
  for (IndexT id : commonViewIds)
  {
      commonViewIds_pairs.push_back(std::make_pair(id, id));
  }
  return computeSimilarityFromCommonViews(sfmDataA, sfmDataB, commonViewIds_pairs, randomNumberGenerator, out_S, out_R, out_t);
}

bool computeSimilarityFromCommonCameras_poseId(
        const sfmData::SfMData& sfmDataA,
        const sfmData::SfMData& sfmDataB,
        std::mt19937 & randomNumberGenerator,
        double* out_S,
        Mat3* out_R,
        Vec3* out_t)
{
    assert(out_S != nullptr);
    assert(out_R != nullptr);
    assert(out_t != nullptr);

    std::vector<IndexT> commonPoseIds;
    getCommonPoseId(sfmDataA, sfmDataB, commonPoseIds);
    ALICEVISION_LOG_DEBUG("Found " << commonPoseIds.size() << " common views.");
    if (commonPoseIds.empty())
    {
        ALICEVISION_LOG_WARNING("Cannot compute similarities without common pose.");
        return false;
    }

    // Move input point in appropriate container
    Mat xA(3, commonPoseIds.size());
    Mat xB(3, commonPoseIds.size());
    for (std::size_t i = 0; i < commonPoseIds.size(); ++i)
    {
        IndexT poseId = commonPoseIds[i];
        xA.col(i) = sfmDataA.getAbsolutePose(poseId).getTransform().center();
        xB.col(i) = sfmDataB.getAbsolutePose(poseId).getTransform().center();
    }
    if (commonPoseIds.size() == 1)
    {
        *out_S = 1.0;
        *out_R = Mat3::Identity();
        *out_t = xB.col(0) - xA.col(0);
        return true;
    }

    // Compute rigid transformation p'i = S R pi + t
    double S;
    Vec3 t;
    Mat3 R;
    std::vector<std::size_t> inliers;

    if (!aliceVision::geometry::ACRansac_FindRTS(xA, xB, randomNumberGenerator, S, t, R, inliers, true))
        return false;

    ALICEVISION_LOG_DEBUG("There are " << commonPoseIds.size() << " common camera poses and " << inliers.size() << " were used to compute the similarity transform.");

    *out_S = S;
    *out_R = R;
    *out_t = t;

    return true;
}


std::map<std::string, IndexT> retrieveMatchingFilepath(
    const sfmData::SfMData& sfmData,
    const std::string& filePatternMatching)
{
    std::set<std::string> duplicates;
    std::map<std::string, IndexT> uniqueFileparts;
    for (auto& viewIt : sfmData.getViews())
    {
        const std::string& imagePath = viewIt.second->getImagePath();
        std::string cumulatedValues;
        if (filePatternMatching.empty())
        {
            cumulatedValues = imagePath;
        }
        else
        {
            std::regex re(filePatternMatching);
            std::smatch matches;
            if (std::regex_match(imagePath, matches, re))
            {
                for(int i = 1; i < matches.size(); ++i)
                {
                    const std::ssub_match& submatch = matches[i];
                    cumulatedValues += submatch.str();
                }
            }
        }
        ALICEVISION_LOG_TRACE("retrieveMatchingFilepath: " << imagePath << " -> " << cumulatedValues);
        auto it = uniqueFileparts.find(cumulatedValues);
        if (it != uniqueFileparts.end())
        {
            duplicates.insert(cumulatedValues);
        }
        else
        {
            uniqueFileparts[cumulatedValues] = viewIt.first;
        }
    }
    for (const std::string& d : duplicates)
    {
        uniqueFileparts.erase(d);
    }
    return uniqueFileparts;
}

void matchViewsByFilePattern(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::string& filePatternMatching,
    std::vector<std::pair<IndexT, IndexT>>& out_commonViewIds)
{
    out_commonViewIds.clear();
    std::map<std::string, IndexT> filepathValuesA = retrieveMatchingFilepath(sfmDataA, filePatternMatching);
    std::map<std::string, IndexT> filepathValuesB = retrieveMatchingFilepath(sfmDataB, filePatternMatching);

    using P = std::pair<std::string, IndexT>;
    std::vector<P> commonMetadataValues;
    std::set_intersection(
        filepathValuesA.begin(), filepathValuesA.end(), // already sorted
        filepathValuesB.begin(), filepathValuesB.end(), // already sorted
        std::back_inserter(commonMetadataValues),
        [](const P& p1, const P& p2) {
            return p1.first < p2.first;
        }
    );

    for (const P& p : commonMetadataValues)
    {
        out_commonViewIds.emplace_back(filepathValuesA.at(p.first), filepathValuesB.at(p.first));
    }
}


bool computeSimilarityFromCommonCameras_imageFileMatching(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::string& filePatternMatching,
    std::mt19937 &randomNumberGenerator,
    double* out_S,
    Mat3* out_R,
    Vec3* out_t)
{
    assert(out_S != nullptr);
    assert(out_R != nullptr);
    assert(out_t != nullptr);

    std::vector<std::pair<IndexT, IndexT>> commonViewIds;
    matchViewsByFilePattern(sfmDataA, sfmDataB, filePatternMatching, commonViewIds);

    ALICEVISION_LOG_DEBUG("Found " << commonViewIds.size() << " common views.");

    return computeSimilarityFromCommonViews(sfmDataA, sfmDataB, commonViewIds, randomNumberGenerator, out_S, out_R, out_t);
}



std::map<std::string, IndexT> retrieveUniqueMetadataValues(
    const sfmData::SfMData& sfmData,
    const std::vector<std::string>& metadataList)
{
    std::set<std::string> duplicates;
    std::map<std::string, IndexT> uniqueMetadataValues;
    for (auto& viewIt : sfmData.getViews())
    {
        const std::map<std::string, std::string>& m = viewIt.second->getMetadata();
        std::string cumulatedValues;
        for (const std::string& k : metadataList)
        {
            const auto mIt = m.find(k);
            if (mIt != m.end())
                cumulatedValues += mIt->second;
        }
        ALICEVISION_LOG_TRACE("retrieveUniqueMetadataValues: " << viewIt.second->getImagePath() << " -> " << cumulatedValues);
        auto it = uniqueMetadataValues.find(cumulatedValues);
        if (it != uniqueMetadataValues.end())
        {
            duplicates.insert(cumulatedValues);
        }
        else
        {
            uniqueMetadataValues[cumulatedValues] = viewIt.first;
        }
    }
    for (const std::string& d: duplicates)
    {
        uniqueMetadataValues.erase(d);
    }
    return uniqueMetadataValues;
}

void matchViewsByMetadataMatching(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::vector<std::string>& metadataList,
    std::vector<std::pair<IndexT, IndexT>>& out_commonViewIds)
{
    out_commonViewIds.clear();
    std::map<std::string, IndexT> metadataValuesA = retrieveUniqueMetadataValues(sfmDataA, metadataList);
    std::map<std::string, IndexT> metadataValuesB = retrieveUniqueMetadataValues(sfmDataB, metadataList);

    using P = std::pair<std::string, IndexT>;
    std::vector<P> commonMetadataValues;
    std::set_intersection(
        metadataValuesA.begin(), metadataValuesA.end(), // already sorted
        metadataValuesB.begin(), metadataValuesB.end(), // already sorted
        std::back_inserter(commonMetadataValues),
        [](const P& p1, const P& p2) {
            return p1.first < p2.first;
        }
    );
    for (const P& p : commonMetadataValues)
    {
        out_commonViewIds.emplace_back(metadataValuesA.at(p.first), metadataValuesB.at(p.first));
    }
}

bool computeSimilarityFromCommonCameras_metadataMatching(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    const std::vector<std::string>& metadataList,
    std::mt19937 &randomNumberGenerator,
    double* out_S,
    Mat3* out_R,
    Vec3* out_t)
{
    assert(out_S != nullptr);
    assert(out_R != nullptr);
    assert(out_t != nullptr);

    std::vector<std::pair<IndexT, IndexT>> commonViewIds;
    matchViewsByMetadataMatching(sfmDataA, sfmDataB, metadataList, commonViewIds);

    ALICEVISION_LOG_DEBUG("Found " << commonViewIds.size() << " common views.");

    return computeSimilarityFromCommonViews(sfmDataA, sfmDataB, commonViewIds, randomNumberGenerator, out_S, out_R, out_t);
}


/**
 * @return map<pair<DescType, markerId>, landmarkId>
 */
std::map<std::pair<feature::EImageDescriberType, int>, IndexT> getUniqueMarkers(const sfmData::SfMData& sfmDataA)
{
    std::set<std::pair<feature::EImageDescriberType, int>> duplicates;
    std::map<std::pair<feature::EImageDescriberType, int>, IndexT> markers;

    for (const auto& landmarkIt : sfmDataA.getLandmarks())
    {
        if (isMarker(landmarkIt.second.descType))
        {
            const auto p = std::make_pair(landmarkIt.second.descType, landmarkIt.second.rgb.r());
            if (markers.find(p) == markers.end())
            {
                markers[p] = landmarkIt.first;
            }
            else
            {
                duplicates.insert(p); // multiple usages
            }
        }
    }
    for (const auto& d : duplicates)
    {
        markers.erase(d);
    }
    return markers;
}


bool computeSimilarityFromCommonMarkers(
    const sfmData::SfMData& sfmDataA,
    const sfmData::SfMData& sfmDataB,
    std::mt19937 & randomNumberGenerator,
    double* out_S,
    Mat3* out_R,
    Vec3* out_t)
{
    assert(out_S != nullptr);
    assert(out_R != nullptr);
    assert(out_t != nullptr);

    std::map<std::pair<feature::EImageDescriberType, int>, IndexT> markers_A = getUniqueMarkers(sfmDataA);
    std::map<std::pair<feature::EImageDescriberType, int>, IndexT> markers_B = getUniqueMarkers(sfmDataB);

    using P = std::pair<std::pair<feature::EImageDescriberType, int>, IndexT>;
    std::vector<P> commonMarkers;
    std::set_intersection(
        markers_A.begin(), markers_A.end(), // already sorted
        markers_B.begin(), markers_B.end(), // already sorted
        std::back_inserter(commonMarkers),
        [](const P& p1, const P& p2) {
            return p1.first < p2.first;
        }
    );

    ALICEVISION_LOG_DEBUG("Found " << commonMarkers.size() << " common markers.");
    if (commonMarkers.empty())
    {
        ALICEVISION_LOG_WARNING("Cannot compute similarities without common marker.");
        return false;
    }

    std::map<IndexT, std::pair<IndexT, IndexT>> commonLandmarks;
    for (auto& m : commonMarkers)
    {
        auto& markerLandmarks = commonLandmarks[m.first.second];
        markerLandmarks.first = markers_A.at(m.first);
        markerLandmarks.second = markers_B.at(m.first);
    }

    // Move input point in appropriate container
    Mat xA(3, commonLandmarks.size());
    Mat xB(3, commonLandmarks.size());
    auto it = commonLandmarks.begin();
    for (std::size_t i = 0; i < commonLandmarks.size(); ++i, ++it)
    {
        const std::pair<IndexT, IndexT>& commonLandmark = it->second;
        xA.col(i) = sfmDataA.getLandmarks().at(commonLandmark.first).X;
        xB.col(i) = sfmDataB.getLandmarks().at(commonLandmark.second).X;
    }

    if (commonLandmarks.size() == 1)
    {
        *out_S = 1.0;
        *out_R = Mat3::Identity();
        *out_t = xB.col(0) - xA.col(0);
        return true;
    }

    // Compute rigid transformation p'i = S R pi + t
    double S;
    Vec3 t;
    Mat3 R;
    std::vector<std::size_t> inliers;

    if (!aliceVision::geometry::ACRansac_FindRTS(xA, xB, randomNumberGenerator, S, t, R, inliers, true))
        return false;

    ALICEVISION_LOG_DEBUG("There are " << commonLandmarks.size() << " common markers and " << inliers.size() << " were used to compute the similarity transform.");

    *out_S = S;
    *out_R = R;
    *out_t = t;

    return true;
}

void computeNewCoordinateSystemFromCameras(const sfmData::SfMData& sfmData,
                                           double& out_S,
                                           Mat3& out_R,
                                           Vec3& out_t)
{
  Mat3X vX(3, sfmData.getLandmarks().size());

  std::size_t i = 0;
  for(const auto& landmark : sfmData.getLandmarks())
  {
    vX.col(i) = landmark.second.X;
    ++i;
  }

  const std::size_t nbCameras = sfmData.getPoses().size();
  Mat3X vCamCenter(3,nbCameras);

  // Compute the mean of the point cloud
  Vec3 meanPoints = Vec3::Zero(3,1);
  i=0;

  for(const auto & pose : sfmData.getPoses())
  {
    const Vec3 center = pose.second.getTransform().center();
    vCamCenter.col(i) = center;
    meanPoints +=  center;
    ++i;
  }

  meanPoints /= nbCameras;

  std::vector<Mat3> vCamRotation; // Camera rotations
  vCamRotation.reserve(nbCameras);
  i=0;
  double rms = 0;
  for(const auto & pose : sfmData.getPoses())
  {
    Vec3 camCenterMean = vCamCenter.col(i) - meanPoints;
    rms += camCenterMean.transpose() * camCenterMean; // squared dist to the mean of camera centers

    vCamRotation.push_back(pose.second.getTransform().rotation().transpose()); // Rotation in the world coordinate system
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

IndexT getViewIdFromExpression(const sfmData::SfMData& sfmData, const std::string & camName)
{
  IndexT viewId = -1;

  std::regex cameraRegex = simpleFilterToRegex_noThrow(camName);

  try
  {
    viewId = boost::lexical_cast<IndexT>(camName);
    if(!sfmData.getViews().count(viewId))
      viewId = -1;
  }
  catch(const boost::bad_lexical_cast &)
  {
    viewId = -1;
  }

  if(viewId == -1)
  {
    for(const auto & view : sfmData.getViews())
    {
      const std::string path = view.second->getImagePath();
      if(std::regex_match(path, cameraRegex))
      {
          viewId = view.second->getViewId();
          break;
      }
    }
  }

  if(viewId == -1)
    throw std::invalid_argument("The camera name \"" + camName + "\" is not found in the sfmData.");
  else if(!sfmData.isPoseAndIntrinsicDefined(viewId))
    throw std::invalid_argument("The camera \"" + camName + "\" exists in the sfmData but is not reconstructed.");

  return viewId;
}

IndexT getCenterCameraView(const sfmData::SfMData& sfmData)
{
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::mean>> accX, accY, accZ;

    for(auto& pose: sfmData.getPoses())
    {
        const auto& c = pose.second.getTransform().center();
        accX(c(0));
        accY(c(1));
        accZ(c(2));
    }
    const Vec3 camerasCenter(extract::mean(accX), extract::mean(accY), extract::mean(accZ));

    double minDist = std::numeric_limits<double>::max();
    IndexT centerViewId = UndefinedIndexT;
    for(auto& viewIt: sfmData.getViews())
    {
        const sfmData::View& v = *viewIt.second;
        if(!sfmData.isPoseAndIntrinsicDefined(&v))
            continue;
        const auto& pose = sfmData.getPose(v);
        const double dist = (pose.getTransform().center() - camerasCenter).norm();

        if(dist < minDist)
        {
            minDist = dist;
            centerViewId = v.getViewId();
        }
    }
    return centerViewId;
}

void computeNewCoordinateSystemFromSingleCamera(const sfmData::SfMData& sfmData, const IndexT viewId,
                                                double& out_S, Mat3& out_R, Vec3& out_t)
{
  sfmData::EEXIFOrientation orientation = sfmData.getView(viewId).getMetadataOrientation();
  ALICEVISION_LOG_TRACE("computeNewCoordinateSystemFromSingleCamera orientation: " << int(orientation));

  const sfmData::View& view = sfmData.getView(viewId);
  switch(orientation)
  {
    case sfmData::EEXIFOrientation::RIGHT:
          ALICEVISION_LOG_TRACE("computeNewCoordinateSystemFromSingleCamera orientation: RIGHT");
          out_R = Eigen::AngleAxisd(degreeToRadian(90.0),  Vec3(0,0,1))
                  * sfmData.getPose(view).getTransform().rotation();
          break;
    case sfmData::EEXIFOrientation::LEFT:
          ALICEVISION_LOG_TRACE("computeNewCoordinateSystemFromSingleCamera orientation: LEFT");
          out_R = Eigen::AngleAxisd(degreeToRadian(270.0),  Vec3(0,0,1))
                  * sfmData.getPose(view).getTransform().rotation();
          break;
    case sfmData::EEXIFOrientation::UPSIDEDOWN:
          ALICEVISION_LOG_TRACE("computeNewCoordinateSystemFromSingleCamera orientation: UPSIDEDOWN");
          out_R = sfmData.getPose(view).getTransform().rotation();
          break;
    case sfmData::EEXIFOrientation::NONE:
          ALICEVISION_LOG_TRACE("computeNewCoordinateSystemFromSingleCamera orientation: NONE");
          out_R = sfmData.getPose(view).getTransform().rotation();
          break;
    default:
          ALICEVISION_LOG_TRACE("computeNewCoordinateSystemFromSingleCamera orientation: default");
          out_R = sfmData.getPose(view).getTransform().rotation();
          break;
  }

  out_t = - out_R * sfmData.getPose(view).getTransform().center();
  out_S = 1.0;
}

void computeNewCoordinateSystemFromLandmarks(const sfmData::SfMData& sfmData,
                                    const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
                                    double& out_S,
                                    Mat3& out_R,
                                    Vec3& out_t)
{
    const std::size_t nbLandmarks = sfmData.getLandmarks().size();

    Mat3X vX(3,nbLandmarks);

    std::size_t i = 0;

    Vec3 meanPoints = Vec3::Zero(3,1);
    std::size_t nbMeanLandmarks = 0;

    bacc::accumulator_set<double, bacc::stats<bacc::tag::min, bacc::tag::max> > accX, accY, accZ;

    for(const auto& landmark : sfmData.getLandmarks())
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


bool computeNewCoordinateSystemFromSpecificMarkers(const sfmData::SfMData& sfmData,
    const feature::EImageDescriberType& imageDescriberType,
    const std::vector<MarkerWithCoord>& markers,
    bool withScaling,
    double& out_S,
    Mat3& out_R,
    Vec3& out_t
    )
{
    std::vector<int> landmarksIds(markers.size(), -1);

    int maxLandmarkIdx = 0;
    for (const auto& landmarkIt : sfmData.getLandmarks())
    {
        if(landmarkIt.first > maxLandmarkIdx)
            maxLandmarkIdx = landmarkIt.first;
        if(landmarkIt.second.descType != imageDescriberType)
            continue;
        for (int i = 0; i < markers.size(); ++i)
        {
            if (landmarkIt.second.rgb.r() == markers[i].id)
            {
                landmarksIds[i] = landmarkIt.first;
            }
        }
    }

    for (int i = 0; i < landmarksIds.size(); ++i)
    {
        int landmarkId = landmarksIds[i];
        if (landmarkId == -1)
        {
            ALICEVISION_LOG_ERROR("Failed to find marker: " << int(markers[i].id));
            ALICEVISION_THROW_ERROR("Failed to find marker: " << int(markers[i].id));
        }
    }

    Mat ptsSrc = Mat3X(3, markers.size());
    Mat ptsDst = Mat3X(3, markers.size());
    for (std::size_t i = 0; i < markers.size(); ++i)
    {
        ptsSrc.col(i) = sfmData.getLandmarks().at(landmarksIds[i]).X;
        ptsDst.col(i) = markers[i].coord;
    }

    if (markers.size() == 1)
    {
        out_S = 1;
        out_R = Mat3::Identity();
        out_t = ptsDst.col(0) - ptsSrc.col(0);
        return true;
    }

    const Mat4 RTS = Eigen::umeyama(ptsSrc, ptsDst, withScaling);

    return geometry::decomposeRTS(RTS, out_S, out_t, out_R);
}



bool computeNewCoordinateSystemFromGpsData(const sfmData::SfMData& sfmData, std::mt19937 &randomNumberGenerator, double& out_S, Mat3& out_R, Vec3& out_t)
{
    std::vector<Vec3> gpsPositions{};
    std::vector<Vec3> centers{};
    gpsPositions.reserve(sfmData.getPoses().size());
    centers.reserve(sfmData.getPoses().size());

    // for each reconstructed view
    for(const auto& v : sfmData.getViews())
    {
        const auto viewID = v.first;
        const auto& view = v.second;
        // skip no pose
        if(!(sfmData.isPoseAndIntrinsicDefined(viewID) && view->hasGpsMetadata()))
        {
            ALICEVISION_LOG_TRACE("Skipping view " << viewID << " because pose " << sfmData.isPoseAndIntrinsicDefined(viewID) << " and gps " << view->hasGpsMetadata());
            continue;
        }
        // extract the gps position
        gpsPositions.push_back(view->getGpsPositionFromMetadata());
        // get the center
        centers.push_back(sfmData.getPose(*view.get()).getTransform().center());
    }

    // if enough data try to find the transformation
    if(gpsPositions.size() < 4)
    {
        ALICEVISION_LOG_INFO("Not enough points to estimate the rototranslation to align the gps");
        return false;
    }

    Mat x1(3, centers.size());
    Mat x2(3, gpsPositions.size());

    for(Mat::Index i = 0; i < gpsPositions.size(); ++i)
    {
        x1.col(i) = centers[i];
        x2.col(i) = gpsPositions[i];
    }

    std::vector<std::size_t> inliers;
    const bool refine{true};
    return aliceVision::geometry::ACRansac_FindRTS(x1, x2, randomNumberGenerator, out_S, out_t, out_R, inliers, refine);
}

} // namespace sfm
} // namespace aliceVision
