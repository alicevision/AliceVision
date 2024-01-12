// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/geometry/rigidTransformation3D.hpp>
#include <aliceVision/stl/regex.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <regex>
#include <numeric>

#include <aliceVision/numeric/gps.hpp>

namespace bacc = boost::accumulators;

namespace aliceVision {
namespace sfm {

std::istream& operator>>(std::istream& in, MarkerWithCoord& marker)
{
    std::string token;
    in >> token;
    std::vector<std::string> markerCoord;
    boost::split(markerCoord, token, boost::algorithm::is_any_of(":="));
    if (markerCoord.size() != 2)
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
                                      std::mt19937& randomNumberGenerator,
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

    ALICEVISION_LOG_DEBUG("There are " << reconstructedCommonViewIds.size() << " common cameras and " << inliers.size()
                                       << " were used to compute the similarity transform.");

    *out_S = S;
    *out_R = R;
    *out_t = t;

    return true;
}

bool computeSimilarityFromCommonCameras_viewId(const sfmData::SfMData& sfmDataA,
                                               const sfmData::SfMData& sfmDataB,
                                               std::mt19937& randomNumberGenerator,
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

bool computeSimilarityFromCommonCameras_poseId(const sfmData::SfMData& sfmDataA,
                                               const sfmData::SfMData& sfmDataB,
                                               std::mt19937& randomNumberGenerator,
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
        const IndexT poseId = commonPoseIds[i];
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

    ALICEVISION_LOG_DEBUG("There are " << commonPoseIds.size() << " common camera poses and " << inliers.size()
                                       << " were used to compute the similarity transform.");

    *out_S = S;
    *out_R = R;
    *out_t = t;

    return true;
}

std::map<std::string, IndexT> retrieveMatchingFilepath(const sfmData::SfMData& sfmData, const std::string& filePatternMatching)
{
    std::set<std::string> duplicates;
    std::map<std::string, IndexT> uniqueFileparts;
    for (auto& viewIt : sfmData.getViews())
    {
        const std::string& imagePath = viewIt.second->getImage().getImagePath();
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
                for (int i = 1; i < matches.size(); ++i)
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

void matchViewsByFilePattern(const sfmData::SfMData& sfmDataA,
                             const sfmData::SfMData& sfmDataB,
                             const std::string& filePatternMatching,
                             std::vector<std::pair<IndexT, IndexT>>& out_commonViewIds)
{
    out_commonViewIds.clear();
    std::map<std::string, IndexT> filepathValuesA = retrieveMatchingFilepath(sfmDataA, filePatternMatching);
    std::map<std::string, IndexT> filepathValuesB = retrieveMatchingFilepath(sfmDataB, filePatternMatching);

    using P = std::pair<std::string, IndexT>;
    std::vector<P> commonMetadataValues;
    std::set_intersection(filepathValuesA.begin(),
                          filepathValuesA.end(),  // already sorted
                          filepathValuesB.begin(),
                          filepathValuesB.end(),  // already sorted
                          std::back_inserter(commonMetadataValues),
                          [](const P& p1, const P& p2) { return p1.first < p2.first; });

    for (const P& p : commonMetadataValues)
    {
        out_commonViewIds.emplace_back(filepathValuesA.at(p.first), filepathValuesB.at(p.first));
    }
}

bool computeSimilarityFromCommonCameras_imageFileMatching(const sfmData::SfMData& sfmDataA,
                                                          const sfmData::SfMData& sfmDataB,
                                                          const std::string& filePatternMatching,
                                                          std::mt19937& randomNumberGenerator,
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

std::map<std::string, IndexT> retrieveUniqueMetadataValues(const sfmData::SfMData& sfmData, const std::vector<std::string>& metadataList)
{
    std::set<std::string> duplicates;
    std::map<std::string, IndexT> uniqueMetadataValues;
    for (auto& viewIt : sfmData.getViews())
    {
        const std::map<std::string, std::string>& m = viewIt.second->getImage().getMetadata();
        std::string cumulatedValues;
        for (const std::string& k : metadataList)
        {
            const auto mIt = m.find(k);
            if (mIt != m.end())
                cumulatedValues += mIt->second;
        }
        ALICEVISION_LOG_TRACE("retrieveUniqueMetadataValues: " << viewIt.second->getImage().getImagePath() << " -> " << cumulatedValues);
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
    for (const std::string& d : duplicates)
    {
        uniqueMetadataValues.erase(d);
    }
    return uniqueMetadataValues;
}

void matchViewsByMetadataMatching(const sfmData::SfMData& sfmDataA,
                                  const sfmData::SfMData& sfmDataB,
                                  const std::vector<std::string>& metadataList,
                                  std::vector<std::pair<IndexT, IndexT>>& out_commonViewIds)
{
    out_commonViewIds.clear();
    std::map<std::string, IndexT> metadataValuesA = retrieveUniqueMetadataValues(sfmDataA, metadataList);
    std::map<std::string, IndexT> metadataValuesB = retrieveUniqueMetadataValues(sfmDataB, metadataList);

    using P = std::pair<std::string, IndexT>;
    std::vector<P> commonMetadataValues;
    std::set_intersection(metadataValuesA.begin(),
                          metadataValuesA.end(),  // already sorted
                          metadataValuesB.begin(),
                          metadataValuesB.end(),  // already sorted
                          std::back_inserter(commonMetadataValues),
                          [](const P& p1, const P& p2) { return p1.first < p2.first; });
    for (const P& p : commonMetadataValues)
    {
        out_commonViewIds.emplace_back(metadataValuesA.at(p.first), metadataValuesB.at(p.first));
    }
}

bool computeSimilarityFromCommonCameras_metadataMatching(const sfmData::SfMData& sfmDataA,
                                                         const sfmData::SfMData& sfmDataB,
                                                         const std::vector<std::string>& metadataList,
                                                         std::mt19937& randomNumberGenerator,
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
                duplicates.insert(p);  // multiple usages
            }
        }
    }
    for (const auto& d : duplicates)
    {
        markers.erase(d);
    }
    return markers;
}

bool computeSimilarityFromCommonMarkers(const sfmData::SfMData& sfmDataA,
                                        const sfmData::SfMData& sfmDataB,
                                        std::mt19937& randomNumberGenerator,
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
    std::set_intersection(markers_A.begin(),
                          markers_A.end(),  // already sorted
                          markers_B.begin(),
                          markers_B.end(),  // already sorted
                          std::back_inserter(commonMarkers),
                          [](const P& p1, const P& p2) { return p1.first < p2.first; });

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

    ALICEVISION_LOG_DEBUG("There are " << commonLandmarks.size() << " common markers and " << inliers.size()
                                       << " were used to compute the similarity transform.");

    *out_S = S;
    *out_R = R;
    *out_t = t;

    return true;
}

/**
 * Image orientation CCW
 */
double orientationToRotationDegree(sfmData::EEXIFOrientation orientation)
{
    switch (orientation)
    {
        case sfmData::EEXIFOrientation::RIGHT:       // 8
            return 90.0;                             // CCW
        case sfmData::EEXIFOrientation::LEFT:        // 6
            return 270.0;                            // CCW
        case sfmData::EEXIFOrientation::UPSIDEDOWN:  // 3
            return 180.0;
        case sfmData::EEXIFOrientation::NONE:
        default:
            return 0.0;
    }
    return 0.0;
}

void computeNewCoordinateSystemFromCamerasXAxis(const sfmData::SfMData& sfmData, double& out_S, Mat3& out_R, Vec3& out_t)
{
    out_S = 1.0;
    out_R = Mat3::Identity();
    out_t = Vec3::Zero();

    // mean of the camera centers
    Vec3 meanCameraCenter = Vec3::Zero(3, 1);
    // Compute mean of the rotation X component
    Eigen::Vector3d meanRx = Eigen::Vector3d::Zero();
    Eigen::Vector3d meanRy = Eigen::Vector3d::Zero();

    std::size_t validPoses = 0;
    for (auto& viewIt : sfmData.getViews())
    {
        const sfmData::View& view = *viewIt.second.get();

        if (sfmData.isPoseAndIntrinsicDefined(&view))
        {
            const sfmData::EEXIFOrientation orientation = view.getImage().getMetadataOrientation();
            const sfmData::CameraPose camPose = sfmData.getPose(view);
            const geometry::Pose3& p = camPose.getTransform();

            // Rotation of image
            Mat3 R_image = Eigen::AngleAxisd(degreeToRadian(orientationToRotationDegree(orientation)), Vec3(0, 0, 1)).toRotationMatrix();

            Eigen::Vector3d oriented_X = R_image * Eigen::Vector3d::UnitX();
            Eigen::Vector3d oriented_Y = R_image * Eigen::Vector3d::UnitY();

            // The X direction is in the "viewed" image
            // If we use the raw X, it will be in the image without the orientation
            // We need to use this orientation to make sure the X spans the horizontal plane.
            const Eigen::Vector3d rX = p.rotation().transpose() * oriented_X;
            const Eigen::Vector3d rY = p.rotation().transpose() * oriented_Y;

            meanRx += rX;
            meanRy += rY;

            meanCameraCenter += p.center();
            ++validPoses;
        }
    }
    if (validPoses == 0)
    {
        return;
    }

    meanRx /= validPoses;
    meanRy /= validPoses;
    meanCameraCenter /= validPoses;

    size_t count = 0;
    double rms = 0.0;
    // Compute covariance matrix of the rotation X component
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (auto& viewIt : sfmData.getViews())
    {
        const sfmData::View& view = *viewIt.second.get();

        if (sfmData.isPoseAndIntrinsicDefined(&view))
        {
            const sfmData::EEXIFOrientation orientation = view.getImage().getMetadataOrientation();
            const sfmData::CameraPose camPose = sfmData.getPose(view);
            const geometry::Pose3& p = camPose.getTransform();

            Mat3 R_image = Eigen::AngleAxisd(degreeToRadian(orientationToRotationDegree(orientation)), Vec3(0, 0, 1)).toRotationMatrix();
            Eigen::Vector3d oriented_X = R_image * Eigen::Vector3d::UnitX();

            const Eigen::Vector3d rX = p.rotation().transpose() * oriented_X;
            C += (rX - meanRx) * (rX - meanRx).transpose();

            count++;
        }
    }

    if (count > 1)
    {
        C = C / double(count - 1);
    }

    Eigen::EigenSolver<Eigen::Matrix3d> solver(C, true);

    // Warning, eigenvalues are not sorted ...
    Vec3 evalues = solver.eigenvalues().real();
    Vec3 aevalues = evalues.cwiseAbs();

    std::vector<int> indices(evalues.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&](int i, int j) { return evalues[i] < evalues[j]; });
    int minCol = indices[0];

    // Make sure we have a clear unique small eigen value
    double ratio1 = evalues[indices[2]] / evalues[indices[0]];
    double ratio2 = evalues[indices[2]] / evalues[indices[1]];
    double unicity = ratio1 / ratio2;
    double largest = std::abs(evalues[indices[2]]);

    ALICEVISION_LOG_DEBUG("computeNewCoordinateSystemFromCamerasXAxis: eigenvalues: " << solver.eigenvalues());
    ALICEVISION_LOG_DEBUG("computeNewCoordinateSystemFromCamerasXAxis: eigenvectors: " << solver.eigenvectors());
    ALICEVISION_LOG_DEBUG("computeNewCoordinateSystemFromCamerasXAxis: unicity: " << unicity);
    ALICEVISION_LOG_DEBUG("computeNewCoordinateSystemFromCamerasXAxis: largest eigen value: " << largest);

    // We assume that the X axis of all or majority of the cameras are on a plane.
    // The covariance is a flat ellipsoid and the min axis is our candidate Y axis.
    Eigen::Vector3d nullestSpace = solver.eigenvectors().col(minCol).real();
    Eigen::Vector3d referenceAxis = Eigen::Vector3d::UnitY();

    if (std::abs(unicity) < 10.0 || largest < 1e-2)
    {
        ALICEVISION_LOG_DEBUG("Algorithm did not find a clear axis. Align with raw Y.");
        nullestSpace = meanRy;
    }

    // Compute the rotation which rotates nullestSpace onto unitY
    out_R = Matrix3d(Quaterniond().setFromTwoVectors(nullestSpace, referenceAxis));
    const double d = (out_R * meanRy).normalized().dot(Eigen::Vector3d::UnitY());
    const bool inverseDirection = (d < 0.0);
    // We have an ambiguity on the Y direction, so if our Y axis is not aligned with the Y axis of the scene
    // we inverse the axis.
    if (inverseDirection)
    {
        nullestSpace = -nullestSpace;
        out_R = Matrix3d(Quaterniond().setFromTwoVectors(nullestSpace, referenceAxis));
    }

    out_S = 1.0;
    out_t = -out_R * meanCameraCenter;
}

void computeNewCoordinateSystemFromCameras(const sfmData::SfMData& sfmData, double& out_S, Mat3& out_R, Vec3& out_t)
{
    const std::size_t nbCameras = sfmData.getPoses().size();
    Mat3X vCamCenter(3, nbCameras);

    // Compute the mean of the point cloud
    Vec3 meanCameraCenter = Vec3::Zero();

    Vec3::Index ncol = 0;
    for (const auto& pose : sfmData.getPoses())
    {
        const Vec3 center = pose.second.getTransform().center();
        vCamCenter.col(ncol) = center;
        meanCameraCenter += center;
        ++ncol;
    }
    meanCameraCenter /= nbCameras;

    // Compute standard deviation
    double stddev = 0;
    for (Vec3::Index i = 0; i < vCamCenter.cols(); ++i)
    {
        Vec3 camCenterMean = vCamCenter.col(i) - meanCameraCenter;
        stddev += camCenterMean.transpose() * camCenterMean;
    }
    stddev /= nbCameras;
    if (stddev < 1e-12)  // If there is no translation change
    {
        stddev = 1.0;
    }

    // Make sure the point cloud is centered and scaled to unit deviation
    for (Vec3::Index i = 0; i < vCamCenter.cols(); ++i)
    {
        vCamCenter.col(i) = (vCamCenter.col(i) - meanCameraCenter) / stddev;
    }

    // Plane fitting of the centered point cloud
    // using Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Mat> svd(vCamCenter.transpose(), Eigen::ComputeFullV);
    Eigen::Vector3d n = svd.matrixV().col(2);

    // Normal vector sign can't really be estimated. Just make sure it's the positive Y half sphere
    if (n(1) < 0)
    {
        n = -n;
    }

    // We want ideal normal to be the Y axis
    out_R = Matrix3d(Quaterniond().setFromTwoVectors(n, Eigen::Vector3d::UnitY()));
    out_S = 1.0 / sqrt(stddev);
    out_t = -out_S * out_R * meanCameraCenter;
}

IndexT getViewIdFromExpression(const sfmData::SfMData& sfmData, const std::string& camName)
{
    IndexT viewId = -1;

    std::regex cameraRegex = simpleFilterToRegex_noThrow(camName);

    try
    {
        viewId = boost::lexical_cast<IndexT>(camName);
        if (!sfmData.getViews().count(viewId))
        {
            bool found = false;
            // check if this view is an ancestor of a view
            for (auto pv : sfmData.getViews())
            {
                for (auto ancestor : pv.second->getAncestors())
                {
                    if (ancestor == viewId)
                    {
                        viewId = pv.first;
                        found = true;
                        break;
                    }
                }

                if (found)
                {
                    break;
                }
            }

            if (!found)
            {
                viewId = -1;
            }
        }
    }
    catch (const boost::bad_lexical_cast&)
    {
        viewId = -1;
    }

    if (viewId == -1)
    {
        for (const auto& view : sfmData.getViews())
        {
            const std::string path = view.second->getImage().getImagePath();
            if (std::regex_match(path, cameraRegex))
            {
                viewId = view.second->getViewId();
                break;
            }
        }
    }

    if (viewId == -1)
        throw std::invalid_argument("The camera name \"" + camName + "\" is not found in the sfmData.");
    else if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        throw std::invalid_argument("The camera \"" + camName + "\" exists in the sfmData but is not reconstructed.");

    return viewId;
}

IndexT getCenterCameraView(const sfmData::SfMData& sfmData)
{
    using namespace boost::accumulators;
    accumulator_set<double, stats<tag::mean>> accX, accY, accZ;

    for (auto& pose : sfmData.getPoses())
    {
        const auto& c = pose.second.getTransform().center();
        accX(c(0));
        accY(c(1));
        accZ(c(2));
    }
    const Vec3 camerasCenter(extract::mean(accX), extract::mean(accY), extract::mean(accZ));

    double minDist = std::numeric_limits<double>::max();
    IndexT centerViewId = UndefinedIndexT;
    for (auto& viewIt : sfmData.getViews())
    {
        const sfmData::View& v = *viewIt.second;
        if (!sfmData.isPoseAndIntrinsicDefined(&v))
            continue;
        const auto& pose = sfmData.getPose(v);
        const double dist = (pose.getTransform().center() - camerasCenter).norm();

        if (dist < minDist)
        {
            minDist = dist;
            centerViewId = v.getViewId();
        }
    }
    return centerViewId;
}

void computeNewCoordinateSystemFromSingleCamera(const sfmData::SfMData& sfmData, const IndexT viewId, double& out_S, Mat3& out_R, Vec3& out_t)
{
    sfmData::EEXIFOrientation orientation = sfmData.getView(viewId).getImage().getMetadataOrientation();
    ALICEVISION_LOG_TRACE("computeNewCoordinateSystemFromSingleCamera orientation: " << int(orientation));

    Mat3 R_image = Eigen::AngleAxisd(degreeToRadian(orientationToRotationDegree(orientation)), Vec3(0, 0, 1)).toRotationMatrix();

    out_R = R_image.transpose() * sfmData.getAbsolutePose(viewId).getTransform().rotation();

    out_t = -out_R * sfmData.getAbsolutePose(viewId).getTransform().center();
    out_S = 1.0;
}

void computeNewCoordinateSystemFromLandmarks(const sfmData::SfMData& sfmData,
                                             const std::vector<feature::EImageDescriberType>& imageDescriberTypes,
                                             double& out_S,
                                             Mat3& out_R,
                                             Vec3& out_t)
{
    Mat3X vX(3, sfmData.getLandmarks().size());

    std::size_t landmarksCount = 0;

    Vec3 meanPoints = Vec3::Zero(3, 1);
    std::size_t nbMeanLandmarks = 0;

    for (const auto& landmark : sfmData.getLandmarks())
    {
        if (!imageDescriberTypes.empty() &&
            std::find(imageDescriberTypes.begin(), imageDescriberTypes.end(), landmark.second.descType) == imageDescriberTypes.end())
        {
            continue;
        }
        const Vec3& position = landmark.second.X;
        vX.col(landmarksCount++) = position;
        meanPoints += position;
    }
    vX.conservativeResize(3, landmarksCount);
    meanPoints /= landmarksCount;

    const std::size_t cacheSize = 10000;
    const double percentile = 0.99;
    using namespace boost::accumulators;
    using AccumulatorMax = accumulator_set<double, stats<tag::tail_quantile<right>>>;
    AccumulatorMax accDist(tag::tail<right>::cache_size = cacheSize);

    // Center the point cloud in [0;0;0]
    for (int i = 0; i < landmarksCount; ++i)
    {
        vX.col(i) -= meanPoints;
        accDist(vX.col(i).norm());
    }

    // Perform an svd over vX*vXT (var-covar)
    const Mat3 dum = vX.leftCols(nbMeanLandmarks) * vX.leftCols(nbMeanLandmarks).transpose();
    Eigen::JacobiSVD<Mat3> svd(dum, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Mat3 U = svd.matrixU();

    // Check whether the determinant is negative in order to keep
    // a direct coordinate system
    if (U.determinant() < 0)
    {
        U.col(2) = -U.col(2);
    }

    const double distMax = quantile(accDist, quantile_probability = percentile);

    out_S = (distMax > 0.00001 ? 1.0 / distMax : 1.0);
    out_R = U.transpose();
    out_R = Eigen::AngleAxisd(degreeToRadian(90.0), Vec3(1, 0, 0)) * out_R;
    out_t = -out_S * out_R * meanPoints;
}

bool computeNewCoordinateSystemFromSpecificMarkers(const sfmData::SfMData& sfmData,
                                                   const feature::EImageDescriberType& imageDescriberType,
                                                   const std::vector<MarkerWithCoord>& markers,
                                                   bool withScaling,
                                                   double& out_S,
                                                   Mat3& out_R,
                                                   Vec3& out_t)
{
    std::vector<int> landmarksIds(markers.size(), -1);

    int maxLandmarkIdx = 0;
    for (const auto& landmarkIt : sfmData.getLandmarks())
    {
        if (landmarkIt.first > maxLandmarkIdx)
            maxLandmarkIdx = landmarkIt.first;
        if (landmarkIt.second.descType != imageDescriberType)
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

bool computeNewCoordinateSystemFromGpsData(const sfmData::SfMData& sfmData,
                                           std::mt19937& randomNumberGenerator,
                                           double& out_S,
                                           Mat3& out_R,
                                           Vec3& out_t)
{
    std::vector<Vec3> gpsPositions{};
    std::vector<Vec3> centers{};
    gpsPositions.reserve(sfmData.getPoses().size());
    centers.reserve(sfmData.getPoses().size());

    // for each reconstructed view
    for (const auto& v : sfmData.getViews())
    {
        const auto viewID = v.first;
        const auto& view = v.second;
        // skip no pose
        if (!(sfmData.isPoseAndIntrinsicDefined(viewID) && view->getImage().hasGpsMetadata()))
        {
            ALICEVISION_LOG_TRACE("Skipping view " << viewID << " because pose " << sfmData.isPoseAndIntrinsicDefined(viewID) << " and gps "
                                                   << view->getImage().hasGpsMetadata());
            continue;
        }
        // extract the gps position
        gpsPositions.push_back(view->getImage().getGpsPositionFromMetadata());
        // get the center
        centers.push_back(sfmData.getPose(*view.get()).getTransform().center());
    }

    // if enough data try to find the transformation
    if (gpsPositions.size() < 4)
    {
        ALICEVISION_LOG_INFO("Not enough points to estimate the rototranslation to align the gps");
        return false;
    }

    Mat x1(3, centers.size());
    Mat x2(3, gpsPositions.size());

    for (Mat::Index i = 0; i < gpsPositions.size(); ++i)
    {
        x1.col(i) = centers[i];
        x2.col(i) = gpsPositions[i];
    }

    std::vector<std::size_t> inliers;
    const bool refine{true};
    return aliceVision::geometry::ACRansac_FindRTS(x1, x2, randomNumberGenerator, out_S, out_t, out_R, inliers, refine);
}

void getRotationNullifyX(Eigen::Matrix3d& out_R, const Eigen::Matrix3d& R)
{
    Eigen::Vector3d alignmentVector = R.transpose() * Eigen::Vector3d::UnitZ();
    getRotationNullifyX(out_R, alignmentVector);
}

void getRotationNullifyX(Eigen::Matrix3d& out_R, const Eigen::Vector3d& pt)
{
    /*
    0 =  [cos(x) 0 -sin(x)][X]
    Y' = [0      1 0      ][Y]
    Z' = [sin(x) 0  cos(x)][Z]

    cos(x)X - sin(x)Z = 0
    sin(x)/cos(x) = X/Z
    tan(x) = X/Z
    x = atan2(X, Z)
    */

    double angle = std::atan2(pt(0), pt(2));
    out_R = Eigen::AngleAxisd(angle, Vec3(0, -1, 0)).toRotationMatrix();
}

Vec3 computeCameraCentersMean(const sfmData::SfMData& sfmData)
{
    // Compute the mean of the point cloud
    Vec3 center = Vec3::Zero();
    size_t count = 0;
    const auto& poses = sfmData.getPoses();

    for (auto v : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(v.first))
        {
            continue;
        }

        const IndexT poseId = v.second->getPoseId();
        const auto& pose = poses.at(poseId);

        center += pose.getTransform().center();
        count++;
    }

    center /= count;
    return center;
}

void computeCentersVarCov(const sfmData::SfMData& sfmData, const Vec3& mean, Eigen::Matrix3d& varCov, size_t& count)
{
    // Compute the mean of the point cloud
    varCov = Eigen::Matrix3d::Zero();
    const auto& poses = sfmData.getPoses();
    count = 0;

    for (auto v : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(v.first))
        {
            continue;
        }

        const IndexT poseId = v.second->getPoseId();
        const auto& pose = poses.at(poseId);

        Vec3 centered = pose.getTransform().center() - mean;
        varCov += centered * centered.transpose();

        count++;
    }
}

void computeNewCoordinateSystemGroundAuto(const sfmData::SfMData& sfmData, Vec3& out_t)
{
    out_t.fill(0.0);

    // Collect landmark positions for ground detection
    // Note: the Y axis is pointing down, therefore +Y is the direction to the ground
    std::vector<Vec3> points;
    for (auto& plandmark : sfmData.getLandmarks())
    {
        // Filter out landmarks with not enough observations
        if (plandmark.second.getObservations().size() < 3)
        {
            continue;
        }

        // Filter out landmarks that lie above all the cameras that observe them
        // This filtering step assumes that cameras should not be underneath the ground level
        const Vec3 X = plandmark.second.X;
        bool foundUnder = false;
        for (const auto& pObs : plandmark.second.getObservations())
        {
            const IndexT viewId = pObs.first;
            const Vec3 camCenter = sfmData.getPose(sfmData.getView(viewId)).getTransform().center();

            if (X(1) > camCenter(1))
            {
                foundUnder = true;
                break;
            }
        }

        // Store landmark position
        if (foundUnder)
        {
            points.push_back(X);
        }
    }

    if (points.empty())
    {
        ALICEVISION_LOG_WARNING("Ground detection failed as there is no valid point");
        return;
    }

    // Filter out statistical noise
    // and take lowest point as the ground level
    const double noiseRatio = 1e-4;
    std::size_t idxGround = static_cast<std::size_t>(static_cast<double>(points.size()) * noiseRatio);
    std::nth_element(points.begin(), points.begin() + idxGround, points.end(), [](const Vec3& pt1, const Vec3& pt2) { return (pt1(1) > pt2(1)); });
    const double Yground = points[idxGround](1);

    out_t = {0.0, -Yground, 0.0};
}

void computeNewCoordinateSystemAuto(const sfmData::SfMData& sfmData, double& out_S, Mat3& out_R, Vec3& out_t)
{
    // For reference, the update is
    // landmark.second.X = S * R * landmark.second.X + t;
    // pose._center = S * R * _center + t;

    // Align with Xaxis, only modify out_R
    sfm::computeNewCoordinateSystemFromCamerasXAxis(sfmData, out_S, out_R, out_t);

    ALICEVISION_LOG_INFO("X axis rotation:" << std::endl << out_R);

    // Compute camera statistics

    Eigen::Matrix3d covCamBase;
    size_t count;
    const Vec3 mean = computeCameraCentersMean(sfmData);
    computeCentersVarCov(sfmData, mean, covCamBase, count);

    ALICEVISION_LOG_INFO("Initial point cloud center: " << mean.transpose());

    // By default, scale to get unit rms
    const double rms = sqrt(covCamBase.trace() / double(count));
    out_S = 1.0 / rms;
    if (rms < 1e-12)
    {
        out_S = 1.0;
    }

    ALICEVISION_LOG_INFO("Initial point cloud scale: " << rms);

    // By default, center all the camera such that their mean is 0
    out_t = -out_S * out_R * mean;

    // Get pairs of gps/camera positions
    const size_t minimalGpsMeasuresCount = 10;
    const double gpsVariance = 4.0;
    const double distVariance = gpsVariance * 2.0;

    const auto& poses = sfmData.getPoses();
    std::list<std::pair<Vec3, Vec3>> list_pairs;
    for (const auto v : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(v.first))
        {
            continue;
        }

        if (!v.second->getImage().hasGpsMetadata())
        {
            continue;
        }

        const IndexT poseId = v.second->getPoseId();
        const auto& pose = poses.at(poseId);

        const Vec3 camCoordinates = pose.getTransform().center();
        const Vec3 gpsCoordinates = v.second->getImage().getGpsPositionFromMetadata();

        list_pairs.push_back(std::make_pair(camCoordinates, gpsCoordinates));
    }

    if (list_pairs.empty())
    {
        ALICEVISION_LOG_INFO("No GPS information available to use it.");
        return;
    }

    if (list_pairs.size() < minimalGpsMeasuresCount)
    {
        ALICEVISION_LOG_INFO("Not enough GPS information available to use it "
                             "(GPS image pairs: "
                             << list_pairs.size() << ", minimal GPS measures count: " << minimalGpsMeasuresCount << ").");
        return;
    }

    Vec3 camCoordinatesSum = Vec3::Zero();
    Vec3 gpsCoordinatesSum = Vec3::Zero();
    for (const auto& pair : list_pairs)
    {
        camCoordinatesSum += pair.first;
        gpsCoordinatesSum += pair.second;
    }

    camCoordinatesSum /= list_pairs.size();
    gpsCoordinatesSum /= list_pairs.size();

    Eigen::Matrix3d camCov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d gpsCov = Eigen::Matrix3d::Zero();
    for (const auto& pair : list_pairs)
    {
        Vec3 centeredCam = pair.first - camCoordinatesSum;
        camCov += centeredCam * centeredCam.transpose();
        Vec3 centeredGps = pair.second - gpsCoordinatesSum;
        gpsCov += centeredGps * centeredGps.transpose();
    }

    // Make sure that gps
    const double var = gpsCov.trace();
    if (var < distVariance)
    {
        ALICEVISION_LOG_INFO("Scene is too small to use GPS information "
                             "(dataset variance: "
                             << var << ", min variance: " << distVariance << ").");
        return;
    }

    ALICEVISION_LOG_INFO("GPS point cloud scale: " << gpsCov.trace());
    ALICEVISION_LOG_INFO("Linked cameras centers scale: " << camCov.trace());
    out_S = sqrt(gpsCov.trace()) / sqrt(camCov.trace());
    out_t = -out_S * out_R * mean;

    // Try to align gps and camera point
    double gpsS;
    Vec3 gpst;
    Mat3 gpsR;
    std::mt19937 randomNumberGenerator;
    if (!computeNewCoordinateSystemFromGpsData(sfmData, randomNumberGenerator, gpsS, gpsR, gpst))
    {
        return;
    }

    // Rotate to align north with Z=1
    const Vec3 northPole = WGS84ToCartesian({90.0, 0.0, 0.0});
    const Vec3 camera_northpole = gpsR.transpose() * (northPole - gpst) * (1.0 / gpsS);
    Vec3 aligned_camera_northpole = out_R * camera_northpole;

    Mat3 nullifyX;
    aligned_camera_northpole(1) = 0;
    aligned_camera_northpole.normalize();
    getRotationNullifyX(nullifyX, aligned_camera_northpole);

    out_R = nullifyX * out_R;
    out_t = -out_S * out_R * mean;
}

}  // namespace sfm
}  // namespace aliceVision
