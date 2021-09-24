// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LocalizationResult.hpp"
#include <aliceVision/sfmDataIO/jsonIO.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <limits>
#include <memory>

namespace aliceVision {
namespace localization {

namespace bpt = boost::property_tree;

LocalizationResult::LocalizationResult() : _isValid(false) {}
LocalizationResult::LocalizationResult(
        const sfm::ImageLocalizerMatchData& matchData,
        const std::vector<IndMatch3D2D>& indMatch3D2D,
        const geometry::Pose3& pose,
        const camera::PinholeRadialK3& intrinsics,
        const std::vector<voctree::DocMatch>& matchedImages,
        bool isValid) :
        _matchData(matchData),
        _indMatch3D2D(indMatch3D2D),
        _pose(pose),
        _intrinsics(intrinsics),
        _matchedImages(matchedImages),
        _isValid(isValid)
{
  // verify data consistency
  assert(_matchData.pt2D.cols() == _matchData.pt3D.cols());
  assert(_matchData.pt2D.cols() == _indMatch3D2D.size());
}
        
LocalizationResult::~LocalizationResult() {}

const Mat LocalizationResult::retrieveUndistortedPt2D() const
{
  const auto& intrinsics =  getIntrinsics();
  const auto& distorted = getPt2D();
  if(!intrinsics.hasDistortion() || !intrinsics.isValid())
  {
    return getPt2D();
  }
  const std::size_t numPts = distorted.cols();
  Mat pt2Dundistorted = Mat2X(2, numPts);
  for(std::size_t iPoint = 0; iPoint < numPts; ++iPoint)
  {
    pt2Dundistorted.col(iPoint) = intrinsics.get_ud_pixel(distorted.col(iPoint));
  }
  return pt2Dundistorted;
}

Mat2X LocalizationResult::computeAllResiduals() const 
{
  const Mat2X& orig2d = getPt2D();
  const Mat3X& orig3d = getPt3D();
  assert(orig2d.cols()==orig3d.cols());
  
  const auto& intrinsics = getIntrinsics();
  return intrinsics.residuals(getPose(), orig3d, orig2d);
}

Mat2X LocalizationResult::computeInliersResiduals() const 
{
  // get the inliers.
  const auto& currInliers = getInliers();
  const std::size_t numInliers = currInliers.size();

  const Mat2X& orig2d = getPt2D();
  const Mat3X& orig3d = getPt3D();
  Mat2X inliers2d = Mat2X(2, numInliers);
  Mat3X inliers3d = Mat3X(3, numInliers);

  for(std::size_t i = 0; i < numInliers; ++i)
  {
    const std::size_t idx = currInliers[i];
    inliers2d.col(i) = orig2d.col(idx);
    inliers3d.col(i) = orig3d.col(idx);
  }
  
  const auto &intrinsics = getIntrinsics();
  return intrinsics.residuals(getPose(), inliers3d, inliers2d);
}

double LocalizationResult::computeInliersRMSE() const 
{
  const auto& residuals = computeInliersResiduals();
  // squared residual for each point
  const auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
  //RMSE
  return std::sqrt(sqrErrors.mean());
}

double LocalizationResult::computeAllRMSE() const 
{
  const auto& residuals = computeAllResiduals();
  // squared residual for each point
  const auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
  //RMSE
  return std::sqrt(sqrErrors.mean());
}

Vec LocalizationResult::computeReprojectionErrorPerInlier() const 
{
  const auto& residuals = computeInliersResiduals();
  // squared residual for each point
  const Vec sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
  //RMSE
  return sqrErrors.array().sqrt();
}

Vec LocalizationResult::computeReprojectionErrorPerPoint() const
{
  const auto& residuals = computeAllResiduals();
  // squared residual for each point
  const Vec sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
  //RMSE
  return sqrErrors.array().sqrt();
}

std::size_t LocalizationResult::selectBestInliers(double maxReprojectionError)
{
   const auto& residuals = computeReprojectionErrorPerPoint();
   auto& inliers = _matchData.vec_inliers;
   ALICEVISION_LOG_DEBUG("Inliers before: " << inliers.size());
   inliers.clear();
   // at worst they could all be inliers
   inliers.reserve(getPt2D().size());
   
   for(std::size_t i = 0; i < residuals.size(); ++i)
   {
     if(residuals[i] < maxReprojectionError)
     {
       inliers.push_back(i);
     }
   }
   ALICEVISION_LOG_DEBUG(" After: " << inliers.size());
   _matchData.error_max = maxReprojectionError;
   return inliers.size();
}

std::size_t LocalizationResult::selectBestInliers()
{  
   const auto threshold = _matchData.error_max;
   return selectBestInliers(threshold);
}


void LocalizationResult::load(std::vector<LocalizationResult>& localizationResults, const std::string& filename)
{
  using namespace aliceVision::sfm;

  // empty output vector
  localizationResults.clear();

  Version version;

  // main tree
  bpt::ptree fileTree;

  // read the json file and initialize the tree
  bpt::read_json(filename, fileTree);

  // version
  {
    Vec3i v;
    sfmDataIO::loadMatrix("version", v, fileTree);
    version = v;
  }

  if(fileTree.count("localizationResults"))
  {
    for(bpt::ptree::value_type& lrNode : fileTree.get_child("localizationResults"))
    {
      // localization result tree
      bpt::ptree lrTree = lrNode.second;

      LocalizationResult lr;

      lr._isValid = lrTree.get<bool>("isValid");
      sfmDataIO::loadPose3("pose", lr._pose, lrTree);

      // indMatch3D2D
      if(lrTree.count("indMatch3D2D"))
      {
        for(bpt::ptree::value_type& itNode : lrTree.get_child("indMatch3D2D"))
        {
          bpt::ptree& itTree = itNode.second;

          IndMatch3D2D indMatch;

          indMatch.landmarkId = itTree.get<IndexT>("landmarkId");
          indMatch.featId = itTree.get<IndexT>("featureId");
          indMatch.descType = feature::EImageDescriberType_stringToEnum(itTree.get<std::string>("descType"));

          lr._indMatch3D2D.emplace_back(indMatch);
        }
      }

      // intrinsic
      {
        IndexT intrinsicId;
        std::shared_ptr<camera::IntrinsicBase> intrinsicPtr;
        sfmDataIO::loadIntrinsic(version, intrinsicId, intrinsicPtr, lrTree.get_child("intrinsic"));
        lr._intrinsics = *(dynamic_cast<camera::PinholeRadialK3*>(intrinsicPtr.get()));
      }

      // inliers
      if(lrTree.count("inliers"))
      {
        for(bpt::ptree::value_type& itNode : lrTree.get_child("inliers"))
          lr._matchData.vec_inliers.emplace_back(itNode.second.get_value<std::size_t>());
      }

      const std::size_t nbPts = lrTree.get<std::size_t>("nbPts");

      lr._matchData.pt3D = Mat(3, nbPts);
      lr._matchData.pt2D = Mat(2, nbPts);

      sfmDataIO::loadMatrix("pt3D", lr._matchData.pt3D, lrTree);
      sfmDataIO::loadMatrix("pt2D", lr._matchData.pt2D, lrTree);
      sfmDataIO::loadMatrix("projectionMatrix", lr._matchData.projection_matrix, lrTree);

      lr._matchData.error_max = std::stod(lrTree.get<std::string>("errorMax"));
      lr._matchData.max_iteration = lrTree.get<std::size_t>("maxIteration");

      // matchedImages;
      if(lrTree.count("matchedImages"))
      {
        for(bpt::ptree::value_type& itNode : lrTree.get_child("matchedImages"))
        {
          bpt::ptree& itTree = itNode.second;

          voctree::DocMatch docMatch;

          docMatch.id = itTree.get<voctree::DocId>("id");
          docMatch.score = itTree.get<float>("score");

          lr._matchedImages.emplace_back(docMatch);
        }
      }
      localizationResults.emplace_back(lr);
    }
  }
}

void LocalizationResult::save(const std::vector<LocalizationResult>& localizationResults, const std::string& filename)
{
  using namespace aliceVision::sfm;

  const Vec3i version = {1, 0, 0};

  // main tree
  bpt::ptree fileTree;

  // file version
  sfmDataIO::saveMatrix("version", version, fileTree);

  // localizationResults tree
  bpt::ptree localizationResultsTree;

  for(const LocalizationResult& lr : localizationResults)
  {
    bpt::ptree lrTree;

    lrTree.put("isValid", lr._isValid);
    sfmDataIO::savePose3("pose", lr._pose, lrTree);

    // indMatch3D2D
    {
      bpt::ptree indMatch3D2DTree;

      for(const IndMatch3D2D& indMatch3D2D : lr._indMatch3D2D)
      {
        bpt::ptree itTree;
        itTree.put("landmarkId", indMatch3D2D.landmarkId);
        itTree.put("featureId", indMatch3D2D.featId);
        itTree.put("descType", feature::EImageDescriberType_enumToString(indMatch3D2D.descType));
        indMatch3D2DTree.push_back(std::make_pair("", itTree));
      }
      lrTree.add_child("indMatch3D2D", indMatch3D2DTree);
    }

    //intrinsic
    {
      std::shared_ptr<camera::PinholeRadialK3> intrinsicPtr(new camera::PinholeRadialK3());
      *intrinsicPtr = lr._intrinsics;
      sfmDataIO::saveIntrinsic("intrinsic", UndefinedIndexT, std::dynamic_pointer_cast<camera::IntrinsicBase>(intrinsicPtr), lrTree);
    }

    // inliers
    {
      bpt::ptree inliersTree;
      for(std::size_t index : lr._matchData.vec_inliers)
      {
        bpt::ptree inlierTree;
        inlierTree.put("",index);
        inliersTree.push_back(std::make_pair("", inlierTree));
      }
      lrTree.add_child("inliers", inliersTree);
    }

    // needed for loading
    lrTree.put("nbPts", lr._matchData.pt3D.cols());

    sfmDataIO::saveMatrix("pt3D", lr._matchData.pt3D, lrTree);
    sfmDataIO::saveMatrix("pt2D", lr._matchData.pt2D, lrTree);
    sfmDataIO::saveMatrix("projectionMatrix", lr._matchData.projection_matrix, lrTree);

    lrTree.put("errorMax", lr._matchData.error_max);
    lrTree.put("maxIteration", lr._matchData.max_iteration);

    // matchedImages
    {
      bpt::ptree matchedImagesTree;

      for(const voctree::DocMatch& docMatch : lr._matchedImages)
      {
        bpt::ptree itTree;
        itTree.put("id", docMatch.id);
        itTree.put("score", docMatch.score);
        matchedImagesTree.push_back(std::make_pair("", itTree));
      }
      lrTree.add_child("matchedImages", matchedImagesTree);
    }
    localizationResultsTree.push_back(std::make_pair("", lrTree));
  }
  fileTree.add_child("localizationResults", localizationResultsTree);

  // write the json file with the tree
  bpt::write_json(filename, fileTree);
}

void updateRigPoses(std::vector<LocalizationResult>& vec_localizationResults,
                    const geometry::Pose3 &rigPose,
                    const std::vector<geometry::Pose3 > &vec_subPoses)
{
  const std::size_t numCams = vec_localizationResults.size();
  assert(numCams==vec_subPoses.size()+1);
  
  // update localization result poses
  for(std::size_t camID = 0; camID < numCams; ++camID)
  {
    geometry::Pose3 pose;
    if(camID == 0)
    {
      pose = rigPose;
    }
    else
    {
      // main camera: q1 ~ [R1 t1] Q = [I 0] A   where A = [R1 t1] Q  
      // another camera: q2 ~ [R2 t2] Q = [R2 t2]*inv([R1 t1]) A 
      // and subPose12 = [R12 t12] = [R2 t2]*inv([R1 t1])
      // With rigResection() we compute [R1 t1] (aka rigPose), hence:
      // subPose12 = [R12 t12] = [R2 t2]*inv([R1 t1]) and we need [R2 t2], ie the absolute pose
      // => [R1 t1] * subPose12 = [R2 t2]
      // => rigPose * subPose12 = [R2 t2]
      pose = vec_subPoses[camID-1] * rigPose;
    }
    
    vec_localizationResults[camID].setPose(pose);
  }
}

} // localization
} // aliceVision
