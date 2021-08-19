// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>

#include <boost/property_tree/ptree.hpp>

#include <string>

namespace aliceVision {
namespace sfmDataIO {

namespace bpt = boost::property_tree;

/**
 * @brief Save an Eigen Matrix (or Vector) in a boost property tree.
 * @param[in] name The node name ( "" = no name )
 * @param[in] matrix The input matrix
 * @param[out] parentTree The parent tree
 */
template<typename Derived>
inline void saveMatrix(const std::string& name, const Eigen::MatrixBase<Derived>& matrix, bpt::ptree& parentTree)
{
  bpt::ptree matrixTree;

  const int size = matrix.size();
  for(int i = 0; i < size; ++i)
  {
    bpt::ptree cellTree;
    cellTree.put("", matrix(i));
    matrixTree.push_back(std::make_pair("", cellTree));
  }

  parentTree.add_child(name, matrixTree);
}

/**
 * @brief Load an Eigen Matrix (or Vector) from a boost property tree.
 * @param[in] name The Matrix name ( "" = no name )
 * @param[out] matrix The output Matrix
 * @param[in,out] matrixTree The input tree
 */
template<typename Derived>
inline void loadMatrix(const std::string& name, Eigen::MatrixBase<Derived>& matrix, bpt::ptree& matrixTree)
{
  const int size = matrix.size();
  int i = 0;

  for(bpt::ptree::value_type &cellNode : matrixTree.get_child(name))
  {
    if(i > size)
      throw std::out_of_range("Invalid matrix / vector type for : " + name);

    matrix(i) = cellNode.second.get_value<typename Derived::Scalar>();
    ++i;
  }
}

/**
 * @brief Save a Pose3 in a boost property tree.
 * @param[in] name The node name ( "" = no name )
 * @param[in] pose The input pose3
 * @param[out] parentTree The parent tree
 */
inline void savePose3(const std::string& name, const geometry::Pose3& pose, bpt::ptree& parentTree)
{
  bpt::ptree pose3Tree;

  saveMatrix("rotation", pose.rotation(), pose3Tree);
  saveMatrix("center", pose.center(), pose3Tree);

  parentTree.add_child(name, pose3Tree);
}

/**
 * @brief Load a Pose3 from a boost property tree.
 * @param[in] name The Pose3 name ( "" = no name )
 * @param[out] pose The output Pose3
 * @param[in,out] pose3Tree The input tree
 */
inline void loadPose3(const std::string& name, geometry::Pose3& pose, bpt::ptree& pose3Tree)
{
  Mat3 rotation;
  Vec3 center;

  loadMatrix(name + ".rotation", rotation, pose3Tree);
  loadMatrix(name + ".center",   center, pose3Tree);

  pose = geometry::Pose3(rotation, center);
}

/**
 * @brief Save a Pose3 in a boost property tree.
 * @param[in] name The node name ( "" = no name )
 * @param[in] pose The input pose3
 * @param[out] parentTree The parent tree
 */
inline void saveCameraPose(const std::string& name, const sfmData::CameraPose& cameraPose, bpt::ptree& parentTree)
{
  bpt::ptree cameraPoseTree;

  savePose3("transform", cameraPose.getTransform(), cameraPoseTree);
  cameraPoseTree.put("locked", static_cast<int>(cameraPose.isLocked())); // convert bool to integer to avoid using "true/false" in exported file instead of "1/0".

  parentTree.add_child(name, cameraPoseTree);
}

/**
 * @brief Load a Pose3 from a boost property tree.
 * @param[in] name The Pose3 name ( "" = no name )
 * @param[out] pose The output Pose3
 * @param[in,out] pose3Tree The input tree
 */
inline void loadCameraPose(const std::string& name, sfmData::CameraPose& cameraPose, bpt::ptree& cameraPoseTree)
{
  geometry::Pose3 pose;

  loadPose3(name + ".transform", pose, cameraPoseTree);
  cameraPose.setTransform(pose);

  if(cameraPoseTree.get<bool>("locked", false))
    cameraPose.lock();
  else
    cameraPose.unlock();
}

/**
 * @brief Save a View in a boost property tree.
 * @param[in] name The node name ( "" = no name )
 * @param[in] view The input View
 * @param[out] parentTree The parent tree
 */
void saveView(const std::string& name, const sfmData::View& view, bpt::ptree& parentTree);

/**
 * @brief Load a View from a boost property tree.
 * @param[out] view The output View
 * @param[in,out] viewTree The input tree
 */
void loadView(sfmData::View& view, bpt::ptree& viewTree);

/**
 * @brief Save an Intrinsic in a boost property tree.
 * @param[in] name The node name ( "" = no name )
 * @param[in] intrinsicId The intrinsic Id
 * @param[in] intrinsic The intrinsic
 * @param[out] parentTree The parent tree
 */
void saveIntrinsic(const std::string& name, IndexT intrinsicId, const std::shared_ptr<camera::IntrinsicBase>& intrinsic, bpt::ptree& parentTree);

/**
 * @brief Load an Intrinsic from a boost property tree.
 * @param[in] version File versioning for dealing with compatibility
 * @param[out] intrinsicId The output Intrinsic Id
 * @param[out] intrinsic The output Intrinsic
 * @param intrinsicTree The input tree
 */
void loadIntrinsic(const Version& version, IndexT& intrinsicId, std::shared_ptr<camera::IntrinsicBase>& intrinsic,
                   bpt::ptree& intrinsicTree);

/**
 * @brief Save a Rig in a boost property tree.
 * @param[in] name The node name ( "" = no name )
 * @param[in] rigId The rig Id
 * @param[in] rig The rig
 * @param[out] parentTree The parent tree
 */
void saveRig(const std::string& name, IndexT rigId, const sfmData::Rig& rig, bpt::ptree& parentTree);

/**
 * @brief Load a Rig from a boost property tree.
 * @param[out] rigId The output Rig Id
 * @param[out] rig The output Rig
 * @param[in,out] rigTree The input tree
 */
void loadRig(IndexT& rigId, sfmData::Rig& rig, bpt::ptree& rigTree);

/**
 * @brief Save a Landmark in a boost property tree.
 * @param[in] name The node name ( "" = no name )
 * @param[in] landmarkId The landmark Id
 * @param[in] landmark The landmark
 * @param[out] parentTree The parent tree
 *
 * Optional:
 * @param[in] saveObservations Save landmark observations (default: true)
 * @param[in] saveFeatures Save landmark observations features (default: true)
 */
void saveLandmark(const std::string& name, IndexT landmarkId, const sfmData::Landmark& landmark, bpt::ptree& parentTree, bool saveObservations = true, bool saveFeatures = true);

/**
 * @brief Load a Landmark from a boost property tree.
 * @param[out] landmarkId The output Landmark Id
 * @param[out] landmark The output Landmmark
 * @param[in,out] landmarkTree The input tree
 *
 * Optional:
 * @param[in] loadObservations Load landmark observations (default: true)
 * @param[in] loadFeatures Load landmark observations features (default: true)
 */
void loadLandmark(IndexT& landmarkId, sfmData::Landmark& landmark, bpt::ptree& landmarkTree, bool loadObservations = true, bool loadFeatures = true);

/**
 * @brief Save an SfMData in a JSON file with a boost property tree.
 * @param[in] sfmData The input SfMData
 * @param[in] filename The filename
 * @param[in] partFlag The ESfMData save flag
 * @return true if completed
 */
bool saveJSON(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

/**
 * @brief Load a JSON SfMData file.
 * @param[out] sfmData The output SfMData
 * @param[in] filename The filename
 * @param[in] partFlag The ESfMData load flag
 * @param[in] incompleteViews If true, try to load incomplete views
 * @param[in] viewIdMethod ViewId generation method to use if incompleteViews is true
 * @param[in] viewIdRegex Optional regex used when viewIdMethod is FILENAME
 * @return true if completed
 */
bool loadJSON(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag, bool incompleteViews = false,
              EViewIdMethod viewIdMethod = EViewIdMethod::METADATA, const std::string& viewIdRegex = "");

} // namespace sfmDataIO
} // namespace aliceVision
