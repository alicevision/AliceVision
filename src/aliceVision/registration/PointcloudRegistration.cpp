// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PointcloudRegistration.hpp"
#include "ICP.hpp"
#include "SICP.hpp"

#include <aliceVision/system/Logger.hpp>
#include <pcl/Vertices.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/from_meshes.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>

#include <chrono>

namespace aliceVision {
namespace registration {

PointcloudRegistration::PointcloudRegistration()
  : rescaleMode(ERescaleMode::None)
  , sourceMeasurements(1.f)
  , targetMeasurements(1.f)
  , scaleRatio(1.f)
  , voxelSize(0.1f)
  , kSearchNormals(10)
{
}

int PointcloudRegistration::loadSourceCloud(const std::string & file)
{
  auto tic = std::chrono::steady_clock::now();

  int res = loadCloud(file, sourceCloud);

  duration.loadSourceCloud = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  return res;
}

int PointcloudRegistration::loadTargetCloud(const std::string & file)
{
  auto tic = std::chrono::steady_clock::now();

  int res = loadCloud(file, targetCloud);

  duration.loadTargetCloud = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  return res;
}

int PointcloudRegistration::tranformAndSaveCloud(
    const std::string & inputFile,
    const Eigen::Matrix4d & T,
    const std::string & outputFile)
{
  if (!boost::filesystem::exists(inputFile))
  {
    ALICEVISION_LOG_ERROR("PointcloudRegistration::saveCloud: The file does not exist '" << inputFile << "'");
    return EXIT_FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZRGB> inputCloud, outputCloud;

  // load input cloud:
  if (inputFile.substr(inputFile.find_last_of(".") + 1) == "ply")
  {
    pcl::io::loadPLYFile(inputFile, inputCloud);
  }
  else if (inputFile.substr(inputFile.find_last_of(".") + 1) == "pcd")
  {
    pcl::io::loadPCDFile(inputFile, inputCloud);
  }
  else if (inputFile.substr(inputFile.find_last_of(".") + 1) == "obj")
  {
    pcl::io::loadOBJFile(inputFile, inputCloud);
  }
  else
  {
    ALICEVISION_LOG_ERROR("PointcloudRegistration::saveCloud: Unknown extension: " << inputFile);
    return EXIT_FAILURE;
  }

  // transform input pointcloud according to T:
  pcl::transformPointCloud(inputCloud, outputCloud, T);

  // save the transformed pointcloud:
  if (outputFile.substr(outputFile.find_last_of(".") + 1) == "ply")
  {
    // pcl::io::savePLYFile(outputFile + "_src.ply", sourceCloud); // save the input point cloud for debug
    pcl::io::savePLYFile(outputFile, outputCloud);
  }
  else
  {
    ALICEVISION_LOG_ERROR("PointcloudRegistration::saveCloud: Unknown extension: " << outputFile);
    return EXIT_FAILURE;
  }

  if (outputCloud.width == inputCloud.width)
  {
    ALICEVISION_LOG_INFO("Transformation & saving succeed: '" << outputFile << "'");
    return EXIT_SUCCESS;
  }
  else
  {
    ALICEVISION_LOG_ERROR("ERROR: Transformation & saving failed: '" << outputFile << "'");
    return EXIT_FAILURE;
  }
}

void PointcloudRegistration::setSourceMeasurement(const float measurement)
{
  if (scaleRatio != 1.f)
  {
    ALICEVISION_LOG_WARNING("Cannot set the source measurement to " << measurement << ": the scale ratio is already set to " << scaleRatio << " (!= 1)." );
    return;
  }

  assert(measurement != 0);

  sourceMeasurements = measurement;
  rescaleMode = ERescaleMode::Manual;
}

void PointcloudRegistration::setTargetMeasurement(const float measurement)
{
  if (scaleRatio != 1.f)
  {
    ALICEVISION_LOG_WARNING("Cannot set the target measurement to " << measurement << ": the scale ratio is already set to " << scaleRatio << " (!= 1)." );
    return;
  }

  assert(measurement != 0);

  targetMeasurements = measurement;
  rescaleMode = ERescaleMode::Manual;
}

void PointcloudRegistration::setScaleRatio(const float ratio)
{
  assert(ratio > 0);

  scaleRatio = ratio;
  rescaleMode = ERescaleMode::Manual;
}

void PointcloudRegistration::showTimeline()
{
  ALICEVISION_LOG_TRACE("Steps duration:\n"
                        "\t| - load target cloud: " << duration.loadTargetCloud << " ms \n"
                        "\t| - load source cloud: " << duration.loadSourceCloud << " ms \n"
                        "\t| - coarse alignment: " << duration.coarseAlignment << " ms \n"
                        "\t| - rescaling: " << duration.rescaling << " ms \n"
                        "\t| - downsampling: " << duration.downsampling << " ms \n"
                        "\t| - normals computation: " << duration.computeNormals << " ms \n"
                        "\t| - refined alignment (gicp): " << duration.refinedAlignment << " ms \n"
                        "\t| -------------------------------------------"
                        );
}

Eigen::Matrix4d PointcloudRegistration::align(EAlignmentMethod mode)
{
    ALICEVISION_LOG_INFO("|- Input source: " << sourceCloud.size());
    ALICEVISION_LOG_INFO("|- Input target: " << targetCloud.size());
    pcl::PointCloud<pcl::PointXYZ> mutableSourceCloud;
    pcl::PointCloud<pcl::PointXYZ> mutableTargetCloud;
    pcl::copyPointCloud(sourceCloud, mutableSourceCloud);
    pcl::copyPointCloud(targetCloud, mutableTargetCloud);

    ALICEVISION_LOG_INFO("-- Move source to the target position");

    auto tic = std::chrono::steady_clock::now();

    // ===========================================================
    // -- Coarse registration: Move source & target to origin
    // Could be replaced by:
    // - correspondences matching
    // - eigen vectors based
    // - ...
    // ===========================================================
    Eigen::Matrix4d sourceInitialTransform = moveToOrigin(mutableSourceCloud);
    Eigen::Matrix4d targetInitialTransform = moveToOrigin(mutableTargetCloud);

    duration.coarseAlignment = std::chrono::duration <double, std::milli>(std::chrono::steady_clock::now() - tic).count();

    ALICEVISION_LOG_INFO("|- T_origin_source = \n" << sourceInitialTransform);
    ALICEVISION_LOG_INFO("|- T_origin_target = \n" << targetInitialTransform);

    Eigen::Matrix4d Ts = preparePointClouds(mutableSourceCloud, mutableTargetCloud);

    Eigen::Matrix4d transform;
    switch (mode)
    {
        case EAlignmentMethod::GICP:
        {
            transform = alignGICP(mutableSourceCloud, mutableTargetCloud);
        }
        case EAlignmentMethod::ICP:
        {
            ICP::Parameters par;
            transform = alignICP(mutableSourceCloud, mutableTargetCloud, par);
            break;
        }
        case EAlignmentMethod::ICP_sim:
        {
            ICP::Parameters par;
            par.useDirectSimilarity = true;
            transform = alignICP(mutableSourceCloud, mutableTargetCloud, par);
            break;
        }
        case EAlignmentMethod::SICP:
        {
            SICP::Parameters par;
            transform = alignSICP(mutableSourceCloud, mutableTargetCloud, par);
            break;
        }
        case EAlignmentMethod::SICP_sim:
        {
            SICP::Parameters par;
            par.useDirectSimilarity = true;
            transform = alignSICP(mutableSourceCloud, mutableTargetCloud, par);
            break;
        }
        case EAlignmentMethod::Undefined:
            throw std::runtime_error("Undefined alignment method");
    }

    finalTransformation = targetInitialTransform.inverse() * transform * Ts * sourceInitialTransform;
    return finalTransformation;
}

Eigen::Matrix4d PointcloudRegistration::preparePointClouds(
    pcl::PointCloud<pcl::PointXYZ>& mutableSourceCloud,
    pcl::PointCloud<pcl::PointXYZ>& mutableTargetCloud)
{
  ALICEVISION_LOG_INFO("|- Input source: " << mutableSourceCloud.size());
  ALICEVISION_LOG_INFO("|- Input target: " << mutableTargetCloud.size());

  // ===========================================================
  // --  Rescale source cloud
  // ===========================================================

  ALICEVISION_LOG_INFO("-- Rescaling step");

  auto tic = std::chrono::steady_clock::now();

  Eigen::Matrix4d Ts = Eigen::Matrix4d::Identity();
  if (rescaleMode == ERescaleMode::Manual)
  {
    ALICEVISION_LOG_INFO("|- Mode: Manual");
    float realScaleRatio = scaleRatio;
    if (scaleRatio == 1.f && (targetMeasurements != 1.f || sourceMeasurements != 1.f))
    {
      realScaleRatio = targetMeasurements / sourceMeasurements;
    }

    ALICEVISION_LOG_INFO("|- scaleRatio: " << realScaleRatio);
    if (realScaleRatio != 1.f )
    {
      Ts = getPureScaleTransformation(realScaleRatio);
      pcl::transformPointCloud(mutableSourceCloud, mutableSourceCloud, Ts);
    }
  }
  else if (rescaleMode == ERescaleMode::Auto)
  {
    Ts = rescaleAuto();
    pcl::transformPointCloud(mutableSourceCloud, mutableSourceCloud, Ts);

    ALICEVISION_LOG_INFO("|- Mode: Auto");
  }
  else // Not rescaled
  {
    ALICEVISION_LOG_INFO("|- Mode: *not rescaled*");
  }

  duration.rescaling = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  ALICEVISION_LOG_INFO("T_rescaling = \n" << Ts);

  // ===========================================================
  // -- VoxelGrid Subsampling
  // ===========================================================

  ALICEVISION_LOG_INFO("-- Apply voxel grid");

  tic = std::chrono::steady_clock::now();

  if (voxelSize > 0.0f)
  {
    applyVoxelGrid(mutableSourceCloud, voxelSize);
    applyVoxelGrid(mutableTargetCloud, voxelSize);

    ALICEVISION_LOG_INFO("|- Voxel size: " << voxelSize);
    ALICEVISION_LOG_INFO("|- Voxel source: " << mutableSourceCloud.size() << " pts");
    ALICEVISION_LOG_INFO("|- Voxel target: " << mutableTargetCloud.size() << " pts");
  }

  duration.downsampling = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  return Ts;
}

Eigen::Matrix4d PointcloudRegistration::alignGICP(
    pcl::PointCloud<pcl::PointXYZ>& mutableSourceCloud,
    pcl::PointCloud<pcl::PointXYZ>& mutableTargetCloud)
{

  // ===========================================================
  // -- Compute normals
  // ===========================================================

  ALICEVISION_LOG_INFO("-- Compute normals");

  pcl::PointCloud<pcl::Normal> sourceNormals, targetNormals;

  auto tic = std::chrono::steady_clock::now();

  estimateNormals(mutableSourceCloud, sourceNormals, kSearchNormals);
  estimateNormals(mutableTargetCloud, targetNormals, kSearchNormals);

  duration.computeNormals = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  ALICEVISION_LOG_INFO("|- Normals source: " << sourceNormals.size() << " pts");
  ALICEVISION_LOG_INFO("|- Normals target: " << targetNormals.size() << " pts");

  // ===========================================================
  // -- Generalized-ICP
  // ===========================================================

  ALICEVISION_LOG_INFO("-- Apply Generalized-ICP");

  Eigen::Matrix4d Ti;
  std::clock_t start;
  start = std::clock();

  duration.computeNormals = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-tic).count();

  Ti = applyGeneralizedICP(mutableSourceCloud, mutableTargetCloud, sourceNormals, targetNormals, mutableSourceCloud);

  duration.refinedAlignment = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-tic).count();

  ALICEVISION_LOG_INFO("|- G-ICP took " << (std::clock() - start) / (double)CLOCKS_PER_SEC << " sec.");
  ALICEVISION_LOG_INFO("|- T_gicp = \n" << Ti);

  // ===========================================================
  // -- Compute complete transformation
  // ===========================================================

  ALICEVISION_LOG_INFO("-- Compute global transformation");

  return Ti;
}

Eigen::Matrix4d PointcloudRegistration::alignICP(
    pcl::PointCloud<pcl::PointXYZ>& mutableSourceCloud,
    pcl::PointCloud<pcl::PointXYZ>& mutableTargetCloud,
    const ICP::Parameters& par)
{
    std::cout << "PointcloudRegistration::alignICP" << std::endl;

    Eigen::Matrix3Xd X(3, mutableSourceCloud.size()); // source, transformed
    for (int i = 0; i < mutableSourceCloud.size(); i++)
    {
        X(0, i) = mutableSourceCloud.points[i].x;
        X(1, i) = mutableSourceCloud.points[i].y;
        X(2, i) = mutableSourceCloud.points[i].z;
    }
    Eigen::Matrix3Xd Y(3, mutableTargetCloud.size()); // target
    for (int i = 0; i < mutableTargetCloud.size(); i++)
    {
        Y(0, i) = mutableTargetCloud.points[i].x;
        Y(1, i) = mutableTargetCloud.points[i].y;
        Y(2, i) = mutableTargetCloud.points[i].z;
    }

    Eigen::Affine3d transform = ICP::point_to_point(X, Y, par); // standard ICP
    finalTransformation = transform.matrix();

    for (int i = 0; i < mutableSourceCloud.size(); i++)
    {
        mutableSourceCloud.points[i].x = X(0, i);
        mutableSourceCloud.points[i].y = X(1, i);
        mutableSourceCloud.points[i].z = X(2, i);
    }

    return finalTransformation;
}

Eigen::Matrix4d PointcloudRegistration::alignSICP(
    pcl::PointCloud<pcl::PointXYZ>& mutableSourceCloud,
    pcl::PointCloud<pcl::PointXYZ>& mutableTargetCloud,
    const SICP::Parameters& par)
{
    std::cout << "PointcloudRegistration::alignSICP" << std::endl;

    Eigen::Matrix3Xd X(3, mutableSourceCloud.size()); // source, transformed
    for (int i = 0; i < mutableSourceCloud.size(); i++)
    {
        X(0, i) = mutableSourceCloud.points[i].x;
        X(1, i) = mutableSourceCloud.points[i].y;
        X(2, i) = mutableSourceCloud.points[i].z;
    }
    Eigen::Matrix3Xd Y(3, mutableTargetCloud.size()); // target
    for (int i = 0; i < mutableTargetCloud.size(); i++)
    {
        Y(0, i) = mutableTargetCloud.points[i].x;
        Y(1, i) = mutableTargetCloud.points[i].y;
        Y(2, i) = mutableTargetCloud.points[i].z;
    }

    Eigen::Affine3d transform = SICP::point_to_point(X, Y, par); // sparse ICP
    finalTransformation = transform.matrix();

    for (int i = 0; i < mutableSourceCloud.size(); i++)
    {
        mutableSourceCloud.points[i].x = X(0, i);
        mutableSourceCloud.points[i].y = X(1, i);
        mutableSourceCloud.points[i].z = X(2, i);
    }

    return finalTransformation;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//        Private member functions
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d PointcloudRegistration::moveToOrigin(pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(cloud, centroid);
  T(0, 3) = -centroid(0);
  T(1, 3) = -centroid(1);
  T(2, 3) = -centroid(2);

  pcl::transformPointCloud(cloud, cloud, T);
  return T;
}

Eigen::Matrix4d PointcloudRegistration::applyGeneralizedICP(
    const pcl::PointCloud <pcl::PointXYZ> & source_cloud,
    const pcl::PointCloud <pcl::PointXYZ> & target_cloud,
    const pcl::PointCloud<pcl::Normal> & source_normals,
    const pcl::PointCloud<pcl::Normal> & target_normals,
    pcl::PointCloud <pcl::PointXYZ> & registered_source_cloud)
{

  std::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > > source_covs(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >);
  std::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > > target_covs(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >);
  pcl::features::computeApproximateCovariances(source_cloud, source_normals, *source_covs);
  pcl::features::computeApproximateCovariances(target_cloud, target_normals, *target_covs);

  //// setup Generalized-ICP
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  //gicp.setMaxCorrespondenceDistance(1);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr target_tree(new pcl::search::KdTree<pcl::PointXYZ>);
  target_tree->setInputCloud(target_cloud.makeShared());
  gicp.setSearchMethodTarget(target_tree);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr source_tree(new pcl::search::KdTree<pcl::PointXYZ>);
  source_tree->setInputCloud(source_cloud.makeShared());
  gicp.setSearchMethodSource(source_tree);

  //gicp.setMaximumIterations(1000000);
  //gicp.setMaximumOptimizerIterations(1000000);
  gicp.setInputSource(source_cloud.makeShared());
  gicp.setInputTarget(target_cloud.makeShared());
  gicp.setSourceCovariances(source_covs);
  gicp.setTargetCovariances(target_covs);

  // gicp.setMaxCorrespondenceDistance(1000000);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  // gicp.setMaxCorrespondenceDistance(0.05);
  // Set the maximum number of iterations (criterion 1)
  // gicp.setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  gicp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  // gicp.setEuclideanFitnessEpsilon(1);

  // run registration and get transformation
  gicp.align(registered_source_cloud);

  return gicp.getFinalTransformation().cast<double>();
}

double PointcloudRegistration::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> sqr_distances(2);
  pcl::search::KdTree<pcl::PointXYZ> tree;

  tree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (!pcl_isfinite((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt(sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void PointcloudRegistration::estimateNormals(const pcl::PointCloud<pcl::PointXYZ> & cloud,
                                          pcl::PointCloud<pcl::Normal> & normals,
                                          int ksearch)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

  // Compute the normals source
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud(cloud.makeShared());
  normalEstimation.setSearchMethod(tree);
  normalEstimation.setKSearch(ksearch);
  normalEstimation.compute(normals);
}

void PointcloudRegistration::applyVoxelGrid(pcl::PointCloud<pcl::PointXYZ> & cloud, const float voxelSize)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpInputCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(cloud, *tmpInputCloud);

  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(tmpInputCloud);
  vg.setLeafSize(voxelSize, voxelSize, voxelSize);
  vg.filter(cloud);
}

Eigen::Matrix4d PointcloudRegistration::getPureScaleTransformation(const float scale)
{
  Eigen::Matrix4d T = Eigen::Matrix4d(Eigen::Matrix4d::Identity());
  T.topLeftCorner(3, 3) *= Eigen::Matrix3d::Identity()*(scale);
  return T;
}

template<typename PointT>
int PointcloudRegistration::loadCloud(const std::string & file, pcl::PointCloud<PointT> & cloud)
{
  if (!boost::filesystem::exists(file))
  {
    ALICEVISION_LOG_ERROR("PointcloudRegistration::loadCloud: The file does not exist '" << file << "'");
    return EXIT_FAILURE;
  }

  int res;
  if (file.substr(file.find_last_of(".") + 1) == "pcd")
    res = pcl::io::loadPCDFile<PointT>(file, cloud);
  else if (file.substr(file.find_last_of(".") + 1) == "ply")
    res = pcl::io::loadPLYFile<PointT>(file, cloud);
  else if (file.substr(file.find_last_of(".") + 1) == "obj")
    res = pcl::io::loadOBJFile<PointT>(file, cloud);
  else
  {
    ALICEVISION_LOG_ERROR("PointcloudRegistration::loadCloud: Unknown extension: " << file);
    return EXIT_FAILURE;
  }

  if (res == 0 && cloud.width > 0)
    return EXIT_SUCCESS;
  else
    return EXIT_FAILURE;
}

Eigen::Matrix4d PointcloudRegistration::moveSourceToTargetPosition()
{
  pcl::PointXYZ sourceMinPoint, sourceMaxPoint, sourcePosition, targetMinPoint, targetMaxPoint, targetPosition;
  pcl::getMinMax3D(sourceCloud, sourceMinPoint, sourceMaxPoint);
  pcl::getMinMax3D(targetCloud, targetMinPoint, targetMaxPoint);
  sourcePosition.x = (sourceMinPoint.x + sourceMaxPoint.x) / 2.f;
  sourcePosition.y = (sourceMinPoint.y + sourceMaxPoint.y) / 2.f;
  sourcePosition.z = (sourceMinPoint.z + sourceMaxPoint.z) / 2.f;
  targetPosition.x = (targetMinPoint.x + targetMaxPoint.x) / 2.f;
  targetPosition.y = (targetMinPoint.y + targetMaxPoint.y) / 2.f;
  targetPosition.z = (targetMinPoint.z + targetMaxPoint.z) / 2.f;

  Eigen::Matrix4d T = Eigen::Matrix4d(Eigen::Matrix4d::Identity());
  T(0, 3) = targetPosition.x - sourcePosition.x;
  T(1, 3) = targetPosition.y - sourcePosition.y;
  T(2, 3) = targetPosition.z - sourcePosition.z;

  pcl::PointCloud<pcl::PointXYZ> movedCloud;
  pcl::transformPointCloud(sourceCloud, movedCloud, T);
  sourceCloud.swap(movedCloud);
  return T;
}


} // namespace registration
} // namespace aliceVision
