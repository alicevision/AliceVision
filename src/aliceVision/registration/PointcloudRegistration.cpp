#include <aliceVision/registration/PointcloudRegistration.hpp>

#include <aliceVision/system/Logger.hpp>

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
#include <pcl/visualization/cloud_viewer.h>
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
  /* ... */
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
    const Eigen::Matrix4f & T, 
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
  scaleRatio = targetMeasurements / sourceMeasurements;
}

void PointcloudRegistration::setTargetMeasurement(const float measurement) 
{
  if (scaleRatio != 1.f)
  {
    ALICEVISION_LOG_WARNING("Cannot set the target measurement to " << measurement << ": the scale ratio is already set to " << scaleRatio << " (!= 1)." );
    return;
  }
  
  assert(measurement != 0);
  assert(sourceMeasurements != 0);
  
  targetMeasurements = measurement; 
  rescaleMode = ERescaleMode::Manual;
  scaleRatio = targetMeasurements / sourceMeasurements;
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

int PointcloudRegistration::align()
{
  pcl::PointCloud<pcl::PointXYZ> mutableSourceCloud, mutableTargetCloud;
  pcl::copyPointCloud(sourceCloud, mutableSourceCloud);
  pcl::copyPointCloud(targetCloud, mutableTargetCloud);
  
  ALICEVISION_LOG_INFO("|- Input source: " << mutableSourceCloud.size());
  ALICEVISION_LOG_INFO("|- Input target: " << mutableTargetCloud.size());
  
  if (showPipeline)
  {
    drawCentered("1. Input", sourceCloud, targetCloud);
  }
  // ===========================================================
  // -- Coarse registration: Move source & target to origin
  // Could be replaced by:
  // - correspondences matching
  // - eigen vectors based
  // - ...
  // ===========================================================

  ALICEVISION_LOG_INFO("-- Move source to the target position");
  
  auto tic = std::chrono::steady_clock::now(); 

  Eigen::Matrix4f To_source = moveToOrigin(mutableSourceCloud);
  Eigen::Matrix4f To_target = moveToOrigin(mutableTargetCloud);
  
  duration.coarseAlignment = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  ALICEVISION_LOG_INFO("|- T_origin_source = \n" << To_source);
  ALICEVISION_LOG_INFO("|- T_origin_target = \n" << To_target);

  if (showPipeline)
  {
    draw("2. Move to origin", mutableSourceCloud, mutableTargetCloud);
  }
  
  // ===========================================================
  // --  Rescale source cloud 
  // ===========================================================

  ALICEVISION_LOG_INFO("-- Rescaling step");
  
  tic = std::chrono::steady_clock::now(); 

  Eigen::Matrix4f Ts = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
  if (rescaleMode == ERescaleMode::Manual)
  {
    ALICEVISION_LOG_INFO("|- Mode: Manual");  
    
    if (scaleRatio == 1.f) // scaleRatio = 1
    {
      ALICEVISION_LOG_WARNING("Manual rescaling mode desired but 'scaleRatio' == 1");
    } 
    else // use scale ratio
    {
      ALICEVISION_LOG_INFO("|- scaleRatio: " << scaleRatio);
      
      Ts = getPureScaleTransformation(scaleRatio);
    }
    pcl::transformPointCloud(mutableSourceCloud, mutableSourceCloud, Ts);
  }
  else if (rescaleMode == ERescaleMode::Auto)
  {
    Ts = rescaleAuto();		
    pcl::transformPointCloud(mutableSourceCloud, mutableSourceCloud, Ts);
    
    ALICEVISION_LOG_INFO("|- Mode: Auto");  

    if (showPipeline)
    {
      draw("3. Rescaling", mutableSourceCloud, mutableTargetCloud);
    }
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

  applyVoxelGrid(mutableSourceCloud, voxelSize);
  applyVoxelGrid(mutableTargetCloud, voxelSize);
  
  duration.downsampling = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  ALICEVISION_LOG_INFO("|- Voxel size: " << voxelSize);
  ALICEVISION_LOG_INFO("|- Voxel source: " << mutableSourceCloud.size() << " pts");
  ALICEVISION_LOG_INFO("|- Voxel target: " << mutableTargetCloud.size() << " pts");
      
  if (showPipeline)
  {
    draw("3. Voxel grid", mutableSourceCloud, mutableTargetCloud);
  }
  
  // ===========================================================
  // -- Compute normals
  // ===========================================================
  
  ALICEVISION_LOG_INFO("-- Compute normals");
  
  pcl::PointCloud<pcl::Normal> sourceNormals, targetNormals;
  
  tic = std::chrono::steady_clock::now(); 

  estimateNormals(mutableSourceCloud, sourceNormals, kSearchNormals);
  estimateNormals(mutableTargetCloud, targetNormals, kSearchNormals);
  
  duration.computeNormals = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  ALICEVISION_LOG_INFO("|- Normals source: " << sourceNormals.size() << " pts");
  ALICEVISION_LOG_INFO("|- Normals target: " << targetNormals.size() << " pts");
  
  if (showPipeline)
  {
    draw("4. Normals", mutableSourceCloud, mutableTargetCloud, sourceNormals, targetNormals);
  }
  
  // ===========================================================
  // -- Generalized-ICP
  // ===========================================================
  
  ALICEVISION_LOG_INFO("-- Apply Generalized-ICP");
  
  Eigen::Matrix4f Ti;
  std::clock_t start;
  start = std::clock();
  
  duration.computeNormals = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  Ti = applyGeneralizedICP(mutableSourceCloud, mutableTargetCloud, sourceNormals, targetNormals, mutableSourceCloud);
  
  duration.refinedAlignment = std::chrono::duration <double, std::milli> (std::chrono::steady_clock::now()-tic).count();

  ALICEVISION_LOG_INFO("|- G-ICP took " << (std::clock() - start) / (double)CLOCKS_PER_SEC << " sec.");
  ALICEVISION_LOG_INFO("|- T_gicp = \n" << Ti);

  if (showPipeline)
  {
    draw("5. Result G-ICP", mutableSourceCloud, mutableTargetCloud);
  }

  // ===========================================================
  // -- Compute complete transformation
  // ===========================================================
  
  ALICEVISION_LOG_INFO("-- Compute global transformation");
  
  Eigen::Matrix4f To_target_inv = To_target.inverse();
  finalTransformation = To_target_inv * Ti * Ts * To_source;
  
  ALICEVISION_LOG_INFO("|- finalTransformation = \n" << finalTransformation);
    
  if (showPipeline)
    goDraw();
  
  return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//        Private member functions
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4f PointcloudRegistration::moveToOrigin(pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> moved_cloud;
  Eigen::Matrix4f T = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
    
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloud, centroid);
  T(0, 3) = -centroid(0);
  T(1, 3) = -centroid(1);
  T(2, 3) = -centroid(2);
  
  pcl::transformPointCloud(cloud, moved_cloud, T);
  cloud.swap(moved_cloud);
  return T;
}

Eigen::Matrix4f PointcloudRegistration::applyGeneralizedICP(
    const pcl::PointCloud <pcl::PointXYZ> & source_cloud,
    const pcl::PointCloud <pcl::PointXYZ> & target_cloud,
    const pcl::PointCloud<pcl::Normal> & source_normals,
    const pcl::PointCloud<pcl::Normal> & target_normals,
    pcl::PointCloud <pcl::PointXYZ> & registered_source_cloud)
{
  
  boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > > source_covs(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >);
  boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > > target_covs(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >);
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
  
  gicp.setMaxCorrespondenceDistance(1000000);
  //gicp.setMaximumIterations(1000000);
  //gicp.setMaximumOptimizerIterations(1000000);
  gicp.setInputSource(source_cloud.makeShared());
  gicp.setInputTarget(target_cloud.makeShared());
  gicp.setSourceCovariances(source_covs);
  gicp.setTargetCovariances(target_covs);
  // run registration and get transformation
  gicp.align(registered_source_cloud);
  return gicp.getFinalTransformation();
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
  pcl::PointCloud<pcl::PointXYZ> voxCloud;// Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud.makeShared());
  vg.setLeafSize(voxelSize, voxelSize, voxelSize);
  vg.filter(voxCloud);
  cloud.swap(voxCloud);
}

Eigen::Matrix4f PointcloudRegistration::getPureScaleTransformation(const float scale)
{
  Eigen::Matrix4f T = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
  T.topLeftCorner(3, 3) *= Eigen::Matrix3f::Identity()*(scale);
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

Eigen::Matrix4f PointcloudRegistration::moveSourceToTargetPosition()
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
  
  Eigen::Matrix4f T = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
  T(0, 3) = targetPosition.x - sourcePosition.x;
  T(1, 3) = targetPosition.y - sourcePosition.y;
  T(2, 3) = targetPosition.z - sourcePosition.z;
  
  pcl::PointCloud<pcl::PointXYZ> movedCloud;
  pcl::transformPointCloud(sourceCloud, movedCloud, T);
  sourceCloud.swap(movedCloud);
  return T;
}

void PointcloudRegistration::draw(
    const std::string & windowName,
    const pcl::PointCloud<pcl::PointXYZ> & source,
    const pcl::PointCloud<pcl::PointXYZ> & target,
    const float voxelSize)
{
  if (voxelSize != 0)
  {
    pcl::PointCloud<pcl::PointXYZ> mutableSourceCloud, mutableTargetCloud;
    pcl::copyPointCloud(source, mutableSourceCloud);
    pcl::copyPointCloud(target, mutableTargetCloud);
    
    applyVoxelGrid(mutableSourceCloud, voxelSize);
    applyVoxelGrid(mutableTargetCloud, voxelSize);
    draw(windowName, mutableSourceCloud, mutableTargetCloud);
  }
  else 
    draw(windowName, source, target);	
}

void PointcloudRegistration::draw(
    const std::string & windowName,
    const pcl::PointCloud<pcl::PointXYZ> & source,
    const pcl::PointCloud<pcl::PointXYZ> & target)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_cloud(source.makeShared(), 80, 150, 80);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_cloud(target.makeShared(), 150, 80, 80);
  
  pcl::visualization::PCLVisualizer viewer(windowName);
  viewer.addPointCloud<pcl::PointXYZ>(source.makeShared(), handler_source_cloud, "Final_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Final_cloud");
  viewer.addPointCloud<pcl::PointXYZ>(target.makeShared(), handler_target_cloud, "target_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
  viewer.addCoordinateSystem(1.0f);
}

void PointcloudRegistration::drawCentered(
    const std::string & windowName,
    const pcl::PointCloud<pcl::PointXYZ> & source,
    const pcl::PointCloud<pcl::PointXYZ> & target,
    const float voxelSize)
{
  pcl::PointCloud<pcl::PointXYZ> mutableSourceCloud, mutableTargetCloud;
  pcl::copyPointCloud(target, mutableTargetCloud);
  
  Eigen::Matrix4f To_target = moveToOrigin(mutableTargetCloud);
  pcl::transformPointCloud(source, mutableSourceCloud, To_target);
  
  draw(windowName, mutableSourceCloud, mutableTargetCloud, voxelSize);
}

void PointcloudRegistration::drawCentered(
    const std::string & windowName,
    const pcl::PointCloud<pcl::PointXYZ> & source,
    const pcl::PointCloud<pcl::PointXYZ> & target)
{
  pcl::PointCloud<pcl::PointXYZ> mutableSourceCloud, mutableTargetCloud;
  pcl::copyPointCloud(target, mutableTargetCloud);
  
  Eigen::Matrix4f To_target = moveToOrigin(mutableTargetCloud);
  pcl::transformPointCloud(source, mutableSourceCloud, To_target);
  
  draw(windowName, mutableSourceCloud, mutableTargetCloud);
}

void PointcloudRegistration::draw(const std::string & windowName, 
                               const pcl::PointCloud<pcl::PointXYZ> & source,
                               const pcl::PointCloud<pcl::PointXYZ> & target,
                               const pcl::PointCloud<pcl::Normal> & sourceNormals, 
                               const pcl::PointCloud<pcl::Normal> & targetNormals)
{
  
  double source_resolution = computeCloudResolution(source.makeShared());
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_source_cloud(source.makeShared(), 80, 150, 80);
  pcl::visualization::PCLVisualizer viewer(windowName + " source");
  viewer.addPointCloud<pcl::PointXYZ>(source.makeShared(), handler_source_cloud, "Final_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Final_cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(source.makeShared(), sourceNormals.makeShared(), 1, source_resolution*3.0, "normals");
  
  double target_resolution = computeCloudResolution(target.makeShared());
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_target_cloud(target.makeShared(), 150, 80, 80);
  pcl::visualization::PCLVisualizer viewert(windowName + " target");
  viewert.addPointCloud<pcl::PointXYZ>(target.makeShared(), handler_target_cloud, "Final_cloud");
  viewert.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Final_cloud");
  viewert.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(target.makeShared(), targetNormals.makeShared(), 1, target_resolution*3.0, "normals");
}

void PointcloudRegistration::goDraw() const 
{
  pcl::visualization::PCLVisualizer viewer("Waiting...");
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

} // namespace registration
} // namespace aliceVision
