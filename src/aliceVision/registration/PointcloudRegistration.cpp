#include <aliceVision/registration/PointcloudRegistration.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/from_meshes.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

namespace aliceVision {
namespace registration {

PointcloudRegistration::PointcloudRegistration()
  : rescaleMode(ERescaleMode::None)
  , sourceMeasurements(1.f)
  , targetMeasurements(1.f)
  , scaleRatio(1.f)
  , voxelSize(0.1f)
  , kSearchNormals(10)
  , verbose(false)
{
  /* ... */
}

int PointcloudRegistration::loadSourceCloud(const std::string & file)
{
  return loadCloud(file, sourceCloud);
}

int PointcloudRegistration::loadTargetCloud(const std::string & file)
{
  return loadCloud(file, targetCloud);
}

int PointcloudRegistration::saveCloud(const std::string & file, const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  int res;
  if (file.substr(file.find_last_of(".") + 1) == "ply")
    res = pcl::io::savePLYFile<pcl::PointXYZ>(file, cloud);
  else
  {
    std::cout << "[ERROR] saveCloud: Unknown extension: " << file << std::endl;
    return EXIT_FAILURE;
  }
  
  if (res == 0)
    return EXIT_SUCCESS;
  else
    return EXIT_FAILURE;
}

int PointcloudRegistration::setSourceMeasurements(const float val) 
{
  if (val <= 0)
  {
    std::cout << "[ERROR] setSourceMeasurements: Cannot be < zero." << std::endl;
    return EXIT_FAILURE;
  }
  sourceMeasurements = val; 
  rescaleMode = ERescaleMode::Manual;
  scaleRatio = targetMeasurements / sourceMeasurements;
  return EXIT_SUCCESS;
}

int PointcloudRegistration::setTargetMeasurements(const float val) 
{
  if (val <= 0)
  {
    std::cout << "[ERROR] setTargetMeasurements: Cannot be < zero." << std::endl;
    return EXIT_FAILURE;
  }
  targetMeasurements = val; 
  rescaleMode = ERescaleMode::Manual;
  scaleRatio = targetMeasurements / sourceMeasurements;
  return EXIT_SUCCESS;
}

int PointcloudRegistration::setScaleRatio(const float val) 
{
  if (val <= 0)
  {
    std::cout << "[ERROR] setScaleRatio: Cannot be < zero." << std::endl;
    return EXIT_FAILURE;
  }
  scaleRatio = val; 
  rescaleMode = ERescaleMode::Manual;
  return EXIT_SUCCESS;
}

int PointcloudRegistration::align(pcl::PointCloud<pcl::PointXYZ> & registeredSourceCloud)
{
  int res = align();
  pcl::transformPointCloud(sourceCloud, registeredSourceCloud, finalTransformation);
  return res;
}

int PointcloudRegistration::align()
{
  pcl::PointCloud<pcl::PointXYZ> mutableSourceCloud, mutableTargetCloud;
  pcl::copyPointCloud(sourceCloud, mutableSourceCloud);
  pcl::copyPointCloud(targetCloud, mutableTargetCloud);
  
  if (verbose)
  {
    std::cout << "|- Input source: " << mutableSourceCloud.size() << std::endl;
    std::cout << "|- Input target: " << mutableTargetCloud.size() << std::endl;
  }
  if (showPipeline)
  {
    draw("1. Input", sourceCloud, targetCloud);
  }
  
  // ===========================================================
  // --  Move source & target to origin
  // (Could be replaced by correspondences matching)
  // ===========================================================
  
  if (verbose)
    std::cout << "\n-- Move source to the target position" << std::endl;
  
  Eigen::Matrix4f To_source = moveToOrigin(mutableSourceCloud);
  Eigen::Matrix4f To_target = moveToOrigin(mutableTargetCloud);
  
  if (verbose)
  {
    std::cout << "|- To_source = \n" << To_source << std::endl;
    std::cout << "|- To_target = \n" << To_target << std::endl;
  }
  if (showPipeline)
  {
    draw("2. Move to origin", mutableSourceCloud, mutableTargetCloud);
  }
  
  // ===========================================================
  // --  Rescale source cloud 
  // ===========================================================
  
  if (verbose)
    std::cout << "\n-- Rescale step" << std::endl;
  
  Eigen::Matrix4f Ts = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
  if (rescaleMode == ERescaleMode::Manual)
  {
    if (verbose)
      std::cout << "|- Mode: Manual" << std::endl;  
    
    if (scaleRatio == 1.f) // scaleRatio = 1
    {
      std::cout << "[WARNING] Manual rescale mode desired but: scaleRatio = 1" << std::endl;
    } 
    else // use scale ratio
    {
      if (verbose)
        std::cout << "|- scaleRatio: " << scaleRatio << std::endl;
      
      Ts = getPureScaleTransformation(scaleRatio);
      
      if (verbose)
        std::cout << "|- Ts = \n" << Ts << std::endl;
    }
    pcl::transformPointCloud(mutableSourceCloud, mutableSourceCloud, Ts);
  }
  else if (rescaleMode == ERescaleMode::Auto)
  {
    Ts = rescaleAuto();		
    pcl::transformPointCloud(mutableSourceCloud, mutableSourceCloud, Ts);
    
    std::cout << "|- Mode: Auto" << std::endl;  
    
    if (verbose)
    {
      std::cout << "|- Mode: Auto" << std::endl;
      std::cout << "|- Ts = \n" << Ts << std::endl;
    }
    if (showPipeline)
    {
      draw("3. Rescaling", mutableSourceCloud, mutableTargetCloud);
    }
  }
  else // Not rescaled
  {
    if (verbose)
      std::cout << "|- Mode: *not rescaled*" << std::endl;  
  }
  
  if (verbose)
    std::cout << "Ts = \n" << Ts << std::endl;
  
  // ===========================================================
  // -- VoxelGrid Subsampling
  // ===========================================================
  
  if (verbose)
    std::cout << "\n-- Apply voxel grid" << std::endl;
  
  applyVoxelGrid(mutableSourceCloud, voxelSize);
  applyVoxelGrid(mutableTargetCloud, voxelSize);
  
  if (verbose)
  {	
    std::cout << "|- Voxel size: " << voxelSize << std::endl;
    std::cout << "|- Voxel source: " << mutableSourceCloud.size() << " pts" << std::endl;
    std::cout << "|- Voxel target: " << mutableTargetCloud.size() << " pts" << std::endl;
  }
  
  if (showPipeline)
  {
    draw("4. Voxel grid", mutableSourceCloud, mutableTargetCloud);
  }
  
  // ===========================================================
  // -- Compute normals
  // ===========================================================
  
  if (verbose)
    std::cout << "\n-- Compute normals" << std::endl;
  
  pcl::PointCloud<pcl::Normal> sourceNormals, targetNormals;
  
  estimateNormals(mutableSourceCloud, sourceNormals, kSearchNormals);
  estimateNormals(mutableTargetCloud, targetNormals, kSearchNormals);
  
  if (verbose)
  {
    std::cout << "|- Normals source: " << sourceNormals.size() << " pts" << std::endl;
    std::cout << "|- Normals target: " << targetNormals.size() << " pts" << std::endl;
  }
  
  if (showPipeline)
  {
    draw("5. Normals", mutableSourceCloud, mutableTargetCloud);
  }
  
  // ===========================================================
  // -- Generalized-ICP
  // ===========================================================
  
  if (verbose)
    std::cout << "\n-- Apply Generalized-ICP\n" << std::endl;
  
  Eigen::Matrix4f Ti;
  pcl::PointCloud<pcl::PointXYZ> regGICP_source;
  std::clock_t start;
  start = std::clock();
  
  Ti = applyGeneralizedICP(mutableSourceCloud, mutableTargetCloud, sourceNormals, targetNormals, mutableSourceCloud);
  
  if (verbose)
  {
    std::cout << "|- G-ICP took " << (std::clock() - start) / (double)CLOCKS_PER_SEC << " sec." << std::endl;
    std::cout << "|- Ti = \n" << Ti << std::endl;
  }
  if (showPipeline)
  {
    draw("6. Result G-ICP", mutableSourceCloud, mutableTargetCloud);
  }
  
  // ===========================================================
  // -- Go back to initial target position
  // ===========================================================
  
  Eigen::Matrix4f To_target_inv = To_target.inverse();
  pcl::PointCloud<pcl::PointXYZ> finalSource, finalTarget;
  pcl::transformPointCloud(mutableSourceCloud, finalSource, To_target_inv);
  pcl::transformPointCloud(mutableTargetCloud, finalTarget, To_target_inv);
  
  // ===========================================================
  // -- Compute complete transformation
  // ===========================================================
  
  if (verbose)
    std::cout << "\n-- Compute global transformation\n" << std::endl;
  
  finalTransformation = To_target_inv * Ti * Ts * To_source;
  
  if (verbose)
    std::cout << "|- finalTransformation = \n" << finalTransformation << std::endl;
  
  pcl::PointCloud<pcl::PointXYZ> regSource;
  pcl::transformPointCloud(sourceCloud, regSource, finalTransformation);
  
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
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(cloud, minPoint, maxPoint);
  
  Eigen::Matrix4f T = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
  T(0, 3) = -(maxPoint.x + minPoint.x) / 2.f;
  T(1, 3) = -(maxPoint.y + minPoint.y) / 2.f;
  T(2, 3) = -(maxPoint.z + minPoint.z) / 2.f;
  
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

int PointcloudRegistration::loadCloud(const std::string & file, pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  int res;
  if (file.substr(file.find_last_of(".") + 1) == "pcd")
    res = pcl::io::loadPCDFile<pcl::PointXYZ>(file, cloud);
  else if (file.substr(file.find_last_of(".") + 1) == "ply")
    res = pcl::io::loadPLYFile<pcl::PointXYZ>(file, cloud);
  else
  {
    std::cout << "[ERROR] loadCloud: Unknown extension: " << file << std::endl;
    return EXIT_FAILURE;
  }
  
  if (res == 0)
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
