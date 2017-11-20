// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfmDataIO_gt.hpp>
#include <aliceVision/sfm/viewIO.hpp>
#include <aliceVision/sfm/utils/uid.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <fstream>
#include <vector>

namespace aliceVision {
namespace sfm {

bool read_aliceVision_Camera(const std::string & camName, camera::Pinhole & cam, geometry::Pose3 & pose)
{
  std::vector<double> val;
  if (stlplus::extension_part(camName) == "bin")
  {
    std::ifstream in(camName.c_str(), std::ios::in|std::ios::binary);
    if (!in.is_open())
    {
      ALICEVISION_LOG_ERROR("Failed to open file '" << camName << "' for reading");
      return false;
    }
    val.resize(12);
    in.read((char*)&val[0],(std::streamsize)12*sizeof(double));
    if (in.fail())
    {
      val.clear();
      return false;
    }
  }
  else
  {
    std::ifstream ifs;
    ifs.open( camName.c_str(), std::ifstream::in);
    if (!ifs.is_open()) {
      ALICEVISION_LOG_ERROR("Failed to open file '" << camName << "' for reading");
      return false;
    }
    while (ifs.good())
    {
      double valT;
      ifs >> valT;
      if (!ifs.fail())
        val.push_back(valT);
    }
  }

  //Update the camera from file value
  Mat34 P;
  if (stlplus::extension_part(camName) == "bin")
  {
    P << val[0], val[3], val[6], val[9],
      val[1], val[4], val[7], val[10],
      val[2], val[5], val[8], val[11];
  }
  else
  {
    P << val[0], val[1], val[2], val[3],
      val[4], val[5], val[6], val[7],
      val[8], val[9], val[10], val[11];
  }
  Mat3 K, R;
  Vec3 t;
  KRt_From_P(P, &K, &R, &t);
  cam = camera::Pinhole(0,0,K);
  // K.transpose() is applied to give [R t] to the constructor instead of P = K [R t]
  pose = geometry::Pose3(K.transpose() * P);
  return true;
}

bool read_Strecha_Camera(const std::string & camName, camera::Pinhole & cam, geometry::Pose3 & pose)
{
  std::ifstream ifs;
  ifs.open( camName.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    ALICEVISION_LOG_ERROR("Failed to open file '" << camName << "' for reading");
    return false;
  }
  std::vector<double> val;
  while (ifs.good() && !ifs.eof())
  {
    double valT;
    ifs >> valT;
    if (!ifs.fail())
      val.push_back(valT);
  }

  if (!(val.size() == 3*3 +3 +3*3 +3 + 3 || val.size() == 26)) //Strecha cam
  {
    ALICEVISION_LOG_ERROR("File " << camName << " is not in Stretcha format ! ");
    return false;
  }
  Mat3 K, R;
  K << val[0], val[1], val[2],
    val[3], val[4], val[5],
    val[6], val[7], val[8];
  R << val[12], val[13], val[14],
    val[15], val[16], val[17],
    val[18], val[19], val[20];

  Vec3 C (val[21], val[22], val[23]);
  // R need to be transposed
  cam = camera::Pinhole(0,0,K);
  cam.setWidth(val[24]);
  cam.setHeight(val[25]);
  pose = geometry::Pose3(R.transpose(), C);
  return true;
}

/**
@brief Reads a set of Pinhole Cameras and its poses from a ground truth dataset.
@param[in] sRootPath, the directory containing an image folder "images" and a GT folder "gt_dense_cameras".
@param[out] sfm_data, the SfMData structure to put views/poses/intrinsics in.
@param[in] useUID, set to false to disable UID".
@return Returns true if data has been read without errors
**/
bool readGt(const std::string & sRootPath, SfMData & sfm_data, bool useUID)
{
  // IF GT_Folder exists, perform evaluation of the quality of rotation estimates
  const std::string sGTPath = stlplus::folder_down(sRootPath, "gt_dense_cameras");
  if (!stlplus::is_folder(sGTPath))
  {
    ALICEVISION_LOG_DEBUG("There is not valid GT data to read from " << sGTPath);
    return false;
  }

  // Switch between case to choose the file reader according to the file types in GT path
  bool (*fcnReadCamPtr)(const std::string &, camera::Pinhole &, geometry::Pose3&);
  std::string suffix;
  if (!stlplus::folder_wildcard(sGTPath, "*.bin", true, true).empty())
  {
    ALICEVISION_LOG_TRACE("Using aliceVision Camera");
    fcnReadCamPtr = &read_aliceVision_Camera;
    suffix = "bin";
  }
  else if (!stlplus::folder_wildcard(sGTPath, "*.camera", true, true).empty())
  {
    ALICEVISION_LOG_TRACE("Using Strechas Camera");
    fcnReadCamPtr = &read_Strecha_Camera;
    suffix = "camera";
  }
  else
  {
    throw std::logic_error(std::string("No camera found in ") + sGTPath);
  }

  ALICEVISION_LOG_TRACE("Read rotation and translation estimates");
  const std::string sImgPath = stlplus::folder_down(sRootPath, "images");
  std::map< std::string, Mat3 > map_R_gt;
  //Try to read .suffix camera (parse camera names)
  std::vector<std::string> vec_camfilenames =
    stlplus::folder_wildcard(sGTPath, "*."+suffix, true, true);
  std::sort(vec_camfilenames.begin(), vec_camfilenames.end());
  if (vec_camfilenames.empty())
  {
    ALICEVISION_LOG_WARNING("No camera found in " << sGTPath);
    return false;
  }
  IndexT index = 0;
  for (std::vector<std::string>::const_iterator iter = vec_camfilenames.begin();
    iter != vec_camfilenames.end(); ++iter, ++index)
  {
    geometry::Pose3 pose;
    std::shared_ptr<camera::Pinhole> pinholeIntrinsic = std::make_shared<camera::Pinhole>();
    bool loaded = fcnReadCamPtr(stlplus::create_filespec(sGTPath, *iter), *pinholeIntrinsic.get(), pose);
    if (!loaded)
    {
      ALICEVISION_LOG_ERROR("Failed to load: " << *iter);
      return false;
    }

    const std::string sImgName = stlplus::basename_part(*iter);
    const std::string sImgFile = stlplus::create_filespec(sImgPath, sImgName);

    std::shared_ptr<View> viewPtr = std::make_shared<View>(stlplus::basename_part(*iter), UndefinedIndexT, index);

    updateIncompleteView(*viewPtr);

    // Update intrinsics with width and height of image
    sfm_data.views.emplace(viewPtr->getViewId(), viewPtr);
    sfm_data.setPose(*sfm_data.views.at(viewPtr->getViewId()), pose);
    sfm_data.intrinsics.emplace(index, pinholeIntrinsic);
  }
  return true;
}

} // namespace sfm
} // namespace aliceVision
