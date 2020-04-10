// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MultiViewParams.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/projection.hpp>

#include <boost/filesystem.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <set>

namespace aliceVision {
namespace mvsUtils {

namespace fs = boost::filesystem;

MultiViewParams::MultiViewParams(const sfmData::SfMData& sfmData,
                                 const std::string& imagesFolder,
                                 const std::string& depthMapsFolder,
                                 const std::string& depthMapsFilterFolder,
                                 bool readFromDepthMaps,
                                 int downscale,
                                 StaticVector<CameraMatrices>* cameras)
    : _sfmData(sfmData)
    , _imagesFolder(imagesFolder + "/")
    , _depthMapsFolder(depthMapsFolder + "/")
    , _depthMapsFilterFolder(depthMapsFilterFolder + "/")
    , _processDownscale(downscale)
{
    verbose = userParams.get<bool>("global.verbose", true);
    simThr = userParams.get<double>("global.simThr", 0.0);
    _useSil = userParams.get<bool>("global.use_silhouettes", _useSil);

    // load image uid, path and dimensions
    {
        std::set<std::pair<int, int>> dimensions; // for print only
        int i = 0;
        for(const auto& viewPair : sfmData.getViews())
        {
          const sfmData::View& view = *(viewPair.second.get());

          if(!sfmData.isPoseAndIntrinsicDefined(&view))
            continue;

          std::string path = view.getImagePath();

          if(readFromDepthMaps)
          {
            path = getFileNameFromViewId(this, view.getViewId(), mvsUtils::EFileType::depthMap, 1);
          }
          else if(_imagesFolder != "/" && !_imagesFolder.empty() && fs::is_directory(_imagesFolder) && !fs::is_empty(_imagesFolder))
          {
            // find folder file extension
            const fs::recursive_directory_iterator end;
            const auto findIt = std::find_if(fs::recursive_directory_iterator(_imagesFolder), end,
                                     [&view](const fs::directory_entry& e) {
                                        return (e.path().stem() == std::to_string(view.getViewId()) &&
                                        (imageIO::isSupportedUndistortFormat(e.path().extension().string())));
                                     });

            if(findIt == end)
              throw std::runtime_error("Cannot find image file " + std::to_string(view.getViewId()) + " in folder " + _imagesFolder);

            path = _imagesFolder + std::to_string(view.getViewId()) + findIt->path().extension().string();
          }

          dimensions.emplace(view.getWidth(), view.getHeight());
          _imagesParams.emplace_back(view.getViewId(), view.getWidth(), view.getHeight(), path);
          _imageIdsPerViewId[view.getViewId()] = i;
          ++i;
        }

        ALICEVISION_LOG_INFO("Found " << dimensions.size() << " image dimension(s): ");
        for(const auto& dim : dimensions)
            ALICEVISION_LOG_INFO("\t- [" << dim.first << "x" << dim.second << "]");
    }

    ncams = getNbCameras(); //TODO : always use getNbCameras() instead of ncams

    // Resize internal structures
    resizeCams(getNbCameras());

    for(int i = 0; i < getNbCameras(); ++i)
    {
        const ImageParams& imgParams = _imagesParams.at(i);

        oiio::ParamValueList metadata;
        imageIO::readImageMetadata(imgParams.path, metadata);

        const auto scaleIt = metadata.find("AliceVision:downscale");
        const auto pIt = metadata.find("AliceVision:P");

        // find image scale information
        if(scaleIt != metadata.end() && scaleIt->type() == oiio::TypeDesc::INT)
        {
            // use aliceVision image metadata
            _imagesScale.at(i) = scaleIt->get_int();
        }
        else
        {
            // use image dimension
            int w, h, channels;
            imageIO::readImageSpec(imgParams.path, w, h, channels);
            const int widthScale = imgParams.width / w;
            const int heightScale = imgParams.height / h;

            if((widthScale != 1) && (heightScale != 1))
                ALICEVISION_LOG_INFO("Reading '" << imgParams.path << "' x" << widthScale << "downscale from file dimension" << std::endl
                                                 << "\t- No 'AliceVision:downscale' metadata found.");

            if(widthScale != heightScale)
                throw std::runtime_error("Scale of file: '" + imgParams.path + "' is not uniform, check image dimension ratio.");

            _imagesScale.at(i) = widthScale;
        }

        FocK1K2Arr.at(i) = Point3d(-1.0, -1.0, -1.0);

        // load camera matrices
        if(cameras != nullptr)
        {
            // use constructor cameras input parameter
            camArr.at(i) = (*cameras)[i].P;
            KArr.at(i) = (*cameras)[i].K;
            RArr.at(i) = (*cameras)[i].R;
            CArr.at(i) = (*cameras)[i].C;
            iKArr.at(i) = (*cameras)[i].iK;
            iRArr.at(i) = (*cameras)[i].iR;
            iCamArr.at(i) = (*cameras)[i].iCam;
            FocK1K2Arr.at(i) = Point3d((*cameras)[i].f, (*cameras)[i].k1, (*cameras)[i].k2);
        }
        else if(pIt != metadata.end() && pIt->type() == oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44))
        {
            ALICEVISION_LOG_DEBUG("Reading view " << getViewId(i) << " projection matrix from image metadata.");
            loadMatricesFromRawProjectionMatrix(i, static_cast<const double*>(pIt->data()));
        }
        else
        {
            // use P matrix file
            const std::string fileNameP = getFileNameFromIndex(this, i, EFileType::P);
            const std::string fileNameD = getFileNameFromIndex(this, i, EFileType::D);

            if(fs::exists(fileNameP), fs::exists(fileNameD))
            {
              ALICEVISION_LOG_DEBUG("Reading view " << getViewId(i) << " projection matrix from file '" << fileNameP << "'.");

              loadMatricesFromTxtFile(i, fileNameP, fileNameD);
            }
            else
            {
              ALICEVISION_LOG_DEBUG("Reading view " << getViewId(i) << " projection matrix from SfMData.");
              loadMatricesFromSfM(i);
            }
        }

        if(KArr[i].m11 > (float)(getWidth(i) * 100))
        {
            ALICEVISION_LOG_WARNING("Camera " << i << " at infinity. Setting to zero");

            KArr[i].m11 = getWidth(i) / 2;
            KArr[i].m12 = 0;
            KArr[i].m13 = getWidth(i) / 2;
            KArr[i].m21 = 0;
            KArr[i].m22 = getHeight(i) / 2;
            KArr[i].m23 = getHeight(i) / 2;
            KArr[i].m31 = 0;
            KArr[i].m32 = 0;
            KArr[i].m33 = 1;

            RArr[i].m11 = 1;
            RArr[i].m12 = 0;
            RArr[i].m13 = 0;
            RArr[i].m21 = 0;
            RArr[i].m22 = 1;
            RArr[i].m23 = 0;
            RArr[i].m31 = 0;
            RArr[i].m32 = 0;
            RArr[i].m33 = 1;

            iRArr[i].m11 = 1;
            iRArr[i].m12 = 0;
            iRArr[i].m13 = 0;
            iRArr[i].m21 = 0;
            iRArr[i].m22 = 1;
            iRArr[i].m23 = 0;
            iRArr[i].m31 = 0;
            iRArr[i].m32 = 0;
            iRArr[i].m33 = 1;

            iKArr[i] = KArr[i].inverse();
            iCamArr[i] = iRArr[i] * iKArr[i];
            CArr[i].x = 0.0;
            CArr[i].y = 0.0;
            CArr[i].z = 0.0;

            camArr[i] = KArr[i] * (RArr[i] | (Point3d(0.0, 0.0, 0.0) - RArr[i] * CArr[i]));
        }

        // find max width and max height
        _maxImageWidth = std::max(_maxImageWidth, imgParams.width);
        _maxImageHeight = std::max(_maxImageHeight, imgParams.height);
    }

    ALICEVISION_LOG_INFO("Overall maximum dimension: [" << _maxImageWidth << "x" << _maxImageHeight << "]");
}


void MultiViewParams::loadMatricesFromTxtFile(int index, const std::string& fileNameP, const std::string& fileNameD)
{
    if(!FileExists(fileNameP))
        throw std::runtime_error(std::string("mv_multiview_params: no such file: ") + fileNameP);

    FILE* f = fopen(fileNameP.c_str(), "r");
    char fc;
    fscanf(f, "%c", &fc);
    if(fc == 'C') // FURUKAWA'S PROJCTION MATRIX FILE FORMAT
    {
        fscanf(f, "%c", &fc);   // O
        fscanf(f, "%c", &fc);   // N
        fscanf(f, "%c", &fc);   // T
        fscanf(f, "%c", &fc);   // O
        fscanf(f, "%c", &fc);   // U
        fscanf(f, "%c\n", &fc); // R
    }
    else
    {
        fclose(f);
        f = fopen(fileNameP.c_str(), "r");
    }

    Matrix3x4& pMatrix = camArr.at(index);

    pMatrix = load3x4MatrixFromFile(f);
    fclose(f);

    // apply scale to camera matrix (camera matrix is scale 1)
    const int imgScale = _imagesScale.at(index) * _processDownscale;
    for(int i=0; i< 8; ++i)
      pMatrix.m[i] /= static_cast<double>(imgScale);

    pMatrix.decomposeProjectionMatrix(KArr[index], RArr[index], CArr[index]);
    iKArr[index] = KArr[index].inverse();
    iRArr[index] = RArr[index].inverse();
    iCamArr[index] = iRArr[index] * iKArr[index];

    if(FileExists(fileNameD))
    {
        FILE* f = fopen(fileNameD.c_str(), "r");
        fscanf(f, "%f %f %f", &FocK1K2Arr[index].x, &FocK1K2Arr[index].y, &FocK1K2Arr[index].z);
        fclose(f);
    }
}

void MultiViewParams::loadMatricesFromRawProjectionMatrix(int index, const double* rawProjMatix)
{
  Matrix3x4& pMatrix = camArr.at(index);
  std::copy_n(rawProjMatix, 12, pMatrix.m);

  // apply scale to camera matrix (camera matrix is scale 1)
  const int imgScale = _imagesScale.at(index) * _processDownscale;
  for(int i = 0; i < 8; ++i)
      pMatrix.m[i] /= static_cast<double>(imgScale);

  pMatrix.decomposeProjectionMatrix(KArr.at(index), RArr.at(index), CArr.at(index));
  iKArr.at(index) = KArr.at(index).inverse();
  iRArr.at(index) = RArr.at(index).inverse();
  iCamArr.at(index) = iRArr.at(index) * iKArr.at(index);
}

void MultiViewParams::loadMatricesFromSfM(int index)
{
  using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  const sfmData::View& view = *(_sfmData.getViews().at(getViewId(index)));
  sfmData::Intrinsics::const_iterator intrinsicIt = _sfmData.getIntrinsics().find(view.getIntrinsicId());
  const Mat34 P = intrinsicIt->second.get()->get_projective_equivalent(_sfmData.getPose(view).getTransform());
  std::vector<double> vP(P.size());
  Eigen::Map<RowMatrixXd>(vP.data(), P.rows(), P.cols()) = P;

  loadMatricesFromRawProjectionMatrix(index, vP.data());
}

MultiViewParams::~MultiViewParams()
{}

const std::map<std::string, std::string>& MultiViewParams::getMetadata(int index) const
{
  return _sfmData.getViews().at(getViewId(index))->getMetadata();
}

bool MultiViewParams::is3DPointInFrontOfCam(const Point3d* X, int rc) const
{
    Point3d XT = camArr[rc] * (*X);

    return XT.z >= 0;
}

void MultiViewParams::getMinMaxMidNbDepth(int index, float& min, float& max, float& mid, std::size_t& nbDepths, float percentile) const
{
  using namespace boost::accumulators;

  const std::size_t cacheSize =  1000;
  accumulator_set<float, stats<tag::tail_quantile<left>>>  accDistanceMin(tag::tail<left>::cache_size = cacheSize);
  accumulator_set<float, stats<tag::tail_quantile<right>>> accDistanceMax(tag::tail<right>::cache_size = cacheSize);

  const IndexT viewId = getViewId(index);

  ALICEVISION_LOG_DEBUG("Compute min/max/mid/nb depth for view id: " << viewId);

  OrientedPoint cameraPlane;
  cameraPlane.p = CArr[index];
  cameraPlane.n = iRArr[index] * Point3d(0.0, 0.0, 1.0);
  cameraPlane.n = cameraPlane.n.normalize();

  Point3d midDepthPoint = Point3d();
  nbDepths = 0;

  for(const auto& landmarkPair : _sfmData.getLandmarks())
  {
    const sfmData::Landmark& landmark = landmarkPair.second;
    const Point3d point(landmark.X(0), landmark.X(1), landmark.X(2));

    for(const auto& observationPair : landmark.observations)
    {
      if(observationPair.first == viewId)
      {
        const float distance = static_cast<float>(pointPlaneDistance(point, cameraPlane.p, cameraPlane.n));
        accDistanceMin(distance);
        accDistanceMax(distance);
        midDepthPoint = midDepthPoint + point;
        ++nbDepths;
      }
    }
  }

  min = quantile(accDistanceMin, quantile_probability = 1.0 - percentile);
  max = quantile(accDistanceMax, quantile_probability = percentile);
  midDepthPoint = midDepthPoint / static_cast<float>(nbDepths);
  mid = pointPlaneDistance(midDepthPoint, cameraPlane.p, cameraPlane.n);
}

void MultiViewParams::getPixelFor3DPoint(Point2d* out, const Point3d& X, int rc) const
{
    getPixelFor3DPoint(out, X, camArr[rc]);
}

void MultiViewParams::getPixelFor3DPoint(Point2d* out, const Point3d& X, const Matrix3x4& P) const
{
    Point3d XT = P * X;

    if(XT.z <= 0)
    {
        out->x = -1.0;
        out->y = -1.0;
    }
    else
    {
        out->x = XT.x / XT.z;
        out->y = XT.y / XT.z;
    }
}

void MultiViewParams::getPixelFor3DPoint(Pixel* out, const Point3d& X, int rc) const
{
    Point3d XT = camArr[rc] * X;

    if(XT.z <= 0)
    {
        out->x = -1;
        out->y = -1;
    }
    else
    {
        //+0.5 is IMPORTANT
        out->x = (int)floor(XT.x / XT.z + 0.5);
        out->y = (int)floor(XT.y / XT.z + 0.5);
    }
}

/**
 * @brief size in 3d space of one pixel at the 3d point depth.
 * @param[in] x0 3d point
 * @param[in] cam camera index
 */
double MultiViewParams::getCamPixelSize(const Point3d& x0, int cam) const
{
    Point2d pix;
    getPixelFor3DPoint(&pix, x0, cam);
    pix.x = pix.x + 1.0;
    Point3d vect = iCamArr[cam] * pix;

    vect = vect.normalize();
    return pointLineDistance3D(x0, CArr[cam], vect);
}

double MultiViewParams::getCamPixelSize(const Point3d& x0, int cam, float d) const
{
    if(d == 0.0f)
    {
        return 0.0f;
    }

    Point2d pix;
    getPixelFor3DPoint(&pix, x0, cam);
    pix.x = pix.x + d;
    Point3d vect = iCamArr[cam] * pix;

    vect = vect.normalize();
    return pointLineDistance3D(x0, CArr[cam], vect);
}

/**
* @brief Return the size of a pixel in space with an offset
* of "d" pixels in the target camera (along the epipolar line).
*/
double MultiViewParams::getCamPixelSizeRcTc(const Point3d& p, int rc, int tc, float d) const
{
    if(d == 0.0f)
    {
        return 0.0f;
    }

    Point3d p1 = CArr[rc] + (p - CArr[rc]) * 0.1f;
    Point2d rpix;
    getPixelFor3DPoint(&rpix, p, rc);

    Point2d pFromTar, pToTar;
    getTarEpipolarDirectedLine(&pFromTar, &pToTar, rpix, rc, tc, this);
    // A vector of 1 pixel length on the epipolar line in tc camera
    // of the 3D point p projected in camera rc.
    Point2d pixelVect = ((pToTar - pFromTar).normalize()) * d;
    // tpix is the point p projected in camera tc
    Point2d tpix;
    getPixelFor3DPoint(&tpix, p, tc);
    // tpix1 is tpix with an offset of d pixels along the epipolar line
    Point2d tpix1 = tpix + pixelVect * d;

    if(!triangulateMatch(p1, rpix, tpix1, rc, tc, this))
    {
        // Fallback to compute the pixel size using only the rc camera
        return getCamPixelSize(p, rc, d);
    }
    // Return the 3D distance between the original point and the newly triangulated one
    return (p - p1).size();
}

double MultiViewParams::getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, int tc, int scale, int step) const
{
    double splaneSeweepAlpha = (double)(scale * step);
    // Compute the 3D volume defined by N pixels in the target camera.
    // We use an offset of splaneSeweepAlpha pixels along the epipolar line
    // (defined by p and the reference camera center) on the target camera.
    double avRcTc = getCamPixelSizeRcTc(p, rc, tc, splaneSeweepAlpha);
    // Compute the 3D volume defined by N pixels in the reference camera
    double avRc = getCamPixelSize(p, rc, splaneSeweepAlpha);
    // Return the average of the pixelSize in rc and tc cameras.
    return (avRcTc + avRc) * 0.5;
}

double MultiViewParams::getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, StaticVector<int>* tcams, int scale,
                                                      int step) const
{
    //float av1 = 0.0f;
    double avmax = 0.0;
    for(int c = 0; c < tcams->size(); c++)
    {
        double dpxs = getCamPixelSizePlaneSweepAlpha(p, rc, (*tcams)[c], scale, step);
        //av1 += dpxs;
        avmax = std::max(avmax, dpxs);
    }
    //av1 /= (float)(tcams->size());
    // return av1;
    return avmax;
}

double MultiViewParams::getCamsMinPixelSize(const Point3d& x0, StaticVector<int>& tcams) const
{
    if(tcams.empty())
    {
        return 0.0f;
    }
    double minPixSize = 1000000000.0;
    for(int ci = 0; ci < (int)tcams.size(); ci++)
    {
        double pixSize = getCamPixelSize(x0, (int)tcams[ci]);
        if(minPixSize > pixSize)
        {
            minPixSize = pixSize;
        }
    }

    return minPixSize;
}

bool MultiViewParams::isPixelInSourceImage(const Pixel& pixRC, int camId, int margin) const
{
    const IndexT viewId = getViewId(camId);
    const sfmData::View& view = *(_sfmData.getViews().at(viewId));
    const camera::IntrinsicBase* intrinsicPtr = _sfmData.getIntrinsicPtr(view.getIntrinsicId());

    const double s = getDownscaleFactor(camId);
    Vec2 pix_disto = intrinsicPtr->get_d_pixel({pixRC.x * s, pixRC.y * s}) / s;
    return isPixelInImage(Pixel(pix_disto.x(), pix_disto.y()), camId, margin);
}

bool MultiViewParams::isPixelInImage(const Pixel& pix, int camId, int margin) const
{
    return ((pix.x >= margin) && (pix.x < getWidth(camId) - margin) &&
            (pix.y >= margin) && (pix.y < getHeight(camId) - margin));
}
bool MultiViewParams::isPixelInImage(const Pixel& pix, int camId) const
{
    return isPixelInImage(pix, camId, g_border);
}

bool MultiViewParams::isPixelInImage(const Point2d& pix, int camId) const
{
    return isPixelInImage(Pixel(pix), camId);
}

bool MultiViewParams::isPixelInImage(const Point2d& pix, int camId, int margin) const
{
    return isPixelInImage(Pixel(pix), camId, margin);
}

void MultiViewParams::decomposeProjectionMatrix(Point3d& Co, Matrix3x3& Ro, Matrix3x3& iRo, Matrix3x3& Ko,
                                                Matrix3x3& iKo, Matrix3x3& iPo, const Matrix3x4& P) const
{
    P.decomposeProjectionMatrix(Ko, Ro, Co);
    iKo = Ko.inverse();
    iRo = Ro.inverse();
    iPo = iRo * iKo;
}


StaticVector<int> MultiViewParams::findNearestCamsFromLandmarks(int rc, int nbNearestCams) const
{
  StaticVector<int> out;
  std::vector<SortedId> ids;
  ids.reserve(getNbCameras());

  for(int tc = 0; tc < getNbCameras(); ++tc)
    ids.push_back(SortedId(tc,0));

  const IndexT viewId = getViewId(rc);
  const sfmData::View& view = *(_sfmData.getViews().at(viewId));
  const geometry::Pose3 pose = _sfmData.getPose(view).getTransform();
  const camera::IntrinsicBase* intrinsicPtr = _sfmData.getIntrinsicPtr(view.getIntrinsicId());

  for(const auto& landmarkPair :_sfmData.getLandmarks())
  {
    const auto& observations = landmarkPair.second.observations;

    auto viewObsIt = observations.find(viewId);
    if(viewObsIt == observations.end())
      continue;

    for(const auto& observationPair : observations)
    {
      const IndexT otherViewId = observationPair.first;

      if(otherViewId == viewId)
       continue;

      const sfmData::View& otherView = *(_sfmData.getViews().at(otherViewId));
      const geometry::Pose3 otherPose = _sfmData.getPose(otherView).getTransform();
      const camera::IntrinsicBase* otherIntrinsicPtr = _sfmData.getIntrinsicPtr(otherView.getIntrinsicId());

      const double angle = camera::AngleBetweenRays(pose, intrinsicPtr, otherPose, otherIntrinsicPtr, viewObsIt->second.x, observationPair.second.x);

      if(angle < _minViewAngle || angle > _maxViewAngle)
        continue;

      const int tc = getIndexFromViewId(otherViewId);
      ++ids.at(tc).value;
    }
  }

  qsort(&ids[0], ids.size(), sizeof(SortedId), qsortCompareSortedIdDesc);

  // ensure the ideal number of target cameras is not superior to the actual number of cameras
  const int maxTc = std::min(std::min(getNbCameras(), nbNearestCams), static_cast<int>(ids.size()));
  out.reserve(maxTc);

  for(int i = 0; i < maxTc; ++i)
  {
    // a minimum of 10 common points is required (10*2 because points are stored in both rc/tc combinations)
    if(ids[i].value > (10 * 2))
      out.push_back(ids[i].id);
  }

  if(out.size() < nbNearestCams)
    ALICEVISION_LOG_INFO("Found only " << out.size() << "/" << nbNearestCams << " nearest cameras for view id: " << getViewId(rc));

  return out;
}

StaticVector<int> MultiViewParams::findCamsWhichIntersectsHexahedron(const Point3d hexah[8], const std::string& minMaxDepthsFileName) const
{
    StaticVector<Point2d>* minMaxDepths = loadArrayFromFile<Point2d>(minMaxDepthsFileName);
    StaticVector<int> tcams;
    tcams.reserve(getNbCameras());
    for(int rc = 0; rc < getNbCameras(); rc++)
    {
        const float minDepth = (*minMaxDepths)[rc].x;
        const float maxDepth = (*minMaxDepths)[rc].y;
        if((minDepth > 0.0f) && (maxDepth > minDepth))
        {
            Point3d rchex[8];
            getCamHexahedron(CArr.at(rc), iCamArr.at(rc), getWidth(rc), getHeight(rc), minDepth, maxDepth, rchex);
            if(intersectsHexahedronHexahedron(rchex, hexah))
                tcams.push_back(rc);
        }
    }
    delete minMaxDepths;
    return tcams;
}

StaticVector<int> MultiViewParams::findCamsWhichIntersectsHexahedron(const Point3d hexah[8]) const
{
    StaticVector<int> tcams;
    tcams.reserve(getNbCameras());
    for(int rc = 0; rc < getNbCameras(); rc++)
    {
        oiio::ParamValueList metadata;
        imageIO::readImageMetadata(getImagePath(rc), metadata);

        const float minDepth = metadata.get_float("AliceVision:minDepth", -1);
        const float maxDepth = metadata.get_float("AliceVision:maxDepth", -1);

        if(minDepth == -1 && maxDepth == -1)
        {
            ALICEVISION_LOG_WARNING("Cannot find min / max depth metadata in image: " << getImagePath(rc) << ". Assumes that all images should be used.");
            tcams.push_back(rc);
        }
        else
        {
            Point3d rchex[8];
            getCamHexahedron(CArr.at(rc), iCamArr.at(rc), getWidth(rc), getHeight(rc), minDepth, maxDepth, rchex);
            if(intersectsHexahedronHexahedron(rchex, hexah))
                tcams.push_back(rc);
        }
    }
    return tcams;
}

} // namespace mvsUtils
} // namespace aliceVision
