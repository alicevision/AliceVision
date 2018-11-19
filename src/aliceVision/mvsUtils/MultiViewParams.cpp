// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MultiViewParams.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/imageIO/image.hpp>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <set>

namespace aliceVision {
namespace mvsUtils {

namespace bfs = boost::filesystem;

void MultiViewParams::initFromConfigFile(const std::string& iniFile)
{
    boost::property_tree::ini_parser::read_ini(iniFile, _ini);
    // debug, dump the read ini file to cout
    // boost::property_tree::write_ini(std::cout, _ini);

    // initialize directory names
    const auto rootPath = bfs::path(iniFile).parent_path().string() + "/";
    mvDir = rootPath;
    prefix = _ini.get<std::string>("global.prefix", prefix);
    verbose = _ini.get<bool>("global.verbose", true);

    ncams = _ini.get<int>("global.ncams", 0);
    simThr = _ini.get<double>("global.simThr", 0.0);
    _imageExt = _ini.get<std::string>("global.imgExt", _imageExt);
    _useSil = _ini.get<bool>("global.use_silhouettes", _useSil);

    assert(ncams > 0);

    // load image uid and dimensions
    std::set<std::pair<int, int>> dimensions;
    {
        boost::optional<boost::property_tree::ptree&> cameras = _ini.get_child_optional("imageResolutions");
        if(!cameras)
        {
         throw std::runtime_error("Can't find images UID and dimensions in .ini file");
        }

        for (const auto v : *cameras)
        {
            const std::string values = v.second.get_value<std::string>();
            std::vector<std::string> valuesVec;
            boost::split(valuesVec, values, boost::algorithm::is_any_of("x"));
            if(valuesVec.size() != 2)
                throw std::runtime_error("Error when loading image sizes from INI file.");

            imageParams imgParams(std::stoi(v.first),
                                  boost::lexical_cast<int>(valuesVec[0]),
                                  boost::lexical_cast<int>(valuesVec[1]));

            _imagesParams.push_back(imgParams);
            dimensions.emplace(imgParams.width, imgParams.height);
        }
    }

    if(getNbCameras() != ncams)
        throw std::runtime_error("Incoherent number of cameras.");

    ALICEVISION_LOG_INFO("Found " << dimensions.size() << " image dimension(s): ");
    for(const auto& dim : dimensions)
        ALICEVISION_LOG_INFO(" - [" << dim.first << "x" << dim.second << "]");
}



MultiViewParams::MultiViewParams(const std::string& iniFile,
                                 const std::string& depthMapFolder,
                                 const std::string& depthMapFilterFolder,
                                 bool readFromDepthMaps,
                                 int downscale,
                                 StaticVector<CameraMatrices>* cameras)
    : _depthMapFolder(depthMapFolder + "/")
    , _depthMapFilterFolder(depthMapFilterFolder + "/")
    , _processDownscale(downscale)
{
    // Parse the .ini file
    initFromConfigFile(iniFile);

    // Resize internal structures
    resizeCams(getNbCameras());

    for(int i = 0; i < ncams; ++i)
    {
        std::string path;

        if(!readFromDepthMaps)
            path = mv_getFileNamePrefix(mvDir, this, i) + "." + _imageExt;
        else
            path = mv_getFileName(this, i, mvsUtils::EFileType::depthMap, 1);

        oiio::ParamValueList metadata;
        imageIO::readImageMetadata(path, metadata);

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
            ALICEVISION_LOG_WARNING("Reading '" << path << "' downscale from file dimension" << std::endl
                                  << "No 'AliceVision:downscale' metadata found.");
            int w, h, channels;
            imageIO::readImageSpec(path, w, h, channels);
            const imageParams& imgParams = _imagesParams.at(i);
            const int widthScale = imgParams.width / w;
            const int heightScale = imgParams.height / h;

            if(widthScale != heightScale)
              throw std::runtime_error("Can't find image scale of file: '" + path + "'");

            _imagesScale.at(i) = widthScale;
        }

        FocK1K2Arr[i] = Point3d(-1.0, -1.0, -1.0);

        // load camera matrices
        if(cameras != nullptr)
        {
            // use constructor cameras input parameter
            camArr[i] = (*cameras)[i].P;
            KArr[i] = (*cameras)[i].K;
            RArr[i] = (*cameras)[i].R;
            CArr[i] = (*cameras)[i].C;
            iKArr[i] = (*cameras)[i].iK;
            iRArr[i] = (*cameras)[i].iR;
            iCamArr[i] = (*cameras)[i].iCam;
            FocK1K2Arr[i] = Point3d((*cameras)[i].f, (*cameras)[i].k1, (*cameras)[i].k2);
        }
        else if(pIt != metadata.end() && pIt->type() == oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44))
        {
            // use aliceVision image metadata
            Matrix3x4& pMatrix = camArr.at(i);
            std::copy_n(static_cast<const double*>(pIt->data()), 12, pMatrix.m);

            // apply scale to camera matrix (camera matrix is scale 1)
            const int imgScale = _imagesScale.at(i) * _processDownscale;
            for(int i=0; i< 8; ++i)
              pMatrix.m[i] /= static_cast<double>(imgScale);

            pMatrix.decomposeProjectionMatrix(KArr.at(i), RArr.at(i), CArr.at(i));
            iKArr.at(i) = KArr.at(i).inverse();
            iRArr.at(i) = RArr.at(i).inverse();
            iCamArr.at(i) = iRArr.at(i) * iKArr.at(i);
        }
        else
        {
            // use P matrix file
            std::string fileNameP = mv_getFileName(this, i, EFileType::P);
            std::string fileNameD = mv_getFileName(this, i, EFileType::D);

            ALICEVISION_LOG_WARNING("Reading " << getViewId(i) << " P matrix from file '" << fileNameP << "'" << std::endl
                                    << "No 'AliceVision:P' metadata found.");

            loadCameraFile(i, fileNameP, fileNameD);
        }


        if(KArr[i].m11 > (float)(getWidth(i) * 100))
        {
            ALICEVISION_LOG_WARNING("Camera " << i << " at infinity ... setting to zero");

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
        _maxImageWidth = std::max(_maxImageWidth, getWidth(i));
        _maxImageHeight = std::max(_maxImageHeight, getHeight(i));
    }

    ALICEVISION_LOG_INFO("Overall maximum dimension: [" << _maxImageWidth << "x" << _maxImageHeight << "]");
}


void MultiViewParams::loadCameraFile(int i, const std::string& fileNameP, const std::string& fileNameD)
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

    Matrix3x4& pMatrix = camArr.at(i);

    pMatrix = load3x4MatrixFromFile(f);
    fclose(f);

    // apply scale to camera matrix (camera matrix is scale 1)
    const int imgScale = _imagesScale.at(i) * _processDownscale;
    for(int i=0; i< 8; ++i)
      pMatrix.m[i] /= static_cast<double>(imgScale);

    pMatrix.decomposeProjectionMatrix(KArr[i], RArr[i], CArr[i]);
    iKArr[i] = KArr[i].inverse();
    iRArr[i] = RArr[i].inverse();
    iCamArr[i] = iRArr[i] * iKArr[i];

    if(FileExists(fileNameD))
    {
        FILE* f = fopen(fileNameD.c_str(), "r");
        fscanf(f, "%f %f %f", &FocK1K2Arr[i].x, &FocK1K2Arr[i].y, &FocK1K2Arr[i].z);
        fclose(f);
    }
}

MultiViewParams::~MultiViewParams()
{}

bool MultiViewParams::is3DPointInFrontOfCam(const Point3d* X, int rc) const
{
    Point3d XT = camArr[rc] * (*X);

    return XT.z >= 0;
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

bool MultiViewParams::isPixelInImage(const Pixel& pix, int d, int camId) const
{
    return ((pix.x >= d) && (pix.x < getWidth(camId) - d) && (pix.y >= d) && (pix.y < getHeight(camId) - d));
}
bool MultiViewParams::isPixelInImage(const Pixel& pix, int camId) const
{
    return ((pix.x >= g_border) && (pix.x < getWidth(camId) - g_border) &&
            (pix.y >= g_border) && (pix.y < getHeight(camId) - g_border));
}

bool MultiViewParams::isPixelInImage(const Point2d& pix, int camId) const
{
    return isPixelInImage(Pixel(pix), camId);
}

void MultiViewParams::decomposeProjectionMatrix(Point3d& Co, Matrix3x3& Ro, Matrix3x3& iRo, Matrix3x3& Ko,
                                                Matrix3x3& iKo, Matrix3x3& iPo, const Matrix3x4& P) const
{
    P.decomposeProjectionMatrix(Ko, Ro, Co);
    iKo = Ko.inverse();
    iRo = Ro.inverse();
    iPo = iRo * iKo;
}

} // namespace mvsUtils
} // namespace aliceVision
