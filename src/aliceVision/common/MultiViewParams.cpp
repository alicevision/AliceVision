// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MultiViewParams.hpp"
#include <aliceVision/structures/geometry.hpp>
#include <aliceVision/structures/Matrix3x4.hpp>
#include <aliceVision/structures/Pixel.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/common/common.hpp>
#include <aliceVision/imageIO/image.hpp>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <set>

namespace aliceVision {

namespace bfs = boost::filesystem;

MultiViewInputParams::MultiViewInputParams(const std::string& file, const std::string& depthMapFolder, const std::string& depthMapFilterFolder)
{
    initFromConfigFile(file);
    _depthMapFolder = depthMapFolder + "/";
    _depthMapFilterFolder = depthMapFilterFolder + "/";
}

void MultiViewInputParams::initFromConfigFile(const std::string& iniFile)
{
    boost::property_tree::ini_parser::read_ini(iniFile, _ini);
    // debug, dump the read ini file to cout
    // boost::property_tree::write_ini(std::cout, _ini);

    // initialize directory names
    const auto rootPath = bfs::path(iniFile).parent_path().string() + "/";
    mvDir = rootPath;
    _depthMapFolder = (bfs::path(rootPath) / "depthMap").string();
    _depthMapFilterFolder = (bfs::path(rootPath) / "depthMapFilter").string();

    imageExt = _ini.get<std::string>("global.imgExt", imageExt);
    prefix = _ini.get<std::string>("global.prefix", prefix);

    usesil = _ini.get<bool>("global.use_silhouettes", usesil);
    int ncams = _ini.get<int>("global.ncams", 0);
    assert(ncams > 0);
    // Load image uid and dimensions
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
            imageParams imgParams(std::stoi(v.first), boost::lexical_cast<int>(valuesVec[0]), boost::lexical_cast<int>(valuesVec[1]));
            _imagesParams.push_back(imgParams);
            maxImageWidth = std::max(maxImageWidth, imgParams.width);
            maxImageHeight = std::max(maxImageHeight, imgParams.height);
            dimensions.emplace(imgParams.width, imgParams.height);
        }
    }
    if(getNbCameras() != ncams)
        throw std::runtime_error("Incoherent number of cameras.");
    std::cout << "Found " << dimensions.size() << " image dimension(s): " << std::endl;
    for(const auto& dim : dimensions)
        std::cout << " - [" << dim.first << "x" << dim.second << "]" << std::endl;
    std::cout << "Overall maximum dimension: [" << maxImageWidth << "x" << maxImageHeight << "]" << std::endl;
}

MultiViewParams::MultiViewParams(int _ncams, MultiViewInputParams* _mip, float _simThr,
                                 StaticVector<CameraMatrices>* cameras)
{
    mip = _mip;

    verbose = (bool)mip->_ini.get<bool>("global.verbose", true);
    CUDADeviceNo = mip->_ini.get<int>("global.CUDADeviceNo", 0);

    minWinSizeHalf = 2;
    simThr = _simThr;

    resizeCams(_ncams);

    long t1 = initEstimate();
    for(int i = 0; i < ncams; i++)
    {
        FocK1K2Arr[i] = Point3d(-1.0f, -1.0f, -1.0f);

        if(cameras != nullptr)
        {
            camArr[i] = (*cameras)[i].P;
            KArr[i] = (*cameras)[i].K;
            RArr[i] = (*cameras)[i].R;
            CArr[i] = (*cameras)[i].C;
            iKArr[i] = (*cameras)[i].iK;
            iRArr[i] = (*cameras)[i].iR;
            iCamArr[i] = (*cameras)[i].iCam;
            FocK1K2Arr[i] = Point3d((*cameras)[i].f, (*cameras)[i].k1, (*cameras)[i].k2);
        }
        else
        {
            std::string fileNameP = mv_getFileName(mip, i, EFileType::P);
            std::string fileNameD = mv_getFileName(mip, i, EFileType::D);
            loadCameraFile(i, fileNameP, fileNameD);
        }

        if(KArr[i].m11 > (float)(mip->getWidth(i) * 100))
        {
            printf("WARNING camera %i at infinity ... settitng to zero\n", i);

            KArr[i].m11 = mip->getWidth(i) / 2;
            KArr[i].m12 = 0;
            KArr[i].m13 = mip->getWidth(i) / 2;
            KArr[i].m21 = 0;
            KArr[i].m22 = mip->getHeight(i) / 2;
            KArr[i].m23 = mip->getHeight(i) / 2;
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
            CArr[i].x = 0.0f;
            CArr[i].y = 0.0f;
            CArr[i].z = 0.0f;

            camArr[i] = KArr[i] * (RArr[i] | (Point3d(0.0, 0.0, 0.0) - RArr[i] * CArr[i]));
        }

        printfEstimate(i, ncams, t1);
    }
    finishEstimate();

    g_border = 10;
    g_maxPlaneNormalViewDirectionAngle = 70;
}


void MultiViewParams::loadCameraFile(int i, const std::string& fileNameP, const std::string& fileNameD)
{
    // std::cout << "MultiViewParams::loadCameraFile: " << fileNameP << std::endl;

    if(!FileExists(fileNameP))
    {
        throw std::runtime_error(std::string("mv_multiview_params: no such file: ") + fileNameP);
    }
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
    camArr[i] = load3x4MatrixFromFile(f);
    fclose(f);

    camArr[i].decomposeProjectionMatrix(KArr[i], RArr[i], CArr[i]);
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
{
    mip = nullptr;
}

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
        out->x = -1.0f;
        out->y = -1.0f;
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
float MultiViewParams::getCamPixelSize(const Point3d& x0, int cam) const
{
    Point2d pix;
    getPixelFor3DPoint(&pix, x0, cam);
    pix.x = pix.x + 1.0;
    Point3d vect = iCamArr[cam] * pix;

    vect = vect.normalize();
    return pointLineDistance3D(x0, CArr[cam], vect);
}

float MultiViewParams::getCamPixelSize(const Point3d& x0, int cam, float d) const
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
float MultiViewParams::getCamPixelSizeRcTc(const Point3d& p, int rc, int tc, float d) const
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

float MultiViewParams::getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, int tc, int scale, int step) const
{
    float splaneSeweepAlpha = (float)(scale * step);
    // Compute the 3D volume defined by N pixels in the target camera.
    // We use an offset of splaneSeweepAlpha pixels along the epipolar line
    // (defined by p and the reference camera center) on the target camera.
    float avRcTc = getCamPixelSizeRcTc(p, rc, tc, splaneSeweepAlpha);
    // Compute the 3D volume defined by N pixels in the reference camera
    float avRc = getCamPixelSize(p, rc, splaneSeweepAlpha);
    // Return the average of the pixelSize in rc and tc cameras.
    return (avRcTc + avRc) * 0.5f;
}

float MultiViewParams::getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, StaticVector<int>* tcams, int scale,
                                                      int step) const
{
    //float av1 = 0.0f;
    float avmax = 0.0f;
    for(int c = 0; c < tcams->size(); c++)
    {
        float dpxs = getCamPixelSizePlaneSweepAlpha(p, rc, (*tcams)[c], scale, step);
        //av1 += dpxs;
        avmax = std::max(avmax, dpxs);
    }
    //av1 /= (float)(tcams->size());
    // return av1;
    return avmax;
}

float MultiViewParams::getCamsMinPixelSize(const Point3d& x0, StaticVector<int>& tcams) const
{
    if(tcams.empty())
    {
        return 0.0f;
    }
    float minPixSize = 1000000000.0;
    for(int ci = 0; ci < (int)tcams.size(); ci++)
    {
        float pixSize = getCamPixelSize(x0, (int)tcams[ci]);
        if(minPixSize > pixSize)
        {
            minPixSize = pixSize;
        }
    }

    return minPixSize;
}

bool MultiViewParams::isPixelInImage(const Pixel& pix, int d, int camId) const
{
    return ((pix.x >= d) && (pix.x < mip->getWidth(camId) - d) && (pix.y >= d) && (pix.y < mip->getHeight(camId) - d));
}
bool MultiViewParams::isPixelInImage(const Pixel& pix, int camId) const
{
    return ((pix.x >= g_border) && (pix.x < mip->getWidth(camId) - g_border) && (pix.y >= g_border) &&
            (pix.y < mip->getHeight(camId) - g_border));
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

} // namespace aliceVision
