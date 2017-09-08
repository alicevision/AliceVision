#include "mv_multiview_params.h"
#include "mv_filesio.h"
#include "mv_geometry.h"
#include "stdafx.h"


#include <boost/filesystem.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <set>

namespace bfs = boost::filesystem;



timeIndex::timeIndex()
{
    index = -1;
    timeStamp = clock();
}

timeIndex::timeIndex(int _index)
{
    index = _index;
    timeStamp = clock();
}

multiviewInputParams::multiviewInputParams(const std::string& file)
{
    initFromConfigFile(file);
}

imageParams multiviewInputParams::addImageFile(const std::string& filename)
{
    auto image = cvLoadImage(filename.c_str());
    if(!image)
        throw std::runtime_error(std::string("can't load image ") + filename);
    imageParams params(image->width, image->height);

    maxImageWidth = std::max(maxImageWidth, image->width);
    maxImageHeight = std::max(maxImageHeight, image->height);

    imps.push_back(params);
    cvReleaseImage(&image);
    return params;
}

void multiviewInputParams::initFromConfigFile(const std::string& iniFile)
{
    boost::property_tree::ini_parser::read_ini(iniFile, _ini);
    // debug, dump the read ini file to cout
    // boost::property_tree::write_ini(std::cout, _ini);

    // initialize directory names
    const auto rootPath = bfs::path(iniFile).parent_path().string() + "/";
    mvDir = rootPath;

    imageExt = _ini.get<std::string>("global.imgExt", imageExt);
    prefix = _ini.get<std::string>("global.prefix", prefix);

    usesil = _ini.get<bool>("global.use_silhouettes", usesil);
    int ncams = _ini.get<int>("global.ncams", 0);
    assert(ncams > 0);
    // Load image dimensions
    std::set<std::pair<int, int>> dimensions;
    {
        boost::optional<boost::property_tree::ptree&> cameras = _ini.get_child_optional("imageResolutions");
        if(!cameras)
        {
            std::cout << "No 'imageResolutions' section, load from image files." << std::endl;
            for(std::size_t i = 0; i < ncams; ++i)
            {
                const std::string filename = mv_getFileNamePrefix(this, static_cast<int>(i + 1)) + "." + imageExt;
                const imageParams params = addImageFile(filename);
                dimensions.emplace(params.width, params.height);
            }
        }
        else
        {
            for (const auto v : *cameras)
            {
                const std::string values = v.second.get_value<std::string>();
                std::vector<std::string> valuesVec;
                boost::split(valuesVec, values, boost::algorithm::is_any_of("x"));
                if(valuesVec.size() != 2)
                    throw std::runtime_error("Error when loading image sizes from INI file.");
                imageParams imgParams(boost::lexical_cast<int>(valuesVec[0]), boost::lexical_cast<int>(valuesVec[1]));
                imps.push_back(imgParams);
                maxImageWidth = std::max(maxImageWidth, imgParams.width);
                maxImageHeight = std::max(maxImageHeight, imgParams.height);

                dimensions.emplace(imgParams.width, imgParams.height);
                // std::cout << " * "  << v.first << ": " << imgParams.width << "x" << imgParams.height << std::endl;
            }
        }
    }
    if(getNbCameras() != ncams)
        throw std::runtime_error("Incoherent number of cameras.");
    std::cout << "Found " << dimensions.size() << " image dimension(s): " << std::endl;
    for(const auto& dim : dimensions)
        std::cout << " - [" << dim.first << "x" << dim.second << "]" << std::endl;
    std::cout << "Overall maximum dimension: [" << maxImageWidth << "x" << maxImageHeight << "]" << std::endl;
}

multiviewParams::multiviewParams(int _ncams, multiviewInputParams* _mip, float _simThr,
                                 staticVector<cameraMatrices>* cameras)
{
    mip = _mip;

    verbose = (bool)mip->_ini.get<bool>("global.verbose", true);
    bool doPrintfKRC = (bool)mip->_ini.get<bool>("global.doPrintfKRC", false);

    CUDADeviceNo = mip->_ini.get<int>("global.CUDADeviceNo", 0);

    minWinSizeHalf = 2;
    simThr = _simThr;

    resizeCams(_ncams);

    long t1 = initEstimate();
    for(int i = 0; i < ncams; i++)
    {
        indexes[i] = i + 1;
        FocK1K2Arr[i] = point3d(-1.0f, -1.0f, -1.0f);

        if(cameras != nullptr)
        {
            camArr[i] = (*cameras)[i].P;
            KArr[i] = (*cameras)[i].K;
            RArr[i] = (*cameras)[i].R;
            CArr[i] = (*cameras)[i].C;
            iKArr[i] = (*cameras)[i].iK;
            iRArr[i] = (*cameras)[i].iR;
            iCamArr[i] = (*cameras)[i].iCam;
            FocK1K2Arr[i] = point3d((*cameras)[i].f, (*cameras)[i].k1, (*cameras)[i].k2);
        }
        else
        {
            std::string fileNameP = mv_getFileName(mip, indexes[i], mip->MV_FILE_TYPE_P);
            std::string fileNameD = mv_getFileNamePrefix(mip, indexes[i]) + "_D.txt";
            loadCameraFile(i, fileNameP, fileNameD);
        }

        if(doPrintfKRC)
        {
            printf("----------------------\n");
            printf("K\n");
            KArr[i].doprintf();
            printf("iK\n");
            iKArr[i].doprintf();
            printf("R\n");
            RArr[i].doprintf();
            printf("iR\n");
            iRArr[i].doprintf();
            printf("cam\n");
            camArr[i].doprintf();
            printf("icam\n");
            iCamArr[i].doprintf();
            printf("C\n");
            CArr[i].doprintf();
        }

        if(KArr[i].m11 > (float)(mip->getWidth(i) * 100))
        {
            printf("\n\n\n\n\n\n WARNING camera %i at infinity ... settitng to zero\n", i);
            printf("\n K\n");
            KArr[i].doprintf();
            printf("\n R\n");
            RArr[i].doprintf();
            printf("\n iR\n");
            iRArr[i].doprintf();
            printf("\n iK\n");
            iKArr[i].doprintf();
            printf("\n P\n");
            camArr[i].doprintf();
            printf("\n iP\n");
            iCamArr[i].doprintf();
            printf("\n C\n");
            CArr[i].doprintf();

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

            camArr[i] = KArr[i] * (RArr[i] | (point3d(0.0, 0.0, 0.0) - RArr[i] * CArr[i]));
        }

        printfEstimate(i, ncams, t1);
    }
    finishEstimate();

    g_border = 10;
    g_maxPlaneNormalViewDirectionAngle = 70;
}


void multiviewParams::loadCameraFile(int i, const std::string& fileNameP, const std::string& fileNameD)
{
    // std::cout << "multiviewParams::loadCameraFile: " << fileNameP << std::endl;

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

void multiviewParams::addCam()
{
    ncams++;
    indexes.resize(ncams);
    camArr.resize(ncams);
    KArr.resize(ncams);
    iKArr.resize(ncams);
    RArr.resize(ncams);
    iRArr.resize(ncams);
    CArr.resize(ncams);
    iCamArr.resize(ncams);

    int i = ncams - 1;
    indexes[i] = i + 1;

    FILE* f;
    f = mv_openFile(mip, indexes[i], mip->MV_FILE_TYPE_P, "r");
    camArr[i] = load3x4MatrixFromFile(f);
    fclose(f);

    camArr[i].decomposeProjectionMatrix(KArr[i], RArr[i], CArr[i]);
    iKArr[i] = KArr[i].inverse();
    iRArr[i] = RArr[i].inverse();
    iCamArr[i] = iRArr[i] * iKArr[i];
}

void multiviewParams::reloadLastCam()
{
    int i = ncams - 1;

    FILE* f;
    f = mv_openFile(mip, indexes[i], mip->MV_FILE_TYPE_P, "r");
    camArr[i] = load3x4MatrixFromFile(f);
    fclose(f);

    camArr[i].decomposeProjectionMatrix(KArr[i], RArr[i], CArr[i]);
    iKArr[i] = KArr[i].inverse();
    iRArr[i] = RArr[i].inverse();
    iCamArr[i] = iRArr[i] * iKArr[i];
}

multiviewParams::~multiviewParams()
{
    mip = nullptr;
}

bool multiviewParams::is3DPointInFrontOfCam(const point3d* X, int rc) const
{
    point3d XT = camArr[rc] * (*X);

    return XT.z >= 0;
}

void multiviewParams::getPixelFor3DPoint(point2d* out, const point3d& X, int rc) const
{
    getPixelFor3DPoint(out, X, camArr[rc]);
}

void multiviewParams::getPixelFor3DPoint(point2d* out, const point3d& X, const matrix3x4& P) const
{
    point3d XT = P * X;

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

void multiviewParams::getPixelFor3DPoint(pixel* out, const point3d& X, int rc) const
{
    point3d XT = camArr[rc] * X;

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
float multiviewParams::getCamPixelSize(const point3d& x0, int cam) const
{
    point2d pix;
    getPixelFor3DPoint(&pix, x0, cam);
    pix.x = pix.x + 1.0;
    point3d vect = iCamArr[cam] * pix;

    vect = vect.normalize();
    return pointLineDistance3D(x0, CArr[cam], vect);
}

float multiviewParams::getCamPixelSize(const point3d& x0, int cam, float d) const
{
    if(d == 0.0f)
    {
        return 0.0f;
    }

    point2d pix;
    getPixelFor3DPoint(&pix, x0, cam);
    pix.x = pix.x + d;
    point3d vect = iCamArr[cam] * pix;

    vect = vect.normalize();
    return pointLineDistance3D(x0, CArr[cam], vect);
}

/**
* @brief Return the size of a pixel in space with an offset
* of "d" pixels in the target camera (along the epipolar line).
*/
float multiviewParams::getCamPixelSizeRcTc(const point3d& p, int rc, int tc, float d) const
{
    if(d == 0.0f)
    {
        return 0.0f;
    }

    point3d p1 = CArr[rc] + (p - CArr[rc]) * 0.1f;
    point2d rpix;
    getPixelFor3DPoint(&rpix, p, rc);

    point2d pFromTar, pToTar;
    getTarEpipolarDirectedLine(&pFromTar, &pToTar, rpix, rc, tc, this);
    // A vector of 1 pixel length on the epipolar line in tc camera
    // of the 3D point p projected in camera rc.
    point2d pixelVect = ((pToTar - pFromTar).normalize()) * d;
    // tpix is the point p projected in camera tc
    point2d tpix;
    getPixelFor3DPoint(&tpix, p, tc);
    // tpix1 is tpix with an offset of d pixels along the epipolar line
    point2d tpix1 = tpix + pixelVect * d;

    if(!triangulateMatch(p1, rpix, tpix1, rc, tc, this))
    {
        // Fallback to compute the pixel size using only the rc camera
        return getCamPixelSize(p, rc, d);
    }
    // Return the 3D distance between the original point and the newly triangulated one
    return (p - p1).size();
}

float multiviewParams::getCamPixelSizePlaneSweepAlpha(const point3d& p, int rc, int tc, int scale, int step) const
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

float multiviewParams::getCamPixelSizePlaneSweepAlpha(const point3d& p, int rc, staticVector<int>* tcams, int scale,
                                                      int step) const
{
    float av1 = 0.0f;
    float avmax = 0.0f;
    for(int c = 0; c < tcams->size(); c++)
    {
        float dpxs = getCamPixelSizePlaneSweepAlpha(p, rc, (*tcams)[c], scale, step);
        av1 += dpxs;
        avmax = std::max(avmax, dpxs);
    }
    av1 /= (float)(tcams->size());
    // return av1;
    return avmax;
}

int multiviewParams::getCamsMinPixelSizeIndex(const point3d& x0, int rc, seedPointCams* tcams) const
{
    int mini = -1;
    float minPixSize = getCamPixelSize(x0, rc);

    for(int ci = 0; ci < (int)tcams->size(); ci++)
    {
        float pixSize = getCamPixelSize(x0, (int)(*tcams)[ci]);
        if(minPixSize > pixSize)
        {
            minPixSize = pixSize;
            mini = ci;
        }
    }

    return mini;
}

int multiviewParams::getCamsMinPixelSizeIndex(const point3d& x0, const staticVector<int> &tcams) const
{
    int mini = 0;
    float minPixSize = getCamPixelSize(x0, tcams[0]);

    for(int ci = 1; ci < tcams.size(); ci++)
    {
        float pixSize = getCamPixelSize(x0, tcams[ci]);
        if(minPixSize > pixSize)
        {
            minPixSize = pixSize;
            mini = ci;
        }
    }

    return mini;
}

float multiviewParams::getCamsMinPixelSize(const point3d& x0, staticVector<int>& tcams) const
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

float multiviewParams::getCamsAveragePixelSize(const point3d& x0, staticVector<int>* tcams) const
{
    if(sizeOfStaticVector<int>(tcams) == 0)
    {
        return 0.0f;
    }
    float avPixSize = 1000000000.0;
    for(int ci = 0; ci < (int)tcams->size(); ci++)
    {
        avPixSize += getCamPixelSize(x0, (int)(*tcams)[ci]);
    }
    avPixSize /= (float)tcams->size();

    return avPixSize;
}

void multiviewParams::computeNormal(point3d& n, const rotation& rot, int refCam) const
{
    float crx, cry, srx, sry, rx, ry;
    rx = (float)rot.rx;
    ry = (float)rot.ry - 90.0;

    crx = cos(rx * (M_PI / 180.0));
    srx = sin(rx * (M_PI / 180.0));
    cry = cos(ry * (M_PI / 180.0));
    sry = sin(ry * (M_PI / 180.0));

    n.x = crx * cry;
    n.y = sry;
    n.z = -srx * cry;

    // n is a vector in the reference camera coordinate system we have to convert it to global coordinate system
    n = iRArr[refCam] * (n);
    n = n.normalize();
}

bool multiviewParams::isPixelInCutOut(const pixel* pix, const pixel* lu, const pixel* rd, int d, int camId) const
{
    return ((pix->x >= std::max(lu->x, d)) && (pix->x <= std::min(rd->x, mip->getWidth(camId) - 1 - d)) &&
            (pix->y >= std::max(lu->y, d)) && (pix->y <= std::min(rd->y, mip->getHeight(camId) - 1 - d)));
}

bool multiviewParams::isPixelInImage(const pixel& pix, int d, int camId) const
{
    return ((pix.x >= d) && (pix.x < mip->getWidth(camId) - d) && (pix.y >= d) && (pix.y < mip->getHeight(camId) - d));
}
bool multiviewParams::isPixelInImage(const pixel& pix, int camId) const
{
    return ((pix.x >= g_border) && (pix.x < mip->getWidth(camId) - g_border) && (pix.y >= g_border) &&
            (pix.y < mip->getHeight(camId) - g_border));
}

bool multiviewParams::isPixelInImage(const point2d& pix, int camId) const
{
    return isPixelInImage(pixel(pix), camId);
}

void multiviewParams::computeHomographyInductedByPlaneRcTc(matrix3x3* H, const point3d& _p, const point3d& _n, int rc,
                                                           int tc) const
{
    point3d _tl = point3d(0.0, 0.0, 0.0) - RArr[rc] * CArr[rc];
    point3d _tr = point3d(0.0, 0.0, 0.0) - RArr[tc] * CArr[tc];

    point3d p = RArr[rc] * (_p - CArr[rc]);
    point3d n = RArr[rc] * _n;
    n = n.normalize();
    float d = -dot(n, p);

    matrix3x3 Rr = RArr[tc] * RArr[rc].transpose();
    point3d tr = _tr - Rr * _tl;

    // hartley zisserman second edition p.327 (13.2)
    *H = (KArr[tc] * (Rr - outerMultiply(tr, n / d))) * iKArr[rc];
}

void multiviewParams::decomposeProjectionMatrix(point3d& Co, matrix3x3& Ro, matrix3x3& iRo, matrix3x3& Ko,
                                                matrix3x3& iKo, matrix3x3& iPo, const matrix3x4& P) const
{
    P.decomposeProjectionMatrix(Ko, Ro, Co);
    iKo = Ko.inverse();
    iRo = Ro.inverse();
    iPo = iRo * iKo;
}

