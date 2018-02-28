// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/SeedPointCams.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/structures.hpp>

#include <boost/property_tree/ptree.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace mvsUtils {

enum class EFileType {
    P = 0,
    K = 1,
    iK = 2,
    R = 3,
    iR = 4,
    C = 5,
    iP = 6,
    har = 7,
    prematched = 8,
    seeds = 9,
    growed = 10,
    op = 11,
    occMap = 12,
    wshed = 13,
    nearMap = 14,
    seeds_prm = 15,
    seeds_flt = 16,
    img = 17,
    seeds_seg = 18,
    graphCutMap = 20,
    graphCutPts = 21,
    growedMap = 22,
    agreedMap = 23,
    agreedPts = 24,
    refinedMap = 25,
    seeds_sfm = 26,
    radial_disortion = 27,
    graphCutMesh = 28,
    agreedMesh = 29,
    nearestAgreedMap = 30,
    segPlanes = 31,
    agreedVisMap = 32,
    diskSizeMap = 33,
    imgT = 34,
    depthMap = 35,
    simMap = 36,
    mapPtsTmp = 37,
    depthMapInfo = 38,
    camMap = 39,
    mapPtsSimsTmp = 40,
    nmodMap = 41,
    D = 42,
};

struct MultiViewInputParams
{
    static const int N_SCALES = 4;
    std::string newDir;
    /// prepareDenseScene data
    std::string mvDir;
    /// depthMapEstimate data folder
    std::string _depthMapFolder;
    /// depthMapFilter data folder
    std::string _depthMapFilterFolder;
    std::string outDir;
    std::string prefix;
    std::string imageExt = ".exr";

    int maxImageWidth = 0;
    int maxImageHeight = 0;
    bool usesil = false;
    boost::property_tree::ptree _ini;

    MultiViewInputParams() = default;
    explicit MultiViewInputParams(const std::string& file, const std::string& depthMapFolder, const std::string& depthMapFilterFolder);

    void initFromConfigFile(const std::string& iniFile);

    inline int getViewId(int index) const
    {
        return _imagesParams.at(index).viewId;
    }

    inline int getWidth(int index) const
    {
        return _imagesParams.at(index).width;
    }

    inline int getHeight(int index) const
    {
        return _imagesParams.at(index).height;
    }

    inline int getSize(int index) const
    {
        return _imagesParams.at(index).size;
    }

    inline int getMaxImageWidth() const
    {
        return maxImageWidth;
    }

    inline int getMaxImageHeight() const
    {
        return maxImageHeight;
    }

    inline int getNbCameras() const
    {
        return _imagesParams.size();
    }

private:
    std::vector<imageParams> _imagesParams;
};


class MultiViewParams
{
public:
    std::vector<Matrix3x4> camArr;
    std::vector<Matrix3x3> KArr;
    std::vector<Matrix3x3> iKArr;
    std::vector<Matrix3x3> RArr;
    std::vector<Matrix3x3> iRArr;
    std::vector<Point3d> CArr;
    std::vector<Matrix3x3> iCamArr;
    std::vector<Point3d> FocK1K2Arr;

    MultiViewInputParams* mip;

    int ncams;
    int CUDADeviceNo;
    float simThr;
    int g_border;
    int g_maxPlaneNormalViewDirectionAngle;
    bool verbose;

    MultiViewParams(int _ncams, MultiViewInputParams* _mip, float _simThr,
                    StaticVector<CameraMatrices>* cameras = nullptr);
    ~MultiViewParams();

    void resizeCams(int _ncams)
    {
        ncams = _ncams;
        camArr.resize(ncams);
        KArr.resize(ncams);
        iKArr.resize(ncams);
        RArr.resize(ncams);
        iRArr.resize(ncams);
        CArr.resize(ncams);
        iCamArr.resize(ncams);
        FocK1K2Arr.resize(ncams);
    }

    void loadCameraFile(int i, const std::string& fileNameP, const std::string& fileNameD);

    bool is3DPointInFrontOfCam(const Point3d* X, int rc) const;
    void getPixelFor3DPoint(Point2d* out, const Point3d& X, const Matrix3x4& P) const;
    void getPixelFor3DPoint(Point2d* out, const Point3d& X, int rc) const;
    void getPixelFor3DPoint(Pixel* out, const Point3d& X, int rc) const;
    float getCamPixelSize(const Point3d& x0, int cam) const;
    float getCamPixelSize(const Point3d& x0, int cam, float d) const;
    float getCamPixelSizeRcTc(const Point3d& p, int rc, int tc, float d) const;
    float getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, int tc, int scale, int step) const;
    float getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, StaticVector<int>* tcams, int scale, int step) const;

    float getCamsMinPixelSize(const Point3d& x0, std::vector<unsigned short>* tcams) const;
    float getCamsMinPixelSize(const Point3d& x0, StaticVector<int>& tcams) const;

    bool isPixelInImage(const Pixel& pix, int d, int camId) const;
    bool isPixelInImage(const Pixel& pix, int camId) const;
    bool isPixelInImage(const Point2d& pix, int camId) const;
    void decomposeProjectionMatrix(Point3d& Co, Matrix3x3& Ro, Matrix3x3& iRo, Matrix3x3& Ko, Matrix3x3& iKo,
                                   Matrix3x3& iPo, const Matrix3x4& P) const;

private:
    int winSizeHalf;
    int minWinSizeHalf;
};

} // namespace mvsUtils
} // namespace aliceVision
