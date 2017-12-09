// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/mv_structures.hpp>

#include <boost/property_tree/ptree.hpp>

#include <vector>
#include <string>

struct timeIndex
{
    int index;
    long timeStamp;

    timeIndex();
    timeIndex(int _index);
};

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

struct multiviewInputParams
{
    static const int N_SCALES = 4;

    multiviewInputParams() = default;

    explicit multiviewInputParams(const std::string& file, const std::string& depthMapFolder, const std::string& depthMapFilterFolder);

    void initFromConfigFile(const std::string& iniFile);

    imageParams addImageFile(const std::string& filename);

    int getWidth(int rc) const { return imps[rc].width; }
    int getHeight(int rc) const { return imps[rc].height; }
    int getSize(int rc) const { return imps[rc].im_size; }
    int getMaxImageWidth() const { return maxImageWidth; }
    int getMaxImageHeight() const { return maxImageHeight; }
    int getNbCameras() const { return imps.size(); }
    int getNbPixelsFromAllCameras() const
    {
        int fullSize = 0;
        for(const auto& p: imps)
            fullSize += p.im_size;
        return fullSize;
    }

    std::string newDir;
    // int occMapScale;
    std::string mvDir; //< prepareDenseScene data
    std::string _depthMapFolder; //< depthMapEstimate data folder
    std::string _depthMapFilterFolder; //< depthMapFilter data folder
    std::string outDir;
    std::string prefix;
    std::string imageExt = "_c.png";

    std::vector<imageParams> imps;
    int maxImageWidth = 0;
    int maxImageHeight = 0;

    bool usesil = false;
    boost::property_tree::ptree _ini;
};


class multiviewParams
{
public:
    std::vector<timeIndex> mapiSort;

    std::vector<matrix3x4> camArr;
    std::vector<matrix3x3> KArr;
    std::vector<matrix3x3> iKArr;
    std::vector<matrix3x3> RArr;
    std::vector<matrix3x3> iRArr;
    std::vector<point3d> CArr;
    std::vector<matrix3x3> iCamArr;
    std::vector<short> indexes;
    std::vector<point3d> FocK1K2Arr;

    multiviewInputParams* mip;

    int ncams;
    int CUDADeviceNo;
    float simThr;
    int g_border;
    int g_maxPlaneNormalViewDirectionAngle;
    bool verbose;

    multiviewParams(int _ncams, multiviewInputParams* _mip, float _simThr,
                    staticVector<cameraMatrices>* cameras = nullptr);
    ~multiviewParams();

    void resizeCams(int _ncams)
    {
        ncams = _ncams;
        indexes.resize(ncams);
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

    void addCam();
    void reloadLastCam();

    bool is3DPointInFrontOfCam(const point3d* X, int rc) const;
    void getPixelFor3DPoint(point2d* out, const point3d& X, const matrix3x4& P) const;
    void getPixelFor3DPoint(point2d* out, const point3d& X, int rc) const;
    void getPixelFor3DPoint(pixel* out, const point3d& X, int rc) const;
    float getCamPixelSize(const point3d& x0, int cam) const;
    float getCamPixelSize(const point3d& x0, int cam, float d) const;
    float getCamPixelSizeRcTc(const point3d& p, int rc, int tc, float d) const;
    float getCamPixelSizePlaneSweepAlpha(const point3d& p, int rc, int tc, int scale, int step) const;
    float getCamPixelSizePlaneSweepAlpha(const point3d& p, int rc, staticVector<int>* tcams, int scale, int step) const;

    float getCamsMinPixelSize(const point3d& x0, std::vector<unsigned short>* tcams) const;
    int getCamsMinPixelSizeIndex(const point3d& x0, int rc, seedPointCams* tcams) const;
    int getCamsMinPixelSizeIndex(const point3d& x0, const staticVector<int>& tcams) const;

    float getCamsMinPixelSize(const point3d& x0, staticVector<int>& tcams) const;
    float getCamsAveragePixelSize(const point3d& x0, staticVector<int>* tcams) const;
    void computeNormal(point3d& n, const rotation& rot, int refCam) const;
    bool isPixelInCutOut(const pixel* pix, const pixel* lu, const pixel* rd, int d, int camId) const;
    bool isPixelInImage(const pixel& pix, int d, int camId) const;
    bool isPixelInImage(const pixel& pix, int camId) const;
    bool isPixelInImage(const point2d& pix, int camId) const;

    void computeHomographyInductedByPlaneRcTc(matrix3x3* H, const point3d& _p, const point3d& _n, int rc, int tc) const;

    void decomposeProjectionMatrix(point3d& Co, matrix3x3& Ro, matrix3x3& iRo, matrix3x3& Ko, matrix3x3& iKo,
                                   matrix3x3& iPo, const matrix3x4& P) const;

private:
    int winSizeHalf;
    int minWinSizeHalf;
};
