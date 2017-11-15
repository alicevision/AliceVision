// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_structures.hpp"

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



struct multiviewInputParams
{
    static const int MV_FILE_TYPE_P = 0;
    static const int MV_FILE_TYPE_K = 1;
    static const int MV_FILE_TYPE_iK = 2;
    static const int MV_FILE_TYPE_R = 3;
    static const int MV_FILE_TYPE_iR = 4;
    static const int MV_FILE_TYPE_C = 5;
    static const int MV_FILE_TYPE_iP = 6;
    static const int MV_FILE_TYPE_har = 7;
    static const int MV_FILE_TYPE_prematched = 8;
    static const int MV_FILE_TYPE_seeds = 9;
    static const int MV_FILE_TYPE_growed = 10;
    static const int MV_FILE_TYPE_op = 11;
    static const int MV_FILE_TYPE_occMap = 12;
    static const int MV_FILE_TYPE_wshed = 13;
    static const int MV_FILE_TYPE_nearMap = 14;
    static const int MV_FILE_TYPE_seeds_prm = 15;
    static const int MV_FILE_TYPE_seeds_flt = 16;
    static const int MV_FILE_TYPE_img = 17;
    static const int MV_FILE_TYPE_seeds_seg = 18;
    static const int MV_FILE_TYPE_graphCutMap = 20;
    static const int MV_FILE_TYPE_graphCutPts = 21;
    static const int MV_FILE_TYPE_growedMap = 22;
    static const int MV_FILE_TYPE_agreedMap = 23;
    static const int MV_FILE_TYPE_agreedPts = 24;
    static const int MV_FILE_TYPE_refinedMap = 25;
    static const int MV_FILE_TYPE_seeds_sfm = 26;
    static const int MV_FILE_TYPE_radial_disortion = 27;
    static const int MV_FILE_TYPE_graphCutMesh = 28;
    static const int MV_FILE_TYPE_agreedMesh = 29;
    static const int MV_FILE_TYPE_nearestAgreedMap = 30;
    static const int MV_FILE_TYPE_segPlanes = 31;
    static const int MV_FILE_TYPE_agreedVisMap = 32;
    static const int MV_FILE_TYPE_diskSizeMap = 33;
    static const int MV_FILE_TYPE_imgT = 34;
    static const int MV_FILE_TYPE_depthMap = 35;
    static const int MV_FILE_TYPE_simMap = 36;
    static const int MV_FILE_TYPE_mapPtsTmp = 37;
    static const int MV_FILE_TYPE_depthMapInfo = 38;
    static const int MV_FILE_TYPE_camMap = 39;
    static const int MV_FILE_TYPE_mapPtsSimsTmp = 40;
    static const int MV_FILE_TYPE_nmodMap = 41;

    static const int N_SCALES = 4;

    multiviewInputParams() = default;

    explicit multiviewInputParams(const std::string& file);

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
    std::string mvDir;
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
