// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/structures.hpp>

#include <boost/property_tree/ptree.hpp>

#include <string>
#include <vector>
#include <map>

namespace aliceVision {

namespace bpt = boost::property_tree;

namespace sfmData {
class SfMData;
}  // namespace sfmData

namespace mvsUtils {

enum class EFileType
{
    P = 0,
    K = 1,
    iK = 2,
    R = 3,
    iR = 4,
    C = 5,
    iP = 6,
    har = 7,
    prematched = 8,
    growed = 10,
    op = 11,
    occMap = 12,
    wshed = 13,
    nearMap = 14,
    img = 17,
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
    depthMapFiltered = 36,
    simMap = 37,
    simMapFiltered = 38,
    normalMap = 39,
    normalMapFiltered = 40,
    thicknessMap = 41,
    pixSizeMap = 42,
    mapPtsTmp = 43,
    camMap = 44,
    mapPtsSimsTmp = 45,
    nmodMap = 46,
    D = 47,
    volume = 48,
    volumeCross = 49,
    volumeTopographicCut = 50,
    stats9p = 51,
    tilePattern = 52,
    none = 9999
};

class MultiViewParams
{
  public:
    /// prepareDenseScene data
    std::string _imagesFolder;
    /// camera projection matrix P
    std::vector<Matrix3x4> camArr;
    /// camera intrinsics matrix K: [focalx skew u; 0 focaly v; 0 0 1]
    std::vector<Matrix3x3> KArr;
    /// inverse camera intrinsics matrix K
    std::vector<Matrix3x3> iKArr;
    /// camera rotation matrix R
    std::vector<Matrix3x3> RArr;
    /// inverse camera rotation matrix R
    std::vector<Matrix3x3> iRArr;
    /// camera position C in world coordinate system
    std::vector<Point3d> CArr;
    /// K * R inverse matrix
    std::vector<Matrix3x3> iCamArr;
    ///
    std::vector<Point3d> FocK1K2Arr;
    /// number of cameras
    int ncams;
    ///
    float simThr;
    int g_border = 2;
    bool verbose;

    boost::property_tree::ptree userParams;

    MultiViewParams(const sfmData::SfMData& sfmData,
                    const std::string& imagesFolder = "",
                    const std::string& depthMapsFolder = "",
                    const std::string& depthMapsFilterFolder = "",
                    bool readFromDepthMaps = false,
                    int downscale = 1);

    ~MultiViewParams();

    inline Point3d backproject(const int camIndex, const Point2d& pix, double depth) const
    {
        const Point3d p = CArr[camIndex] + (iCamArr[camIndex] * pix).normalize() * depth;
        return p;
    }
    inline const std::string& getImagePath(int index) const { return _imagesParams.at(index).path; }

    inline int getViewId(int index) const { return _imagesParams.at(index).viewId; }

    inline int getOriginalWidth(int index) const { return _imagesParams.at(index).width; }

    inline int getOriginalHeight(int index) const { return _imagesParams.at(index).height; }

    inline int getOriginalSize(int index) const { return _imagesParams.at(index).size; }

    inline int getWidth(int index) const { return _imagesParams.at(index).width / getDownscaleFactor(index); }

    inline int getHeight(int index) const { return _imagesParams.at(index).height / getDownscaleFactor(index); }

    inline int getSize(int index) const { return _imagesParams.at(index).size / getDownscaleFactor(index); }

    inline const std::vector<ImageParams>& getImagesParams() const { return _imagesParams; }

    inline const ImageParams& getImageParams(int i) const { return _imagesParams.at(i); }

    inline int getDownscaleFactor(int index) const { return _imagesScale.at(index) * _processDownscale; }

    inline int getProcessDownscale() const { return _processDownscale; }

    inline int getMaxImageOriginalWidth() const { return _maxImageWidth; }

    inline int getMaxImageOriginalHeight() const { return _maxImageHeight; }

    inline int getMaxImageWidth() const { return _maxImageWidth / getProcessDownscale(); }

    inline int getMaxImageHeight() const { return _maxImageHeight / getProcessDownscale(); }

    inline int getNbCameras() const { return _imagesParams.size(); }

    inline int getIndexFromViewId(IndexT viewId) const { return _imageIdsPerViewId.at(viewId); }

    inline float getMinViewAngle() const { return _minViewAngle; }

    inline float getMaxViewAngle() const { return _maxViewAngle; }

    inline std::vector<double> getOriginalP(int index) const
    {
        std::vector<double> p44;                  // projection matrix (4x4) scale 1
        const Matrix3x4& p34 = camArr.at(index);  // projection matrix (3x4) scale = getDownscaleFactor()
        const int downscale = getDownscaleFactor(index);
        p44.assign(p34.m, p34.m + 12);
        std::transform(p44.begin(), p44.begin() + 8, p44.begin(), [&](double p) { return p * downscale; });
        p44.push_back(0);
        p44.push_back(0);
        p44.push_back(0);
        p44.push_back(1);
        return p44;
    }

    inline const std::string& getDepthMapsFolder() const { return _depthMapsFolder; }

    inline const std::string& getDepthMapsFilterFolder() const { return _depthMapsFilterFolder; }

    inline const sfmData::SfMData& getInputSfMData() const { return _sfmData; }

    const std::map<std::string, std::string>& getMetadata(int index) const;

    bool is3DPointInFrontOfCam(const Point3d* X, int rc) const;

    void getPixelFor3DPoint(Point2d* out, const Point3d& X, const Matrix3x4& P) const;
    void getPixelFor3DPoint(Point2d* out, const Point3d& X, int rc) const;
    void getPixelFor3DPoint(Pixel* out, const Point3d& X, int rc) const;
    double getCamPixelSize(const Point3d& x0, int cam) const;
    double getCamPixelSize(const Point3d& x0, int cam, float d) const;
    double getCamPixelSizeRcTc(const Point3d& p, int rc, int tc, float d) const;
    double getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, int tc, int scale, int step) const;
    double getCamPixelSizePlaneSweepAlpha(const Point3d& p, int rc, StaticVector<int>* tcams, int scale, int step) const;

    double getCamsMinPixelSize(const Point3d& x0, std::vector<unsigned short>* tcams) const;
    double getCamsMinPixelSize(const Point3d& x0, const StaticVector<int>& tcams) const;

    bool isPixelInSourceImage(const Pixel& pixRC, int camId, int margin) const;
    bool isPixelInImage(const Pixel& pix, int camId, int margin) const;
    bool isPixelInImage(const Pixel& pix, int camId) const;
    bool isPixelInImage(const Point2d& pix, int camId) const;
    bool isPixelInImage(const Point2d& pix, int camId, int margin) const;
    void decomposeProjectionMatrix(Point3d& Co, Matrix3x3& Ro, Matrix3x3& iRo, Matrix3x3& Ko, Matrix3x3& iKo, Matrix3x3& iPo, const Matrix3x4& P)
      const;

    /**
     * @brief findCamsWhichIntersectsHexahedron
     * @param hexah 0-3 frontal face, 4-7 back face
     * @param minMaxDepthsFileName
     * @return
     */
    StaticVector<int> findCamsWhichIntersectsHexahedron(const Point3d hexah[8], const std::string& minMaxDepthsFileName) const;

    /**
     * @brief findCamsWhichIntersectsHexahedron
     * @param hexah 0-3 frontal face, 4-7 back face
     * @return
     */
    StaticVector<int> findCamsWhichIntersectsHexahedron(const Point3d hexah[8]) const;

    /**
     * @brief findNearestCamsFromLandmarks
     * @param rc
     * @param nbNearestCams
     * @return
     */
    StaticVector<int> findNearestCamsFromLandmarks(int rc, int nbNearestCams) const;

    /**
     * @brief Find nearest cameras for a given tile
     * @param[in] rc R camera id
     * @param[in] nbNearestCams maximum number of desired nearest cameras
     * @param[in] tCams a given list of pre-selected nearest cameras
     * @param[in] roi the tile 2d region of interest
     * @return nearest cameras list for the given tile
     */
    std::vector<int> findTileNearestCams(int rc, int nbNearestCams, const std::vector<int>& tCams, const ROI& roi) const;

    inline void setMinViewAngle(float minViewAngle) { _minViewAngle = minViewAngle; }

    inline void setMaxViewAngle(float maxViewAngle) { _maxViewAngle = maxViewAngle; }

  private:
    /// image params list (width, height, size)
    std::vector<ImageParams> _imagesParams;
    /// image id per view id
    std::map<IndexT, int> _imageIdsPerViewId;
    /// image scale list
    std::vector<int> _imagesScale;
    /// downscale apply to input images during process
    int _processDownscale = 1;
    /// maximum width
    int _maxImageWidth = 0;
    /// maximum height
    int _maxImageHeight = 0;
    /// depthMapEstimate data folder
    std::string _depthMapsFolder;
    /// depthMapFilter data folder
    std::string _depthMapsFilterFolder;
    /// use silhouettes
    bool _useSil = false;
    /// minimum view angle
    float _minViewAngle = 2.0f;
    /// maximum view angle
    float _maxViewAngle = 70.0f;  // WARNING: may be too low, especially when using seeds from SfM
    /// input sfmData
    const sfmData::SfMData& _sfmData;

    void loadMatricesFromTxtFile(int index, const std::string& fileNameP, const std::string& fileNameD);
    void loadMatricesFromRawProjectionMatrix(int index, const double* rawProjMatix);
    void loadMatricesFromSfM(int index);

    inline void resizeCams(int _ncams)
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
        _imagesScale.resize(ncams, 1);
    }
};

}  // namespace mvsUtils
}  // namespace aliceVision
