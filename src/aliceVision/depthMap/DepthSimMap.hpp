// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Universe.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/ROI.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>

struct float2;

namespace aliceVision {
namespace depthMap {

template <class Type, unsigned Dim>
class CudaDeviceMemoryPitched;

class DepthSim
{
public:
    union {
        struct
        {
            float depth, sim;
        };
        float m[2];
    };

    inline DepthSim()
    {
        depth = 0.0;
        sim = 0.0;
    }

    inline DepthSim(float _depth, float _sim)
    {
        depth = _depth;
        sim = _sim;
    }

    inline DepthSim& operator=(const DepthSim& v)
    {
        depth = v.depth;
        sim = v.sim;
        return *this;
    }

    inline DepthSim operator+(const DepthSim& v) const
    {
        DepthSim out;
        out.depth = depth + v.depth;
        out.sim = sim + v.sim;
        return out;
    }
    inline DepthSim operator-(const DepthSim& v) const
    {
        DepthSim out;
        out.depth = depth - v.depth;
        out.sim = sim - v.sim;
        return out;
    }
    inline DepthSim operator*(float v) const
    {
        DepthSim out;
        out.depth = depth * v;
        out.sim = sim * v;
        return out;
    }
    inline DepthSim operator/(float v) const
    {
        DepthSim out;
        out.depth = depth / v;
        out.sim = sim / v;
        return out;
    }
    inline bool operator<(const DepthSim& other) const
    {
        if(depth == other.depth)
            return sim < other.sim;
        return (depth < other.depth);
    }
};

/*
 * @class DepthSimMap
 * @brief Support class to maintain a depth/similarity buffer.
 */
class DepthSimMap
{
public:

    /**
     * @brief DepthSimMap constructor.
     * @param[in] rc the related R camera index
     * @param[in] mp the multi-view parameters
     * @param[in] scale the depth/sim map scale factor from the original R image
     * @param[in] step the depth/sim map step factor from the original R image
     */
    DepthSimMap(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step);

    /**
     * @brief DepthSimMap constructor.
     * @param[in] rc the related R camera index
     * @param[in] mp the multi-view parameters
     * @param[in] tileParams tile workflow parameters
     * @param[in] scale the depth/sim map scale factor from the original R image
     * @param[in] step the depth/sim map step factor from the original R image
     * @param[in] roi the 2d region of interest of the R image without any downscale apply
     */
    DepthSimMap(int rc, const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, int scale, int step, const ROI& roi);

    // default destructor
    ~DepthSimMap() = default;

    // public accessors

    inline int getRc() const { return _rc; }
    inline int getWidth() const { return _width; }
    inline int getHeight() const { return _height; }
    inline int getScale() const { return _scale; }
    inline int getStep() const { return _step; }

    inline const ROI& getRoi() const { return _roi; }
    inline ROI getDownscaledRoi() const { return downscaleROI(_roi, _mp.getProcessDownscale() * _scale * _step); }

    inline const DepthSim& getDepthSim(int x, int y) const { return _dsm[y * _width + x]; }
    inline DepthSim& getDepthSim(int x, int y) { return _dsm[y * _width + x]; }

    inline const StaticVector<DepthSim>& getData() const { return _dsm; }
    inline StaticVector<DepthSim>& getData() { return _dsm; }

    inline Point2d getCorrespondingImagePoint(int x, int y) const
    {
        Point2d imagePoint;
        imagePoint.x = _roi.x.begin / double(_mp.getProcessDownscale()) + (double(x) * _scale * _step);
        imagePoint.y = _roi.y.begin / double(_mp.getProcessDownscale()) + (double(y) * _scale * _step);
        return imagePoint;
    }
    
    // public methods

    /**
     * @brief Initialize with a given smaller depth/sim map.
     * @param[in] depthSimMap The smaller depth/sim map
     */
    void initFromSmaller(const DepthSimMap& depthSimMap);

    /**
     * @brief Get the max/min depth value.
     * @return Point2d(x = max, y = min)
     */
    Point2d getMaxMinDepth() const;

    /**
     * @brief Get the max/min similarity value.
     * @return Point2d(x = max, y = min)
     */
    Point2d getMaxMinSim() const;

    /**
     * @brief Get the percentile depth.
     * @param[in] percentile the percentile
     * @return percentile depth
     */
    float getPercentileDepth(float percentile) const;

    /**
     * @brief Get the depth map buffer without step computed from an interpolated subpart (based on the step).
     * @note the output depth map buffer is downscaled with the multi-view process downscale and the internal downscale.
     * @param[out] out_depthMap the output depth map
     */
    void getDepthMapStep1(StaticVector<float>& out_depthMap) const;

    /**
     * @brief Get the similarity map buffer without step computed from an interpolated subpart (based on the step).
     * @note the output similarity map buffer is downscaled with the multi-view process downscale and the internal downscale.
     * @param[out] out_simMap the output similarity map
     */
    void getSimMapStep1(StaticVector<float>& out_simMap) const;

    /**
     * @brief Get the depth map buffer .
     * @note the output depth map buffer is downscaled with the multi-view process downscale and the internal downscale/step.
     * @param[out] out_depthMap the output depth map
     */
    void getDepthMap(StaticVector<float>& out_depthMap) const;

    /**
     * @brief Get the similarity map buffer.
     * @note the output similarity map buffer is downscaled with the multi-view process downscale and the internal downscale/step.
     * @param[out] out_simMap the output similarity map
     */
    void getSimMap(StaticVector<float>& out_simMap) const;

    /**
     * @brief Copy depth/sim map buffer to an array in device memory. 
     * @param[out] out_depthSimMap_dmp the output depth/sim array in device memory
     */
    void copyTo(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp) const;
 
    /**
     * @brief Copy depth/sim map buffer from an array in device memory.
     * @param[in] in_depthSimMap_dmp the input depth/sim array in device memory
     */
    void copyFrom(const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp);

    /**
     * @brief Copy depth/sim map buffer from two arrays in device memory.
     * @param[in] in_depthMap_dmp the input depth array in device memory
     * @param[in] in_simMap_dmp the input similarity array in device memory
     */
    void copyFrom(const CudaDeviceMemoryPitched<float, 2>& in_depthMap_dmp, const CudaDeviceMemoryPitched<float, 2>& in_simMap_dmp);

    /**
     * @brief Save the depth/sim to a single image colored from a JetColorMap
     * @param[in] filename the output image filename
     * @param[in] simThr the filter similarity threshold
     */
    void saveToImage(const std::string& filename, float simThr) const;

    /**
     * @brief Save the depth map and the similarity map in two different images
     * @param[in] customSuffix the filename custom suffix
     * @param[in] useStep1 whether or not to use no step
     */
    void save(const std::string& customSuffix = "", bool useStep1 = false) const;

    /**
     * @brief Load the depth map and the similarity map from multple tile maps
     * @param[in] tileRoiList the tile region of interest list
     * @param[in] customSuffix the filename custom suffix
     */
    void loadFromTiles(const std::vector<ROI>& tileRoiList, const std::string& customSuffix = "");

private:

    // private methods

    /**
     * @brief Add the tile depth map and the tile similarity map with border weight
     * @param[in] roi the tile region of interest list
     * @param[in] customSuffix the filename custom suffix
     */
    void loadTileWeighted(const ROI& tileRoi, const std::string& customSuffix);

    // private members

    const mvsUtils::MultiViewParams& _mp;    // multi-view parameters
    const mvsUtils::TileParams _tileParams;  // tile workflow parameters
    const ROI _roi;                          // 2d region of interest of the original R image without any downscale apply
    const int _rc;                           // related R camera index
    const int _width;                        // depth/sim map width
    const int _height;                       // depth/sim map height
    const int _scale;                        // depth/sim map scale factor from the original R image
    const int _step;                         // depth/sim map step factor from the original R image
    StaticVector<DepthSim> _dsm;             // depth similarity map

};

} // namespace depthMap
} // namespace aliceVision
