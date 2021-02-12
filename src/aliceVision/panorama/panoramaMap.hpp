#pragma once

#include "boundingBox.hpp"

#include <aliceVision/types.hpp>

#include <list>
#include <map>

namespace aliceVision
{


class PanoramaMap
{
public:
    PanoramaMap(int width, int height, int scale, int borderSize) : 
    _panoramaWidth(width), 
    _panoramaHeight(height),
    _scale(scale),
    _borderSize(borderSize)
    {
    }

    bool append(IndexT index, const BoundingBox & box);

    bool getOverlaps(std::vector<IndexT> & overlaps, IndexT reference) const;

    bool getOverlaps(std::vector<IndexT> & overlaps, const BoundingBox & referenceBoundingBox) const;

    int getWidth() const
    {
        return _panoramaWidth;
    }

    int getHeight() const
    {
        return _panoramaHeight;
    }

    size_t getScale() const
    {
        return _scale;
    }

    size_t getBorderSize() const
    {
        return _borderSize;
    }

    bool getBoundingBox(BoundingBox & bb, const IndexT & id) const
    {
        if (_map.find(id) == _map.end()) 
        {
            return false;
        }

        bb = _map.at(id);

        return true;
    }

    bool getIntersectionsList(std::vector<BoundingBox> & intersections, std::vector<BoundingBox> & currentBoundingBoxes, const IndexT & referenceIndex, const IndexT & otherIndex) const;
    
    bool getIntersectionsList(std::vector<BoundingBox> & intersections, std::vector<BoundingBox> & currentBoundingBoxes, const BoundingBox & referenceBoundingBox, const IndexT & otherIndex) const;

    bool optimizeChunks(std::vector<std::vector<IndexT>> & chunks, int chunkSize);

private:
    bool intersect(const BoundingBox & box1, const BoundingBox & box2) const;

private:
    std::map<IndexT, BoundingBox> _map;

    int _panoramaWidth;
    int _panoramaHeight;
    int _scale;
    int _borderSize;
};

}