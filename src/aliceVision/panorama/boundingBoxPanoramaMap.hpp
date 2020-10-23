#pragma once

#include "boundingBox.hpp"

#include <aliceVision/types.hpp>

#include <list>
#include <map>

namespace aliceVision
{

class BoundingBoxPanoramaMap
{
public:
    using uniqueIndices = std::set<IndexT>;
    using stackItem = std::pair<IndexT, uniqueIndices::iterator>;
public:
    BoundingBoxPanoramaMap(int width, int height) : _panoramaWidth(width), _panoramaHeight(height) 
    {

    }

    bool append(IndexT index, const BoundingBox & box, int maxScale, int borderSize);
    bool getBestInitial(std::list<IndexT> & best_items) const;

private:
    bool intersect(const BoundingBox & box1, const BoundingBox & box2) const;
    bool isValid(const std::list<stackItem> & stack) const;

private:
    std::map<IndexT, BoundingBox> _map;
    int _panoramaWidth;
    int _panoramaHeight;
};

}