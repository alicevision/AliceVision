#pragma once

#include "boundingBox.hpp"

#include <aliceVision/types.hpp>
#include <map>

namespace aliceVision
{

class BoundingBoxMap
{
public:
    BoundingBoxMap() = default;

    bool append(IndexT index, const BoundingBox & box);

    bool getIntersectionList(std::map<IndexT, BoundingBox> & intersections, IndexT reference) const;

private:
    std::map<IndexT, BoundingBox> _map;
};

}