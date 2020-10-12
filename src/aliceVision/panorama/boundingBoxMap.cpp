#include "boundingBoxMap.hpp"

#include <iostream>

namespace aliceVision
{

bool BoundingBoxMap::append(IndexT index, const BoundingBox & box)
{
    _map[index] = box;

    return true;
}

bool BoundingBoxMap::getIntersectionList(std::map<IndexT, BoundingBox> & intersections, IndexT reference) const
{   
    auto it = _map.find(reference);

    if (it == _map.end()) 
    {
        return false;
    }

    intersections.clear();

    BoundingBox refBB = it->second;
    if (refBB.isEmpty()) 
    {
        return false;
    }

    for (auto item : _map)
    {
        if (item.first == reference)
        {
            continue;
        }

        BoundingBox intersection = refBB.intersectionWith(item.second);
        if (intersection.isEmpty()) 
        {
            continue;
        }

        intersections[item.first] = intersection;
    }

    return true;
}


}