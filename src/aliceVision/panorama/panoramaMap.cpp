#include "panoramaMap.hpp"

#include <iostream>
#include <list>

namespace aliceVision
{

bool PanoramaMap::append(IndexT index, const BoundingBox & box)
{
    int maxFactor = pow(2, _scale);
    
    BoundingBox scaled = box.divide(maxFactor);

    BoundingBox scaledWithBorders = scaled.dilate(_borderSize);

    _map[index] = scaledWithBorders.multiply(maxFactor);
    _mapRaw[index] = box;

    return true;
}

bool PanoramaMap::intersect(const BoundingBox & box1, const BoundingBox & box2) const
{
    BoundingBox otherBbox = box2;
    BoundingBox otherBboxLoop = box2;
    otherBboxLoop.left = otherBbox.left - _panoramaWidth;
    BoundingBox otherBboxLoopRight = box2;
    otherBboxLoopRight.left = otherBbox.left + _panoramaWidth;

    if (!box1.intersectionWith(otherBbox).isEmpty()) 
    {
        /*BoundingBox sbox1 = box1.divide(_scale);
        BoundingBox sotherBbox = otherBbox.divide(_scale);
        BoundingBox bb = sbox1.intersectionWith(sotherBbox);
        BoundingBox bbb = bb.multiply(_scale);

        std::cout << bbb << std::endl;
        std::cout << "(" << box2 << ")" << std::endl;*/

        return true;
    }

    if (!box1.intersectionWith(otherBboxLoop).isEmpty()) 
    {
        /*BoundingBox sbox1 = box1.divide(_scale);
        BoundingBox sotherBbox = otherBboxLoop.divide(_scale);
        BoundingBox bb = sbox1.intersectionWith(sotherBbox);
        BoundingBox bbb = bb.multiply(_scale);

        std::cout << bbb << std::endl;
        std::cout << "(" << box2 << ")" << std::endl;*/

        return true;
    }

    if (!box1.intersectionWith(otherBboxLoopRight).isEmpty()) 
    {
        /*BoundingBox sbox1 = box1.divide(_scale);
        BoundingBox sotherBbox = otherBboxLoopRight.divide(_scale);
        BoundingBox bb = sbox1.intersectionWith(sotherBbox);
        BoundingBox bbb = bb.multiply(_scale);

        std::cout << bbb << std::endl;
        std::cout << "(" << box2 << ")" << std::endl;*/

        return true;
    }

    return false;
}

bool PanoramaMap::getOverlaps(std::list<IndexT> & overlaps, IndexT reference)
{
    if (_map.find(reference) == _map.end())
    {
        return false;
    }

    BoundingBox bbref = _map[reference];

    for (auto it : _map)
    {
        if (it.first == reference)
        {
            continue;
        }

        std::cout << it.first << " " << it.second  << " " << bbref << std::endl;

        if (intersect(bbref, it.second))
        {
            overlaps.push_back(it.first);
        }
    }

    return true;
}

bool PanoramaMap::getOverlaps(std::list<IndexT> & overlaps, BoundingBox bbref)
{


    for (auto it : _map)
    {
        if (intersect(bbref, it.second))
        {
            overlaps.push_back(it.first);
        }
    }

    return true;
}

}