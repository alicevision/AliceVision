#include "panoramaMap.hpp"

#include <iostream>
#include <list>

namespace aliceVision
{

bool PanoramaMap::append(IndexT index, const BoundingBox & box)
{
    BoundingBox bbox = box;
    
    // if the reference bounding box is looping on the right of the panorama, 
    // make it loop on the left instead to reduce the number of further possibilities
    if (bbox.getRight() >= _panoramaWidth && bbox.width < _panoramaWidth) 
    {
        bbox.left -= _panoramaWidth;    
    }

    _map[index] = bbox;

    return true;
}

bool PanoramaMap::intersect(const BoundingBox & box1, const BoundingBox & box2) const
{
    BoundingBox extentedBox1 = box1.divide(_scale).dilate(_borderSize).multiply(_scale);
    BoundingBox extentedBox2 = box2.divide(_scale).dilate(_borderSize).multiply(_scale);

    BoundingBox otherBbox = extentedBox2;
    BoundingBox otherBboxLoop = extentedBox2;
    otherBboxLoop.left = otherBbox.left - _panoramaWidth;
    BoundingBox otherBboxLoopRight = extentedBox2;
    otherBboxLoopRight.left = otherBbox.left + _panoramaWidth;

    if (!extentedBox1.intersectionWith(otherBbox).isEmpty()) 
    {
        return true;
    }

    if (!extentedBox1.intersectionWith(otherBboxLoop).isEmpty()) 
    {
        return true;
    }

    if (!extentedBox1.intersectionWith(otherBboxLoopRight).isEmpty()) 
    {
        return true;
    }

    return false;
}

bool PanoramaMap::getOverlaps(std::list<IndexT> & overlaps, IndexT reference) const
{
    if (_map.find(reference) == _map.end())
    {
        return false;
    }

    BoundingBox bbref = _map.at(reference);

    for (auto it : _map)
    {
        if (it.first == reference)
        {
            continue;
        }

        if (intersect(bbref, it.second))
        {
            overlaps.push_back(it.first);
        }
    }

    return true;
}

bool PanoramaMap::getIntersectionsList(std::vector<BoundingBox> & intersections, std::vector<BoundingBox> & currentBoundingBoxes, const IndexT & referenceIndex, const IndexT & otherIndex) const
{
    BoundingBox referenceBoundingBox = _map.at(referenceIndex);
    BoundingBox referenceBoundingBoxReduced = referenceBoundingBox.divide(_scale).dilate(_borderSize);

    
    BoundingBox otherBoundingBox = _map.at(otherIndex);

    // Base compare
    {
        BoundingBox otherBoundingBoxReduced = otherBoundingBox.divide(_scale).dilate(_borderSize);
        BoundingBox intersectionSmall = referenceBoundingBoxReduced.intersectionWith(otherBoundingBoxReduced);
        if (!intersectionSmall.isEmpty())
        {
            currentBoundingBoxes.push_back(otherBoundingBox);

            BoundingBox intersection = intersectionSmall.multiply(_scale);
            intersection = intersection.limitInside(otherBoundingBox);
            intersections.push_back(intersection);
        }
    }

    // Shift to check loop
    {
        BoundingBox otherBoundingBoxLoop = otherBoundingBox;
        otherBoundingBoxLoop.left -= _panoramaWidth;
        BoundingBox otherBoundingBoxReduced = otherBoundingBoxLoop.divide(_scale).dilate(_borderSize);
        BoundingBox intersectionSmall = referenceBoundingBoxReduced.intersectionWith(otherBoundingBoxReduced);
        if (!intersectionSmall.isEmpty())
        {
            currentBoundingBoxes.push_back(otherBoundingBoxLoop);
            
            BoundingBox intersection = intersectionSmall.multiply(_scale);
            intersection = intersection.limitInside(otherBoundingBoxLoop);
            intersections.push_back(intersection);
        }
    }

    // Shift to check loop
    {
        BoundingBox otherBoundingBoxLoop = otherBoundingBox;
        otherBoundingBoxLoop.left += _panoramaWidth;
        BoundingBox otherBoundingBoxReduced = otherBoundingBoxLoop.divide(_scale).dilate(_borderSize);
        BoundingBox intersectionSmall = referenceBoundingBoxReduced.intersectionWith(otherBoundingBoxReduced);
        if (!intersectionSmall.isEmpty())
        {
            currentBoundingBoxes.push_back(otherBoundingBoxLoop);
            
            BoundingBox intersection = intersectionSmall.multiply(_scale);
            intersection = intersection.limitInside(otherBoundingBoxLoop);
            intersections.push_back(intersection);
        }
    }

    return true;
}

}