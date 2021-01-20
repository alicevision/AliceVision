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

bool PanoramaMap::getOverlaps(std::vector<IndexT> & overlaps, IndexT reference) const
{
    if (_map.find(reference) == _map.end())
    {
        return false;
    }

    BoundingBox bbref = _map.at(reference);

    return getOverlaps(overlaps, bbref);
}

bool PanoramaMap::getOverlaps(std::vector<IndexT> & overlaps, const BoundingBox & referenceBoundingBox) const
{
    for (auto it : _map)
    {
        if (intersect(referenceBoundingBox, it.second))
        {
            overlaps.push_back(it.first);
        }
    }

    return true;
}


bool PanoramaMap::getIntersectionsList(std::vector<BoundingBox> & intersections, std::vector<BoundingBox> & currentBoundingBoxes, const IndexT & referenceIndex, const IndexT & otherIndex) const
{
    BoundingBox referenceBoundingBox = _map.at(referenceIndex);
    
    return getIntersectionsList(intersections, currentBoundingBoxes, referenceBoundingBox, otherIndex);
}

bool PanoramaMap::getIntersectionsList(std::vector<BoundingBox> & intersections, std::vector<BoundingBox> & currentBoundingBoxes, const BoundingBox & referenceBoundingBox, const IndexT & otherIndex) const
{
    BoundingBox referenceBoundingBoxReduced = referenceBoundingBox.divide(_scale).dilate(_borderSize);

    BoundingBox otherBoundingBox = _map.at(otherIndex);

    // Base compare
    {
        BoundingBox otherBoundingBoxReduced = otherBoundingBox.divide(_scale).dilate(_borderSize);
        BoundingBox intersectionSmall = referenceBoundingBoxReduced.intersectionWith(otherBoundingBoxReduced);
        if (!intersectionSmall.isEmpty())
        {
            BoundingBox intersection = intersectionSmall.multiply(_scale);
            intersection = intersection.limitInside(otherBoundingBox);

            if (!intersection.isEmpty()) 
            {
                intersections.push_back(intersection);
                currentBoundingBoxes.push_back(otherBoundingBox);
            }
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
            BoundingBox intersection = intersectionSmall.multiply(_scale);
            intersection = intersection.limitInside(otherBoundingBoxLoop);

            if (!intersection.isEmpty()) 
            {
                intersections.push_back(intersection);
                currentBoundingBoxes.push_back(otherBoundingBoxLoop);
            }
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
            BoundingBox intersection = intersectionSmall.multiply(_scale);
            intersection = intersection.limitInside(otherBoundingBoxLoop);

            if (!intersection.isEmpty()) 
            {
                intersections.push_back(intersection);
                currentBoundingBoxes.push_back(otherBoundingBoxLoop);
            }
        }
    }

    // Shift to check loop
    {
        BoundingBox otherBoundingBoxLoop = otherBoundingBox;
        otherBoundingBoxLoop.left += _panoramaWidth + _panoramaWidth;
        BoundingBox otherBoundingBoxReduced = otherBoundingBoxLoop.divide(_scale).dilate(_borderSize);
        BoundingBox intersectionSmall = referenceBoundingBoxReduced.intersectionWith(otherBoundingBoxReduced);
        if (!intersectionSmall.isEmpty())
        {            
            BoundingBox intersection = intersectionSmall.multiply(_scale);
            intersection = intersection.limitInside(otherBoundingBoxLoop);

            if (!intersection.isEmpty()) 
            {
                intersections.push_back(intersection);
                currentBoundingBoxes.push_back(otherBoundingBoxLoop);
            }
        }
    }

    return true;
}

bool PanoramaMap::optimizeChunks(std::vector<std::vector<IndexT>> & chunks, int chunkSize) {

    int countViews = _map.size();
    int countChunks =  int(std::ceil(double(countViews) / double(chunkSize)));

    chunks.clear();
    chunks.resize(countChunks);

    for (auto item : _map) 
    {
        std::sort(chunks.begin(), chunks.end(), 
            [this](const std::vector<IndexT> & first, const std::vector<IndexT> & second)
            {
                size_t size_first = 0;
                for (int i = 0; i < first.size(); i++) 
                {
                    IndexT curIndex = first[i];
                    size_first += _map.at(curIndex).area();
                }

                size_t size_second = 0;
                for (int i = 0; i < second.size(); i++) 
                {
                    IndexT curIndex = second[i];
                    size_second += _map.at(curIndex).area();
                }

                return (size_first < size_second);
            }
        );

        chunks[0].push_back(item.first);
    }

    return true;
}

}