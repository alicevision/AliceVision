#include "boundingBoxPanoramaMap.hpp"

#include <iostream>
#include <list>

namespace aliceVision
{



bool BoundingBoxPanoramaMap::append(IndexT index, const BoundingBox & box, int maxScale, int borderSize)
{
    int maxFactor = pow(2, maxScale);
    
    BoundingBox scaled = box.divide(maxFactor);

    BoundingBox scaledWithBorders = scaled.dilate(borderSize);

    _map[index] = scaledWithBorders.multiply(maxFactor);

    return true;
}

bool BoundingBoxPanoramaMap::intersect(const BoundingBox & box1, const BoundingBox & box2) const
{
    BoundingBox otherBbox = box2;
    BoundingBox otherBboxLoop = box2;
    otherBboxLoop.left = otherBbox.left - _panoramaWidth;
    BoundingBox otherBboxLoopRight = box2;
    otherBboxLoopRight.left = otherBbox.left + _panoramaWidth;

    if (!box1.intersectionWith(otherBbox).isEmpty()) 
    {
        return true;
    }

    if (!box1.intersectionWith(otherBboxLoop).isEmpty()) 
    {
        return true;
    }

    if (!box1.intersectionWith(otherBboxLoopRight).isEmpty()) 
    {
        return true;
    }

    return false;
}

bool BoundingBoxPanoramaMap::isValid(const std::list<stackItem> & stack) const 
{
    if (stack.size() == 0) 
    {
        return false;
    }
 
    if (stack.size() == 1)
    {
        return true;
    }

    auto & item = stack.back();
    IndexT top = *(item.second);
    BoundingBox topBb = _map.at(top);

    for (auto it : stack) 
    {
        IndexT current = *(it.second);
        if (current == top) continue;

        BoundingBox currentBb = _map.at(current);

        if (intersect(topBb, currentBb)) 
        {
            return false;
        }
    }

    return true;
}

bool BoundingBoxPanoramaMap::getBestInitial(std::list<IndexT> & best_items) const 
{

    //For each view, get the list of non overlapping views
    //with an id which is superior to them
    std::set<IndexT> initialList;
    std::map<IndexT, std::set<IndexT>> nonOverlapPerBox;
    for (auto it = _map.begin(); it != _map.end(); it++)
    {
        std::set<IndexT> listNonOverlap;

        initialList.insert(it->first);

        for (auto nextIt = std::next(it); nextIt != _map.end(); nextIt++)
        {
            if (intersect(it->second, nextIt->second)) 
            {
                continue;
            }

            listNonOverlap.insert(nextIt->first);
        }

        nonOverlapPerBox[it->first] = listNonOverlap;
    }
    nonOverlapPerBox[UndefinedIndexT] = initialList;

    size_t best_score = 0;
    best_items.clear();
    std::list<stackItem> stack;
    stack.push_back(std::make_pair(UndefinedIndexT, nonOverlapPerBox.at(UndefinedIndexT).begin()));

    while (!stack.empty())
    {
        if (isValid(stack))
        {
            std::list<IndexT> items;
            size_t score = 0;
            for (auto it : stack)
            {
                IndexT item;
                items.push_back(item);
                BoundingBox box = _map.at(item);
                score += box.area();
                if (score > best_score) 
                {
                    best_items = items;
                    best_score = score;
                }
            }

            IndexT id = *stack.back().second;
            std::set<IndexT> & addList = nonOverlapPerBox.at(id);

            if (!addList.empty())
            {
                stack.push_back(std::make_pair(id, addList.begin()));
                continue;
            }
        }
        
        while (!stack.empty())
        {
            //Iterate over the last list
            stackItem & lastItem = stack.back();
            uniqueIndices & list = nonOverlapPerBox.at(lastItem.first);
            lastItem.second++;
            
            //If we reached the end of the list
            //Pop back this list
            if (stack.back().second == list.end()) 
            {
                stack.pop_back();
            }
            else 
            {
                break;
            }
        }
    }

    return true;
}


}