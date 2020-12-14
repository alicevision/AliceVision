#pragma once

#include <algorithm>
#include <stdint.h>
#include <cmath>
#include <iostream>

namespace aliceVision
{

struct BoundingBox
{

    int left;
    int top;
    int width;
    int height;

    BoundingBox()
    {
        left = -1;
        top = -1;
        width = 0;
        height = 0;
    };

    BoundingBox(int l, int t, int w, int h)
        : left(l)
        , top(t)
        , width(w)
        , height(h)
    {
    }

    int getRight() const { 
        return left + width - 1; 
    }

    int getBottom() const { 
        return top + height - 1; 
    }

    bool isEmpty() const {
        return (width <= 0 || height <= 0);
    }

    int area() const {
        return width * height;
    }

    void snapToGrid(uint32_t gridSize)
    {

        int right = getRight();
        int bottom = getBottom();

        int leftBounded = int(std::floor(double(left) / double(gridSize))) * int(gridSize);
        int topBounded = int(std::floor(double(top) / double(gridSize))) * int(gridSize);
        int widthBounded = int(std::ceil(double(right - leftBounded + 1) / double(gridSize))) * int(gridSize);
        int heightBounded = int(std::ceil(double(bottom - topBounded + 1) / double(gridSize))) * int(gridSize);

        left = leftBounded;
        top = topBounded;
        width = widthBounded;
        height = heightBounded;
    }

    BoundingBox unionWith(const BoundingBox& other) const
    {
        BoundingBox ret;

        if (left < 0 && top < 0)
        {
            ret.left = other.left;
            ret.top = other.top;
            ret.width = other.width;
            ret.height = other.height;
            return ret;
        }

        int rt = getRight();
        int ro = other.getRight();
        int bt = getBottom();
        int bo = other.getBottom();

        ret.left = std::min(left, other.left);
        ret.top = std::min(top, other.top);

        int maxr = std::max(rt, ro);
        int maxb = std::max(bt, bo);

        ret.width = maxr - ret.left + 1;
        ret.height = maxb - ret.top + 1;

        return ret;
    }

    BoundingBox intersectionWith(const BoundingBox& other) const
    {
        BoundingBox intersection;

        intersection.left = std::max(left, other.left);
        intersection.top = std::max(top, other.top);

        int right = std::min(getRight(), other.getRight());
        int bottom = std::min(getBottom(), other.getBottom());

        intersection.width = std::max(0, right - intersection.left + 1);
        intersection.height = std::max(0, bottom - intersection.top + 1);

        return intersection;
    }

    bool isInside(const BoundingBox& other) const 
    {

        if (other.left > left) return false;
        if (other.top > top) return false;

        if (other.getRight() < getRight()) return false;
        if (other.getBottom() < getBottom()) return false;

        return true;
    }

    BoundingBox dilate(int units) 
    {
        BoundingBox b;
        
        b.left = left - units;
        b.top = top - units;
        b.width = width + units * 2;
        b.height = height + units * 2;

        return b;
    }

    void clampLeft() 
    {
        if (left < 0) 
        {
            width += left;
            left = 0;
        }
    }

    void clampRight(int maxRight)
    {
        if (getRight() > maxRight)
        {
            int removal = getRight() - maxRight;
            width -= removal;
        }
    }

    void clampTop() 
    {
        if (top < 0) 
        {
            height += top;
            top = 0;
        }
    }

    void clampBottom(int maxBottom)
    {
        if (getBottom() > maxBottom)
        {
            int removal = getBottom() - maxBottom;
            height -= removal;
        }
    }

    BoundingBox doubleSize() const
    {
        BoundingBox b;

        b.left = left * 2;
        b.top = top * 2;
        b.width = width * 2;
        b.height = height * 2;

        return b;
    }

    BoundingBox multiply(int factor) const
    {
        BoundingBox b;

        b.left = left * factor;
        b.top = top * factor;
        b.width = width * factor;
        b.height = height * factor;

        return b;
    }

    BoundingBox divide(int factor) const
    {
        BoundingBox b;

        b.left = int(floor(double(left) / double(factor)));
        b.top = int(floor(double(top) / double(factor)));

        int right = int(ceil(double(getRight()) / double(factor)));
        int bottom = int(ceil(double(getBottom()) / double(factor)));

        b.width = right - b.left + 1;
        b.height = bottom - b.top + 1;

        return b;
    }
};

std::ostream& operator<<(std::ostream& os, const BoundingBox& in);

}