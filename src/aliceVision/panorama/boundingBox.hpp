#pragma once

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

    int getRight() const { return left + width - 1; }

    int getBottom() const { return top + height - 1; }

    void snapToGrid(uint32_t gridSize)
    {

        int right = getRight();
        int bottom = getBottom();

        int leftBounded = int(floor(double(left) / double(gridSize))) * int(gridSize);
        int topBounded = int(floor(double(top) / double(gridSize))) * int(gridSize);
        int widthBounded = int(ceil(double(right - leftBounded + 1) / double(gridSize))) * int(gridSize);
        int heightBounded = int(ceil(double(bottom - topBounded + 1) / double(gridSize))) * int(gridSize);

        left = leftBounded;
        top = topBounded;
        width = widthBounded;
        height = heightBounded;
    }

    void unionWith(const BoundingBox& other)
    {

        if(left < 0 && top < 0)
        {
            left = other.left;
            top = other.top;
            width = other.width;
            height = other.height;
            return;
        }

        int rt = getRight();
        int ro = other.getRight();
        int bt = getBottom();
        int bo = other.getBottom();

        left = std::min(left, other.left);
        top = std::min(top, other.top);

        int maxr = std::max(rt, ro);
        int maxb = std::max(bt, bo);

        width = maxr - left + 1;
        height = maxb - top + 1;
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

    BoundingBox doubleSize()
    {
        BoundingBox b;

        b.left = left * 2;
        b.top = top * 2;
        b.width = width * 2;
        b.height = height * 2;

        return b;
    }
};
