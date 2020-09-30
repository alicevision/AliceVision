#pragma once


struct BoundingBox {
    int left;
    int top;
    int width;
    int height;

    BoundingBox() = default;

    BoundingBox(int l, int t, int w, int h) : left(l), top(t), width(w), height(h) {}

    int getRight() const {
        return left + width;
    }

    int getBottom() const {
        return top + height;
    }
};