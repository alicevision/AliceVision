// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once
#include "mv_point3d.hpp"

class matrix3x4;
class matrix3x3;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class double3d
{
public:
    union {
        struct
        {
            double x, y, z;
        };
        double m[3];
    };

    double3d();
    double3d(const double _x, const double _y, const double _z);
    double3d& operator=(const double3d& param);
    bool operator==(const double3d& param) const;
    double3d operator-(const double3d& _p) const;
    double3d operator-() const;
    double3d operator+(const double3d& _p) const;
    double3d operator*(const double d) const;
    double3d operator/(const double d) const;
    double3d normalize() const;

    friend double dot(const double3d& p1, const double3d& p2);
    friend double3d cross(const double3d& a, const double3d& b);

    double size() const;
    double size2() const;
    void doprintf() const;
    void saveToFile(const std::string& fileName) const;
    void loadFromFile(const std::string& fileName);

    friend double3d proj(double3d& e, double3d& a);
};

double3d operator*(const matrix3x4& M, const double3d& _p);
double3d operator*(const matrix3x3& M, const double3d& _p);

double3d point3d2double3d(const point3d& p);
