// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <Eigen/Dense>

namespace aliceVision {
namespace fuseCut {

struct RayInfo
{
    IndexT start;
    IndexT end;
};

class Node 
{
public:
    using uptr = std::unique_ptr<Node>;
    using ptr = Node *;

public:
    Node(const Eigen::Vector3d & bbmin, const Eigen::Vector3d & bbmax)
    {
        _bbMin = bbmin;
        _bbMax = bbmax;

        //Enlarge intersection bbox
        Eigen::Vector3d center = (_bbMin + _bbMax) * 0.5;
        _intersectionBbMin = center + 1.0 * (_bbMin - center);
        _intersectionBbMax = center + 1.0 * (_bbMax - center);
    }

    double getSize()
    {
        Eigen::Vector3d rel = _bbMax - _bbMin;
        return rel.maxCoeff();
    }

    void storeRay(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const RayInfo & info)
    {
        //Check that the ray intersect this cell
        if (!intersect(start, end))
        {
            return;
        }        

        double size = getSize();
        if (size > _minSize)
        {
            subdivide();

            for (auto& item : _nodes)
            {
                if (item == nullptr)
                {
                    continue;
                }

                item->storeRay(start, end, info);
            }
        }
        else 
        {
            _rayInfos.push_back(info);
        }
    }

    bool isInside(const Eigen::Vector3d & pt)
    {
        return (pt.x() >= _intersectionBbMin.x() && 
                pt.y() >= _intersectionBbMin.y() && 
                pt.z() >= _intersectionBbMin.z() && 
                pt.x() <= _intersectionBbMax.x() && 
                pt.y() <= _intersectionBbMax.y() && 
                pt.z() <= _intersectionBbMax.z());
    }

    void visit(std::vector<Node::ptr> & list)
    {
        if (_rayInfos.size() > 0)
        {
            list.push_back(this);
        }

        for (auto &item : _nodes)
        {
            if (item)
            {
                item->visit(list);
            }
        }
    }

    bool intersect(const Eigen::Vector3d & start, const Eigen::Vector3d & end)
    {
        // x+lambda*dx>bbmin.x
        // y+lambda*dy>bbmin.y
        // z+lambda*dz>bbmin.z
        // x+lambda*dx<bbmax.x
        // y+lambda*dy<bbmax.y
        // z+lambda*dz<bbmax.z

        // lambda > (bbmin.x-x)/dx
        // lambda > (bbmin.y-y)/dy
        // lambda > (bbmin.z-z)/dz
        // lambda < (bbmax.x-x)/dx
        // lambda < (bbmax.y-y)/dy
        // lambda < (bbmax.z-z)/dz
        

        Eigen::Vector3d direction = end - start;

        double boundsMin = std::numeric_limits<double>::lowest();
        double boundsMax = std::numeric_limits<double>::max();

        if (std::abs(direction.x()) > 1e-12)
        {
            double lmin = (_intersectionBbMin.x() - start.x()) / direction.x();
            double lmax = (_intersectionBbMax.x() - start.x()) / direction.x();

            if (direction.x() > 0.0)
            {
                boundsMin = std::max(boundsMin, lmin);
                boundsMax = std::min(boundsMax, lmax);
            }
            else
            {
                boundsMin = std::max(boundsMin, lmax);
                boundsMax = std::min(boundsMax, lmin);
            }
        }

        if (std::abs(direction.y()) > 1e-12)
        {
            double lmin = (_intersectionBbMin.y() - start.y()) / direction.y();
            double lmax = (_intersectionBbMax.y() - start.y()) / direction.y();

            if (direction.y() > 0.0)
            {
                boundsMin = std::max(boundsMin, lmin);
                boundsMax = std::min(boundsMax, lmax);
            }
            else
            {
                boundsMin = std::max(boundsMin, lmax);
                boundsMax = std::min(boundsMax, lmin);
            }
        }   

        if (std::abs(direction.z()) > 1e-12)
        {
            double lmin = (_intersectionBbMin.z() - start.z()) / direction.z();
            double lmax = (_intersectionBbMax.z() - start.z()) / direction.z();

            if (direction.z() > 0.0)
            {
                boundsMin = std::max(boundsMin, lmin);
                boundsMax = std::min(boundsMax, lmax);
            }
            else
            {
                boundsMin = std::max(boundsMin, lmax);
                boundsMax = std::min(boundsMax, lmin);
            }
        }  

        boundsMin = std::min(boundsMin, 1.0);
        boundsMax = std::min(boundsMax, 1.0);

        return (boundsMin < boundsMax);
    }

    void subdivide()
    {
        if (_nodes[0] != nullptr)
        {
            return;
        }

        Eigen::Vector3d center = (_bbMin + _bbMax) * 0.5;
        
        
        double xs[] = {_bbMin.x(), center.x(), _bbMax.x()};
        double ys[] = {_bbMin.y(), center.y(), _bbMax.y()};
        double zs[] = {_bbMin.z(), center.z(), _bbMax.z()};

        int pos = 0;
        for (int ix = 0; ix < 2; ix++)
        {
            for (int iy = 0; iy < 2; iy++)
            {
                for (int iz = 0; iz < 2; iz++)
                {
                    Eigen::Vector3d lmin;
                    lmin.x() = xs[ix];
                    lmin.y() = ys[iy];
                    lmin.z() = zs[iz];

                    Eigen::Vector3d lmax;
                    lmax.x() = xs[ix + 1];
                    lmax.y() = ys[iy + 1];
                    lmax.z() = zs[iz + 1];
                    
                    _nodes[pos] = std::make_unique<Node>(lmin, lmax);
                    pos++;
                }
            }
        }
    }

private:
    std::array<Node::uptr, 8> _nodes;

    Eigen::Vector3d _bbMin;
    Eigen::Vector3d _bbMax;
    Eigen::Vector3d _intersectionBbMin;
    Eigen::Vector3d _intersectionBbMax;

    std::vector<RayInfo> _rayInfos;
private:
    double _minSize = 10.0;
};

}
}