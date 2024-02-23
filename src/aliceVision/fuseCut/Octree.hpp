// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


#include <aliceVision/types.hpp>

#include <memory>
#include <Eigen/Dense>

namespace aliceVision {
namespace fuseCut {

struct RayInfo
{
    IndexT start;
    IndexT end;
};

class SimpleNode
{
  public:
    using uptr = std::unique_ptr<SimpleNode>;
    using ptr = SimpleNode*;

  public:
    SimpleNode(const Eigen::Vector3d& bbmin, const Eigen::Vector3d& bbmax)
    {
        _bbMin = bbmin;
        _bbMax = bbmax;
    }

    double getSize()
    {
        Eigen::Vector3d rel = _bbMax - _bbMin;
        return rel.maxCoeff();
    }

    void store(const Eigen::Vector3d& pt)
    {
        if (!isInside(pt))
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

                item->store(pt);
            }
        }
        else
        {
            _count++;
        }
    }

    bool isInside(const Eigen::Vector3d& pt) const
    {
        return (pt.x() >= _bbMin.x() && pt.y() >= _bbMin.y() && pt.z() >= _bbMin.z() && pt.x() <= _bbMax.x() && pt.y() <= _bbMax.y() &&
                pt.z() <= _bbMax.z());
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

                    _nodes[pos] = std::make_unique<SimpleNode>(lmin, lmax);
                    pos++;
                }
            }
        }
    }

    void visit(std::vector<SimpleNode::ptr>& list)
    {
        if (_count > 0)
        {
            list.push_back(this);
        }

        for (auto& item : _nodes)
        {
            if (item)
            {
                item->visit(list);
            }
        }
    }

    void regroup(size_t maxPointsPerNode)
    {
        size_t count = getCount();

        if (count == 0)
        {
            return;
        }

        if (count < maxPointsPerNode)
        {
            _count = count;

            for (auto& item : _nodes)
            {
                item.reset();
                item = nullptr;
            }

            return;
        }

        for (auto& item : _nodes)
        {
            if (item)
            {
                item->regroup(maxPointsPerNode);
            }
        }
    }

    const size_t getCount() const
    {
        size_t count = _count;

        for (const auto& item : _nodes)
        {
            if (item)
            {
                count += item->getCount();
            }
        }

        return count;
    }

    const Eigen::Vector3d& getBBMin() const { return _bbMin; }

    const Eigen::Vector3d& getBBMax() const { return _bbMax; }

  private:
    std::array<SimpleNode::uptr, 8> _nodes;
    Eigen::Vector3d _bbMin;
    Eigen::Vector3d _bbMax;
    size_t _count = 0;
    double _minSize = 2.0;
};

class Node
{
  public:
    using uptr = std::unique_ptr<Node>;
    using ptr = Node*;

  public:
    Node(const Eigen::Vector3d& bbmin, const Eigen::Vector3d& bbmax)
    {
        _bbMin = bbmin;
        _bbMax = bbmax;
    }

    double getSize()
    {
        Eigen::Vector3d rel = _bbMax - _bbMin;
        return rel.maxCoeff();
    }

    void storeRay(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const RayInfo& info)
    {
        // Check that the ray intersect this cell
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
            if (isInside(end))
                _rayInfos.push_back(info);
        }
    }

    bool isInside(const Eigen::Vector3d& pt) const
    {
        return (pt.x() >= _bbMin.x() && pt.y() >= _bbMin.y() && pt.z() >= _bbMin.z() && pt.x() <= _bbMax.x() && pt.y() <= _bbMax.y() &&
                pt.z() <= _bbMax.z());
    }

    void visit(std::vector<Node::ptr>& list)
    {
        if (_rayInfos.size() > 0)
        {
            list.push_back(this);
        }

        for (auto& item : _nodes)
        {
            if (item)
            {
                item->visit(list);
            }
        }
    }

    void getIntersect(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double& boundsMin, double& boundsMax) const
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

        boundsMin = std::numeric_limits<double>::lowest();
        boundsMax = std::numeric_limits<double>::max();

        if (std::abs(direction.x()) > 1e-12)
        {
            double lmin = (_bbMin.x() - start.x()) / direction.x();
            double lmax = (_bbMax.x() - start.x()) / direction.x();

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
            double lmin = (_bbMin.y() - start.y()) / direction.y();
            double lmax = (_bbMax.y() - start.y()) / direction.y();

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
            double lmin = (_bbMin.z() - start.z()) / direction.z();
            double lmax = (_bbMax.z() - start.z()) / direction.z();

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
    }

    bool intersect(const Eigen::Vector3d& start, const Eigen::Vector3d& end) const
    {
        double boundsMin, boundsMax;

        getIntersect(start, end, boundsMin, boundsMax);

        return (boundsMin < boundsMax);
    }

    bool getPointLeaving(const Eigen::Vector3d& start, const Eigen::Vector3d& end, Eigen::Vector3d& output) const
    {
        double boundsMin, boundsMax;

        getIntersect(start, end, boundsMin, boundsMax);

        output = start + boundsMax * (end - start);

        return (boundsMin < boundsMax);
    }

    bool intersectTriangle(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C) { return true; }

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

    const std::vector<RayInfo>& getRayInfos() { return _rayInfos; }

    const Eigen::Vector3d& getBBMin() const { return _bbMin; }

    const Eigen::Vector3d& getBBMax() const { return _bbMax; }

  private:
    std::array<Node::uptr, 8> _nodes;

    Eigen::Vector3d _bbMin;
    Eigen::Vector3d _bbMax;

    std::vector<RayInfo> _rayInfos;

  public:
    double _minSize = 40.0;
};

}  // namespace fuseCut
}  // namespace aliceVision