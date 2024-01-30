// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>

#include <Eigen/Dense>

#include <E57SimpleData.h>
#include <E57SimpleReader.h>

namespace aliceVision {
namespace dataio {

class E57Reader
{
public: 
    struct PointInfo
    {
        unsigned short idMesh;
        Eigen::Vector3d coords;
        float intensity;
    };

public:
    E57Reader(const std::vector<std::string> & paths)
    : 
    _paths(paths),
    _idPath(-1),
    _idMesh(-1),
    _countMeshesForFile(0)
    {

    }

    void reset()
    {
        _idPath = -1;
        _idMesh = -1;
        _countMeshesForFile = 0;
    }

    bool getNext(Eigen::Vector3d & sensorPosition, std::vector<PointInfo> & vertices, Eigen::Matrix<size_t, -1, -1> & grid)
    {
        // Go to next mesh of current file
        _idMesh++;

        // Load next file if needed
        if (_idMesh >= _countMeshesForFile)
        {
            _idPath++;
            _idMesh = 0;
            if (_idPath >= _paths.size())
            {
                return false;
            }

            // Create reader
            _reader = std::make_unique<e57::Reader>(_paths[_idPath], e57::ReaderOptions());
            if (!_reader->IsOpen())
            {
                return false;
            }

            e57::E57Root root;
            if (!_reader->GetE57Root(root))
            {
                return false;
            }

            // Compute number of meshes in file
            _countMeshesForFile = _reader->GetData3DCount();
        }

        if (_idMesh >= _countMeshesForFile)
        {
            return false;
        }

        if (!_reader)
        {
            return false;
        }

        // Get header
        e57::Data3D scanHeader;
        if (!_reader->ReadData3D(_idMesh, scanHeader))
        {
            ALICEVISION_LOG_ERROR("Error reading mesh #" << _idMesh);
            return false;
        }

        // Get sensor pose (worldTsensor)
        Eigen::Quaternion<double> q(scanHeader.pose.rotation.w, 
                                    scanHeader.pose.rotation.x, 
                                    scanHeader.pose.rotation.y, 
                                    scanHeader.pose.rotation.z);

        Eigen::Matrix3d R = q.normalized().toRotationMatrix();

        Eigen::Vector3d t;
        t(0) = scanHeader.pose.translation.x;
        t(1) = scanHeader.pose.translation.y;
        t(2) = scanHeader.pose.translation.z;

        int64_t maxRows = 0;
        int64_t maxColumns = 0;
        int64_t countPoints = 0;
        int64_t countGroups = 0;
        int64_t maxGroupSize = 0;
        bool isColumnIndex;

        if (!_reader->GetData3DSizes(0, maxRows, maxColumns, countPoints, countGroups, maxGroupSize, isColumnIndex))
        {
            ALICEVISION_LOG_ERROR("Error reading content of mesh #" << _idMesh);
            return false;
        }

        e57::Data3DPointsFloat data3DPoints(scanHeader);
        e57::CompressedVectorReader datareader = _reader->SetUpData3DPointsData(_idMesh, countPoints, data3DPoints);

        // Prepare list of vertices
        vertices.clear();
        vertices.reserve(countPoints);

        //Prepare structured grid for sensor
        grid.resize(maxRows, maxColumns);
        grid.fill(std::numeric_limits<size_t>::max());

        unsigned readCount = 0;
        while ((readCount = datareader.read()) > 0)
        {
            // Check input compatibility
            if (data3DPoints.sphericalRange != nullptr)
            {
                ALICEVISION_LOG_ERROR("Data contains spherical coordinates, this is not currently supported");
                continue;
            }

            if (data3DPoints.cartesianX == nullptr)
            {
                ALICEVISION_LOG_ERROR("Data contains no cartesian coordinates");
                continue;
            }

            if (data3DPoints.columnIndex == nullptr)
            {
                ALICEVISION_LOG_ERROR("Data contains no 2d column coordinates");
                continue;
            }

            if (data3DPoints.rowIndex == nullptr)
            {
                ALICEVISION_LOG_ERROR("Data contains no 2d row coordinates");
                continue;
            }

            if (data3DPoints.intensity == nullptr)
            {
                ALICEVISION_LOG_ERROR("Data contains no intensities");
                continue;
            }

            for (int pos = 0; pos < readCount; pos++)
            {
                if (data3DPoints.cartesianInvalidState[pos])
                {
                    continue;
                }

                Eigen::Vector3d pt;
                pt(0) = data3DPoints.cartesianX[pos];
                pt(1) = data3DPoints.cartesianY[pos];
                pt(2) = data3DPoints.cartesianZ[pos];

                // Transform point in the world frame
                PointInfo pi;
                pi.coords = (R * pt + t);
                pi.idMesh = _idMesh;
                pi.intensity = data3DPoints.intensity[pos],

                grid(data3DPoints.rowIndex[pos], data3DPoints.columnIndex[pos]) = vertices.size();
                vertices.push_back(pi);
            }
        }

        sensorPosition = t;

        return true;
    }

    bool getNext(Eigen::Vector3d & sensorPosition)
    {
        // Go to next mesh of current file
        _idMesh++;

        // Load next file if needed
        if (_idMesh >= _countMeshesForFile)
        {
            _idPath++;
            _idMesh = 0;
            if (_idPath >= _paths.size())
            {
                return false;
            }

            // Create reader
            _reader = std::make_unique<e57::Reader>(_paths[_idPath], e57::ReaderOptions());
            if (!_reader->IsOpen())
            {
                return false;
            }

            e57::E57Root root;
            if (!_reader->GetE57Root(root))
            {
                return false;
            }

            // Compute number of meshes in file
            _countMeshesForFile = _reader->GetData3DCount();
        }

        if (_idMesh >= _countMeshesForFile)
        {
            return false;
        }

        if (!_reader)
        {
            return false;
        }

        // Get header
        e57::Data3D scanHeader;
        if (!_reader->ReadData3D(_idMesh, scanHeader))
        {
            ALICEVISION_LOG_ERROR("Error reading mesh #" << _idMesh);
            return false;
        }

        // Get sensor pose (worldTsensor)
        Eigen::Quaternion<double> q(scanHeader.pose.rotation.w, 
                                    scanHeader.pose.rotation.x, 
                                    scanHeader.pose.rotation.y, 
                                    scanHeader.pose.rotation.z);

        Eigen::Matrix3d R = q.normalized().toRotationMatrix();

        Eigen::Vector3d t;
        t(0) = scanHeader.pose.translation.x;
        t(1) = scanHeader.pose.translation.y;
        t(2) = scanHeader.pose.translation.z;

        sensorPosition = t;

        return true;
    }

    int getIdMesh()
    {
        return _idMesh;
    }

private:
    std::vector<std::string> _paths;
    std::unique_ptr<e57::Reader> _reader;

    int _idPath;
    int _idMesh;
    size_t _countMeshesForFile;
};

}  // namespace dataio
}  // namespace aliceVision
