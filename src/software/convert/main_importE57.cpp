// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/fuseCut/Octree.hpp>
#include <aliceVision/fuseCut/InputSet.hpp>
#include <aliceVision/dataio/E57Reader.hpp>
#include <aliceVision/camera/camera.hpp>
#include <filesystem>

#include <boost/program_options.hpp>
#include "nanoflann.hpp"

#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

struct PointInfoVectorAdaptator
{
    using Derived = PointInfoVectorAdaptator;  //!< In this case the dataset class is myself.

    const std::vector<dataio::E57Reader::PointInfo>& _data;

    PointInfoVectorAdaptator(const std::vector<dataio::E57Reader::PointInfo>& data)
      : _data(data)
    {}

    inline const Derived& derived() const { return *static_cast<const Derived*>(this); }

    inline Derived& derived() { return *static_cast<Derived*>(this); }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return _data.size(); }

    inline double kdtree_get_pt(const size_t idx, int dim) const { return _data.at(idx).coords(dim); }

    // Let flann compute bbox
    template<class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
    {
        return false;
    }
};

using PointInfoKdTree =
  nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointInfoVectorAdaptator>, PointInfoVectorAdaptator, 3>;

template<typename DistanceType, typename IndexType = size_t>
class BestPointInRadius
{
  public:
    const DistanceType m_radius;
    const std::vector<dataio::E57Reader::PointInfo>& m_points;
    const std::map<int, Eigen::Vector3d>& m_cameras;

    size_t m_result = 0;
    const int m_i;
    bool found = false;

    BestPointInRadius(DistanceType radius_,
                      const std::vector<dataio::E57Reader::PointInfo>& points,
                      const std::map<int, Eigen::Vector3d>& cameras,
                      int i)
      : m_radius(radius_),
        m_points(points),
        m_cameras(cameras),
        m_i(i)
    {
        init();
    }

    inline void init() { clear(); }

    inline void clear() { m_result = 0; }

    inline size_t size() const { return m_result; }

    inline bool full() const { return found; }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are sufficient
     */
    inline bool addPoint(DistanceType dist, IndexType index)
    {
        if (index == m_i)
        {
            return true;
        }

        if (dist < m_radius)
        {
            Eigen::Vector3d cam_i = m_cameras.at(m_points[m_i].idMesh);
            Eigen::Vector3d cam_index = m_cameras.at(m_points[index].idMesh);

            double distcam_i = (cam_i - m_points[m_i].coords).norm();
            double distcam_index = (cam_index - m_points[index].coords).norm();

            ++m_result;
            if (distcam_index < distcam_i)
            {
                found = true;
                return false;
            }
        }
        return true;
    }

    // Upper bound on distance
    inline DistanceType worstDist() const { return m_radius; }
};

// convert from a SfMData format to another
int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::vector<std::string> e57filenames;
    std::string outputJsonFilename;
    double maxDensity = 0.0;
    double minIntensity = 0.03;
    size_t maxPointsPerBlock = 1000000;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::vector<std::string>>(&e57filenames)->multitoken()->required(),
         "Path to E57 files.")
        ("output,o", po::value<std::string>(&outputJsonFilename)->required(),
         "Path to the output JSON file.");
    
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("maxDensity", po::value<double>(&maxDensity)->default_value(maxDensity),
         "Ensure each point has no neighbour closer than maxDensity meters.")
        ("minIntensity", po::value<double>(&minIntensity)->default_value(minIntensity),
         "Minimal intensity required to use LiDAR measure.")
        ("maxPointsPerBlock", po::value<size_t>(&maxPointsPerBlock)->default_value(maxPointsPerBlock),
         "Maximal number of points per computation block.");
    // clang-format on

    CmdLine cmdline("AliceVision importE57");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    if (maxPointsPerBlock == 0)
    {
        maxPointsPerBlock = std::numeric_limits<size_t>::max();
    }

    sfmData::SfMData sfmData;

    // Create intrinsics
    auto cam = camera::createEquidistant(aliceVision::camera::EDISTORTION::DISTORTION_NONE, 1, 1, 1, 0, 0);
    sfmData.getIntrinsics().emplace(0, cam);

    // Create a reader using all files
    dataio::E57Reader reader(e57filenames);
    reader.setRequiredIntensity(minIntensity);

    // Retrieve all sensor positions
    ALICEVISION_LOG_INFO("Extracting sensors");
    Eigen::Vector3d sensorPosition;
    std::map<int, Eigen::Vector3d> cameras;
    while (reader.getNext(sensorPosition))
    {
        cameras[reader.getIdMesh()] = sensorPosition;

        // Create pose for sfmData
        const int idMesh = reader.getIdMesh();

        Eigen::Vector3d correctedSensorPosition;
        correctedSensorPosition.x() = sensorPosition.x();
        correctedSensorPosition.y() = -sensorPosition.z();
        correctedSensorPosition.z() = sensorPosition.y();

        geometry::Pose3 pose(Eigen::Matrix3d::Identity(), correctedSensorPosition);
        sfmData.getPoses().emplace(idMesh, pose);

        // Create view for sfmData
        sfmData::View::sptr view = std::make_shared<sfmData::View>("nopath", idMesh, 0, idMesh, 1, 1);
        sfmData.getViews().emplace(idMesh, view);
    }
    reader.reset();

    // Create buffers for reading
    std::vector<dataio::E57Reader::PointInfo> vertices;
    Eigen::Matrix<size_t, -1, -1> grid;

    ALICEVISION_LOG_INFO("Extracting Meshes");
    // Loop through all meshes
    std::vector<dataio::E57Reader::PointInfo> allVertices;
    while (reader.getNext(sensorPosition, vertices, grid))
    {
        const int idMesh = reader.getIdMesh();
        ALICEVISION_LOG_INFO("Extracting Mesh " << idMesh);

        PointInfoVectorAdaptator pointCloudRef(vertices);

        nanoflann::KDTreeSingleIndexAdaptorParams params(10, nanoflann::KDTreeSingleIndexAdaptorFlags::None, 0);
        PointInfoKdTree tree(3, pointCloudRef, params);

        ALICEVISION_LOG_INFO("Building tree");
        tree.buildIndex();
        ALICEVISION_LOG_INFO("Built tree");

        // Angular definition of a ray
        double angularRes = 2.0 * M_PI / std::max(grid.rows(), grid.cols());
        double cord = 2.0 * sin(angularRes * 0.5);
        double maxLength = 1.5 * maxDensity / cord;

        size_t originalSize = allVertices.size();

        std::vector<std::vector<dataio::E57Reader::PointInfo>> vec_allVertices(omp_get_max_threads());

// Loop trough all vertices
#pragma omp parallel for
        for (int vIndex = 0; vIndex < vertices.size(); ++vIndex)
        {
            bool found = true;

            // Check if the point is close to the sensor enough to be theorically close to a neighboor
            double length = (vertices[vIndex].coords - sensorPosition).norm();
            if (length < maxLength)
            {
                found = false;

                // Search in the vicinity if a better point exists
                const double radius = maxDensity;
                static const nanoflann::SearchParameters searchParams(0.f, false);
                BestPointInRadius<double, std::size_t> resultSet(radius * radius, vertices, cameras, vIndex);
                tree.findNeighbors(resultSet, vertices[vIndex].coords.data(), searchParams);

                if (!resultSet.found)
                {
                    found = true;
                }
            }

            if (found)
            {
                vec_allVertices[omp_get_thread_num()].push_back(vertices[vIndex]);
            }
        }

        for (const auto& item : vec_allVertices)
        {
            allVertices.insert(allVertices.end(), item.begin(), item.end());
        }

        ALICEVISION_LOG_INFO("Mesh has " << allVertices.size() - originalSize << " points");
    }

    {
        ALICEVISION_LOG_INFO("Building final point cloud");
        PointInfoVectorAdaptator pointCloudRef(allVertices);

        nanoflann::KDTreeSingleIndexAdaptorParams params(10, nanoflann::KDTreeSingleIndexAdaptorFlags::None, 0);
        PointInfoKdTree tree(3, pointCloudRef, params);

        ALICEVISION_LOG_INFO("Building tree");
        tree.buildIndex();
        ALICEVISION_LOG_INFO("Built tree");

        int num_threads = omp_get_max_threads();

        auto& landmarks = sfmData.getLandmarks();

        std::vector<sfmData::Landmarks> vec_landmarks(num_threads);

#pragma omp parallel for
        for (int vIndex = 0; vIndex < allVertices.size(); ++vIndex)
        {
            // Search in the vicinity if a better point exists
            const double radius = maxDensity;
            static const nanoflann::SearchParameters searchParams(0.001f, false);
            BestPointInRadius<double, std::size_t> resultSet(radius * radius, allVertices, cameras, vIndex);
            tree.findNeighbors(resultSet, allVertices[vIndex].coords.data(), searchParams);

            if (!resultSet.found)
            {
                auto& ls = vec_landmarks[omp_get_thread_num()];

                const auto& v = allVertices[vIndex];

                Eigen::Vector3d pt;
                pt.x() = v.coords.x();
                pt.y() = -v.coords.z();
                pt.z() = v.coords.y();

                sfmData::Observation obs(Vec2(0.0, 0.0), ls.size(), 1.0);
                sfmData::Landmark landmark(pt, feature::EImageDescriberType::SIFT);
                landmark.getObservations().emplace(v.idMesh, obs);
                ls.emplace(vIndex, landmark);
            }
        }

        for (const auto& item : vec_landmarks)
        {
            landmarks.insert(item.begin(), item.end());
        }

        ALICEVISION_LOG_INFO("Final point cloud has " << landmarks.size() << " points");
    }

    ALICEVISION_LOG_INFO("Get Final Global bounding box");
    Eigen::Vector3d bbmin, bbmax;
    sfmData.getBoundingBox(bbmin, bbmax);

    double sx = std::abs(bbmax.x() - bbmin.x());
    double sy = std::abs(bbmax.y() - bbmin.y());
    double sz = std::abs(bbmax.z() - bbmin.z());
    ALICEVISION_LOG_INFO("Global Bounding box: " << sx << " x " << sy << " x " << sz);

    fuseCut::SimpleNode octree(bbmin, bbmax);
    for (const auto& pt : sfmData.getLandmarks())
    {
        octree.store(pt.second.X);
    }

    // Now regroup cells as much as we can
    octree.regroup(maxPointsPerBlock);

    std::vector<fuseCut::SimpleNode::ptr> list;
    octree.visit(list);

    ALICEVISION_LOG_INFO("Generating " << list.size() << "sub regions");

    fuseCut::InputSet inputs;

    for (int id = 0; id < list.size(); id++)
    {
        const auto& item = list[id];

        Eigen::Vector3d bbmin = item->getBBMin();
        Eigen::Vector3d bbmax = item->getBBMax();

        // Add borders of 20 cm
        Eigen::Vector3d center = (bbmin + bbmax) * 0.2;
        bbmin -= Eigen::Vector3d::Ones() * 0.2;
        bbmax += Eigen::Vector3d::Ones() * 0.2;

        double sx = std::abs(bbmax.x() - bbmin.x());
        double sy = std::abs(bbmax.y() - bbmin.y());
        double sz = std::abs(bbmax.z() - bbmin.z());

        ALICEVISION_LOG_INFO("Local Bounding box: " << sx << " x " << sy << " x " << sz);

        sfmData::SfMData subSfmData(sfmData, bbmin, bbmax);

        ALICEVISION_LOG_INFO("local count : " << subSfmData.getLandmarks().size() << " points");

        std::filesystem::path p = outputJsonFilename;
        std::filesystem::path transformed = p.parent_path() / "sfm";
        transformed += "_";
        transformed += std::to_string(id);
        transformed += ".abc";

        fuseCut::Input input;
        input.bbMin = bbmin;
        input.bbMax = bbmax;
        input.sfmPath = transformed.string();

        ALICEVISION_LOG_INFO("Saving to " << input.sfmPath);
        sfmDataIO::save(subSfmData, input.sfmPath, sfmDataIO::ESfMData::ALL);

        inputs.push_back(input);
    }

    std::ofstream of(outputJsonFilename);
    boost::json::value jv = boost::json::value_from(inputs);
    of << boost::json::serialize(jv);
    of.close();

    return EXIT_SUCCESS;
}
