// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/colorize.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/fuseCut/LargeScale.hpp>
#include <aliceVision/fuseCut/ReconstructionPlan.hpp>
#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <Eigen/Geometry>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <cmath>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 4
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;

enum EPartitioningMode
{
    ePartitioningUndefined = 0,
    ePartitioningSingleBlock = 1,
    ePartitioningAuto = 2,
};

EPartitioningMode EPartitioning_stringToEnum(const std::string& s)
{
    if(s == "singleBlock")
        return ePartitioningSingleBlock;
    if(s == "auto")
        return ePartitioningAuto;
    return ePartitioningUndefined;
}

inline std::istream& operator>>(std::istream& in, EPartitioningMode& out_mode)
{
    std::string s;
    in >> s;
    out_mode = EPartitioning_stringToEnum(s);
    return in;
}

enum ERepartitionMode
{
    eRepartitionUndefined = 0,
    eRepartitionMultiResolution = 1,
};

ERepartitionMode ERepartitionMode_stringToEnum(const std::string& s)
{
    if(s == "multiResolution")
        return eRepartitionMultiResolution;
    return eRepartitionUndefined;
}

inline std::istream& operator>>(std::istream& in, ERepartitionMode& out_mode)
{
    std::string s;
    in >> s;
    out_mode = ERepartitionMode_stringToEnum(s);
    return in;
}

/// Create a dense SfMData based on reference \p sfmData, 
/// using \p vertices as landmarks and \p ptCams as observations
void createDenseSfMData(const sfmData::SfMData& sfmData,
                        const mvsUtils::MultiViewParams& mp,
                        const std::vector<Point3d>& vertices,
                        const StaticVector<StaticVector<int>>& ptsCams,
                        sfmData::SfMData& outSfmData
)
{
  outSfmData = sfmData;
  outSfmData.getLandmarks().clear();

  const double unknownScale = 0.0;
  for(std::size_t i = 0; i < vertices.size(); ++i)
  {
    const Point3d& point = vertices.at(i);
    const Vec3 pt3D(point.x, point.y, point.z);
    sfmData::Landmark landmark(pt3D, feature::EImageDescriberType::UNKNOWN);
    // set landmark observations from ptsCams if any
    if(!ptsCams[i].empty())
    {
      for(int cam : ptsCams[i])
      {
        const sfmData::View& view = sfmData.getView(mp.getViewId(cam));
        const camera::IntrinsicBase* intrinsicPtr = sfmData.getIntrinsicPtr(view.getIntrinsicId());
        const sfmData::Observation observation(intrinsicPtr->project(sfmData.getPose(view).getTransform(), pt3D.homogeneous(), true), UndefinedIndexT, unknownScale); // apply distortion
        landmark.observations[view.getViewId()] = observation;
      }
    }
    outSfmData.getLandmarks()[i] = landmark;
  }
}

/// Remove all landmarks without observations from \p sfmData.
void removeLandmarksWithoutObservations(sfmData::SfMData& sfmData)
{
  auto& landmarks = sfmData.getLandmarks();
  for(auto it = landmarks.begin(); it != landmarks.end();)
  {
    if(it->second.observations.empty())
      it = landmarks.erase(it);
    else
      ++it;
  }
}

/// BoundingBox Structure stocking ordered values from the command line
struct BoundingBox
{
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d scale = Eigen::Vector3d::Zero();

    inline bool isInitialized() const 
    { 
        return scale(0) != 0.0;
    }

    Eigen::Matrix4d modelMatrix() const
    {
        // Compute the translation matrix
        Eigen::Matrix4d translateMat = Eigen::Matrix4d::Identity();
        translateMat.col(3).head<3>() << translation.x(), translation.y(), translation.z();

        // Compute the rotation matrix from quaternion made with Euler angles in that order: ZXY (same as Qt algorithm)
        Eigen::Matrix4d rotateMat = Eigen::Matrix4d::Identity();

        {
            double pitch = rotation.x() * M_PI / 180;
            double yaw = rotation.y() * M_PI / 180;
            double roll = rotation.z() * M_PI / 180;

            pitch *= 0.5;
            yaw *= 0.5;
            roll *= 0.5;

            const double cy = std::cos(yaw);
            const double sy = std::sin(yaw);
            const double cr = std::cos(roll);
            const double sr = std::sin(roll);
            const double cp = std::cos(pitch);
            const double sp = std::sin(pitch);
            const double cycr = cy * cr;
            const double sysr = sy * sr;

            const double w = cycr * cp + sysr * sp;
            const double x = cycr * sp + sysr * cp;
            const double y = sy * cr * cp - cy * sr * sp;
            const double z = cy * sr * cp - sy * cr * sp;

            Eigen::Quaterniond quaternion(w, x, y, z);
            rotateMat.block<3, 3>(0, 0) = quaternion.matrix();
        }

        // Compute the scale matrix
        Eigen::Matrix4d scaleMat = Eigen::Matrix4d::Identity();
        scaleMat.diagonal().head<3>() << scale.x(), scale.y(), scale.z();

        // Model matrix
        Eigen::Matrix4d modelMat = translateMat * rotateMat * scaleMat;

        return modelMat;
    }

    void toHexahedron(Point3d* hexah) const
    {
        Eigen::Matrix4d modelMat = modelMatrix();

        // Retrieve the eight vertices of the bounding box
        // Based on VoxelsGrid::getHexah implementation
        Eigen::Vector4d origin = Eigen::Vector4d(-1, -1, -1, 1);
        Eigen::Vector4d vvx = Eigen::Vector4d(2, 0, 0, 0);
        Eigen::Vector4d vvy = Eigen::Vector4d(0, 2, 0, 0);
        Eigen::Vector4d vvz = Eigen::Vector4d(0, 0, 2, 0);

        Eigen::Vector4d vertex0 = modelMat * origin;
        Eigen::Vector4d vertex1 = modelMat * (origin + vvx);
        Eigen::Vector4d vertex2 = modelMat * (origin + vvx + vvy);
        Eigen::Vector4d vertex3 = modelMat * (origin + vvy);
        Eigen::Vector4d vertex4 = modelMat * (origin + vvz);
        Eigen::Vector4d vertex5 = modelMat * (origin + vvz + vvx);
        Eigen::Vector4d vertex6 = modelMat * (origin + vvz + vvx + vvy);
        Eigen::Vector4d vertex7 = modelMat * (origin + vvz + vvy);

        // Apply those eight vertices to the hexah
        hexah[0] = Point3d(vertex0.x(), vertex0.y(), vertex0.z());
        hexah[1] = Point3d(vertex1.x(), vertex1.y(), vertex1.z());
        hexah[2] = Point3d(vertex2.x(), vertex2.y(), vertex2.z());
        hexah[3] = Point3d(vertex3.x(), vertex3.y(), vertex3.z());
        hexah[4] = Point3d(vertex4.x(), vertex4.y(), vertex4.z());
        hexah[5] = Point3d(vertex5.x(), vertex5.y(), vertex5.z());
        hexah[6] = Point3d(vertex6.x(), vertex6.y(), vertex6.z());
        hexah[7] = Point3d(vertex7.x(), vertex7.y(), vertex7.z());
    }
};

inline std::istream& operator>>(std::istream& in, BoundingBox& out_bbox)
{
    std::string s;
    in >> s;

    std::vector<std::string> dataStr;
    boost::split(dataStr, s, boost::is_any_of(","));
    if(dataStr.size() != 9)
    {
        throw std::runtime_error("Invalid number of values for bounding box.");
    }

    std::vector<double> data;
    data.reserve(9);
    for (const std::string& elt : dataStr) 
    {
        data.push_back(boost::lexical_cast<double>(elt));
    }

    out_bbox.translation << data[0], data[1], data[2];
    out_bbox.rotation << data[3], data[4], data[5];
    out_bbox.scale << data[6], data[7], data[8];

    return in;
}


int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outputMesh;
    std::string outputDensePointCloud;
    std::string depthMapsFolder;
    EPartitioningMode partitioningMode = ePartitioningSingleBlock;
    ERepartitionMode repartitionMode = eRepartitionMultiResolution;
    std::size_t estimateSpaceMinObservations = 3;
    float estimateSpaceMinObservationAngle = 10.0f;
    double universePercentile = 0.999;
    int maxPtsPerVoxel = 6000000;
    bool meshingFromDepthMaps = true;
    bool estimateSpaceFromSfM = true;
    bool addLandmarksToTheDensePointCloud = false;
    bool saveRawDensePointCloud = false;
    bool colorizeOutput = false;
    bool voteFilteringForWeaklySupportedSurfaces = true;
    int invertTetrahedronBasedOnNeighborsNbIterations = 10;
    double minSolidAngleRatio = 0.2;
    int nbSolidAngleFilteringIterations = 2;
    unsigned int seed = 0;
    BoundingBox boundingBox;

    fuseCut::FuseParams fuseParams;

    int helperPointsGridSize = 10;
    int densifyNbFront = 0;
    int densifyNbBack = 0;
    double densifyScale = 1.0;
    double nPixelSizeBehind = 4.0;
    double fullWeight = 1.0;
    bool exportDebugTetrahedralization = false;
    int maxNbConnectedHelperPoints = 50;

    po::options_description allParams("AliceVision meshing");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
          "SfMData file.")
        ("output,o", po::value<std::string>(&outputDensePointCloud)->required(),
          "Output Dense SfMData file.")
        ("outputMesh,o", po::value<std::string>(&outputMesh)->required(),
          "Output mesh");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("depthMapsFolder", po::value<std::string>(&depthMapsFolder),
            "Input filtered depth maps folder.")
        ("boundingBox", po::value<BoundingBox>(&boundingBox),
            "Specifies a bounding box to reconstruct: position, rotation (Euler ZXY) and scale.")
        ("maxInputPoints", po::value<int>(&fuseParams.maxInputPoints)->default_value(fuseParams.maxInputPoints),
            "Max input points loaded from images.")
        ("maxPoints", po::value<int>(&fuseParams.maxPoints)->default_value(fuseParams.maxPoints),
            "Max points at the end of the depth maps fusion.")
        ("maxPointsPerVoxel", po::value<int>(&maxPtsPerVoxel)->default_value(maxPtsPerVoxel),
            "Max points per voxel.")
        ("minStep", po::value<int>(&fuseParams.minStep)->default_value(fuseParams.minStep),
            "The step used to load depth values from depth maps is computed from maxInputPts. Here we define the minimal value for this step, "
            "so on small datasets we will not spend too much time at the beginning loading all depth values.")
        ("simFactor", po::value<float>(&fuseParams.simFactor)->default_value(fuseParams.simFactor),
            "simFactor")
        ("angleFactor", po::value<float>(&fuseParams.angleFactor)->default_value(fuseParams.angleFactor),
            "angleFactor")
        ("minVis", po::value<int>(&fuseParams.minVis)->default_value(fuseParams.minVis),
            "Filter points based on their number of observations")
        ("partitioning", po::value<EPartitioningMode>(&partitioningMode)->default_value(partitioningMode),
            "Partitioning: 'singleBlock' or 'auto'.")
        ("repartition", po::value<ERepartitionMode>(&repartitionMode)->default_value(repartitionMode),
            "Repartition: 'multiResolution' or 'regularGrid'.")
        ("estimateSpaceFromSfM", po::value<bool>(&estimateSpaceFromSfM)->default_value(estimateSpaceFromSfM),
            "Estimate the 3d space from the SfM.")
        ("addLandmarksToTheDensePointCloud", po::value<bool>(&addLandmarksToTheDensePointCloud)->default_value(addLandmarksToTheDensePointCloud),
            "Add SfM Landmarks into the dense point cloud (created from depth maps). If only the SfM is provided in input, SfM landmarks will be used regardless of this option.")
        ("colorizeOutput", po::value<bool>(&colorizeOutput)->default_value(colorizeOutput),
            "Whether to colorize output dense point cloud and mesh.");

    po::options_description advancedParams("Advanced parameters");
    advancedParams.add_options()
        ("universePercentile", po::value<double>(&universePercentile)->default_value(universePercentile),
            "universe percentile")
        ("estimateSpaceMinObservations", po::value<std::size_t>(&estimateSpaceMinObservations)->default_value(estimateSpaceMinObservations),
            "Minimum number of observations for SfM space estimation.")
        ("estimateSpaceMinObservationAngle", po::value<float>(&estimateSpaceMinObservationAngle)->default_value(estimateSpaceMinObservationAngle),
            "Minimum angle between two observations for SfM space estimation.")
        ("pixSizeMarginInitCoef", po::value<double>(&fuseParams.pixSizeMarginInitCoef)->default_value(fuseParams.pixSizeMarginInitCoef),
            "pixSizeMarginInitCoef")
        ("pixSizeMarginFinalCoef", po::value<double>(&fuseParams.pixSizeMarginFinalCoef)->default_value(fuseParams.pixSizeMarginFinalCoef),
            "pixSizeMarginFinalCoef")
        ("voteMarginFactor", po::value<float>(&fuseParams.voteMarginFactor)->default_value(fuseParams.voteMarginFactor),
            "voteMarginFactor")
        ("contributeMarginFactor", po::value<float>(&fuseParams.contributeMarginFactor)->default_value(fuseParams.contributeMarginFactor),
            "contributeMarginFactor")
        ("simGaussianSizeInit", po::value<float>(&fuseParams.simGaussianSizeInit)->default_value(fuseParams.simGaussianSizeInit),
            "simGaussianSizeInit")
        ("simGaussianSize", po::value<float>(&fuseParams.simGaussianSize)->default_value(fuseParams.simGaussianSize),
            "simGaussianSize")
        ("minAngleThreshold", po::value<double>(&fuseParams.minAngleThreshold)->default_value(fuseParams.minAngleThreshold),
            "minAngleThreshold")
        ("refineFuse", po::value<bool>(&fuseParams.refineFuse)->default_value(fuseParams.refineFuse),
            "refineFuse")
        ("helperPointsGridSize", po::value<int>(&helperPointsGridSize)->default_value(helperPointsGridSize),
            "Helper points grid size.")
        ("densifyNbFront", po::value<int>(&densifyNbFront)->default_value(densifyNbFront),
            "Number of points in front of the vertices to densify the scene.")
        ("densifyNbBack", po::value<int>(&densifyNbBack)->default_value(densifyNbBack),
            "Number of points behind the vertices to densify the scene.")
        ("densifyScale", po::value<double>(&densifyScale)->default_value(densifyScale),
            "Scale between points used to densify the scene.")
        ("maskHelperPointsWeight", po::value<float>(&fuseParams.maskHelperPointsWeight)->default_value(fuseParams.maskHelperPointsWeight),
            "Mask helper points weight. Zero to disable it.")
        ("maskBorderSize", po::value<int>(&fuseParams.maskBorderSize)->default_value(fuseParams.maskBorderSize),
            "How many pixels on mask borders? 1 by default.")
        ("nPixelSizeBehind", po::value<double>(&nPixelSizeBehind)->default_value(nPixelSizeBehind),
            "Number of pixel size units to vote behind the vertex with FULL status.")
        ("fullWeight", po::value<double>(&fullWeight)->default_value(fullWeight),
            "Weighting of the FULL cells.")
        ("saveRawDensePointCloud", po::value<bool>(&saveRawDensePointCloud)->default_value(saveRawDensePointCloud),
            "Save dense point cloud before cut and filtering.")
        ("voteFilteringForWeaklySupportedSurfaces", po::value<bool>(&voteFilteringForWeaklySupportedSurfaces)->default_value(voteFilteringForWeaklySupportedSurfaces),
            "Improve support of weakly supported surfaces with a tetrahedra fullness score filtering.")
        ("invertTetrahedronBasedOnNeighborsNbIterations", po::value<int>(&invertTetrahedronBasedOnNeighborsNbIterations)->default_value(invertTetrahedronBasedOnNeighborsNbIterations),
            "Invert cells status around surface to improve smoothness.")
        ("minSolidAngleRatio", po::value<double>(&minSolidAngleRatio)->default_value(minSolidAngleRatio),
            "Filter cells status on surface around vertices to improve smoothness using solid angle ratio between full/empty parts.")
        ("nbSolidAngleFilteringIterations", po::value<int>(&nbSolidAngleFilteringIterations)->default_value(nbSolidAngleFilteringIterations),
            "Number of iterations to filter the status cells based on solid angle ratio.")
        ("maxNbConnectedHelperPoints", po::value<int>(&maxNbConnectedHelperPoints)->default_value(maxNbConnectedHelperPoints),
            "Maximum number of connected helper points before we remove them.")
        ("exportDebugTetrahedralization", po::value<bool>(&exportDebugTetrahedralization)->default_value(exportDebugTetrahedralization),
            "Export debug cells score as tetrahedral mesh. WARNING: could create huge meshes, only use on very small datasets.")        
        ("seed", po::value<unsigned int>(&seed)->default_value(seed),
            "Seed used in random processes. (0 to use a random seed).");

    po::options_description logParams("Log parameters");
    logParams.add_options()
      ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(advancedParams).add(logParams);

    po::variables_map vm;

    try
    {
      po::store(po::parse_command_line(argc, argv, allParams), vm);

      if(vm.count("help") || (argc == 1))
      {
        ALICEVISION_COUT(allParams);
        return EXIT_SUCCESS;
      }

      po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    if(depthMapsFolder.empty())
    {
      if(depthMapsFolder.empty() &&
         repartitionMode == eRepartitionMultiResolution &&
         partitioningMode == ePartitioningSingleBlock)
      {
        meshingFromDepthMaps = false;
        addLandmarksToTheDensePointCloud = true;
      }
      else
      {
        ALICEVISION_LOG_ERROR("Invalid input options:\n"
                              "- Meshing from depth maps require --depthMapsFolder option.\n"
                              "- Meshing from SfM require option --partitioning set to 'singleBlock' and option --repartition set to 'multiResolution'.");
        return EXIT_FAILURE;
      }
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
      return EXIT_FAILURE;
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, "", "", depthMapsFolder, meshingFromDepthMaps);

    mp.userParams.put("LargeScale.universePercentile", universePercentile);
    mp.userParams.put("LargeScale.helperPointsGridSize", helperPointsGridSize);
    mp.userParams.put("LargeScale.densifyNbFront", densifyNbFront);
    mp.userParams.put("LargeScale.densifyNbBack", densifyNbBack);
    mp.userParams.put("LargeScale.densifyScale", densifyScale);

    mp.userParams.put("delaunaycut.seed", seed);
    mp.userParams.put("delaunaycut.nPixelSizeBehind", nPixelSizeBehind);
    mp.userParams.put("delaunaycut.fullWeight", fullWeight);
    mp.userParams.put("delaunaycut.voteFilteringForWeaklySupportedSurfaces", voteFilteringForWeaklySupportedSurfaces);
    mp.userParams.put("hallucinationsFiltering.invertTetrahedronBasedOnNeighborsNbIterations", invertTetrahedronBasedOnNeighborsNbIterations);
    mp.userParams.put("hallucinationsFiltering.minSolidAngleRatio", minSolidAngleRatio);
    mp.userParams.put("hallucinationsFiltering.nbSolidAngleFilteringIterations", nbSolidAngleFilteringIterations);

    int ocTreeDim = mp.userParams.get<int>("LargeScale.gridLevel0", 1024);
    const auto baseDir = mp.userParams.get<std::string>("LargeScale.baseDirName", "root01024");

    fs::path outDirectory = fs::path(outputMesh).parent_path();
    if(!fs::is_directory(outDirectory))
        fs::create_directory(outDirectory);

    fs::path tmpDirectory = outDirectory / "tmp";

    ALICEVISION_LOG_WARNING("repartitionMode: " << repartitionMode);
    ALICEVISION_LOG_WARNING("partitioningMode: " << partitioningMode);

    mesh::Mesh* mesh = nullptr;
    StaticVector<StaticVector<int>> ptsCams;

    switch(repartitionMode)
    {
        case eRepartitionMultiResolution:
        {
            switch(partitioningMode)
            {
                case ePartitioningAuto:
                {
                    throw std::invalid_argument("Meshing mode: 'multiResolution', partitioning: 'auto' is not yet implemented.");
                }
                case ePartitioningSingleBlock:
                {
                    ALICEVISION_LOG_INFO("Meshing mode: multi-resolution, partitioning: single block.");
                    std::array<Point3d, 8> hexah;

                    float minPixSize;
                    fuseCut::Fuser fs(mp);

                    if (boundingBox.isInitialized())
                        boundingBox.toHexahedron(&hexah[0]);
                    else if(meshingFromDepthMaps && (!estimateSpaceFromSfM || sfmData.getLandmarks().empty()))
                      fs.divideSpaceFromDepthMaps(&hexah[0], minPixSize);
                    else
                      fs.divideSpaceFromSfM(sfmData, &hexah[0], estimateSpaceMinObservations, estimateSpaceMinObservationAngle);

                    {
                        const double length = hexah[0].x - hexah[1].x;
                        const double width = hexah[0].y - hexah[3].y;
                        const double height = hexah[0].z - hexah[4].z;

                        ALICEVISION_LOG_INFO("bounding Box : length: " << length << ", width: " << width << ", height: " << height);
                    }

                    StaticVector<int> cams;
                    if(meshingFromDepthMaps)
                    {
                      cams = mp.findCamsWhichIntersectsHexahedron(&hexah[0]);
                    }
                    else
                    {
                      cams.resize(mp.getNbCameras());
                      for(int i = 0; i < cams.size(); ++i)
                          cams[i] = i;
                    }

                    if(cams.empty())
                        throw std::logic_error("No camera to make the reconstruction");
                    
                    fuseCut::DelaunayGraphCut delaunayGC(mp);
                    delaunayGC.createDensePointCloud(&hexah[0], cams, addLandmarksToTheDensePointCloud ? &sfmData : nullptr, meshingFromDepthMaps ? &fuseParams : nullptr);
                    if(saveRawDensePointCloud)
                    {
                      ALICEVISION_LOG_INFO("Save dense point cloud before cut and filtering.");
                      StaticVector<StaticVector<int>> ptsCams;
                      delaunayGC.createPtsCams(ptsCams);
                      sfmData::SfMData densePointCloud;
                      createDenseSfMData(sfmData, mp, delaunayGC._verticesCoords, ptsCams, densePointCloud);
                      removeLandmarksWithoutObservations(densePointCloud);
                      if(colorizeOutput)
                        sfmData::colorizeTracks(densePointCloud);
                      sfmDataIO::Save(densePointCloud, (outDirectory/"densePointCloud_raw.abc").string(), sfmDataIO::ESfMData::ALL_DENSE);
                    }

                    delaunayGC.createGraphCut(&hexah[0], cams, outDirectory.string() + "/",
                                              outDirectory.string() + "/SpaceCamsTracks/", false,
                                              exportDebugTetrahedralization);

                    delaunayGC.graphCutPostProcessing(&hexah[0], outDirectory.string()+"/");

                    mesh = delaunayGC.createMesh(maxNbConnectedHelperPoints);
                    delaunayGC.createPtsCams(ptsCams);
                    mesh::meshPostProcessing(mesh, ptsCams, mp, outDirectory.string()+"/", nullptr, &hexah[0]);

                    break;
                }
                case ePartitioningUndefined:
                default:
                    throw std::invalid_argument("Partitioning mode is not defined");
            }
            break;
        }
        case eRepartitionUndefined:
        default:
            throw std::invalid_argument("Repartition mode is not defined");
    }

    // Generate output files: 
    // - dense point-cloud with observations as sfmData
    // - mesh as .obj

    if(mesh == nullptr || mesh->pts.empty() || mesh->tris.empty())
      throw std::runtime_error("No valid mesh was generated.");

    if(ptsCams.empty())
      throw std::runtime_error("Points visibilities data has not been initialized.");

    sfmData::SfMData densePointCloud;
    createDenseSfMData(sfmData, mp, mesh->pts.getData(), ptsCams, densePointCloud);

    if(colorizeOutput)
    {
      sfmData::colorizeTracks(densePointCloud);
      // colorize output mesh before landmarks filtering
      // to have a 1:1 mapping between points and mesh vertices
      const auto& landmarks = densePointCloud.getLandmarks();
      std::vector<rgb>& colors = mesh->colors();
      colors.resize(mesh->pts.size(), {0, 0, 0});
      for(std::size_t i = 0; i < mesh->pts.size(); ++i)
      {
        const auto& c = landmarks.at(i).rgb;
        colors[i] = {c.r(), c.g(), c.b()};
      }
    }

    removeLandmarksWithoutObservations(densePointCloud);
    ALICEVISION_LOG_INFO("Save dense point cloud.");
    sfmDataIO::Save(densePointCloud, outputDensePointCloud, sfmDataIO::ESfMData::ALL_DENSE);

    ALICEVISION_LOG_INFO("Save obj mesh file.");
    ALICEVISION_LOG_INFO("OUTPUT MESH " << outputMesh);
    mesh->save(outputMesh);
    delete mesh;


    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
