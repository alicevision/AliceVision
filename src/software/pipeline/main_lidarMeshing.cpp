// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/fuseCut/InputSet.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/fuseCut/Fuser.hpp>
#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>

#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>

#include <boost/program_options.hpp>
#include <fstream>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

struct BoundingBox
{
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d scale = Eigen::Vector3d::Zero();

    inline bool isInitialized() const { return scale(0) != 0.0; }

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

    static BoundingBox fromHexahedron(const Point3d* hexah)
    {
        BoundingBox bbox;

        // Compute the scale
        bbox.scale(0) = (hexah[0] - hexah[1]).size() / 2.;
        bbox.scale(1) = (hexah[0] - hexah[3]).size() / 2.;
        bbox.scale(2) = (hexah[0] - hexah[4]).size() / 2.;

        // Compute the translation
        Point3d cg(0., 0., 0.);
        for (int i = 0; i < 8; i++)
        {
            cg += hexah[i];
        }
        cg /= 8.;

        bbox.translation(0) = cg.x;
        bbox.translation(1) = cg.y;
        bbox.translation(2) = cg.z;

        // Compute the rotation matrix
        Eigen::Matrix3d rotateMat = Eigen::Matrix3d::Identity();
        Point3d cx = ((hexah[1] + hexah[2] + hexah[5] + hexah[6]) / 4. - cg).normalize();
        Point3d cy = ((hexah[3] + hexah[2] + hexah[7] + hexah[6]) / 4. - cg).normalize();
        Point3d cz = ((hexah[7] + hexah[4] + hexah[5] + hexah[6]) / 4. - cg).normalize();
        rotateMat.col(0).head<3>() << cx.x, cx.y, cx.z;
        rotateMat.col(1).head<3>() << cy.x, cy.y, cy.z;
        rotateMat.col(2).head<3>() << cz.x, cz.y, cz.z;

        // Euler rotation angles
        Eigen::Vector3d ea = rotateMat.eulerAngles(1, 0, 2) * 180. / M_PI;
        bbox.rotation(0) = ea(1);
        bbox.rotation(1) = ea(0);
        bbox.rotation(2) = ea(2);

        return bbox;
    }
};

inline std::istream& operator>>(std::istream& in, BoundingBox& out_bbox)
{
    std::string token(std::istreambuf_iterator<char>(in), {});

    std::vector<std::string> dataStr;
    boost::split(dataStr, token, boost::is_any_of(","));
    if (dataStr.size() != 9)
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

bool computeSubMesh(const std::string & pathSfmData, std::string & outputFile, const BoundingBox & boundingBox)
{
    //initialization
    StaticVector<StaticVector<int>> ptsCams;

    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, pathSfmData, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << pathSfmData << "' cannot be read.");
        return false;
    }

    //Create multiview params
    mvsUtils::MultiViewParams mp(sfmData, "", "", "", false);
    mp.userParams.put("LargeScale.forcePixelSize", 0.01);
    mp.userParams.put("LargeScale.forceWeight", 32.0);
    mp.userParams.put("LargeScale.helperPointsGridSize", 10);
    mp.userParams.put("LargeScale.densifyNbFront", 0);
    mp.userParams.put("LargeScale.densifyNbBack", 0);
    mp.userParams.put("LargeScale.densifyScale", 1.0);
    mp.userParams.put("delaunaycut.seed", 0);
    mp.userParams.put("delaunaycut.nPixelSizeBehind", 4);
    mp.userParams.put("delaunaycut.fullWeight", 1.0);
    mp.userParams.put("delaunaycut.voteFilteringForWeaklySupportedSurfaces", true);
    mp.userParams.put("hallucinationsFiltering.invertTetrahedronBasedOnNeighborsNbIterations", 10);
    mp.userParams.put("hallucinationsFiltering.minSolidAngleRatio", 0.2);
    mp.userParams.put("hallucinationsFiltering.nbSolidAngleFilteringIterations", 2);

    //Set the input cams ids
    StaticVector<int> cams(mp.getNbCameras());
    for (int i = 0; i < cams.size(); ++i)
    {
        cams[i] = i;
    }

    if (cams.empty())
    {
        ALICEVISION_LOG_ERROR("No available camera/sensor");
        return false;
    }

    //Compute boundingbox
    Eigen::Vector3d bbMin, bbMax;
    sfmData.getBoundingBox(bbMin, bbMax);

    //Check bounding box validity
    double w = std::abs(bbMin.x() - bbMax.x());
    double h = std::abs(bbMin.y() - bbMax.y());
    double l = std::abs(bbMin.z() - bbMax.z());
    if (w < 1e-6 || h < 1e-6 || l < 1e-6) return true;

    //Use mp format
    std::array<Point3d, 8> lhexah;
    lhexah[0].x = bbMin.x(); lhexah[0].y = bbMin.y(); lhexah[0].z = bbMin.z();
    lhexah[1].x = bbMax.x(); lhexah[1].y = bbMin.y(); lhexah[1].z = bbMin.z();
    lhexah[2].x = bbMax.x(); lhexah[2].y = bbMax.y(); lhexah[2].z = bbMin.z();
    lhexah[3].x = bbMin.x(); lhexah[3].y = bbMax.y(); lhexah[3].z = bbMin.z();
    lhexah[4].x = bbMin.x(); lhexah[4].y = bbMin.y(); lhexah[4].z = bbMax.z();
    lhexah[5].x = bbMax.x(); lhexah[5].y = bbMin.y(); lhexah[5].z = bbMax.z();
    lhexah[6].x = bbMax.x(); lhexah[6].y = bbMax.y(); lhexah[6].z = bbMax.z();
    lhexah[7].x = bbMin.x(); lhexah[7].y = bbMax.y(); lhexah[7].z = bbMax.z();

    fuseCut::DelaunayGraphCut delaunayGC(mp);
    delaunayGC.createDensePointCloud(&lhexah[0], cams, &sfmData, nullptr);
    delaunayGC.createGraphCut(&lhexah[0], cams, "/", "/SpaceCamsTracks/", false, false);
    delaunayGC.graphCutPostProcessing(&lhexah[0], "/");
    mesh::Mesh * mesh = delaunayGC.createMesh(0);
    delaunayGC.createPtsCams(ptsCams);
    mesh::meshPostProcessing(mesh, ptsCams, mp, "/", nullptr, &lhexah[0]);
    mesh->save(outputFile);
    delete mesh;
    
    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;
    int rangeStart = -1;
    int rangeSize = 1;

    std::string jsonFilename = "";
    std::string outputDirectory = "";
    std::string outputJsonFilename = "";

    BoundingBox boundingBox;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&jsonFilename)->required(),
         "json file.")
        ("output,o", po::value<std::string>(&outputDirectory)->required(),
         "Output directory.")
        ("outputJson", po::value<std::string>(&outputJsonFilename)->required(),
         "Output scene description.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("boundingBox", po::value<BoundingBox>(&boundingBox),
         "Specifies a bounding box to reconstruct: position, rotation (Euler ZXY) and scale.")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.");
    // clang-format on

    CmdLine cmdline("AliceVision lidarMeshing");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    std::ifstream inputfile(jsonFilename);
    if (!inputfile.is_open())
    {
        ALICEVISION_LOG_ERROR("Cannot open json input file");
        return EXIT_FAILURE;
    }
    
    
    std::stringstream buffer;
    buffer << inputfile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    fuseCut::InputSet inputsets(boost::json::value_to<fuseCut::InputSet>(jv));

    size_t setSize = inputsets.size();
    if (rangeStart != -1)
    {
        if (rangeStart < 0 || rangeSize < 0)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if (rangeStart + rangeSize > setSize)
            rangeSize = setSize - rangeStart;

        if (rangeSize <= 0)
        {
            ALICEVISION_LOG_WARNING("Nothing to compute.");
            return EXIT_SUCCESS;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = setSize;
    }

    for (int idSub = rangeStart; idSub < rangeStart + rangeSize; idSub++)
    {
        const fuseCut::Input & input = inputsets[idSub];
        std::string ss = outputDirectory + "/subobj_" + std::to_string(idSub) + ".obj";

        ALICEVISION_LOG_INFO("Computing sub mesh " << idSub + 1 << " / " << setSize);
        if (!computeSubMesh(input.sfmPath, ss, boundingBox))
        {
            ALICEVISION_LOG_ERROR("Error computing sub mesh");
            return EXIT_FAILURE;
        }

        inputsets[idSub].subMeshPath = ss;

        ALICEVISION_LOG_INFO(ss);
    }

    std::ofstream of(outputJsonFilename);
    jv = boost::json::value_from(inputsets);
    of << boost::json::serialize(jv);
    of.close();

    
    return EXIT_SUCCESS;
}
