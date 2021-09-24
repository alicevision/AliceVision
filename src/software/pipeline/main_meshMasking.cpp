// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/sfmMvsUtils/visibility.hpp>
#include <aliceVision/camera/cameraUndistortImage.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <memory>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


bool tryLoadMask(image::Image<unsigned char>* mask, const std::vector<std::string>& masksFolders, const IndexT viewId, const std::string& srcImage)
{
    for (const auto& masksFolder_str : masksFolders)
    {
        if (!masksFolder_str.empty() && fs::exists(masksFolder_str))
        {
            const auto masksFolder = fs::path(masksFolder_str);
            const auto idMaskPath = masksFolder / fs::path(std::to_string(viewId)).replace_extension("png");
            const auto nameMaskPath = masksFolder / fs::path(srcImage).filename().replace_extension("png");

            if (fs::exists(idMaskPath))
            {
                image::readImage(idMaskPath.string(), *mask, image::EImageColorSpace::LINEAR);
                return true;
            }
            else if (fs::exists(nameMaskPath))
            {
                image::readImage(nameMaskPath.string(), *mask, image::EImageColorSpace::LINEAR);
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Basic cache system to manage masks.
 * 
 * It keeps the latest "maxSize" (defaut = 16) masks in memory.
 */
struct MaskCache
{
    MaskCache(const mvsUtils::MultiViewParams& mp, const std::vector<std::string>& masksFolders, bool undistortMasks, int maxSize = 16)
        : _mp(mp)
        , _masksFolders(masksFolders)
        , _undistortMasks(undistortMasks)
        , _maxSize(maxSize)
    {
    }

    image::Image<unsigned char>* lock(int camId)
    {
        Item * item = findItem(camId);
        if (item)
        {
            assert(item->locks >= 0);
            ++item->locks;
            item = pushToLowPriority(camId);
            assert(item->camId == camId);
            return item->mask.get();
        }
        else
        {
            if (_cache.size() >= _maxSize)
            {
                tryFreeUnlockedItem();
            }

            _cache.push_back({ camId, std::make_unique<image::Image<unsigned char>>(), 0 });
            item = &_cache.back();
            const IndexT viewId = _mp.getViewId(camId);
            auto * const mask = item->mask.get();
            const bool loaded = tryLoadMask(mask, _masksFolders, viewId, _mp.getImagePath(camId));
            if (loaded)
            {
                if (_undistortMasks)
                {
                    const auto& sfm = _mp.getInputSfMData();
                    const IndexT intrinsicId = sfm.getView(viewId).getIntrinsicId();
                    const auto intrinsicIt = sfm.intrinsics.find(intrinsicId);
                    if (intrinsicIt != sfm.intrinsics.end())
                    {
                        const auto& intrinsic = intrinsicIt->second;
                        if (intrinsic->isValid() && intrinsic->hasDistortion())
                        {
                            image::Image<unsigned char> mask_ud;
                            camera::UndistortImage(*mask, intrinsic.get(), mask_ud, (unsigned char)0);
                            mask->swap(mask_ud);
                        }
                    }
                }

                item->locks = 1;
                return mask;
            }
            else
            {
                _cache.pop_back();
                return nullptr;
            }
        }
    }

    void unlock(int camId)
    {
        Item* item = findItem(camId);
        assert(item);
        assert(item->locks > 0);
        --item->locks;
    }

private:
    struct Item
    {
        int camId;
        std::unique_ptr<image::Image<unsigned char>> mask;
        int locks;
    };

    std::vector<Item>::iterator findItemIt(int camId)
    {
        return std::find_if(_cache.begin(), _cache.end(), [camId](const Item& item) { return item.camId == camId; });
    }

    Item* findItem(int camId)
    {
        const auto it = findItemIt(camId);
        return (it == _cache.end()) ? nullptr : &*it;
    }

    void tryFreeUnlockedItem()
    {
        const auto it = std::find_if(_cache.begin(), _cache.end(), [](const Item& item) { return item.locks <= 0; });
        if (it != _cache.end())
        {
            _cache.erase(it);
        }
    }

    Item* pushToLowPriority(int camId)
    {
        const auto it = findItemIt(camId);
        assert(it != _cache.end());
        std::rotate(it, it+1, _cache.end());
        return &_cache.back();
    }

private:
    mvsUtils::MultiViewParams _mp;
    std::vector<std::string> _masksFolders;
    bool _undistortMasks;
    int _maxSize;
    std::vector<Item> _cache;
};

StaticVector<int> computeDiffVisibilities(const StaticVector<int>& A, const StaticVector<int>& B)
{
    assert(std::is_sorted(A.begin(), A.end()));
    assert(std::is_sorted(B.begin(), B.end()));
    StaticVector<int> diff;
    std::set_symmetric_difference(A.begin(), A.end(), B.begin(), B.end(), std::back_inserter(diff.getDataWritable()));
    return diff;
}

Point3d findBoundaryVertex(
    const mesh::Mesh& mesh,
    int visibleVertexId,
    int hiddenVertexId,
    const mvsUtils::MultiViewParams& mp,
    MaskCache & maskCache,
    int threshold,
    bool invert
    )
{
    // find the cameras that make a difference, we only need to sample those
    const auto& visibleVertexVisibilities = mesh.pointsVisibilities[visibleVertexId];
    const auto& hiddenVertexVisibilities = mesh.pointsVisibilities[hiddenVertexId];
    assert(visibleVertexVisibilities.size() > hiddenVertexVisibilities.size());
    assert(hiddenVertexVisibilities.size() < threshold);

    const auto diffVisibilities = computeDiffVisibilities(visibleVertexVisibilities, hiddenVertexVisibilities);
    assert(diffVisibilities.size() > 0);
    assert(std::is_sorted(diffVisibilities.begin(), diffVisibilities.end()));

    // compute the visibility that is already acquired and that does not change along the edge.
    // <=> is in hiddenVertexVisibilities but not in diffVisibilities
    const int baseVisibility = std::count_if(hiddenVertexVisibilities.begin(), hiddenVertexVisibilities.end(),
        [&diffVisibilities] (int camId) { return diffVisibilities.indexOfSorted(camId) == -1; });

    // preload masks
    std::map<int, image::Image<unsigned char>*> masks;
    for (const int camId : diffVisibilities)
    {
        masks[camId] = maskCache.lock(camId);
        assert(masks[camId]);  // invalid masks are already ignored.
    }

    // compute the minimal distance according to the mask resolution (it is our search stop condition)
    const Point3d leftPoint = mesh.pts[visibleVertexId];
    const Point3d rightPoint = mesh.pts[hiddenVertexId];
    const float minDistance = [&]
    {
        const int camId = diffVisibilities[0];  // use a single mask, supposing they are all equivalent
        Pixel leftPixel, rightPixel;
        mp.getPixelFor3DPoint(&leftPixel, leftPoint, camId);
        mp.getPixelFor3DPoint(&rightPixel, rightPoint, camId);
        const int manhattanDistance = std::abs(rightPixel.x - leftPixel.x) + std::abs(rightPixel.y - leftPixel.y);
        return 1.f / float(manhattanDistance);
    }();

    // binary search in continuous space along the edge
    Point3d result;
    float left = 0.f;  // is the visible area
    float right = 1.f;  // is the hidden area
    while (true)
    {
        const float mid = (left + right) * 0.5f;
        const Point3d midPoint = leftPoint + (rightPoint - leftPoint) * mid;

        if (mid - left < minDistance)
        {
            result = midPoint;
            break;  // stop the search
        }

        int diffVisibility = 0;
        for (const int camId : diffVisibilities)
        {
            const auto& mask = *masks[camId];

            Pixel projectedPixel;
            mp.getPixelFor3DPoint(&projectedPixel, midPoint, camId);
            if (mp.isPixelInImage(projectedPixel, camId))
            {
                const bool maskValue = (mask(projectedPixel.y, projectedPixel.x) == 0);
                const bool masked = invert ? !maskValue : maskValue;
                if (!masked)
                {
                    ++diffVisibility;
                }
            }
        }

        const bool isVisible = (baseVisibility + diffVisibility) >= threshold;
        float& newBoundary = isVisible ? left : right;
        newBoundary = mid;
    }

    for (const int camId : diffVisibilities)
    {
        maskCache.unlock(camId);
    }

    return result;
}

void smoothenBoundary(
    mesh::Mesh & filteredMesh,
    const StaticVector<int> & filteredVertexVisibilityCounters,
    const mvsUtils::MultiViewParams& mp,
    MaskCache& maskCache,
    int threshold,
    bool invert
    )
{
    const auto isVertexVisible = [&filteredVertexVisibilityCounters, threshold](const int vertexId)
    {
        return filteredVertexVisibilityCounters[vertexId] >= threshold;
    };

    std::unordered_set<int> boundaryVertexIds;  // vertex that are already moved
    for (int triangleId = 0; triangleId < filteredMesh.tris.size(); ++triangleId)
    {
        const auto& triangle = filteredMesh.tris[triangleId];
        const auto visibleCount = std::count_if(std::begin(triangle.v), std::end(triangle.v), isVertexVisible);
        if (visibleCount == 3)
        {
            // do nothing
        }
        else if (visibleCount == 2)
        {
            // 2 out of 3 are visible: we need to move the 3rd vertex o the border.
            // we move it toward the farthest visible vertex so the triangle has little chance to be degenerate.
            const auto hiddenIdx = std::find_if_not(std::begin(triangle.v), std::end(triangle.v), isVertexVisible) - std::begin(triangle.v);
            const auto hiddenVertexId = triangle.v[hiddenIdx];
            if (boundaryVertexIds.find(hiddenVertexId) != boundaryVertexIds.end())
            {
                // already moved, ignore
                continue;
            }

            // move along the longest edge to avoid degenerate triangles
            const auto visibleVertexId = [&]
            {
                const auto visibleVertexId1 = triangle.v[(hiddenIdx + 1) % 3];
                const double length1 = (filteredMesh.pts[visibleVertexId1] - filteredMesh.pts[hiddenVertexId]).size();

                const auto visibleVertexId2 = triangle.v[(hiddenIdx + 2) % 3];
                const double length2 = (filteredMesh.pts[visibleVertexId2] - filteredMesh.pts[hiddenVertexId]).size();

                return length1 > length2 ? visibleVertexId1 : visibleVertexId2;
            }();

            const auto boundaryPoint = findBoundaryVertex(filteredMesh, visibleVertexId, hiddenVertexId, mp, maskCache, threshold, invert);
            filteredMesh.pts[hiddenVertexId] = boundaryPoint;
            boundaryVertexIds.insert(hiddenVertexId);
        }
        else if (visibleCount == 1)
        {
            // only 1 vertex is visible: we move the other 2 in its direction.
            const auto visibleIdx = std::find_if(std::begin(triangle.v), std::end(triangle.v), isVertexVisible) - std::begin(triangle.v);
            const auto visibleVertexId = triangle.v[visibleIdx];
            for (int i : {1, 2})
            {
                const auto hiddenIdx = (visibleIdx + i) % 3;
                const auto hiddenVertexId = triangle.v[hiddenIdx];
                if (boundaryVertexIds.find(hiddenVertexId) != boundaryVertexIds.end())
                {
                    // already moved, ignore
                    continue;
                }
                const auto boundaryPoint = findBoundaryVertex(filteredMesh, visibleVertexId, hiddenVertexId, mp, maskCache, threshold, invert);
                filteredMesh.pts[hiddenVertexId] = boundaryPoint;
                boundaryVertexIds.insert(hiddenVertexId);
            }
        }
        else
        {
            ALICEVISION_LOG_WARNING("A triangle was visible but is not visible anymore.");
        }
    }
}

void removeCameraVisibility(mesh::Mesh& inputMesh, int camId)
{
    #pragma omp parallel for
    for (int vertexId = 0; vertexId < inputMesh.pts.size(); ++vertexId)
    {
        auto& pointVisibilities = inputMesh.pointsVisibilities[vertexId];
        const int pointVisibilityIndex = pointVisibilities.indexOf(camId);
        if (pointVisibilityIndex != -1)
        {
            pointVisibilities.remove(pointVisibilityIndex);
        }
    }
}

void meshMasking(
    const mvsUtils::MultiViewParams & mp,
    mesh::Mesh & inputMesh,
    const std::vector<std::string> & masksFolders,
    const std::string & outputMeshPath,
    const int threshold,
    const bool invert,
    const bool smoothBoundary,
    const bool undistortMasks,
    const bool usePointsVisibilities
    )
{
    MaskCache maskCache(mp, masksFolders, undistortMasks);

    // compute visibility for every vertex
    // also update inputMesh.pointsVisibilities according to the masks
    ALICEVISION_LOG_INFO("Compute vertex visibilities");
    StaticVector<int> vertexVisibilityCounters;
    vertexVisibilityCounters.resize_with(inputMesh.pts.size(), 0);
    for (int camId = 0; camId < mp.getNbCameras(); ++camId)
    {
        auto* maskPtr = maskCache.lock(camId);
        if (!maskPtr)
        {
            if (usePointsVisibilities)
            {
                removeCameraVisibility(inputMesh, camId);
            }
            continue;
        }

        const auto& mask = *maskPtr;
        if (mp.getWidth(camId) != mask.Width() || mp.getHeight(camId) != mask.Height())
        {
            ALICEVISION_LOG_WARNING("Invalid mask size: mask is ignored.");
            if (usePointsVisibilities)
            {
                removeCameraVisibility(inputMesh, camId);
            }
            maskCache.unlock(camId);
            continue;
        }

        #pragma omp parallel for
        for (int vertexId = 0; vertexId < inputMesh.pts.size(); ++vertexId)
        {
            const auto& vertex = inputMesh.pts[vertexId];

            // check if the vertex is visible by the camera
            auto& pointVisibilities = inputMesh.pointsVisibilities[vertexId];
            const int pointVisibilityIndex = pointVisibilities.indexOf(camId);
            if (usePointsVisibilities && pointVisibilityIndex == -1)
            {
                continue;
            }

            // project vertex on mask
            Pixel projectedPixel;
            mp.getPixelFor3DPoint(&projectedPixel, vertex, camId);
            if (projectedPixel.x < 0 || projectedPixel.x >= mask.Width()
             || projectedPixel.y < 0 || projectedPixel.y >= mask.Height())
            {
                if (usePointsVisibilities)
                {
                    pointVisibilities.remove(pointVisibilityIndex);  // not visible
                }
                continue;
            }

            // get the mask value
            const bool maskValue = (mask(projectedPixel.y, projectedPixel.x) == 0);
            const bool masked = invert ? !maskValue : maskValue;
            if (masked)
            {
                if (usePointsVisibilities)
                {
                    pointVisibilities.remove(pointVisibilityIndex);  // not visible
                }
            }
            else
            {
                if (!usePointsVisibilities)
                {
                    pointVisibilities.push_back(camId);
                }
                ++vertexVisibilityCounters[vertexId];
            }
        }

        maskCache.unlock(camId);
    }

    // filter masked vertex (remove adjacent triangles)
    ALICEVISION_LOG_INFO("Filter triangles");
    mesh::Mesh filteredMesh;
    StaticVector<int> inputPtIdToFilteredPtId;

    {
        const auto isVertexVisible = [&vertexVisibilityCounters, threshold] (const int vertexId)
        {
            return vertexVisibilityCounters[vertexId] >= threshold;
        };

        StaticVector<int> visibleTriangles;
        visibleTriangles.reserve(inputMesh.tris.size() / 2);  // arbitrary size initial buffer
        for (int triangleId = 0; triangleId < inputMesh.tris.size(); ++triangleId)
        {
            const auto& triangle = inputMesh.tris[triangleId];
            const bool visible = smoothBoundary ?
                std::any_of(std::begin(triangle.v), std::end(triangle.v), isVertexVisible) :
                std::all_of(std::begin(triangle.v), std::end(triangle.v), isVertexVisible);
            if (visible)
            {
                visibleTriangles.push_back(triangleId);
            }
        }

        inputMesh.generateMeshFromTrianglesSubset(visibleTriangles, filteredMesh, inputPtIdToFilteredPtId);
    }

    if (smoothBoundary)
    {
        ALICEVISION_LOG_INFO("Smoothen boundary triangles");
        // build visibility counters + cameraId/point visibilities for the filtered mesh
        StaticVector<int> filteredVertexVisibilityCounters;
        filteredVertexVisibilityCounters.resize_with(filteredMesh.pts.size(), 0);
        filteredMesh.pointsVisibilities.resize(filteredMesh.pts.size());
        for (int inputPtId = 0; inputPtId < inputMesh.pts.size(); ++inputPtId)
        {
            const int filteredPtId = inputPtIdToFilteredPtId[inputPtId];
            if (filteredPtId >= 0)
            {
                filteredVertexVisibilityCounters[filteredPtId] = vertexVisibilityCounters[inputPtId];

                // fill visibilities and ensure they are sorted (we rely on it in findBoundaryVertex)
                auto& pointVisibilities = filteredMesh.pointsVisibilities[filteredPtId];
                pointVisibilities = inputMesh.pointsVisibilities[inputPtId];
                if (!std::is_sorted(pointVisibilities.begin(), pointVisibilities.end()))
                {
                    std::sort(pointVisibilities.begin(), pointVisibilities.end());
                }
            }
        }

        smoothenBoundary(filteredMesh, filteredVertexVisibilityCounters, mp, maskCache, threshold, invert);
    }

    // Save output mesh
    filteredMesh.save(outputMeshPath);

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputMeshPath << "\" saved.");
}


/**
 * @brief Write mask images from input images based on chosen algorithm.
 */
int main(int argc, char **argv)
{
    // command-line parameters
    std::string sfmFilePath;
    std::string inputMeshPath;
    std::vector<std::string> masksFolders;
    std::string outputMeshPath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    int threshold = 1;
    bool invert = false;
    bool smoothBoundary = false;
    bool undistortMasks = false;
    bool usePointsVisibilities = false;

    po::options_description allParams("AliceVision masking");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmFilePath)->default_value(sfmFilePath)->required(),
            "A SfMData file (*.sfm).")
        ("inputMesh,i", po::value<std::string>(&inputMeshPath)->required(),
            "Input Mesh")
        ("masksFolders", po::value<std::vector<std::string>>(&masksFolders)->multitoken(),
            "Use masks from specific folder(s).\n"
            "Filename should be the same or the image uid.")
        ("outputMesh,o", po::value<std::string>(&outputMeshPath)->required(),
            "Output mesh")
        ("threshold", po::value<int>(&threshold)->default_value(threshold)->notifier(optInRange(1, INT_MAX, "threshold"))->required(),
            "The minimum number of visibility to keep a vertex.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("invert", po::value<bool>(&invert)->default_value(invert),
            "Invert the mask.")
        ("smoothBoundary", po::value<bool>(&smoothBoundary)->default_value(smoothBoundary),
            "Modify the triangles at the boundary to fit the masks.")
        ("undistortMasks", po::value<bool>(&undistortMasks)->default_value(undistortMasks),
            "Undistort the masks with the same parameters as the matching image. Use it if the masks are drawn on the original images.")
        ("usePointsVisibilities", po::value<bool>(&usePointsVisibilities)->default_value(usePointsVisibilities),
            "Use the points visibilities from the meshing to filter triangles. Example: when they are occluded, back-face, etc.")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // check user choose at least one input option
    if(sfmFilePath.empty())
    {
        ALICEVISION_LOG_ERROR("Program need -i option" << std::endl << "No input images.");
        return EXIT_FAILURE;
    }

    // check input mesh
    ALICEVISION_LOG_INFO("Load input mesh.");
    mesh::Mesh inputMesh;
    inputMesh.load(inputMeshPath);

    // check sfm file
    if(!sfmFilePath.empty() && !fs::exists(sfmFilePath) && !fs::is_regular_file(sfmFilePath))
    {
        ALICEVISION_LOG_ERROR("The input sfm file doesn't exist");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmFilePath, sfmDataIO::ESfMData::ALL_DENSE))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmFilePath + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // check output string
    if(outputMeshPath.empty())
    {
        ALICEVISION_LOG_ERROR("Invalid output");
        return EXIT_FAILURE;
    }

    // ensure output folder exists
    fs::path outputDirectory = fs::path(outputMeshPath).parent_path();
    if(!outputDirectory.empty() && !fs::exists(outputDirectory))
    {
        if(!fs::create_directory(outputDirectory))
        {
            ALICEVISION_LOG_ERROR("Cannot create output folder");
            return EXIT_FAILURE;
        }
    }

    // execute
    system::Timer timer;

    mvsUtils::MultiViewParams mp(sfmData);

    if (usePointsVisibilities)
    {
        // load reference dense point cloud with visibilities
        ALICEVISION_LOG_INFO("Convert dense point cloud into ref mesh");
        mesh::Mesh refMesh;
        mvsUtils::createRefMeshFromDenseSfMData(refMesh, sfmData, mp);
        inputMesh.remapVisibilities(mesh::EVisibilityRemappingMethod::PullPush, refMesh);
    }
    else
    {
        // initialize points visibilities because it is later used to store vertex/mask visibilities
        inputMesh.pointsVisibilities.resize(inputMesh.pts.size());
    }

    ALICEVISION_LOG_INFO("Mask mesh");
    meshMasking(mp, inputMesh, masksFolders, outputMeshPath, threshold, invert, smoothBoundary, undistortMasks, usePointsVisibilities);
    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
