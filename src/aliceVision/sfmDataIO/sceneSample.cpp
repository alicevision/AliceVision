// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sceneSample.hpp"
#include <aliceVision/stl/hash.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/sfmData/ImageSequence.hpp>

namespace aliceVision {
namespace sfmDataIO {

ESceneType ESceneType_stringToEnum(const std::string& SScene)
{
    std::string scene = SScene;
    std::transform(scene.begin(), scene.end(), scene.begin(), ::tolower);

    if (scene == "cube")
    {
        return ESceneType::SCENE_CUBE;
    }
    if (scene == "sphere")
    {
        return ESceneType::SCENE_SPHERE;
    }
    if (scene == "planar")
    {
        return ESceneType::SCENE_PLANAR;
    }
    if (scene == "collinear")
    {
        return ESceneType::SCENE_COLLINEAR;
    }
    if (scene == "forwardmotion")
    {
        return ESceneType::SCENE_FORWARD_MOTION;
    }
    if (scene == "narrowbaseline")
    {
        return ESceneType::SCENE_NARROW_BASELINE;
    }
    if (scene == "unevencoverage")
    {
        return ESceneType::SCENE_UNEVEN_COVERAGE;
    }
    if (scene == "purerotation")
    {
        return ESceneType::SCENE_PURE_ROTATION;
    }
    if (scene == "sparseobservations")
    {
        return ESceneType::SCENE_SPARSE_OBSERVATIONS;
    }
    if (scene == "clusteredobservations")
    {
        return ESceneType::SCENE_CLUSTERED_OBSERVATIONS;
    }
    throw std::invalid_argument("Invalid scene type: " + scene);
}

std::string ESceneType_enumToString(const ESceneType EScene)
{
    if (EScene == ESceneType::SCENE_CUBE)
    {
        return "cube";
    }
    if (EScene == ESceneType::SCENE_SPHERE)
    {
        return "sphere";
    }
    if (EScene == ESceneType::SCENE_PLANAR)
    {
        return "planar";
    }
    if (EScene == ESceneType::SCENE_COLLINEAR)
    {
        return "collinear";
    }
    if (EScene == ESceneType::SCENE_FORWARD_MOTION)
    {
        return "forwardmotion";
    }
    if (EScene == ESceneType::SCENE_NARROW_BASELINE)
    {
        return "narrowbaseline";
    }
    if (EScene == ESceneType::SCENE_UNEVEN_COVERAGE)
    {
        return "unevencoverage";
    }
    if (EScene == ESceneType::SCENE_PURE_ROTATION)
    {
        return "purerotation";
    }
    if (EScene == ESceneType::SCENE_SPARSE_OBSERVATIONS)
    {
        return "sparseobservations";
    }
    if (EScene == ESceneType::SCENE_CLUSTERED_OBSERVATIONS)
    {
        return "clusteredobservations";
    }
    throw std::invalid_argument("Unrecognized ESceneType: " + std::to_string(int(EScene)));
}

std::ostream& operator<<(std::ostream& os, ESceneType p) { return os << ESceneType_enumToString(p); }

std::istream& operator>>(std::istream& in, ESceneType& p)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    p = ESceneType_stringToEnum(token);
    return in;
}


void generateSampleScene(sfmData::SfMData& output, ESceneType scene)
{
    // Cleanup sfmData
    output.clear();

    if (scene == ESceneType::SCENE_CUBE)
    {
        generateCubeScene(output);
    }
    else if (scene == ESceneType::SCENE_SPHERE)
    {
        generateSphereScene(output, 1000, 240);
    }
    else if (scene == ESceneType::SCENE_PLANAR)
    {
        generatePlanarScene(output, 30, 60);
    }
    else if (scene == ESceneType::SCENE_COLLINEAR)
    {
        generateCollinearCamerasScene(output, 500, 20);
    }
    else if (scene == ESceneType::SCENE_FORWARD_MOTION)
    {
        generateForwardMotionScene(output, 500, 30);
    }
    else if (scene == ESceneType::SCENE_NARROW_BASELINE)
    {
        generateNarrowBaselineScene(output, 1000, 240);
    }
    else if (scene == ESceneType::SCENE_UNEVEN_COVERAGE)
    {
        generateUnevenCoverageScene(output, 200, 800, 240);
    }
    else if (scene == ESceneType::SCENE_PURE_ROTATION)
    {
        generatePureRotationScene(output, 1000, 240);
    }
    else if (scene == ESceneType::SCENE_SPARSE_OBSERVATIONS)
    {
        generateSparseCameraObservationsScene(output, 100, 60, 5);
    }
    else if (scene == ESceneType::SCENE_CLUSTERED_OBSERVATIONS)
    {
        generateClusteredObservationsScene(output, 200, 240);
    }
    else
    {
        throw std::out_of_range("Invalid ESceneType");
    }
}


void generateCubeScene(sfmData::SfMData& output)
{
    // Generate points on a cube
    IndexT idpt = 0;
    for (int x = -10; x <= 10; ++x)
    {
        for (int y = -10; y <= 10; ++y)
        {
            for (int z = -10; z <= 10; ++z)
            {
                output.getLandmarks().emplace(idpt, sfmData::Landmark(Vec3(x, y, z), feature::EImageDescriberType::UNKNOWN));
                ++idpt;
            }
        }
    }

    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 1000.0;
    const double focalLengthPixY = 2000.0;
    const double offsetX = -26;
    const double offsetY = 16;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY));
    output.getIntrinsics().emplace(
      1,
      camera::createPinhole(camera::DISTORTION_RADIALK3, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, {0.1, 0.05, -0.001}));

    // Generate poses on another cube
    IndexT idpose = 0;
    IndexT idview = 0;
    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            for (int z = -1; z <= 1; ++z)
            {
                const Eigen::Vector3d thetau(x, y, z);
                const Eigen::AngleAxis<double> aa(thetau.norm(), thetau.normalized());

                geometry::Pose3 pose(aa.toRotationMatrix(), Vec3(x, y, z));

                output.getPoses().assign(idpose, sfmData::CameraPose(pose));

                for (const auto itIntrinsic : output.getIntrinsics())
                {
                    output.getViews().emplace(idview, std::make_shared<sfmData::View>("", idview, itIntrinsic.first, idpose, w, h));
                    ++idview;
                }

                ++idpose;
            }
        }
    }
}

void generateSphereScene(sfmData::SfMData& output, int pointsNb, int posesNb)
{
    // Generate random points on a sphere

    size_t sphereSceneHash = 4546135487;

    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 10000.0;
    const double focalLengthPixY = 20000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, 0, 0));

    size_t imageGroupID = sphereSceneHash;
    stl::hash_combine(imageGroupID, pointsNb);
    stl::hash_combine(imageGroupID, posesNb);

    // Generate poses on a circle
    for (IndexT idPV = 0; idPV < posesNb; idPV++)
    {
        double angle2d = (double(idPV) * 2. * M_PI) / posesNb;
        Eigen::AngleAxis<double> aa(angle2d + .5 * M_PI, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d rotation = aa.toRotationMatrix();
        Eigen::Vector3d position(std::cos(angle2d), 0., std::sin(angle2d));
        geometry::Pose3 pose(rotation, 10. * position);
        output.getPoses().assign(idPV, sfmData::CameraPose(pose));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
        output.getView(idPV).setFrameId(idPV);
        output.getView(idPV).setImageGroupId(imageGroupID);
    }

    auto imageGroupPtr = std::make_shared<sfmData::ImageSequence>();
    output.getImageGroups().emplace(imageGroupID, imageGroupPtr);

    // Generate random points on a sphere and corresponding observations in each view
    const double arbitraryScale = 1.0;
    for (int landmarkId = 0; landmarkId < pointsNb; landmarkId++)
    {
        Eigen::Vector3d point3D = Eigen::Vector3d::Random();
        point3D = point3D / point3D.norm();

        sfmData::Landmark landmark(point3D, feature::EImageDescriberType::UNKNOWN);
        for (int viewId = 0; viewId < posesNb; viewId++)
        {
            const sfmData::View& view = output.getView(viewId);
            const geometry::Pose3 pose = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intrinsic = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intrinsic.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, landmarkId, arbitraryScale));
        }
        output.getLandmarks().emplace(landmarkId, landmark);
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

namespace
{

/**
 * @brief Build a world-to-camera rotation matrix from an eye position, a look-at
 * target, and a world-up vector.
 *
 * The resulting matrix R satisfies: X_cam = R * (X_world - eye).
 *
 * Axes:
 *   zAxis (optical axis, into scene)  = (target - eye).normalized()
 *   xAxis (right)                     = worldUp × zAxis, normalized
 *   yAxis (down in camera frame)      = zAxis × xAxis
 *
 * When worldUp is nearly parallel to zAxis (degenerate case), a fallback
 * direction (0,0,1) is used for the cross product.
 *
 * @param eye      Camera centre in world coordinates.
 * @param target   World point that the camera looks towards.
 * @param worldUp  World up direction (does not need to be perpendicular to the axis).
 * @return World-to-camera rotation matrix (3×3, rows = camera basis vectors).
 */
Eigen::Matrix3d computeLookAtRotation(const Eigen::Vector3d& eye,
                                      const Eigen::Vector3d& target,
                                      const Eigen::Vector3d& worldUp)
{
    Eigen::Vector3d zAxis = (target - eye).normalized();

    Eigen::Vector3d xAxis = worldUp.cross(zAxis);
    if (xAxis.norm() < 1e-6)
    {
        // worldUp is (nearly) parallel to zAxis – use a fallback
        const Eigen::Vector3d fallback = (std::abs(zAxis.z()) < 0.9)
                                           ? Eigen::Vector3d::UnitZ()
                                           : Eigen::Vector3d::UnitX();
        xAxis = fallback.cross(zAxis);
    }
    xAxis.normalize();

    const Eigen::Vector3d yAxis = zAxis.cross(xAxis);

    Eigen::Matrix3d R;
    R.row(0) = xAxis;
    R.row(1) = yAxis;
    R.row(2) = zAxis;
    return R;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// New scene generators
// ---------------------------------------------------------------------------

void generatePlanarScene(sfmData::SfMData& output, int gridSize, int posesNb)
{
    const int w = 4092;
    const int h = 2048;
    const double focal = 10000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focal, focal, 0, 0));

    // Landmarks on a regular grid in the z=0 plane, x/y in [-1, 1].
    IndexT landmarkId = 0;
    for (int ix = 0; ix < gridSize; ++ix)
    {
        for (int iy = 0; iy < gridSize; ++iy)
        {
            const double x = (gridSize > 1) ? -1.0 + 2.0 * ix / (gridSize - 1) : 0.0;
            const double y = (gridSize > 1) ? -1.0 + 2.0 * iy / (gridSize - 1) : 0.0;
            output.getLandmarks().emplace(landmarkId,
                sfmData::Landmark(Eigen::Vector3d(x, y, 0.0), feature::EImageDescriberType::UNKNOWN));
            ++landmarkId;
        }
    }

    // Cameras on a circle of radius 3 at height z = 5, looking at the origin.
    // worldUp = +Z because the plane lies in XY and cameras are above it.
    const double cameraRadius = 3.0;
    const double cameraHeight = 5.0;
    const Eigen::Vector3d sceneCenter(0.0, 0.0, 0.0);
    const Eigen::Vector3d worldUp(0.0, 0.0, 1.0);

    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        const double angle = (double(idPV) * 2.0 * M_PI) / posesNb;
        Eigen::Vector3d eye(cameraRadius * std::cos(angle), cameraRadius * std::sin(angle), cameraHeight);
        Eigen::Matrix3d R = computeLookAtRotation(eye, sceneCenter, worldUp);
        output.getPoses().assign(idPV, sfmData::CameraPose(geometry::Pose3(R, eye)));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    // Build observations by projecting every landmark into every view.
    const double arbitraryScale = 1.0;
    for (auto& [lmId, landmark] : output.getLandmarks())
    {
        for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
        {
            const sfmData::View& view       = output.getView(viewId);
            const geometry::Pose3 pose      = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, lmId, arbitraryScale));
        }
    }
}

void generateCollinearCamerasScene(sfmData::SfMData& output, int pointsNb, int posesNb)
{
    const int w = 4092;
    const int h = 2048;
    const double focal = 10000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focal, focal, 0, 0));

    // Random landmarks in a box ahead of the cameras (+Z direction).
    std::srand(12345);
    for (int id = 0; id < pointsNb; ++id)
    {
        Eigen::Vector3d pt = Eigen::Vector3d::Random(); // components in [-1, 1]
        pt.x() *= 3.0;
        pt.y() *= 3.0;
        pt.z() = 5.0 + (pt.z() + 1.0) * 2.5; // remap to [5, 10]
        output.getLandmarks().emplace(id,
            sfmData::Landmark(pt, feature::EImageDescriberType::UNKNOWN));
    }

    // Cameras uniformly spaced along the X axis, all looking in the +Z direction.
    // The spacing between cameras is 0.3; the array is centred at the origin.
    const double spacing = 0.3;
    const Eigen::Vector3d lookDir(0.0, 0.0, 1.0);
    const Eigen::Vector3d worldUp(0.0, 1.0, 0.0);

    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        const double xPos = spacing * (double(idPV) - double(posesNb - 1) / 2.0);
        Eigen::Vector3d eye(xPos, 0.0, 0.0);
        Eigen::Matrix3d R = computeLookAtRotation(eye, eye + lookDir, worldUp);
        output.getPoses().assign(idPV, sfmData::CameraPose(geometry::Pose3(R, eye)));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    // Build observations.
    const double arbitraryScale = 1.0;
    for (auto& [lmId, landmark] : output.getLandmarks())
    {
        for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
        {
            const sfmData::View& view         = output.getView(viewId);
            const geometry::Pose3 pose        = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, lmId, arbitraryScale));
        }
    }
}

void generateForwardMotionScene(sfmData::SfMData& output, int pointsNb, int posesNb)
{
    const int w = 4092;
    const int h = 2048;
    const double focal = 10000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focal, focal, 0, 0));

    // Camera step along Z.
    const double step = 0.4;
    const double cameraTrainLength = step * (posesNb - 1);

    // Random landmarks in a box well ahead of the entire camera train.
    std::srand(23456);
    for (int id = 0; id < pointsNb; ++id)
    {
        Eigen::Vector3d pt = Eigen::Vector3d::Random();
        pt.x() *= 1.0; // x in [-1, 1]
        pt.y() *= 1.0; // y in [-1, 1]
        // z well ahead of the last camera: [cameraTrainLength + 4, cameraTrainLength + 8]
        pt.z() = cameraTrainLength + 4.0 + (pt.z() + 1.0) * 2.0;
        output.getLandmarks().emplace(id,
            sfmData::Landmark(pt, feature::EImageDescriberType::UNKNOWN));
    }

    // Cameras at (0, 0, i*step), all looking in +Z.
    const Eigen::Vector3d lookDir(0.0, 0.0, 1.0);
    const Eigen::Vector3d worldUp(0.0, 1.0, 0.0);

    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        Eigen::Vector3d eye(0.0, 0.0, double(idPV) * step);
        Eigen::Matrix3d R = computeLookAtRotation(eye, eye + lookDir, worldUp);
        output.getPoses().assign(idPV, sfmData::CameraPose(geometry::Pose3(R, eye)));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    // Build observations.
    const double arbitraryScale = 1.0;
    for (auto& [lmId, landmark] : output.getLandmarks())
    {
        for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
        {
            const sfmData::View& view         = output.getView(viewId);
            const geometry::Pose3 pose        = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, lmId, arbitraryScale));
        }
    }
}

void generateNarrowBaselineScene(sfmData::SfMData& output, int pointsNb, int posesNb)
{
    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 10000.0;
    const double focalLengthPixY = 20000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, 0, 0));

    // Very small camera circle radius (0.05) vs scene depth (~1.0) → narrow baseline.
    const double cameraRadius = 0.05;

    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        const double angle2d = (double(idPV) * 2.0 * M_PI) / posesNb;
        Eigen::AngleAxis<double> aa(angle2d + 0.5 * M_PI, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d rotation = aa.toRotationMatrix();
        Eigen::Vector3d position(std::cos(angle2d), 0.0, std::sin(angle2d));
        geometry::Pose3 pose(rotation, cameraRadius * position);
        output.getPoses().assign(idPV, sfmData::CameraPose(pose));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    // Landmarks uniformly distributed on the unit sphere.
    std::srand(34567);
    const double arbitraryScale = 1.0;
    for (int landmarkId = 0; landmarkId < pointsNb; ++landmarkId)
    {
        Eigen::Vector3d point3D = Eigen::Vector3d::Random().normalized();

        sfmData::Landmark landmark(point3D, feature::EImageDescriberType::UNKNOWN);
        for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
        {
            const sfmData::View& view         = output.getView(viewId);
            const geometry::Pose3 pose        = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            // Skip landmarks behind the camera (negative depth in camera space).
            // For cameras at radius 0.05 looking inward, roughly half of unit-sphere
            // landmarks lie behind each camera and must not generate observations.
            // Also skip near-grazing observations (z_c < 0.1): these have tiny depth
            // values that produce enormous Jacobian entries (f/z_c >> 1) which inflate
            // column norms and create artificially small covariance estimates.
            const double minDepth = 0.1;
            const Vec3 transformed = pose(landmark.getX());
            if (transformed.z() <= minDepth)
            {
                continue;
            }

            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, landmarkId, arbitraryScale));
        }
        output.getLandmarks().emplace(landmarkId, landmark);
    }
}

void generateUnevenCoverageScene(sfmData::SfMData& output, int wellObservedNb, int poorlyObservedNb, int posesNb)
{
    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 10000.0;
    const double focalLengthPixY = 20000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, 0, 0));

    // Cameras arranged on a circle of radius 10, identical to the sphere scene.
    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        const double angle2d = (double(idPV) * 2.0 * M_PI) / posesNb;
        Eigen::AngleAxis<double> aa(angle2d + 0.5 * M_PI, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d rotation = aa.toRotationMatrix();
        Eigen::Vector3d position(std::cos(angle2d), 0.0, std::sin(angle2d));
        geometry::Pose3 pose(rotation, 10.0 * position);
        output.getPoses().assign(idPV, sfmData::CameraPose(pose));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    const double arbitraryScale = 1.0;
    IndexT landmarkId = 0;

    // Well-observed landmarks: random points near the origin (|X| <= 0.3),
    // observed by every camera.
    std::srand(45678);
    for (int i = 0; i < wellObservedNb; ++i)
    {
        Eigen::Vector3d point3D = Eigen::Vector3d::Random() * 0.3;
        sfmData::Landmark landmark(point3D, feature::EImageDescriberType::UNKNOWN);
        for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
        {
            const sfmData::View& view         = output.getView(viewId);
            const geometry::Pose3 pose        = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, landmarkId, arbitraryScale));
        }
        output.getLandmarks().emplace(landmarkId, landmark);
        ++landmarkId;
    }

    // Poorly-observed landmarks: random points on the unit sphere, each seen
    // by only 2 adjacent cameras (k and k+1 mod posesNb).
    for (int i = 0; i < poorlyObservedNb; ++i)
    {
        Eigen::Vector3d point3D = Eigen::Vector3d::Random().normalized();
        sfmData::Landmark landmark(point3D, feature::EImageDescriberType::UNKNOWN);

        const IndexT k = static_cast<IndexT>(i % posesNb);
        const IndexT views[2] = {k, (k + 1) % static_cast<IndexT>(posesNb)};
        for (IndexT viewId : views)
        {
            const sfmData::View& view         = output.getView(viewId);
            const geometry::Pose3 pose        = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, landmarkId, arbitraryScale));
        }
        output.getLandmarks().emplace(landmarkId, landmark);
        ++landmarkId;
    }
}

void generatePureRotationScene(sfmData::SfMData& output, int pointsNb, int posesNb)
{
    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 10000.0;
    const double focalLengthPixY = 20000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, 0, 0));

    // All cameras share the origin but have distinct orientations (sphere scene angles).
    // The Jacobian d(proj)/d(depth) is identically zero for any point under perspective
    // projection when the camera centre coincides with the world origin and the point
    // lies on the unit sphere, so radial depth is completely unobservable.
    const Eigen::Vector3d sharedCenter(0.0, 0.0, 0.0);
    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        const double angle2d = (double(idPV) * 2.0 * M_PI) / posesNb;
        Eigen::AngleAxis<double> aa(angle2d + 0.5 * M_PI, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d rotation = aa.toRotationMatrix();
        // All cameras are at the origin; only the rotation changes.
        geometry::Pose3 pose(rotation, sharedCenter);
        output.getPoses().assign(idPV, sfmData::CameraPose(pose));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    // Landmarks on the unit sphere, projected into all views.
    std::srand(56789);
    const double arbitraryScale = 1.0;
    for (int landmarkId = 0; landmarkId < pointsNb; ++landmarkId)
    {
        Eigen::Vector3d point3D = Eigen::Vector3d::Random().normalized();

        sfmData::Landmark landmark(point3D, feature::EImageDescriberType::UNKNOWN);
        for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
        {
            const sfmData::View& view         = output.getView(viewId);
            const geometry::Pose3 pose        = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, landmarkId, arbitraryScale));
        }
        output.getLandmarks().emplace(landmarkId, landmark);
    }
}

void generateSparseCameraObservationsScene(sfmData::SfMData& output, int pointsNb, int posesNb, int obsPerCamera)
{
    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 10000.0;
    const double focalLengthPixY = 20000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, 0, 0));

    // Cameras arranged on a circle of radius 10, all looking at the origin.
    // worldUp = +Y (camera centres lie in the XZ plane).
    const Eigen::Vector3d sceneCenter(0.0, 0.0, 0.0);
    const Eigen::Vector3d worldUp(0.0, 1.0, 0.0);
    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        const double angle2d = (double(idPV) * 2.0 * M_PI) / posesNb;
        const Eigen::Vector3d eye(10.0 * std::cos(angle2d), 0.0, 10.0 * std::sin(angle2d));
        const Eigen::Matrix3d R = computeLookAtRotation(eye, sceneCenter, worldUp);
        output.getPoses().assign(idPV, sfmData::CameraPose(geometry::Pose3(R, eye)));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    // Landmarks randomly distributed in the cube [-0.5, 0.5]^3 near the origin.
    // All landmarks are visible from every camera (cameras are at radius 10, looking inward).
    std::srand(78901);
    for (int id = 0; id < pointsNb; ++id)
    {
        const Eigen::Vector3d pt = Eigen::Vector3d::Random() * 0.5;
        output.getLandmarks().emplace(id, sfmData::Landmark(pt, feature::EImageDescriberType::UNKNOWN));
    }

    // Each camera observes exactly obsPerCamera landmarks using a round-robin
    // assignment over the landmark list.  With obsPerCamera << pointsNb the
    // bundle-adjustment Jacobian block for each camera has very few rows, so
    // the camera translation is weakly determined and carries high positional
    // uncertainty.  Each landmark is seen by approximately
    //   posesNb * obsPerCamera / pointsNb
    // cameras and is therefore reasonably well constrained.
    const double arbitraryScale = 1.0;
    for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
    {
        const sfmData::View& view         = output.getView(viewId);
        const geometry::Pose3 pose        = output.getPose(view).getTransform();
        const camera::IntrinsicBase& intr = output.getIntrinsic(0);

        for (int k = 0; k < obsPerCamera; ++k)
        {
            const IndexT lmId = (static_cast<IndexT>(viewId) * obsPerCamera + k) % pointsNb;
            auto& landmark    = output.getLandmarks().at(lmId);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, lmId, arbitraryScale));
        }
    }
}

void generateClusteredObservationsScene(sfmData::SfMData& output, int pointsNb, int posesNb)
{
    const int w = 4092;
    const int h = 2048;
    const double focalLengthPixX = 10000.0;
    const double focalLengthPixY = 20000.0;
    output.getIntrinsics().emplace(
      0, camera::createPinhole(camera::DISTORTION_NONE, camera::UNDISTORTION_NONE, w, h, focalLengthPixX, focalLengthPixY, 0, 0));

    // Cameras arranged on a circle of radius 10, all looking at the origin.
    // worldUp = +Y (camera centres lie in the XZ plane).
    const Eigen::Vector3d sceneCenter(0.0, 0.0, 0.0);
    const Eigen::Vector3d worldUp(0.0, 1.0, 0.0);
    for (IndexT idPV = 0; idPV < static_cast<IndexT>(posesNb); ++idPV)
    {
        const double angle2d = (double(idPV) * 2.0 * M_PI) / posesNb;
        const Eigen::Vector3d eye(10.0 * std::cos(angle2d), 0.0, 10.0 * std::sin(angle2d));
        const Eigen::Matrix3d R = computeLookAtRotation(eye, sceneCenter, worldUp);
        output.getPoses().assign(idPV, sfmData::CameraPose(geometry::Pose3(R, eye)));
        output.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, w, h));
    }

    // Landmarks placed in a narrow cone pointing towards +Z direction from the origin.
    // The cone has angular width ~±7.5° (≈15% of image width/height for a typical camera).
    // All landmarks, when projected from any camera, fall into a small clustered region.
    // This creates severe directional ambiguity: cameras see all features in nearly the
    // same direction, leading to high depth and lateral position uncertainty.
    std::srand(89012);
    const double maxConeAngle = 7.5 * M_PI / 180.0; // ±7.5 degrees in radians (~13% field of view)
    const double coneDepthMin = 0.5;
    const double coneDepthMax = 2.0;

    for (int id = 0; id < pointsNb; ++id)
    {
        // Random direction within a narrow cone (small angles around +Z axis)
        double theta = (std::rand() / double(RAND_MAX)) * 2.0 * maxConeAngle - maxConeAngle; // azimuth angle within cone
        double phi = (std::rand() / double(RAND_MAX)) * 2.0 * maxConeAngle - maxConeAngle;   // elevation angle within cone
        double depth = coneDepthMin + (std::rand() / double(RAND_MAX)) * (coneDepthMax - coneDepthMin);

        // Convert spherical coordinates to Cartesian (with small angles, cos(angle) ≈ 1)
        Eigen::Vector3d point3D;
        point3D.x() = depth * std::sin(theta);
        point3D.y() = depth * std::sin(phi);
        point3D.z() = depth * std::cos(theta) * std::cos(phi); // ≈ depth for small angles

        output.getLandmarks().emplace(id, sfmData::Landmark(point3D, feature::EImageDescriberType::UNKNOWN));
    }

    // Build observations: all landmarks projected into all views.
    // The narrow clustering ensures that all observations fall within ~15% × 15% of the image.
    const double arbitraryScale = 1.0;
    for (auto& [lmId, landmark] : output.getLandmarks())
    {
        for (IndexT viewId = 0; viewId < static_cast<IndexT>(posesNb); ++viewId)
        {
            const sfmData::View& view         = output.getView(viewId);
            const geometry::Pose3 pose        = output.getPose(view).getTransform();
            const camera::IntrinsicBase& intr = output.getIntrinsic(0);
            const Eigen::Vector2d pt = intr.transformProject(pose, landmark.getX().homogeneous(), true);
            landmark.getObservations().emplace(viewId, sfmData::Observation(pt, lmId, arbitraryScale));
        }
    }
}

}  // namespace sfmDataIO
}  // namespace aliceVision
