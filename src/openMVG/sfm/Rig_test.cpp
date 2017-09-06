#include "sfm_data.hpp"

#include "testing/testing.h"

#include <memory>

using namespace openMVG;
using namespace openMVG::sfm;

TEST(Rig, initialization)
{
  static constexpr IndexT rigId = 0;
  static constexpr std::size_t nbSubPoses = 5;

  SfM_Data sfmData;

  sfmData.intrinsics.emplace(0, std::make_shared<cameras::Pinhole_Intrinsic>(1000, 1000, 36.0, std::rand()%10000, std::rand()%10000));
  sfmData.getRigs().emplace(rigId, Rig(nbSubPoses));

  std::vector<View> rigViews;

  for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
  {
    rigViews.emplace_back("", subPoseId, 0, 0, 10, 10, rigId, subPoseId);
  }

  for(const View& view : rigViews)
  {
    sfmData.views.emplace(view.getViewId(), std::make_shared<View>(view));
  }

  const Mat3 r = Mat3::Random();
  const Vec3 c = Vec3::Random();
  const geometry::Pose3 firstPose = geometry::Pose3(r, c);
  const IndexT firstPoseId = std::rand() % nbSubPoses;

  const Rig& rig = sfmData.getRig(rigViews.front());

  // rig uninitialized
  EXPECT_FALSE(rig.isInitialized());

  sfmData.setPose(rigViews.at(firstPoseId), firstPose);

  // setPose done, rig initialized
  EXPECT_TRUE(rig.isInitialized());

  // Check rig sub-poses
  for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
  {
    const RigSubPose& subPose = rig.getSubPose(subPoseId);

    if(subPoseId == firstPoseId)
    {
      // first sub-pose should be initialized
      EXPECT_TRUE(subPose.status != ERigSubPoseStatus::UNINITIALIZED);
    }
    else
    {
      // other sub-poses are uninitialized
      EXPECT_TRUE(subPose.status == ERigSubPoseStatus::UNINITIALIZED);
    }

    // all sub-poses should be at identity
    EXPECT_TRUE(subPose.pose == geometry::Pose3())
  }

  // Check rig pose
  for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
  {
    const View& view = rigViews.at(subPoseId);
    EXPECT_TRUE(sfmData.getPose(view) == firstPose);
  }
}

TEST(Rig, setPose)
{
  static constexpr IndexT rigId = 0;
  static constexpr std::size_t nbSubPoses = 5;
  static constexpr std::size_t nbPoses = 2;

  SfM_Data sfmData;

  sfmData.intrinsics.emplace(0, std::make_shared<cameras::Pinhole_Intrinsic>(1000, 1000, 36.0, std::rand()%10000, std::rand()%10000));
  sfmData.getRigs().emplace(rigId, Rig(nbSubPoses));

  std::vector<View> rigViews;

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      rigViews.emplace_back("", viewId, 0, poseId, 10, 10, rigId, subPoseId);
    }
  }

  for(const View& view : rigViews)
  {
    sfmData.views.emplace(view.getViewId(), std::make_shared<View>(view));
  }

  const Rig& rig = sfmData.getRig(rigViews.front());

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      const View& view = *(sfmData.views.at(viewId));
      const RigSubPose& subPose = rig.getSubPose(subPoseId);

      if(subPoseId == 0)
      {
        // first sub-pose, rig pose is unknown
        EXPECT_FALSE(sfmData.existsPose(view));

        if(poseId == 0)
        {
          // first rig pose, first sub-pose, sub-pose uninitialized
          EXPECT_TRUE(subPose.status == ERigSubPoseStatus::UNINITIALIZED)
        }
        else
        {
          // not first rig pose, first sub-pose, sub-pose initialized
          EXPECT_TRUE(subPose.status != ERigSubPoseStatus::UNINITIALIZED)
        }

        const Mat3 r = Mat3::Random();
        const Vec3 c = Vec3::Random();
        const geometry::Pose3 firstPose = geometry::Pose3(r, c);

        sfmData.setPose(view, firstPose);

        // setPose done, sub-pose must be initialized
        EXPECT_TRUE(subPose.status != ERigSubPoseStatus::UNINITIALIZED)

        // setPose done, rig initialized
        EXPECT_TRUE(sfmData.existsPose(view));
        EXPECT_TRUE(sfmData.getPose(view) == firstPose);
      }
      else
      {

        // rig pose should be initialized
        EXPECT_TRUE(sfmData.existsPose(view));

        if(poseId == 0) //other poses are redundant
        {
          const Mat3 r = Mat3::Random();
          const Vec3 c = Vec3::Random();
          const geometry::Pose3 pose = geometry::Pose3(r, c);

          // first rig pose, sub-pose must be uninitialized
          EXPECT_TRUE(subPose.status == ERigSubPoseStatus::UNINITIALIZED)

          sfmData.setPose(view, pose);

          // setPose done, sub-pose must be initialized
          EXPECT_TRUE(subPose.status != ERigSubPoseStatus::UNINITIALIZED)
        }
      }
    }
  }
}

TEST(Rig, getPose)
{
  static constexpr IndexT rigId = 0;
  static constexpr std::size_t nbSubPoses = 5;
  static constexpr std::size_t nbPoses = 2;

  SfM_Data sfmData;

  sfmData.intrinsics.emplace(0, std::make_shared<cameras::Pinhole_Intrinsic>(1000, 1000, 36.0, std::rand()%10000, std::rand()%10000));
  sfmData.getRigs().emplace(rigId, Rig(nbSubPoses));

  std::vector<View> rigViews;

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      rigViews.emplace_back("", viewId, 0, poseId, 10, 10, rigId, subPoseId);
    }
  }

  for(const View& view : rigViews)
  {
    sfmData.views.emplace(view.getViewId(), std::make_shared<View>(view));
  }

  const Rig& rig = sfmData.getRig(rigViews.front());

  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subPoseId = 0; subPoseId < nbSubPoses; ++subPoseId)
    {
      const IndexT viewId = poseId * nbSubPoses + subPoseId;
      const View& view = *(sfmData.views.at(viewId));
      const RigSubPose& subPose = rig.getSubPose(subPoseId);

      if(subPoseId == 0)
      {
        const Mat3 r = Mat3::Random();
        const Vec3 c = Vec3::Random();
        const geometry::Pose3 firstPose = geometry::Pose3(r, c);

        sfmData.setPose(view, firstPose);

        // setPose done, rig initialized
        if(poseId == 0)
        {
          // the rig pose is the first pose
          EXPECT_TRUE(sfmData.getPose(view) == firstPose);
        }
        else
        {
          // the rig pose is the sub-pose inverse multiply by the rig pose
          EXPECT_TRUE(sfmData.getPose(view) == (subPose.pose.inverse() * firstPose));
        }

      }
      else
      {
        const geometry::Pose3& rigPose = sfmData.getPose(view);

        if(poseId == 0) //other poses are redundant
        {
          const Mat3 r = Mat3::Random();
          const Vec3 c = Vec3::Random();
          const geometry::Pose3 absolutePose = geometry::Pose3(r, c);

          sfmData.setPose(view, absolutePose);

          // the view sub-pose is the absolute pose multiply by the rig pose inverse
          EXPECT_TRUE(subPose.pose == (absolutePose * rigPose.inverse()))
        }

        // the view absolute pose is the sub-pose multiply by the rig pose
        EXPECT_TRUE(sfmData.getPose(view) == (subPose.pose * sfmData.GetPoses().at(view.getPoseId())));
      }
    }
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
