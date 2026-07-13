#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "piper_mtc_tasks/pregrasp_geometry.hpp"

namespace piper_mtc_tasks
{
namespace
{

geometry_msgs::msg::PoseStamped make_grasp_pose()
{
  geometry_msgs::msg::PoseStamped grasp_pose;
  grasp_pose.header.frame_id = "world";
  grasp_pose.header.stamp.sec = 42;
  grasp_pose.pose.position.x = 0.6921;
  grasp_pose.pose.position.y = 0.0198;
  grasp_pose.pose.position.z = 0.2052;
  const double half_pitch = -std::acos(-1.0) / 4.0;
  grasp_pose.pose.orientation.x = 0.0;
  grasp_pose.pose.orientation.y = std::sin(half_pitch);
  grasp_pose.pose.orientation.z = 0.0;
  grasp_pose.pose.orientation.w = std::cos(half_pitch);
  return grasp_pose;
}

TEST(MakePlanningFramePregraspPose, PreservesWorldApproachForPitchedGrasp)
{
  const auto grasp_pose = make_grasp_pose();

  const auto pregrasp_pose = make_planning_frame_pregrasp_pose(
    grasp_pose, XYZ{1.0, 0.0, 0.0}, 0.07);

  ASSERT_TRUE(pregrasp_pose.has_value());
  EXPECT_NEAR(pregrasp_pose->pose.position.x, 0.6221, 1e-9);
  EXPECT_DOUBLE_EQ(grasp_pose.pose.position.y, pregrasp_pose->pose.position.y);
  EXPECT_DOUBLE_EQ(grasp_pose.pose.position.z, pregrasp_pose->pose.position.z);
  EXPECT_EQ(grasp_pose.header, pregrasp_pose->header);
  EXPECT_EQ(grasp_pose.pose.orientation, pregrasp_pose->pose.orientation);
}

TEST(MakePlanningFramePregraspPose, NormalizesNonZeroXYDirection)
{
  const auto grasp_pose = make_grasp_pose();

  const auto pregrasp_pose = make_planning_frame_pregrasp_pose(
    grasp_pose, XYZ{3.0, 4.0, 0.0}, 0.10);

  ASSERT_TRUE(pregrasp_pose.has_value());
  EXPECT_NEAR(grasp_pose.pose.position.x - pregrasp_pose->pose.position.x, 0.06, 1e-9);
  EXPECT_NEAR(grasp_pose.pose.position.y - pregrasp_pose->pose.position.y, 0.08, 1e-9);
  EXPECT_DOUBLE_EQ(grasp_pose.pose.position.z, pregrasp_pose->pose.position.z);
}

TEST(MakePlanningFramePregraspPose, RejectsZeroDirection)
{
  EXPECT_FALSE(make_planning_frame_pregrasp_pose(
      make_grasp_pose(), XYZ{0.0, 0.0, 0.0}, 0.07)
      .has_value());
}

TEST(MakePlanningFramePregraspPose, RejectsNonFiniteDistance)
{
  const auto grasp_pose = make_grasp_pose();

  EXPECT_FALSE(make_planning_frame_pregrasp_pose(
      grasp_pose, XYZ{1.0, 0.0, 0.0}, std::numeric_limits<double>::quiet_NaN())
      .has_value());
  EXPECT_FALSE(make_planning_frame_pregrasp_pose(
      grasp_pose, XYZ{1.0, 0.0, 0.0}, std::numeric_limits<double>::infinity())
      .has_value());
  EXPECT_FALSE(make_planning_frame_pregrasp_pose(
      grasp_pose, XYZ{1.0, 0.0, 0.0}, -std::numeric_limits<double>::infinity())
      .has_value());
}

TEST(IsCompleteCartesianPathFraction, AcceptsOnlyCompleteFiniteFractions)
{
  EXPECT_TRUE(is_complete_cartesian_path_fraction(1.0));
  EXPECT_TRUE(is_complete_cartesian_path_fraction(1.0 - 5e-7));
  EXPECT_FALSE(is_complete_cartesian_path_fraction(0.99));
  EXPECT_FALSE(is_complete_cartesian_path_fraction(0.538));
  EXPECT_FALSE(is_complete_cartesian_path_fraction(std::numeric_limits<double>::quiet_NaN()));
}

TEST(MakePlanningFramePostgraspEscapePose, UsesRecordedRadialSideEscapeOffset)
{
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = "piper_base_link";
  current_pose.header.stamp.sec = 42;
  current_pose.pose.position.x = 0.6998;
  current_pose.pose.position.y = -0.2243;
  current_pose.pose.position.z = 0.2047;
  current_pose.pose.orientation.y = 0.7071;
  current_pose.pose.orientation.w = 0.7071;

  const auto escape_pose = make_planning_frame_postgrasp_escape_pose(
    current_pose, XYZ{1.0, 0.0, 0.0}, 0.05, 0.06);

  ASSERT_TRUE(escape_pose.has_value());
  EXPECT_NEAR(escape_pose->pose.position.x, 0.6498, 1e-9);
  EXPECT_NEAR(escape_pose->pose.position.y, -0.2243, 1e-9);
  EXPECT_NEAR(escape_pose->pose.position.z, 0.2647, 1e-9);
  EXPECT_EQ(current_pose.header, escape_pose->header);
  EXPECT_EQ(current_pose.pose.orientation, escape_pose->pose.orientation);
}

TEST(MakePlanningFramePostgraspEscapePose, NormalizesNonAxialDirection)
{
  const auto current_pose = make_grasp_pose();

  const auto escape_pose = make_planning_frame_postgrasp_escape_pose(
    current_pose, XYZ{3.0, 4.0, 0.0}, 0.10, 0.06);

  ASSERT_TRUE(escape_pose.has_value());
  EXPECT_NEAR(current_pose.pose.position.x - escape_pose->pose.position.x, 0.06, 1e-9);
  EXPECT_NEAR(current_pose.pose.position.y - escape_pose->pose.position.y, 0.08, 1e-9);
  EXPECT_NEAR(escape_pose->pose.position.z - current_pose.pose.position.z, 0.06, 1e-9);
}

TEST(MakePlanningFramePostgraspEscapePose, RejectsNonPositiveDistancesAndNonHorizontalApproach)
{
  const auto current_pose = make_grasp_pose();

  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{1.0, 0.0, 0.0}, -0.05, 0.06)
      .has_value());
  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{1.0, 0.0, 0.0}, 0.0, 0.06)
      .has_value());
  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{1.0, 0.0, 0.0}, 0.05, -0.06)
      .has_value());
  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{1.0, 0.0, 0.0}, 0.05, 0.0)
      .has_value());
  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{1.0, 0.0, 1e-5}, 0.05, 0.06)
      .has_value());
}

TEST(MakePlanningFramePostgraspEscapePose, RejectsInvalidInputs)
{
  const auto current_pose = make_grasp_pose();

  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{0.0, 0.0, 0.0}, 0.05, 0.06)
      .has_value());
  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{std::numeric_limits<double>::infinity(), 0.0, 0.0}, 0.05, 0.06)
      .has_value());
  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{1.0, 0.0, 0.0}, std::numeric_limits<double>::quiet_NaN(), 0.06)
      .has_value());
  EXPECT_FALSE(make_planning_frame_postgrasp_escape_pose(
      current_pose, XYZ{1.0, 0.0, 0.0}, 0.05, std::numeric_limits<double>::infinity())
      .has_value());
}

}  // namespace
}  // namespace piper_mtc_tasks
