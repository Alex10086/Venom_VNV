#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "piper_mtc_tasks/task_factory.hpp"
#include "piper_mtc_tasks/task_parameters.hpp"

namespace piper_mtc_tasks
{
namespace
{

class TaskParametersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TaskParametersTest, LoadsClassificationObserveJointPositionsInOrder)
{
  auto node = std::make_shared<rclcpp::Node>("task_parameters_observe_joint_positions");
  declare_task_parameters(*node);

  const std::vector<double> expected{
    0.002599156, 1.108618532, -0.067578056,
    -0.046627812, -0.957291832, 0.094616256};
  ASSERT_TRUE(node->set_parameter(
      rclcpp::Parameter("classification_place.observe_joint_positions", expected)).successful);

  const auto parameters = load_task_parameters(*node);

  EXPECT_EQ(parameters.classification_place.observe_joint_positions, expected);
}

TEST_F(TaskParametersTest, DefaultsClassificationObserveJointPositionsToEmpty)
{
  auto node = std::make_shared<rclcpp::Node>("task_parameters_default_observe_joint_positions");
  declare_task_parameters(*node);

  const auto parameters = load_task_parameters(*node);

  EXPECT_TRUE(parameters.classification_place.observe_joint_positions.empty());
}

TEST_F(TaskParametersTest, ValidatesClassificationObserveJointPositions)
{
  std::string error_message;
  EXPECT_TRUE(validate_classification_observe_joint_positions({}, error_message));
  EXPECT_TRUE(error_message.empty());

  EXPECT_TRUE(validate_classification_observe_joint_positions(
      {0.002599156, 1.108618532, -0.067578056,
       -0.046627812, -0.957291832, 0.094616256},
      error_message));
  EXPECT_TRUE(error_message.empty());
}

TEST_F(TaskParametersTest, RejectsInvalidClassificationObserveJointPositions)
{
  std::string error_message;
  EXPECT_FALSE(validate_classification_observe_joint_positions({0.0, 1.0}, error_message));
  EXPECT_FALSE(error_message.empty());

  EXPECT_FALSE(validate_classification_observe_joint_positions(
      {0.0, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}, error_message));
  EXPECT_FALSE(error_message.empty());

  EXPECT_FALSE(validate_classification_observe_joint_positions(
      {0.0, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::infinity()}, error_message));
  EXPECT_FALSE(error_message.empty());
}

TEST_F(TaskParametersTest, LoadsClassificationReleaseCorrectionsInIndexOrder)
{
  auto node = std::make_shared<rclcpp::Node>("task_parameters_release_corrections");
  declare_task_parameters(*node);

  const std::vector<XYZ> expected{
    {0.001, 0.002, 0.003},
    {0.004, 0.005, 0.006}};
  ASSERT_TRUE(node->set_parameter(
      rclcpp::Parameter(
          "classification_place.release_corrections_xyz",
          std::vector<double>{0.001, 0.002, 0.003, 0.004, 0.005, 0.006}))
      .successful);

  const auto parameters = load_task_parameters(*node);

  ASSERT_EQ(parameters.classification_place.release_corrections.size(), expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    const auto& actual = parameters.classification_place.release_corrections[index];
    EXPECT_DOUBLE_EQ(actual.x, expected[index].x);
    EXPECT_DOUBLE_EQ(actual.y, expected[index].y);
    EXPECT_DOUBLE_EQ(actual.z, expected[index].z);
  }
}

TEST_F(TaskParametersTest, DefaultsClassificationReleaseCorrectionsToEmpty)
{
  auto node = std::make_shared<rclcpp::Node>("task_parameters_default_release_corrections");
  declare_task_parameters(*node);

  const auto parameters = load_task_parameters(*node);

  EXPECT_TRUE(parameters.classification_place.release_corrections.empty());
}

TEST_F(TaskParametersTest, ValidatesClassificationReleaseCorrections)
{
  std::string error_message;
  EXPECT_TRUE(validate_classification_release_corrections({}, 2, error_message));
  EXPECT_TRUE(error_message.empty());

  EXPECT_TRUE(validate_classification_release_corrections(
       {{0.001, 0.002, 0.003}, {0.004, 0.005, 0.006}}, 2, error_message));
  EXPECT_TRUE(error_message.empty());
}

TEST_F(TaskParametersTest, RejectsInvalidClassificationReleaseCorrections)
{
  std::string error_message;
  EXPECT_FALSE(validate_classification_release_corrections(
      {{0.0, 0.0, 0.0}}, 2, error_message));
  EXPECT_FALSE(error_message.empty());

  EXPECT_FALSE(validate_classification_release_corrections(
      {{0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}, {0.0, 0.0, 0.0}},
      2,
      error_message));
  EXPECT_FALSE(error_message.empty());

  EXPECT_FALSE(validate_classification_release_corrections(
      {{0.0, 0.0, 0.0}, {0.0, 0.0, std::numeric_limits<double>::infinity()}},
      2,
      error_message));
  EXPECT_FALSE(error_message.empty());
}

}  // namespace
}  // namespace piper_mtc_tasks
