#ifndef PLAN_PREVIEW_RVIZ_PLUGIN__PLAN_PREVIEW_TOOL_HPP_
#define PLAN_PREVIEW_RVIZ_PLUGIN__PLAN_PREVIEW_TOOL_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"

namespace rviz_common::properties
{
class StringProperty;
}  // namespace rviz_common::properties

namespace nav2_plan_preview
{

/**
 * @brief RViz tool: click and drag a goal like "2D Goal Pose"; instead of driving there, the planner
 * server is asked for a path from the robot to that pose (ComputePathToPose action) and the path is
 * published on its own topic, so it can be shown next to the plan being driven. Nothing is sent to
 * the controller.
 *
 * Properties: Action (the planner server's ComputePathToPose action), Planner ID (empty: the
 * server's default), Topic (where the path is published, latched).
 */
class PlanPreviewTool : public rviz_default_plugins::tools::PoseTool
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;

  PlanPreviewTool();
  ~PlanPreviewTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  /// (Re)creates the action client and the publisher if their names changed in the properties.
  void connect();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ComputePath>::SharedPtr client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  std::string action_name_;
  std::string topic_name_;

  rviz_common::properties::StringProperty * action_property_{nullptr};
  rviz_common::properties::StringProperty * planner_property_{nullptr};
  rviz_common::properties::StringProperty * topic_property_{nullptr};

  // Results arrive later (through RViz's node, on the GUI thread); they are dropped if the tool is
  // gone by then.
  std::shared_ptr<int> alive_{std::make_shared<int>(0)};
};

}  // namespace nav2_plan_preview

#endif  // PLAN_PREVIEW_RVIZ_PLUGIN__PLAN_PREVIEW_TOOL_HPP_
