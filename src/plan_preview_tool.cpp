#include "nav2_plan_preview/plan_preview_tool.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_rendering/objects/arrow.hpp"

namespace nav2_plan_preview
{

PlanPreviewTool::PlanPreviewTool()
{
  shortcut_key_ = 'v';
}

PlanPreviewTool::~PlanPreviewTool() = default;

void PlanPreviewTool::onInitialize()
{
  PoseTool::onInitialize();
  arrow_->setColor(0.2f, 0.5f, 1.0f, 1.0f);
  setName("Plan Preview");

  action_property_ = new rviz_common::properties::StringProperty(
    "Action", "/compute_path_to_pose",
    "ComputePathToPose action of the Nav2 planner server.", getPropertyContainer());
  planner_property_ = new rviz_common::properties::StringProperty(
    "Planner ID", "",
    "planner_id sent with the request; empty: the planner server's default planner.",
    getPropertyContainer());
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/plan_preview",
    "The planned path is published here (nav_msgs/Path, latched). Show it with a Path display.",
    getPropertyContainer());

  // RViz's own node: RViz spins it in its update loop, so the results arrive on the GUI thread.
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
}

void PlanPreviewTool::connect()
{
  const std::string action = action_property_->getStdString();
  const std::string topic = topic_property_->getStdString();
  if (!client_ || action != action_name_) {
    client_ = rclcpp_action::create_client<ComputePath>(node_, action);
    action_name_ = action;
  }
  if (!publisher_ || topic != topic_name_) {
    publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
      topic, rclcpp::QoS(1).transient_local().reliable());
    topic_name_ = topic;
  }
}

void PlanPreviewTool::onPoseSet(double x, double y, double theta)
{
  connect();
  if (!client_->action_server_is_ready()) {
    setStatus(QString::fromStdString("Plan Preview: no planner server at " + action_name_));
    return;
  }

  ComputePath::Goal goal;
  goal.goal.header.frame_id = context_->getFixedFrame().toStdString();  // stamp 0: latest transform
  goal.goal.pose.position.x = x;
  goal.goal.pose.position.y = y;
  goal.goal.pose.orientation = orientationAroundZAxis(theta);
  goal.planner_id = planner_property_->getStdString();
  goal.use_start = false;  // from where the robot is now
  logPose(
    "plan preview goal", goal.goal.pose.position, goal.goal.pose.orientation, theta,
    goal.goal.header.frame_id);

  rclcpp_action::Client<ComputePath>::SendGoalOptions options;
  std::weak_ptr<int> alive = alive_;
  auto publisher = publisher_;
  options.goal_response_callback =
    [this, alive](const rclcpp_action::ClientGoalHandle<ComputePath>::SharedPtr & handle) {
      if (!alive.lock()) {
        return;
      }
      if (!handle) {
        setStatus("Plan Preview: the planner server rejected the request");
      }
    };
  options.result_callback =
    [this, alive, publisher](
    const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & result) {
      if (!alive.lock()) {
        return;
      }
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
        setStatus("Plan Preview: no path (the planner failed or was cancelled)");
        return;
      }
      const auto & path = result.result->path;
      publisher->publish(path);
      double length = 0.0;
      for (size_t i = 1; i < path.poses.size(); ++i) {
        length += std::hypot(
          path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
          path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
      }
      const double ms = result.result->planning_time.sec * 1e3 +
        result.result->planning_time.nanosec * 1e-6;
      char text[160];
      std::snprintf(
        text, sizeof(text), "Plan Preview: %zu poses, %.2f m, planned in %.0f ms (on %s)",
        path.poses.size(), length, ms, topic_name_.c_str());
      setStatus(text);
    };
  client_->async_send_goal(goal, options);
  setStatus("Plan Preview: planning ...");
}

}  // namespace nav2_plan_preview

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_plan_preview::PlanPreviewTool, rviz_common::Tool)
