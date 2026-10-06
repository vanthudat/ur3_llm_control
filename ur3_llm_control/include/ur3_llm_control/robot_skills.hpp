#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

namespace ur3_llm_control
{
enum class SkillStatus
{
  SUCCESS,
  FAILED,
  INVALID_OBJECT,
  INVALID_ZONE,
  INVALID_STATE,
  PLANNING_FAILED
};

const char * toString(SkillStatus status);

class RobotSkills
{
public:
  explicit RobotSkills(const rclcpp::Node::SharedPtr & node);

  bool validateEnvironmentPlan(nlohmann::json & plan);
  bool checkTransfer(const std::string & object, std::string & destination);
  bool verifyFinalPlan(const nlohmann::json & plan);
  SkillStatus home();
  SkillStatus pick(const std::string & object_name);
  SkillStatus place(const std::string & object_name, const std::string & zone_name);
  SkillStatus moveAbove(const std::string & object_name);
  SkillStatus openGripper();
  SkillStatus closeGripper();
  SkillStatus moveToZone(const std::string & zone_name);

private:
  using GripperTrajectory = control_msgs::action::FollowJointTrajectory;

  moveit::core::RobotStatePtr getLatestState(const std::string & label);
  bool isStateValid(const moveit::core::RobotState & state, const std::string & group_name);
  bool normalizeTrajectory(
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const moveit::core::RobotState & current_state, const std::string & label);
  geometry_msgs::msg::Pose makeToolPose(const geometry_msgs::msg::Point & point) const;
  SkillStatus planToPose(
    const geometry_msgs::msg::Pose & pose, const std::string & label,
    const moveit::core::RobotStatePtr & current_state,
    moveit::planning_interface::MoveGroupInterface::Plan & plan);
  SkillStatus planCartesianToPose(
    const geometry_msgs::msg::Pose & pose, const std::string & label,
    const moveit::core::RobotStatePtr & current_state,
    moveit::planning_interface::MoveGroupInterface::Plan & plan);
  SkillStatus moveToPose(const geometry_msgs::msg::Pose & pose, const std::string & label);
  SkillStatus moveCartesianToPose(
    const geometry_msgs::msg::Pose & pose, const std::string & label);
  bool executeWithGraspMonitor(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan, const std::string & label);
  bool commandGripper(double position, const std::string & label);
  void publishSubskillStatus(const std::string & name, SkillStatus status);
  void addPlanningScene();
  void addCube(const std::string & name, const geometry_msgs::msg::Point & point);
  bool removeCube(const std::string & name);
  bool attachCube(const std::string & name);
  bool detachCube(const std::string & name);
  bool refreshCameraScene(bool require_all = true);
  bool acquireFullCameraScene();
  bool refreshPlacementScene(const geometry_msgs::msg::Point & target,
    const std::string & object, double baseline_observed_at);
  bool verifyObserved(const std::string & object, const geometry_msgs::msg::Point & expected,
                      bool lifted = false);
  bool verifyHeldObject(const std::string & object);
  bool canReachDestination(const geometry_msgs::msg::Point & target);
  bool destinationFree(const geometry_msgs::msg::Point & target, const std::string & exclude,
                       double clearance = 0.095) const;

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp_action::Client<GripperTrajectory>::SharedPtr gripper_action_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wrist_camera_subscription_;
  std::mutex camera_mutex_;
  nlohmann::json camera_scene_;
  std::chrono::steady_clock::time_point camera_received_{};
  std::mutex wrist_camera_mutex_;
  nlohmann::json wrist_camera_observation_;
  std::chrono::steady_clock::time_point wrist_camera_received_{};
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  std::mutex state_mutex_;
  moveit::core::RobotStatePtr latest_state_;
  rclcpp::Time latest_state_time_{0, 0, RCL_ROS_TIME};

  std::map<std::string, geometry_msgs::msg::Point> object_positions_;
  std::map<std::string, geometry_msgs::msg::Point> zone_positions_;
  std::string held_object_;
  std::string pending_destination_;
  std::string frame_id_;
  std::string home_target_;
  std::string end_effector_link_;
  double cube_size_;
  double grasp_offset_;
  std::string ik_seed_target_;
  double approach_height_;
  double placement_clearance_;
  double gripper_open_position_;
  double gripper_closed_position_;
  double gripper_timeout_;
  double ik_timeout_;
  double max_joint_delta_;
  double transit_joint_margin_;
  double cartesian_eef_step_;
  double cartesian_jump_threshold_;
  double cartesian_max_joint_step_;
  double trajectory_detour_allowance_;
  double min_cartesian_fraction_;
  double velocity_scale_;
  double acceleration_scale_;
  double cartesian_velocity_scale_;
  double cartesian_acceleration_scale_;
  bool execute_motion_;
  geometry_msgs::msg::Quaternion tool_orientation_;
  geometry_msgs::msg::Quaternion default_tool_orientation_;
};
}  // namespace ur3_llm_control
