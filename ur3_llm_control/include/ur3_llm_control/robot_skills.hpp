#pragma once

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
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

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
  SkillStatus moveToPose(const geometry_msgs::msg::Pose & pose, const std::string & label);
  SkillStatus moveCartesianToPose(
    const geometry_msgs::msg::Pose & pose, const std::string & label);
  bool commandGripper(double position, const std::string & label, bool accept_contact = false);
  void publishSubskillStatus(const std::string & name, SkillStatus status);
  void addPlanningScene();
  void addCube(const std::string & name, const geometry_msgs::msg::Point & point);
  bool removeCube(const std::string & name);
  bool attachCube(const std::string & name);
  bool detachCube(const std::string & name);
  void syncGazeboPose(const std::string & object_name, const geometry_msgs::msg::Pose & pose);

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp_action::Client<GripperTrajectory>::SharedPtr gripper_action_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr gazebo_pose_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  std::mutex state_mutex_;
  moveit::core::RobotStatePtr latest_state_;
  rclcpp::Time latest_state_time_{0, 0, RCL_ROS_TIME};

  std::map<std::string, geometry_msgs::msg::Point> object_positions_;
  std::map<std::string, geometry_msgs::msg::Point> zone_positions_;
  std::string held_object_;
  std::string frame_id_;
  std::string home_target_;
  std::string ik_seed_target_;
  std::string end_effector_link_;
  double cube_size_;
  double grasp_offset_;
  double approach_height_;
  double gripper_open_position_;
  double gripper_closed_position_;
  double gripper_timeout_;
  double ik_timeout_;
  double max_joint_delta_;
  double transit_joint_margin_;
  double cartesian_eef_step_;
  double cartesian_jump_threshold_;
  double min_cartesian_fraction_;
  double gazebo_sync_period_;
  double velocity_scale_;
  double acceleration_scale_;
  double cartesian_velocity_scale_;
  double cartesian_acceleration_scale_;
  bool execute_motion_;
  bool sync_gazebo_;
  rclcpp::Time last_gazebo_sync_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Quaternion tool_orientation_;
};
}  // namespace ur3_llm_control
