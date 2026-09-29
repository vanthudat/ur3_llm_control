#include "ur3_llm_control/robot_skills.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace ur3_llm_control
{
namespace
{
template<typename T>
T getParameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

geometry_msgs::msg::Point pointFromVector(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::vector<double> & default_value)
{
  const auto values = getParameter(node, name, default_value);
  if (values.size() != 3U) {
    throw std::runtime_error("Parameter '" + name + "' must contain [x, y, z]");
  }
  geometry_msgs::msg::Point point;
  point.x = values[0];
  point.y = values[1];
  point.z = values[2];
  return point;
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string & id, const std::string & frame_id,
  const geometry_msgs::msg::Point & center, const std::vector<double> & dimensions)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id;
  object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.assign(dimensions.begin(), dimensions.end());
  geometry_msgs::msg::Pose pose;
  pose.position = center;
  pose.orientation.w = 1.0;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

bool normalizeRevoluteTargets(
  const moveit::core::JointModelGroup * group, const std::vector<double> & current_values,
  std::vector<double> & target_values)
{
  if (current_values.size() != target_values.size()) {
    return false;
  }

  std::size_t variable_index = 0;
  for (const auto * joint_model : group->getActiveJointModels()) {
    const auto variable_count = joint_model->getVariableCount();
    if (variable_index + variable_count > target_values.size()) {
      return false;
    }
    if (joint_model->getType() == moveit::core::JointModel::REVOLUTE &&
      variable_count == 1U)
    {
      const auto wrapped = current_values[variable_index] + std::remainder(
        target_values[variable_index] - current_values[variable_index], 2.0 * M_PI);
      const auto & bounds = joint_model->getVariableBounds().front();
      if (!bounds.position_bounded_) {
        target_values[variable_index] = wrapped;
      } else {
        double best_value = 0.0;
        double best_delta = std::numeric_limits<double>::infinity();
        for (int shift = -2; shift <= 2; ++shift) {
          const auto candidate = wrapped + static_cast<double>(shift) * 2.0 * M_PI;
          if (candidate < bounds.min_position_ || candidate > bounds.max_position_) {
            continue;
          }
          const auto delta = std::abs(candidate - current_values[variable_index]);
          if (delta < best_delta) {
            best_value = candidate;
            best_delta = delta;
          }
        }
        if (!std::isfinite(best_delta)) {
          return false;
        }
        target_values[variable_index] = best_value;
      }
    }
    variable_index += variable_count;
  }
  return variable_index == target_values.size();
}

moveit_msgs::msg::Constraints makeJointWindowConstraints(
  const moveit::core::JointModelGroup * group,
  const std::vector<double> & start_values, const std::vector<double> & target_values,
  const double margin)
{
  moveit_msgs::msg::Constraints constraints;
  const auto & variable_names = group->getVariableNames();
  if (start_values.size() != variable_names.size() ||
    target_values.size() != variable_names.size())
  {
    return constraints;
  }

  constraints.name = "local_joint_window";
  constraints.joint_constraints.reserve(variable_names.size());
  for (std::size_t index = 0; index < variable_names.size(); ++index) {
    moveit_msgs::msg::JointConstraint constraint;
    const auto half_distance = std::abs(target_values[index] - start_values[index]) * 0.5;
    constraint.joint_name = variable_names[index];
    constraint.position = (start_values[index] + target_values[index]) * 0.5;
    constraint.tolerance_above = half_distance + margin;
    constraint.tolerance_below = half_distance + margin;
    constraint.weight = 1.0;
    constraints.joint_constraints.push_back(constraint);
  }
  return constraints;
}
}  // namespace

const char * toString(const SkillStatus status)
{
  switch (status) {
    case SkillStatus::SUCCESS:
      return "SUCCESS";
    case SkillStatus::FAILED:
      return "FAILED";
    case SkillStatus::INVALID_OBJECT:
      return "INVALID_OBJECT";
    case SkillStatus::INVALID_ZONE:
      return "INVALID_ZONE";
    case SkillStatus::INVALID_STATE:
      return "INVALID_STATE";
    case SkillStatus::PLANNING_FAILED:
      return "PLANNING_FAILED";
  }
  return "FAILED";
}

RobotSkills::RobotSkills(const rclcpp::Node::SharedPtr & node)
: node_(node),
  move_group_(node, getParameter(node, "planning_group", std::string("ur_manipulator"))),
  frame_id_(getParameter(node, "frame_id", std::string("world"))),
  home_target_(getParameter(node, "home_target", std::string("home"))),
  cube_size_(getParameter(node, "cube_size", 0.04)),
  grasp_offset_(getParameter(node, "grasp_offset", 0.116)),
  ik_seed_target_(getParameter(node, "ik_seed_target", std::string("test_configuration"))),
  approach_height_(getParameter(node, "approach_height", 0.12)),
  gripper_open_position_(getParameter(node, "gripper_open_position", 0.0)),
  gripper_closed_position_(getParameter(node, "gripper_closed_position", 0.015)),
  gripper_timeout_(getParameter(node, "gripper_timeout", 5.0)),
  ik_timeout_(getParameter(node, "ik_timeout", 0.05)),
  max_joint_delta_(getParameter(node, "max_joint_delta", 2.2)),
  transit_joint_margin_(getParameter(node, "transit_joint_margin", 0.20)),
  cartesian_eef_step_(getParameter(node, "cartesian_eef_step", 0.005)),
  cartesian_jump_threshold_(getParameter(node, "cartesian_jump_threshold", 0.0)),
  min_cartesian_fraction_(getParameter(node, "min_cartesian_fraction", 0.98)),
  gazebo_sync_period_(getParameter(node, "gazebo_sync_period", 0.05)),
  velocity_scale_(getParameter(node, "velocity_scale", 0.10)),
  acceleration_scale_(getParameter(node, "acceleration_scale", 0.10)),
  cartesian_velocity_scale_(getParameter(node, "cartesian_velocity_scale", 0.10)),
  cartesian_acceleration_scale_(getParameter(node, "cartesian_acceleration_scale", 0.10)),
  execute_motion_(getParameter(node, "execute_motion", true)),
  sync_gazebo_(getParameter(node, "sync_gazebo", true))
{
  object_positions_["red_cube"] = pointFromVector(
    node, "objects.red_cube", {0.25, 0.14, 0.742});
  object_positions_["yellow_cube"] = pointFromVector(
    node, "objects.yellow_cube", {0.25, 0.0, 0.742});
  object_positions_["blue_cube"] = pointFromVector(
    node, "objects.blue_cube", {0.25, -0.14, 0.742});
  zone_positions_["zone_a"] = pointFromVector(node, "zones.zone_a", {0.40, 0.14, 0.742});
  zone_positions_["zone_b"] = pointFromVector(node, "zones.zone_b", {0.40, 0.0, 0.742});
  zone_positions_["zone_c"] = pointFromVector(node, "zones.zone_c", {0.40, -0.14, 0.742});

  const auto orientation = getParameter(
    node, "tool_orientation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
  if (orientation.size() != 4U) {
    throw std::runtime_error("Parameter 'tool_orientation' must contain [x, y, z, w]");
  }
  if (ik_timeout_ <= 0.0 || max_joint_delta_ <= 0.0) {
    throw std::runtime_error("IK timeout and max joint delta must be positive");
  }
  if (std::abs(gripper_open_position_ - gripper_closed_position_) < 1e-6 ||
    gripper_timeout_ <= 0.0)
  {
    throw std::runtime_error("Gripper parameters are invalid");
  }
  if (transit_joint_margin_ <= 0.0 || cartesian_eef_step_ <= 0.0 ||
    cartesian_jump_threshold_ < 0.0 || min_cartesian_fraction_ <= 0.0 ||
    min_cartesian_fraction_ > 1.0 || gazebo_sync_period_ <= 0.0 ||
    velocity_scale_ <= 0.0 || velocity_scale_ > 1.0 ||
    acceleration_scale_ <= 0.0 || acceleration_scale_ > 1.0 ||
    cartesian_velocity_scale_ <= 0.0 || cartesian_velocity_scale_ > 1.0 ||
    cartesian_acceleration_scale_ <= 0.0 || cartesian_acceleration_scale_ > 1.0)
  {
    throw std::runtime_error("Cartesian motion and synchronization parameters are invalid");
  }
  tool_orientation_.x = orientation[0];
  tool_orientation_.y = orientation[1];
  tool_orientation_.z = orientation[2];
  tool_orientation_.w = orientation[3];

  move_group_.setPoseReferenceFrame(frame_id_);
  move_group_.setPlannerId(
    getParameter(node, "planner_id", std::string("RRTConnectkConfigDefault")));
  move_group_.setPlanningTime(getParameter(node, "planning_time", 10.0));
  move_group_.setNumPlanningAttempts(
    static_cast<int>(getParameter(node, "planning_attempts", static_cast<int64_t>(1))));
  move_group_.setMaxVelocityScalingFactor(velocity_scale_);
  move_group_.setMaxAccelerationScalingFactor(acceleration_scale_);
  end_effector_link_ = move_group_.getEndEffectorLink();

  const auto gripper_action = getParameter(
    node, "gripper_action", std::string("/gripper_controller/follow_joint_trajectory"));
  joint_state_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  gripper_action_client_ = rclcpp_action::create_client<GripperTrajectory>(
    node_, gripper_action, joint_state_callback_group_);
  const auto status_topic = getParameter(node_, "status_topic", std::string("/task_status"));
  status_publisher_ = node_->create_publisher<std_msgs::msg::String>(status_topic, 10);
  const auto gazebo_service = getParameter(
    node, "gazebo_set_pose_service", std::string("/world/llm_robot/set_pose"));
  gazebo_pose_client_ = node->create_client<ros_gz_interfaces::srv::SetEntityPose>(gazebo_service);
  state_validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity", rmw_qos_profile_services_default, joint_state_callback_group_);
  rclcpp::SubscriptionOptions joint_state_options;
  joint_state_options.callback_group = joint_state_callback_group_;
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      if (message->name.size() != message->position.size()) {
        return;
      }
      std::string object_to_sync;
      geometry_msgs::msg::Pose object_pose;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_state_) {
          latest_state_ = std::make_shared<moveit::core::RobotState>(move_group_.getRobotModel());
          latest_state_->setToDefaultValues();
        }
        latest_state_->setVariablePositions(message->name, message->position);
        latest_state_->update();
        latest_state_time_ = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
        const auto elapsed = (latest_state_time_ - last_gazebo_sync_time_).seconds();
        if (!held_object_.empty() &&
          (last_gazebo_sync_time_.nanoseconds() == 0 || elapsed < 0.0 ||
          elapsed >= gazebo_sync_period_))
        {
          const auto object_transform = latest_state_->getGlobalLinkTransform(end_effector_link_) *
            Eigen::Translation3d(0.0, 0.0, grasp_offset_);
          const Eigen::Quaterniond orientation(object_transform.rotation());
          object_pose.position.x = object_transform.translation().x();
          object_pose.position.y = object_transform.translation().y();
          object_pose.position.z = object_transform.translation().z();
          object_pose.orientation.x = orientation.x();
          object_pose.orientation.y = orientation.y();
          object_pose.orientation.z = orientation.z();
          object_pose.orientation.w = orientation.w();
          object_to_sync = held_object_;
          last_gazebo_sync_time_ = latest_state_time_;
        }
      }
      if (!object_to_sync.empty()) {
        syncGazeboPose(object_to_sync, object_pose);
      }
    }, joint_state_options);

  if (!state_validity_client_->wait_for_service(std::chrono::seconds(10))) {
    throw std::runtime_error("MoveIt state validity service is unavailable");
  }
  if (!move_group_.getCurrentState(20.0)) {
    throw std::runtime_error("MoveIt current robot state is unavailable");
  }
  if (!commandGripper(gripper_open_position_, "initialize open gripper")) {
    throw std::runtime_error("Gripper action server is unavailable");
  }
  addPlanningScene();
  if (execute_motion_) {
    const auto home_status = home();
    if (home_status != SkillStatus::SUCCESS) {
      RCLCPP_ERROR(
        node_->get_logger(), "Startup home failed with %s; executor remains available.",
        toString(home_status));
    }
  }
  RCLCPP_INFO(
    node_->get_logger(), "Robot skills ready (group=%s, eef=%s, execute=%s).",
    move_group_.getName().c_str(), end_effector_link_.c_str(),
    execute_motion_ ? "true" : "false");
}

moveit::core::RobotStatePtr RobotSkills::getLatestState(const std::string & label)
{
  moveit::core::RobotStatePtr state;
  rclcpp::Time state_time(0, 0, RCL_ROS_TIME);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (latest_state_) {
      state = std::make_shared<moveit::core::RobotState>(*latest_state_);
      state_time = latest_state_time_;
    }
  }
  if (!state) {
    RCLCPP_ERROR(node_->get_logger(), "No joint state is available for %s.", label.c_str());
    return {};
  }
  if ((node_->now() - state_time).seconds() > 0.5) {
    RCLCPP_ERROR(node_->get_logger(), "Joint state is stale for %s.", label.c_str());
    return {};
  }
  return state;
}

bool RobotSkills::isStateValid(
  const moveit::core::RobotState & state, const std::string & group_name)
{
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  moveit::core::robotStateToRobotStateMsg(state, request->robot_state);
  request->group_name = group_name;
  auto future = state_validity_client_->async_send_request(request);
  if (future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
    RCLCPP_WARN(node_->get_logger(), "Timed out while checking IK state validity.");
    return false;
  }
  return future.get()->valid;
}

bool RobotSkills::normalizeTrajectory(
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit::core::RobotState & current_state, const std::string & label)
{
  auto & trajectory = plan.trajectory_.joint_trajectory;
  std::vector<double> previous_values;
  previous_values.reserve(trajectory.joint_names.size());
  for (const auto & variable_name : trajectory.joint_names) {
    previous_values.push_back(current_state.getVariablePosition(variable_name));
  }

  const auto robot_model = current_state.getRobotModel();
  for (auto & point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid trajectory dimensions for %s.", label.c_str());
      return false;
    }
    for (std::size_t index = 0; index < point.positions.size(); ++index) {
      const auto & variable_name = trajectory.joint_names[index];
      const auto * joint_model = robot_model->getJointOfVariable(variable_name);
      auto normalized = point.positions[index];
      if (joint_model->getType() == moveit::core::JointModel::REVOLUTE) {
        const auto wrapped = previous_values[index] + std::remainder(
          normalized - previous_values[index], 2.0 * M_PI);
        const auto & bounds = robot_model->getVariableBounds(variable_name);
        if (!bounds.position_bounded_) {
          normalized = wrapped;
        } else {
          double best_delta = std::numeric_limits<double>::infinity();
          for (int shift = -2; shift <= 2; ++shift) {
            const auto candidate = wrapped + static_cast<double>(shift) * 2.0 * M_PI;
            if (candidate < bounds.min_position_ || candidate > bounds.max_position_) {
              continue;
            }
            const auto delta = std::abs(candidate - previous_values[index]);
            if (delta < best_delta) {
              normalized = candidate;
              best_delta = delta;
            }
          }
          if (!std::isfinite(best_delta)) {
            RCLCPP_ERROR(
              node_->get_logger(), "No bounded trajectory angle for %s joint %s.",
              label.c_str(), variable_name.c_str());
            return false;
          }
        }
      }
      const auto delta = std::abs(normalized - previous_values[index]);
      if (delta > max_joint_delta_ + 1e-6) {
        RCLCPP_ERROR(
          node_->get_logger(), "Rejected %s trajectory: joint %s jumps %.3f rad.",
          label.c_str(), variable_name.c_str(), delta);
        return false;
      }
      point.positions[index] = normalized;
      previous_values[index] = normalized;
    }
  }
  return true;
}

geometry_msgs::msg::Pose RobotSkills::makeToolPose(
  const geometry_msgs::msg::Point & point) const
{
  geometry_msgs::msg::Pose pose;
  pose.position = point;
  pose.position.z += grasp_offset_;
  pose.orientation = tool_orientation_;
  return pose;
}

SkillStatus RobotSkills::moveToPose(
  const geometry_msgs::msg::Pose & pose, const std::string & label)
{
  const auto current_state = getLatestState(label);
  if (!current_state) {
    return SkillStatus::PLANNING_FAILED;
  }

  const auto * joint_model_group = current_state->getJointModelGroup(move_group_.getName());
  if (!joint_model_group) {
    RCLCPP_ERROR(node_->get_logger(), "Planning group '%s' does not exist.", move_group_.getName().c_str());
    return SkillStatus::PLANNING_FAILED;
  }

  std::vector<double> current_values;
  current_state->copyJointGroupPositions(joint_model_group, current_values);

  const auto valid_ik =
    [this, &current_values](
    moveit::core::RobotState * candidate_state,
    const moveit::core::JointModelGroup * group, const double * values)
    {
      std::vector<double> normalized_values(values, values + group->getVariableCount());
      if (!normalizeRevoluteTargets(group, current_values, normalized_values)) {
        return false;
      }
      candidate_state->setJointGroupPositions(group, normalized_values);
      candidate_state->update();
      if (!candidate_state->satisfiesBounds(group) ||
        !isStateValid(*candidate_state, group->getName()))
      {
        return false;
      }
      for (std::size_t index = 0; index < normalized_values.size(); ++index) {
        if (std::abs(normalized_values[index] - current_values[index]) > max_joint_delta_) {
          return false;
        }
      }
      return true;
    };

  moveit::core::RobotState target_state(*current_state);
  if (!target_state.setToDefaultValues(joint_model_group, ik_seed_target_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "IK seed target '%s' does not exist.", ik_seed_target_.c_str());
    return SkillStatus::PLANNING_FAILED;
  }
  target_state.update();
  if (!target_state.setFromIK(
      joint_model_group, pose, end_effector_link_, ik_timeout_, valid_ik))
  {
    RCLCPP_ERROR(node_->get_logger(), "IK failed for %s.", label.c_str());
    return SkillStatus::PLANNING_FAILED;
  }

  std::vector<double> target_values;
  target_state.copyJointGroupPositions(joint_model_group, target_values);
  if (!normalizeRevoluteTargets(joint_model_group, current_values, target_values)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not normalize IK target for %s.", label.c_str());
    return SkillStatus::PLANNING_FAILED;
  }
  target_state.setJointGroupPositions(joint_model_group, target_values);
  target_state.update();

  if (!target_state.satisfiesBounds(joint_model_group)) {
    RCLCPP_ERROR(node_->get_logger(), "IK target violates joint limits for %s.", label.c_str());
    return SkillStatus::PLANNING_FAILED;
  }
  const auto & variable_names = joint_model_group->getVariableNames();
  for (std::size_t index = 0; index < target_values.size(); ++index) {
    const auto delta = std::abs(target_values[index] - current_values[index]);
    if (delta > max_joint_delta_) {
      RCLCPP_ERROR(
        node_->get_logger(), "Rejected %s: joint %s would rotate %.3f rad.",
        label.c_str(), variable_names[index].c_str(), delta);
      return SkillStatus::PLANNING_FAILED;
    }
  }

  move_group_.setStartState(*current_state);
  if (!move_group_.setJointValueTarget(target_state)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not set joint target for %s.", label.c_str());
    return SkillStatus::PLANNING_FAILED;
  }

  const auto joint_window = makeJointWindowConstraints(
    joint_model_group, current_values, target_values, transit_joint_margin_);
  move_group_.setPathConstraints(joint_window);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planning_result = move_group_.plan(plan);
  move_group_.clearPathConstraints();
  if (!static_cast<bool>(planning_result)) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed for %s.", label.c_str());
    return SkillStatus::PLANNING_FAILED;
  }
  if (!normalizeTrajectory(plan, *current_state, label)) {
    return SkillStatus::PLANNING_FAILED;
  }
  if (execute_motion_ && !static_cast<bool>(move_group_.execute(plan))) {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed for %s.", label.c_str());
    return SkillStatus::FAILED;
  }
  return SkillStatus::SUCCESS;
}

SkillStatus RobotSkills::moveCartesianToPose(
  const geometry_msgs::msg::Pose & pose, const std::string & label)
{
  const auto current_state = getLatestState(label);
  if (!current_state) {
    return SkillStatus::PLANNING_FAILED;
  }

  move_group_.setStartState(*current_state);
  moveit_msgs::msg::RobotTrajectory trajectory;
  const auto fraction = move_group_.computeCartesianPath(
    {pose}, cartesian_eef_step_, cartesian_jump_threshold_, trajectory, true);
  if (fraction < min_cartesian_fraction_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cartesian path for %s reached only %.1f%%; refusing a non-linear fallback.",
      label.c_str(), fraction * 100.0);
    return SkillStatus::PLANNING_FAILED;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::robotStateToRobotStateMsg(*current_state, plan.start_state_);
  plan.trajectory_ = std::move(trajectory);
  if (!normalizeTrajectory(plan, *current_state, label)) {
    return SkillStatus::PLANNING_FAILED;
  }

  robot_trajectory::RobotTrajectory timed_trajectory(
    move_group_.getRobotModel(), move_group_.getName());
  timed_trajectory.setRobotTrajectoryMsg(*current_state, plan.trajectory_);
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  if (!time_parameterization.computeTimeStamps(
      timed_trajectory, cartesian_velocity_scale_, cartesian_acceleration_scale_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed for %s.", label.c_str());
    return SkillStatus::PLANNING_FAILED;
  }
  timed_trajectory.getRobotTrajectoryMsg(plan.trajectory_);

  if (execute_motion_ && !static_cast<bool>(move_group_.execute(plan))) {
    RCLCPP_ERROR(node_->get_logger(), "Cartesian execution failed for %s.", label.c_str());
    return SkillStatus::FAILED;
  }
  return SkillStatus::SUCCESS;
}

bool RobotSkills::commandGripper(
  const double position, const std::string & label, const bool accept_contact)
{
  if (!execute_motion_) {
    return true;
  }

  const auto timeout = std::chrono::milliseconds(
    static_cast<int64_t>(gripper_timeout_ * 1000.0));
  if (!gripper_action_client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server is unavailable for %s.", label.c_str());
    return false;
  }

  GripperTrajectory::Goal goal;
  goal.trajectory.joint_names = {"gripper_finger_joint", "gripper_fixed_finger_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {position, position};
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 800000000;
  goal.trajectory.points = {point};

  const auto goal_future = gripper_action_client_->async_send_goal(goal);
  if (goal_future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out sending gripper command for %s.", label.c_str());
    return false;
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper rejected command for %s.", label.c_str());
    return false;
  }

  const auto result_future = gripper_action_client_->async_get_result(goal_handle);
  if (result_future.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out moving gripper for %s.", label.c_str());
    return false;
  }
  const auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return true;
  }
  if (accept_contact && result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_INFO(node_->get_logger(), "Gripper contact accepted for %s.", label.c_str());
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "Gripper command failed for %s.", label.c_str());
  return false;
}

void RobotSkills::addPlanningScene()
{
  const auto table_center = pointFromVector(node_, "table.position", {0.35, 0.0, 0.7075});
  const auto table_size = getParameter(
    node_, "table.size", std::vector<double>{0.55, 0.50, 0.025});
  if (table_size.size() != 3U) {
    throw std::runtime_error("Parameter 'table.size' must contain [x, y, z]");
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(makeBox("work_table", frame_id_, table_center, table_size));
  for (const auto & item : object_positions_) {
    objects.push_back(makeBox(
      item.first, frame_id_, item.second, {cube_size_, cube_size_, cube_size_}));
  }
  if (!planning_scene_.applyCollisionObjects(objects)) {
    throw std::runtime_error("Failed to apply the MoveIt planning scene");
  }
}

void RobotSkills::addCube(
  const std::string & name, const geometry_msgs::msg::Point & point)
{
  if (!planning_scene_.applyCollisionObject(
      makeBox(name, frame_id_, point, {cube_size_, cube_size_, cube_size_})))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to restore collision object '%s'.", name.c_str());
  }
}

bool RobotSkills::removeCube(const std::string & name)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = frame_id_;
  object.id = name;
  object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!planning_scene_.applyCollisionObject(object)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to remove collision object '%s'.", name.c_str());
    return false;
  }
  return true;
}

bool RobotSkills::attachCube(const std::string & name)
{
  geometry_msgs::msg::Point center;
  center.z = grasp_offset_;
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = end_effector_link_;
  attached.object = makeBox(
    name, end_effector_link_, center, {cube_size_, cube_size_, cube_size_});
  attached.touch_links = {
    end_effector_link_,
    "simple_gripper_base_link",
    "gripper_fixed_finger_link",
    "gripper_moving_finger_link"
  };
  if (!planning_scene_.applyAttachedCollisionObject(attached)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to attach collision object '%s'.", name.c_str());
    return false;
  }
  return true;
}

bool RobotSkills::detachCube(const std::string & name)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = end_effector_link_;
  attached.object.id = name;
  attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!planning_scene_.applyAttachedCollisionObject(attached)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to detach collision object '%s'.", name.c_str());
    return false;
  }
  return true;
}

SkillStatus RobotSkills::home()
{
  const auto current_state = getLatestState("home");
  if (!current_state) {
    return SkillStatus::PLANNING_FAILED;
  }
  move_group_.setStartState(*current_state);
  const auto home_positions = getParameter(
    node_, "home_joint_positions", std::vector<double>{});
  if (!home_positions.empty()) {
    const auto * group = current_state->getJointModelGroup(move_group_.getName());
    if (!group || home_positions.size() != group->getVariableCount() ||
      !move_group_.setJointValueTarget(home_positions))
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid startup home joint target.");
      return SkillStatus::PLANNING_FAILED;
    }
  } else if (!move_group_.setNamedTarget(home_target_)) {
    RCLCPP_ERROR(node_->get_logger(), "Named target '%s' does not exist.", home_target_.c_str());
    return SkillStatus::PLANNING_FAILED;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(move_group_.plan(plan))) {
    return SkillStatus::PLANNING_FAILED;
  }
  if (!normalizeTrajectory(plan, *current_state, "home")) {
    return SkillStatus::PLANNING_FAILED;
  }
  if (execute_motion_ && !static_cast<bool>(move_group_.execute(plan))) {
    return SkillStatus::FAILED;
  }
  return SkillStatus::SUCCESS;
}

SkillStatus RobotSkills::moveAbove(const std::string & object_name)
{
  const auto object = object_positions_.find(object_name);
  if (object == object_positions_.end()) {
    return SkillStatus::INVALID_OBJECT;
  }
  auto pose = makeToolPose(object->second);
  pose.position.z += approach_height_;
  return moveToPose(pose, "above " + object_name);
}

SkillStatus RobotSkills::openGripper()
{
  return commandGripper(gripper_open_position_, "open gripper") ?
         SkillStatus::SUCCESS : SkillStatus::FAILED;
}

SkillStatus RobotSkills::closeGripper()
{
  return commandGripper(gripper_closed_position_, "close gripper", true) ?
         SkillStatus::SUCCESS : SkillStatus::FAILED;
}

void RobotSkills::publishSubskillStatus(const std::string & name, const SkillStatus status)
{
  RCLCPP_INFO(node_->get_logger(), "SUBSKILL: %s -> %s", name.c_str(), toString(status));
  std_msgs::msg::String message;
  message.data = "SUBSKILL: " + name + " " + toString(status);
  status_publisher_->publish(message);
}

SkillStatus RobotSkills::moveToZone(const std::string & zone_name)
{
  const auto zone = zone_positions_.find(zone_name);
  if (zone == zone_positions_.end()) {
    return SkillStatus::INVALID_ZONE;
  }
  auto pose = makeToolPose(zone->second);
  pose.position.z += approach_height_;
  return moveToPose(pose, "above " + zone_name);
}

SkillStatus RobotSkills::pick(const std::string & object_name)
{
  const auto object = object_positions_.find(object_name);
  if (object == object_positions_.end()) {
    return SkillStatus::INVALID_OBJECT;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!held_object_.empty()) {
      return SkillStatus::INVALID_STATE;
    }
  }

  auto grasp_pose = makeToolPose(object->second);
  auto above_pose = grasp_pose;
  above_pose.position.z += approach_height_;
  const auto gripper_status = openGripper();
  publishSubskillStatus("open_gripper", gripper_status);
  if (gripper_status != SkillStatus::SUCCESS) {
    return SkillStatus::FAILED;
  }
  auto status = moveToPose(above_pose, "above " + object_name);
  publishSubskillStatus("move_above(" + object_name + ")", status);
  if (status != SkillStatus::SUCCESS) {
    return status;
  }
  if (!removeCube(object_name)) {
    return SkillStatus::FAILED;
  }
  status = moveCartesianToPose(grasp_pose, "descend to " + object_name);
  if (status != SkillStatus::SUCCESS) {
    addCube(object_name, object->second);
    return status;
  }
  const auto close_status = closeGripper();
  publishSubskillStatus("close_gripper", close_status);
  if (close_status != SkillStatus::SUCCESS) {
    addCube(object_name, object->second);
    return SkillStatus::FAILED;
  }
  if (!attachCube(object_name)) {
    addCube(object_name, object->second);
    return SkillStatus::FAILED;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_ = object_name;
    last_gazebo_sync_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  status = moveCartesianToPose(above_pose, "lift " + object_name);
  if (status != SkillStatus::SUCCESS) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      held_object_.clear();
    }
    detachCube(object_name);
    addCube(object_name, object->second);
    geometry_msgs::msg::Pose original_pose;
    original_pose.position = object->second;
    original_pose.orientation.w = 1.0;
    syncGazeboPose(object_name, original_pose);
  }
  return status;
}

SkillStatus RobotSkills::place(
  const std::string & object_name, const std::string & zone_name)
{
  if (object_positions_.count(object_name) == 0U) {
    return SkillStatus::INVALID_OBJECT;
  }
  const auto zone = zone_positions_.find(zone_name);
  if (zone == zone_positions_.end()) {
    return SkillStatus::INVALID_ZONE;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (held_object_ != object_name) {
      return SkillStatus::INVALID_STATE;
    }
  }

  auto place_pose = makeToolPose(zone->second);
  auto above_pose = place_pose;
  above_pose.position.z += approach_height_;
  auto status = moveToPose(above_pose, "above " + zone_name);
  publishSubskillStatus("move_to_zone(" + zone_name + ")", status);
  if (status != SkillStatus::SUCCESS) {
    return status;
  }
  status = moveCartesianToPose(place_pose, "lower " + object_name);
  if (status != SkillStatus::SUCCESS) {
    return status;
  }
  const auto release_status = openGripper();
  publishSubskillStatus("open_gripper", release_status);
  if (release_status != SkillStatus::SUCCESS) {
    return SkillStatus::FAILED;
  }
  if (!detachCube(object_name)) {
    return SkillStatus::FAILED;
  }

  object_positions_[object_name] = zone->second;
  addCube(object_name, zone->second);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_.clear();
  }
  geometry_msgs::msg::Pose gazebo_pose;
  gazebo_pose.position = zone->second;
  gazebo_pose.orientation.w = 1.0;
  syncGazeboPose(object_name, gazebo_pose);

  return moveCartesianToPose(above_pose, "retreat from " + zone_name);
}

void RobotSkills::syncGazeboPose(
  const std::string & object_name, const geometry_msgs::msg::Pose & pose)
{
  if (!sync_gazebo_ || !execute_motion_) {
    return;
  }
  if (!gazebo_pose_client_->service_is_ready()) {
    RCLCPP_WARN(
      node_->get_logger(), "Gazebo set_pose service is unavailable; MoveIt scene remains correct.");
    return;
  }
  auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
  request->entity.name = object_name;
  request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
  request->pose = pose;
  gazebo_pose_client_->async_send_request(request);
}
}  // namespace ur3_llm_control
