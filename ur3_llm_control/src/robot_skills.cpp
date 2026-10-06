#include "ur3_llm_control/robot_skills.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <set>
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
constexpr double kHeldObjectCameraToleranceM = 0.06;
constexpr unsigned int kHeldObjectCameraConfirmations = 2U;

double wallTimeSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

bool cameraObservationFresh(
  const nlohmann::json & scene, const std::chrono::steady_clock::time_point received)
{
  if (scene.at("source") != "rgbd_camera" || !scene.at("observed_at").is_number() ||
    !scene.at("stamp").is_number() || !scene.at("stream_id").is_string() ||
    !scene.at("frame_id").is_number_integer())
  {return false;}
  const double stamp = scene.at("stamp").get<double>();
  const double age = wallTimeSeconds() - scene.at("observed_at").get<double>();
  const double silence = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - received).count();
  return std::isfinite(stamp) && stamp >= 0.0 && std::isfinite(age) &&
         age >= -0.25 && age <= 2.0 && silence >= 0.0 && silence <= 2.0;
}

bool wristCameraObservationFresh(
  const nlohmann::json & observation,
  const std::chrono::steady_clock::time_point received)
{
  if (observation.at("source") != "wrist_camera_rgb" ||
    !observation.at("observed_at").is_number() ||
    !observation.at("stamp").is_number() ||
    !observation.at("stream_id").is_string() ||
    !observation.at("frame_id").is_number_integer() ||
    !observation.at("detections").is_object())
  {return false;}
  const double age = wallTimeSeconds() - observation.at("observed_at").get<double>();
  const double silence = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - received).count();
  return std::isfinite(age) && age >= -0.25 && age <= 1.0 &&
         silence >= 0.0 && silence <= 1.0;
}

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
  placement_clearance_(getParameter(node, "placement_clearance", 0.002)),
  gripper_open_position_(getParameter(node, "gripper_open_position", 0.0)),
  gripper_closed_position_(getParameter(node, "gripper_closed_position", 0.018)),
  gripper_timeout_(getParameter(node, "gripper_timeout", 5.0)),
  ik_timeout_(getParameter(node, "ik_timeout", 0.05)),
  max_joint_delta_(getParameter(node, "max_joint_delta", 2.2)),
  transit_joint_margin_(getParameter(node, "transit_joint_margin", 0.20)),
  cartesian_eef_step_(getParameter(node, "cartesian_eef_step", 0.005)),
  cartesian_jump_threshold_(getParameter(node, "cartesian_jump_threshold", 0.0)),
  cartesian_max_joint_step_(getParameter(node, "cartesian_max_joint_step", 0.10)),
  trajectory_detour_allowance_(getParameter(node, "trajectory_detour_allowance", 0.75)),
  min_cartesian_fraction_(getParameter(node, "min_cartesian_fraction", 1.0)),
  velocity_scale_(getParameter(node, "velocity_scale", 0.10)),
  acceleration_scale_(getParameter(node, "acceleration_scale", 0.10)),
  cartesian_velocity_scale_(getParameter(node, "cartesian_velocity_scale", 0.05)),
  cartesian_acceleration_scale_(getParameter(node, "cartesian_acceleration_scale", 0.03)),
  execute_motion_(getParameter(node, "execute_motion", true))
{
  zone_positions_["zone_a"] = pointFromVector(node, "zones.zone_a", {0.40, 0.14, 0.740});
  zone_positions_["zone_b"] = pointFromVector(node, "zones.zone_b", {0.40, 0.0, 0.740});
  zone_positions_["zone_c"] = pointFromVector(node, "zones.zone_c", {0.40, -0.14, 0.740});
  std::vector<std::vector<double>> temporary;
  for (const double x : {0.22, 0.26, 0.30, 0.34, 0.38}) {
    for (const double y : {-0.27, -0.24, 0.24, 0.27}) {
      temporary.push_back({x, y, 0.740});
    }
  }
  for (std::size_t i = 0; i < temporary.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = temporary[i][0]; p.y = temporary[i][1]; p.z = temporary[i][2];
    zone_positions_["temp_" + std::to_string(i)] = p;
  }

  const auto orientation = getParameter(
    node, "tool_orientation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
  if (orientation.size() != 4U) {
    throw std::runtime_error("Parameter 'tool_orientation' must contain [x, y, z, w]");
  }
  if (ik_timeout_ <= 0.0 || max_joint_delta_ <= 0.0) {
    throw std::runtime_error("IK timeout and max joint delta must be positive");
  }
  if (!std::isfinite(placement_clearance_) || placement_clearance_ < 0.0 || placement_clearance_ > 0.01) {
    throw std::runtime_error("Placement clearance must be between 0 and 0.01 metres");
  }
  if (std::abs(gripper_open_position_ - gripper_closed_position_) < 1e-6 ||
    gripper_timeout_ <= 0.0)
  {
    throw std::runtime_error("Gripper parameters are invalid");
  }
  if (transit_joint_margin_ <= 0.0 || cartesian_eef_step_ <= 0.0 ||
    cartesian_jump_threshold_ < 0.0 || cartesian_max_joint_step_ <= 0.0 ||
    trajectory_detour_allowance_ < 0.0 || min_cartesian_fraction_ <= 0.0 ||
    min_cartesian_fraction_ > 1.0 ||
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
  default_tool_orientation_ = tool_orientation_;

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
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!latest_state_) {
          latest_state_ = std::make_shared<moveit::core::RobotState>(move_group_.getRobotModel());
          latest_state_->setToDefaultValues();
        }
        latest_state_->setVariablePositions(message->name, message->position);
        latest_state_->update();
        latest_state_time_ = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
      }
    }, joint_state_options);

  camera_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/environment_state", 1,
    [this](const std_msgs::msg::String::SharedPtr message) {
      try {
        auto scene = nlohmann::json::parse(message->data);
        if (!scene.at("stream_id").is_string() || !scene.at("frame_id").is_number_integer()) {
          return;
        }
        std::lock_guard<std::mutex> lock(camera_mutex_);
        if (!camera_scene_.is_null() && scene.at("stream_id") == camera_scene_.at("stream_id") &&
          scene.at("frame_id").get<int64_t>() <= camera_scene_.at("frame_id").get<int64_t>())
        {return;}
        camera_scene_ = std::move(scene);
        camera_received_ = std::chrono::steady_clock::now();
      } catch (const nlohmann::json::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "Invalid camera observation: %s", e.what());
      }
    }, joint_state_options);

  wrist_camera_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/wrist_camera/observation", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      try {
        auto observation = nlohmann::json::parse(message->data);
        if (observation.at("source") != "wrist_camera_rgb" ||
          !observation.at("stream_id").is_string() ||
          !observation.at("frame_id").is_number_integer())
        {return;}
        std::lock_guard<std::mutex> lock(wrist_camera_mutex_);
        if (!wrist_camera_observation_.is_null() &&
          observation.at("stream_id") == wrist_camera_observation_.at("stream_id") &&
          observation.at("frame_id").get<int64_t>() <=
          wrist_camera_observation_.at("frame_id").get<int64_t>())
        {return;}
        wrist_camera_observation_ = std::move(observation);
        wrist_camera_received_ = std::chrono::steady_clock::now();
      } catch (const nlohmann::json::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "Invalid wrist-camera observation: %s", e.what());
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
  const auto response = future.get();
  if (!response->valid && !response->contacts.empty()) {
    const auto & contact = response->contacts.front();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "IK collision between %s and %s.", contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
  }
  return response->valid;
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
  if (trajectory.points.empty()) {return false;}
  const auto initial_values = previous_values;
  std::vector<double> total_travel(previous_values.size(), 0.0);

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
      total_travel[index] += delta;
      previous_values[index] = normalized;
    }
  }
  for (std::size_t index = 0; index < total_travel.size(); ++index) {
    const double direct = std::abs(previous_values[index] - initial_values[index]);
    if (total_travel[index] > direct + trajectory_detour_allowance_) {
      RCLCPP_ERROR(node_->get_logger(), "Rejected %s: joint %s travels %.3f rad for a %.3f rad change.",
        label.c_str(), trajectory.joint_names[index].c_str(), total_travel[index], direct);
      return false;
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
  bool carrying;
  {std::lock_guard<std::mutex> lock(state_mutex_); carrying = !held_object_.empty();}
  if (carrying) {return moveCartesianToPose(pose, label);}
  const auto current_state = getLatestState(label);
  if (!current_state) {
    return SkillStatus::PLANNING_FAILED;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto status = planToPose(pose, label, current_state, plan);
  if (status != SkillStatus::SUCCESS) {return status;}
  if (execute_motion_ && !static_cast<bool>(move_group_.execute(plan))) {
    publishSubskillStatus("execute(" + label + ")", SkillStatus::FAILED);
    return SkillStatus::FAILED;
  }
  return SkillStatus::SUCCESS;
}

SkillStatus RobotSkills::planToPose(
  const geometry_msgs::msg::Pose & pose, const std::string & label,
  const moveit::core::RobotStatePtr & current_state,
  moveit::planning_interface::MoveGroupInterface::Plan & plan)
{

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
  // Prefer the current elbow/wrist branch instead of resetting the seed on every transfer.
  if (!target_state.setFromIK(
      joint_model_group, pose, end_effector_link_, ik_timeout_, valid_ik))
  {
    if (!target_state.setToDefaultValues(joint_model_group, ik_seed_target_)) {
      return SkillStatus::PLANNING_FAILED;
    }
    target_state.update();
    if (!target_state.setFromIK(
        joint_model_group, pose, end_effector_link_, ik_timeout_, valid_ik))
    {
      RCLCPP_ERROR(node_->get_logger(), "IK failed for %s.", label.c_str());
      return SkillStatus::PLANNING_FAILED;
    }
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
  const auto planning_result = move_group_.plan(plan);
  move_group_.clearPathConstraints();
  if (!static_cast<bool>(planning_result)) {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed for %s.", label.c_str());
    return SkillStatus::PLANNING_FAILED;
  }
  if (!normalizeTrajectory(plan, *current_state, label)) {
    publishSubskillStatus("trajectory_bounds(" + label + ")", SkillStatus::PLANNING_FAILED);
    return SkillStatus::PLANNING_FAILED;
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
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto status = planCartesianToPose(pose, label, current_state, plan);
  if (status != SkillStatus::SUCCESS) {return status;}
  if (execute_motion_ && !executeWithGraspMonitor(plan, label)) {
    publishSubskillStatus("execute(" + label + ")", SkillStatus::FAILED);
    return SkillStatus::FAILED;
  }
  return SkillStatus::SUCCESS;
}

SkillStatus RobotSkills::planCartesianToPose(
  const geometry_msgs::msg::Pose & pose, const std::string & label,
  const moveit::core::RobotStatePtr & current_state,
  moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  move_group_.setStartState(*current_state);
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = 0.0;
  bool continuous = false;
  for (const double step : {cartesian_eef_step_, cartesian_eef_step_ * 0.5, cartesian_eef_step_ * 0.25}) {
    trajectory = moveit_msgs::msg::RobotTrajectory();
    fraction = move_group_.computeCartesianPath(
      {pose}, step, cartesian_jump_threshold_, trajectory, true);
    if (!std::isfinite(fraction) || fraction < min_cartesian_fraction_) {continue;}
    continuous = !trajectory.joint_trajectory.points.empty();
    std::vector<double> previous;
    for (const auto & name : trajectory.joint_trajectory.joint_names) {
      previous.push_back(current_state->getVariablePosition(name));
    }
    for (const auto & point : trajectory.joint_trajectory.points) {
      if (point.positions.size() != previous.size()) {continuous = false; break;}
      for (std::size_t i = 0; i < previous.size(); ++i) {
        const double delta = std::remainder(point.positions[i] - previous[i], 2.0 * M_PI);
        if (!std::isfinite(delta) || std::abs(delta) > cartesian_max_joint_step_) {
          continuous = false;
          break;
        }
        previous[i] += delta;
      }
      if (!continuous) {break;}
    }
    if (continuous) {break;}
  }
  // The remote Cartesian solver can pick another IK branch before the post-check.
  // Retry with consistency limits enforced inside each seeded IK solve instead.
  if (!continuous) {
    const auto * group = current_state->getJointModelGroup(move_group_.getName());
    if (!group) {return SkillStatus::PLANNING_FAILED;}
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-9) {
      return SkillStatus::PLANNING_FAILED;
    }
    target.linear() = rotation.normalized().toRotationMatrix();
    target.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    target = current_state->getFrameTransform(frame_id_) * target;
    const auto origin = current_state->getGlobalLinkTransform(end_effector_link_);
    const Eigen::Quaterniond initial_rotation(origin.rotation());
    const Eigen::Quaterniond final_rotation(target.rotation());
    const double distance = (target.translation() - origin.translation()).norm();
    const double angle = initial_rotation.angularDistance(final_rotation);
    const std::vector<double> consistency(group->getVariableCount(), cartesian_max_joint_step_);
    for (const double step : {cartesian_eef_step_ * 0.5,
        cartesian_eef_step_ * 0.25, cartesian_eef_step_ * 0.125}) {
      moveit::core::RobotState state(*current_state);
      moveit_msgs::msg::RobotTrajectory candidate;
      auto & joints = candidate.joint_trajectory;
      joints.joint_names = group->getVariableNames();
      trajectory_msgs::msg::JointTrajectoryPoint initial;
      state.copyJointGroupPositions(group, initial.positions);
      joints.points.push_back(initial);
      const auto count = static_cast<std::size_t>(std::max(1.0,
        std::max(std::ceil(distance / step), std::ceil(angle / 0.01))));
      bool solved = true;
      for (std::size_t index = 1; index <= count; ++index) {
        const double ratio = static_cast<double>(index) / static_cast<double>(count);
        Eigen::Isometry3d waypoint = Eigen::Isometry3d::Identity();
        waypoint.translation() = origin.translation() + ratio * (target.translation() - origin.translation());
        waypoint.linear() = initial_rotation.slerp(ratio, final_rotation).toRotationMatrix();
        const auto previous_state = state;
        const auto previous = joints.points.back().positions;
        const auto valid = [this, &previous, &previous_state](moveit::core::RobotState * solution,
          const moveit::core::JointModelGroup * solution_group, const double * values) {
            std::vector<double> normalized(values, values + solution_group->getVariableCount());
            if (!normalizeRevoluteTargets(solution_group, previous, normalized)) {return false;}
            double maximum = 0.0;
            for (std::size_t i = 0; i < normalized.size(); ++i) {
              const double delta = std::abs(normalized[i] - previous[i]);
              if (!std::isfinite(delta) || delta > cartesian_max_joint_step_) {return false;}
              maximum = std::max(maximum, delta);
            }
            // Check intermediate joint states as well as each Cartesian sample.
            const auto samples = static_cast<std::size_t>(std::max(1.0, std::ceil(maximum / 0.02)));
            for (std::size_t sample = 1; sample <= samples; ++sample) {
              moveit::core::RobotState intermediate(previous_state);
              auto interpolated = previous;
              for (std::size_t i = 0; i < interpolated.size(); ++i) {
                interpolated[i] += (normalized[i] - previous[i]) *
                  static_cast<double>(sample) / static_cast<double>(samples);
              }
              intermediate.setJointGroupPositions(solution_group, interpolated);
              intermediate.update();
              if (!intermediate.satisfiesBounds(solution_group) ||
                !isStateValid(intermediate, solution_group->getName())) {return false;}
            }
            solution->setJointGroupPositions(solution_group, normalized);
            solution->update();
            return true;
          };
        if (!state.setFromIK(group, waypoint, end_effector_link_, consistency, ik_timeout_, valid)) {
          solved = false;
          std_msgs::msg::String detail;
          detail.data = "PLANNING_DETAIL: seeded Cartesian IK/collision failed for " + label +
            " at sample " + std::to_string(index) + "/" + std::to_string(count);
          status_publisher_->publish(detail);
          break;
        }
        trajectory_msgs::msg::JointTrajectoryPoint point;
        state.copyJointGroupPositions(group, point.positions);
        joints.points.push_back(point);
      }
      if (solved) {
        trajectory = std::move(candidate);
        fraction = 1.0;
        continuous = true;
        std_msgs::msg::String detail;
        detail.data = "PLANNING_DETAIL: continuous seeded Cartesian path accepted for " + label;
        status_publisher_->publish(detail);
        break;
      }
    }
  }
  if (!std::isfinite(fraction) || fraction < min_cartesian_fraction_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cartesian path for %s reached only %.1f%%; refusing a non-linear fallback.",
      label.c_str(), fraction * 100.0);
    std_msgs::msg::String message;
    message.data = "PLANNING_DETAIL: " + label + " Cartesian path reached " +
      std::to_string(fraction * 100.0) + "% (IK/collision/jump filter); no motion executed";
    status_publisher_->publish(message);
    return SkillStatus::PLANNING_FAILED;
  }
  if (!continuous) {
    publishSubskillStatus("joint_jump(" + label + ")", SkillStatus::PLANNING_FAILED);
    return SkillStatus::PLANNING_FAILED;
  }

  moveit::core::robotStateToRobotStateMsg(*current_state, plan.start_state_);
  plan.trajectory_ = std::move(trajectory);
  if (!normalizeTrajectory(plan, *current_state, label)) {
    publishSubskillStatus("trajectory_bounds(" + label + ")", SkillStatus::PLANNING_FAILED);
    return SkillStatus::PLANNING_FAILED;
  }

  // Reject wrist/elbow branch jumps even when the end-effector path is straight.
  std::vector<double> previous;
  for (const auto & name : plan.trajectory_.joint_trajectory.joint_names) {
    previous.push_back(current_state->getVariablePosition(name));
  }
  for (const auto & point : plan.trajectory_.joint_trajectory.points) {
    for (std::size_t i = 0; i < previous.size(); ++i) {
      if (std::abs(point.positions[i] - previous[i]) > cartesian_max_joint_step_) {
        RCLCPP_ERROR(node_->get_logger(), "Cartesian IK branch jump for %s.", label.c_str());
        publishSubskillStatus("joint_jump(" + label + ")", SkillStatus::PLANNING_FAILED);
        return SkillStatus::PLANNING_FAILED;
      }
      previous[i] = point.positions[i];
    }
  }

  robot_trajectory::RobotTrajectory timed_trajectory(
    move_group_.getRobotModel(), move_group_.getName());
  timed_trajectory.setRobotTrajectoryMsg(*current_state, plan.trajectory_);
  trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
  if (!time_parameterization.computeTimeStamps(
      timed_trajectory, cartesian_velocity_scale_, cartesian_acceleration_scale_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed for %s.", label.c_str());
    publishSubskillStatus("time_parameterization(" + label + ")", SkillStatus::PLANNING_FAILED);
    return SkillStatus::PLANNING_FAILED;
  }
  timed_trajectory.getRobotTrajectoryMsg(plan.trajectory_);

  return SkillStatus::SUCCESS;
}

bool RobotSkills::executeWithGraspMonitor(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan, const std::string & label)
{
  std::string held;
  {std::lock_guard<std::mutex> lock(state_mutex_); held = held_object_;}
  if (held.empty()) {return static_cast<bool>(move_group_.execute(plan));}

  auto execution = std::async(std::launch::async, [this, &plan]() {
    return static_cast<bool>(move_group_.execute(plan));
  });
  std::string previous_stream;
  int64_t previous_frame = -1;
  unsigned int mismatches = 0;
  while (execution.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    nlohmann::json scene;
    std::chrono::steady_clock::time_point received;
    {std::lock_guard<std::mutex> lock(camera_mutex_); scene = camera_scene_; received = camera_received_;}
    moveit::core::RobotStatePtr state;
    {std::lock_guard<std::mutex> lock(state_mutex_);
      if (latest_state_) {state = std::make_shared<moveit::core::RobotState>(*latest_state_);}}
    try {
      if (state && cameraObservationFresh(scene, received)) {
        const auto stream = scene.at("stream_id").get<std::string>();
        const auto frame = scene.at("frame_id").get<int64_t>();
        if (stream != previous_stream || frame > previous_frame) {
          previous_stream = stream;
          previous_frame = frame;
          if (scene.at("objects").contains(held)) {
            const auto & p = scene.at("objects").at(held);
            const Eigen::Vector3d measured(p.at(0).get<double>(), p.at(1).get<double>(), p.at(2).get<double>());
            const Eigen::Vector3d expected = state->getGlobalLinkTransform(end_effector_link_) *
              Eigen::Vector3d(0.0, 0.0, grasp_offset_);
            if (measured.allFinite() &&
              (measured - expected).norm() < kHeldObjectCameraToleranceM) {
              mismatches = 0;
            } else {
              ++mismatches;
            }
          }
        }
      }
    } catch (const nlohmann::json::exception &) {
      // Missing/occluded detections are unknown, not proof that the cube was dropped.
    }
    // Stop only on repeated fresh detections that positively place the cube away
    // from the gripper. A missing detection is treated as occlusion, not a drop.
    if (!rclcpp::ok() || mismatches >= 2U) {
      move_group_.stop();
      (void)execution.get();
      RCLCPP_ERROR(node_->get_logger(),
        "Stopped %s: camera repeatedly detected %s away from the gripper.",
        label.c_str(), held.c_str());
      publishSubskillStatus("grasp_monitor(" + held + ")", SkillStatus::FAILED);
      return false;
    }
  }
  return execution.get();
}

bool RobotSkills::commandGripper(
  const double position, const std::string & label)
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
  point.time_from_start.sec = 1;
  point.time_from_start.nanosec = 500000000;
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
    gripper_action_client_->async_cancel_goal(goal_handle);
    RCLCPP_ERROR(node_->get_logger(), "Timed out moving gripper for %s.", label.c_str());
    return false;
  }
  const auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    return true;
  }
  // An aborted effort-controller goal may switch to holding its current position,
  // removing the commanded squeeze. Never count that as a successful grasp.
  RCLCPP_ERROR(node_->get_logger(), "Gripper command failed for %s.", label.c_str());
  return false;
}

void RobotSkills::addPlanningScene()
{
  const auto table_center = pointFromVector(node_, "table.position", {0.25, 0.0, 0.70});
  const auto table_size = getParameter(
    node_, "table.size", std::vector<double>{0.80, 0.65, 0.04});
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
  if (!refreshCameraScene()) {return SkillStatus::INVALID_STATE;}
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
  return commandGripper(gripper_closed_position_, "close gripper") ?
         SkillStatus::SUCCESS : SkillStatus::FAILED;
}

void RobotSkills::publishSubskillStatus(const std::string & name, const SkillStatus status)
{
  RCLCPP_DEBUG(node_->get_logger(), "SUBSKILL: %s -> %s", name.c_str(), toString(status));
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
  if (!acquireFullCameraScene()) {return SkillStatus::INVALID_STATE;}
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

  const auto object_center = object->second;
  geometry_msgs::msg::Pose grasp_pose;
  geometry_msgs::msg::Pose above_pose;
  const auto gripper_status = openGripper();
  publishSubskillStatus("open_gripper", gripper_status);
  if (gripper_status != SkillStatus::SUCCESS) {
    return SkillStatus::FAILED;
  }
  bool descended = false;
  SkillStatus status = SkillStatus::PLANNING_FAILED;
  const Eigen::Quaterniond base_orientation(
    default_tool_orientation_.w, default_tool_orientation_.x,
    default_tool_orientation_.y, default_tool_orientation_.z);
  // A square cube permits these jaw orientations without changing the grasp target.
  for (const double yaw : {0.0, M_PI / 2.0, -M_PI / 2.0, M_PI}) {
    const Eigen::Quaterniond orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) * base_orientation;
    tool_orientation_.x = orientation.x(); tool_orientation_.y = orientation.y();
    tool_orientation_.z = orientation.z(); tool_orientation_.w = orientation.w();
    grasp_pose = makeToolPose(object_center);
    for (const double height : {approach_height_, approach_height_ * 0.8, approach_height_ * 0.6}) {
      above_pose = grasp_pose;
      above_pose.position.z += height;
      auto start = getLatestState("grasp preflight " + object_name);
      if (!start) {tool_orientation_ = default_tool_orientation_; return SkillStatus::INVALID_STATE;}
      moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
      status = planToPose(above_pose, "above " + object_name, start, approach_plan);
      if (status != SkillStatus::SUCCESS) {continue;}

      auto approach_end = std::make_shared<moveit::core::RobotState>(*start);
      const auto & joints = approach_plan.trajectory_.joint_trajectory;
      approach_end->setVariablePositions(joints.joint_names, joints.points.back().positions);
      approach_end->update();
      // Only the grasped object may be contacted; table and other cubes remain checked.
      if (!removeCube(object_name)) {return SkillStatus::FAILED;}
      moveit::planning_interface::MoveGroupInterface::Plan descent_plan;
      status = planCartesianToPose(grasp_pose, "preflight descend to " + object_name,
        approach_end, descent_plan);
      if (status == SkillStatus::SUCCESS && !pending_destination_.empty()) {
        const auto destination = zone_positions_.find(pending_destination_);
        if (destination == zone_positions_.end()) {
          addCube(object_name, object_center);
          return SkillStatus::INVALID_ZONE;
        }
        auto carry_pose = makeToolPose(destination->second);
        carry_pose.position.z += approach_height_ + placement_clearance_;
        moveit::planning_interface::MoveGroupInterface::Plan carry_plan;
        status = planCartesianToPose(carry_pose, "preflight carry to " + pending_destination_,
          approach_end, carry_plan);
      }
      addCube(object_name, object_center);
      if (status != SkillStatus::SUCCESS) {continue;}

      publishSubskillStatus("grasp_preflight(" + object_name + ")", SkillStatus::SUCCESS);
      if (execute_motion_ && !static_cast<bool>(move_group_.execute(approach_plan))) {
        return SkillStatus::FAILED;
      }
      publishSubskillStatus("move_above(" + object_name + ")", SkillStatus::SUCCESS);
      if (!removeCube(object_name)) {return SkillStatus::FAILED;}
      // Recompute from actual controller feedback, not the ideal planned end state.
      status = moveCartesianToPose(grasp_pose, "descend to " + object_name);
      publishSubskillStatus("descend(" + object_name + ")", status);
      if (status == SkillStatus::SUCCESS) {descended = true; break;}
      addCube(object_name, object_center);
      if (status != SkillStatus::PLANNING_FAILED) {return status;}
    }
    if (descended) {break;}
  }
  if (!descended) {
    tool_orientation_ = default_tool_orientation_;
    publishSubskillStatus("grasp_preflight(" + object_name + ")", SkillStatus::PLANNING_FAILED);
    return SkillStatus::PLANNING_FAILED;
  }
  const auto close_status = closeGripper();
  publishSubskillStatus("close_gripper", close_status);
  if (close_status != SkillStatus::SUCCESS) {
    addCube(object_name, object_center);
    return SkillStatus::FAILED;
  }
  if (!attachCube(object_name)) {
    addCube(object_name, object_center);
    return SkillStatus::FAILED;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_ = object_name;
  }

  status = moveCartesianToPose(above_pose, "lift " + object_name);
  if (status != SkillStatus::SUCCESS) {
    // Retain the held state: a failed motion does not imply that the fingers released.
    return status;
  }
  if (execute_motion_) {
    if (!verifyHeldObject(object_name)) {
      RCLCPP_ERROR(node_->get_logger(), "Camera could not confirm %s at the measured gripper pose after lift.",
        object_name.c_str());
      publishSubskillStatus("verify_grasp(" + object_name + ")", SkillStatus::FAILED);
      return SkillStatus::FAILED;
    }
    publishSubskillStatus("verify_grasp(" + object_name + ")", SkillStatus::SUCCESS);
  }
  return SkillStatus::SUCCESS;
}

SkillStatus RobotSkills::place(
  const std::string & object_name, const std::string & zone_name)
{
  if (!refreshCameraScene(false)) {return SkillStatus::INVALID_STATE;}
  double baseline_observed_at;
  {std::lock_guard<std::mutex> lock(camera_mutex_);
    baseline_observed_at = camera_scene_.at("observed_at").get<double>();}
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

  if (!destinationFree(zone->second, object_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Destination %s is occupied.", zone_name.c_str());
    return SkillStatus::INVALID_STATE;
  }
  auto place_pose = makeToolPose(zone->second);
  // Keep a 2 mm collision margin above the table; gravity settles the released cube.
  place_pose.position.z += placement_clearance_;
  auto above_pose = place_pose;
  above_pose.position.z += approach_height_;
  auto status = moveCartesianToPose(above_pose, "carry straight to " + zone_name);
  publishSubskillStatus("move_to_zone(" + zone_name + ")", status);
  if (status != SkillStatus::SUCCESS) {
    return status;
  }
  if (!verifyHeldObject(object_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot confirm %s is still held above %s.",
      object_name.c_str(), zone_name.c_str());
    publishSubskillStatus("verify_grasp(" + object_name + ")", SkillStatus::INVALID_STATE);
    return SkillStatus::INVALID_STATE;
  }
  publishSubskillStatus("verify_grasp(" + object_name + ")", SkillStatus::SUCCESS);
  const auto observe_scene = [this, &zone, &object_name, baseline_observed_at]() {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      do {
        if (refreshPlacementScene(zone->second, object_name, baseline_observed_at)) {return true;}
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
      return false;
    };
  if (!observe_scene()) {
    RCLCPP_ERROR(node_->get_logger(), "Camera cannot confirm the destination before placing %s; keeping gripper closed.",
      object_name.c_str());
    publishSubskillStatus("verify_scene(" + zone_name + ")", SkillStatus::INVALID_STATE);
    return SkillStatus::INVALID_STATE;
  }
  if (!destinationFree(zone->second, object_name)) {
    RCLCPP_ERROR(node_->get_logger(), "Destination %s became occupied; keeping gripper closed.", zone_name.c_str());
    publishSubskillStatus("verify_destination(" + zone_name + ")", SkillStatus::INVALID_STATE);
    return SkillStatus::INVALID_STATE;
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

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_.clear();
  }
  const auto retreat = moveCartesianToPose(above_pose, "retreat from " + zone_name);
  if (retreat != SkillStatus::SUCCESS) {return retreat;}
  if (!execute_motion_) {
    // Planning-only mode has no physical outcome and never reports a completed task.
    return SkillStatus::INVALID_STATE;
  }
  if (!verifyObserved(object_name, zone->second)) {return SkillStatus::FAILED;}
  return refreshPlacementScene(zone->second, object_name, baseline_observed_at) ?
    SkillStatus::SUCCESS : SkillStatus::INVALID_STATE;
}

bool RobotSkills::refreshPlacementScene(
  const geometry_msgs::msg::Point & target, const std::string & object,
  const double baseline_observed_at)
{
  // Other blocks are passive during this transfer. Preserve their camera-derived
  // collision objects when occluded far from the destination; never erase them.
  nlohmann::json scene;
  std::chrono::steady_clock::time_point received;
  {std::lock_guard<std::mutex> lock(camera_mutex_); scene = camera_scene_; received = camera_received_;}
  try {
    if (!cameraObservationFresh(scene, received)) {return false;}
    const auto & objects = scene.at("objects");
    if (!objects.is_object()) {return false;}
    auto updated = object_positions_;
    std::string held;
    {std::lock_guard<std::mutex> lock(state_mutex_); held = held_object_;}
    for (auto & entry : updated) {
      if (entry.first == held) {continue;}
      if (!objects.contains(entry.first)) {
        const double baseline_age = wallTimeSeconds() - baseline_observed_at;
        if (entry.first == object || baseline_age < 0.0 || baseline_age > 30.0 ||
          std::hypot(entry.second.x - target.x, entry.second.y - target.y) < 0.135) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "Placement observation missing required block %s.", entry.first.c_str());
          return false;
        }
        continue;
      }
      const auto & values = objects.at(entry.first);
      if (!values.is_array() || values.size() != 3U) {return false;}
      geometry_msgs::msg::Point point;
      point.x = values.at(0).get<double>(); point.y = values.at(1).get<double>();
      point.z = values.at(2).get<double>();
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        point.x < 0.10 || point.x > 0.58 || std::abs(point.y) > 0.28 ||
        std::abs(point.z - 0.740) > 0.015) {return false;}
      entry.second = point;
    }
    object_positions_ = std::move(updated);
    for (const auto & entry : object_positions_) {
      if (entry.first != held) {addCube(entry.first, entry.second);}
    }
    return true;
  } catch (const nlohmann::json::exception &) {return false;}
}

bool RobotSkills::destinationFree(
  const geometry_msgs::msg::Point & target, const std::string & exclude,
  const double clearance) const
{
  for (const auto & entry : object_positions_) {
    if (entry.first != exclude &&
      std::hypot(entry.second.x - target.x, entry.second.y - target.y) < clearance)
    {return false;}
  }
  return true;
}

bool RobotSkills::refreshCameraScene(const bool require_all)
{
  std::string held;
  {std::lock_guard<std::mutex> lock(state_mutex_); held = held_object_;}
  nlohmann::json scene;
  std::chrono::steady_clock::time_point received;
  {std::lock_guard<std::mutex> lock(camera_mutex_); scene = camera_scene_; received = camera_received_;}
  try {
    if (!cameraObservationFresh(scene, received)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Camera scene unavailable: observation is stale.");
      return false;
    }
    const std::set<std::string> names{"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"};
    std::map<std::string, geometry_msgs::msg::Point> observed;
    const auto & objects = scene.at("objects");
    if (!objects.is_object()) {return false;}
    for (const auto & name : names) {
      if (name == held && !require_all) {continue;}
      if (!objects.contains(name)) {
        const auto previous = object_positions_.find(name);
        if (!held.empty() && !require_all && previous != object_positions_.end()) {
          // The arm or held cube can hide another block from the fixed camera.
          // No other object moves during this transfer, so retain its last
          // camera-measured pose until the next complete scene is available.
          observed[name] = previous->second;
          RCLCPP_DEBUG(node_->get_logger(),
            "Keeping last camera pose for occluded %s while holding %s.",
            name.c_str(), held.c_str());
          continue;
        }
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Camera scene incomplete: %s is not visible (held object: %s).", name.c_str(), held.c_str());
        return false;
      }
      const auto & values = objects.at(name);
      if (!values.is_array() || values.size() != 3U) {return false;}
      geometry_msgs::msg::Point p;
      p.x = values.at(0).get<double>(); p.y = values.at(1).get<double>(); p.z = values.at(2).get<double>();
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          p.x < 0.10 || p.x > 0.58 || std::abs(p.y) > 0.28 || std::abs(p.z - 0.740) > 0.015)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "Camera reports invalid table position for %s: [%.3f, %.3f, %.3f].",
          name.c_str(), p.x, p.y, p.z);
        return false;
      }
      observed[name] = p;
    }
    if (!held.empty() && object_positions_.count(held)) {observed[held] = object_positions_.at(held);}
    object_positions_ = observed;
    for (const auto & entry : observed) {
      if (entry.first != held) {addCube(entry.first, entry.second);}
    }
    return true;
  } catch (const nlohmann::json::exception &) {return false;}
}

bool RobotSkills::verifyObserved(
  const std::string & object, const geometry_msgs::msg::Point & expected, const bool lifted)
{
  const auto after = wallTimeSeconds();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  unsigned int confirmations = 0;
  std::string previous_stream;
  int64_t previous_frame = -1;
  bool saw_object = false;
  double last_xy_error = std::numeric_limits<double>::infinity();
  double last_z_error = std::numeric_limits<double>::infinity();
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    nlohmann::json scene;
    std::chrono::steady_clock::time_point received;
    {std::lock_guard<std::mutex> lock(camera_mutex_); scene = camera_scene_; received = camera_received_;}
    try {
      if (cameraObservationFresh(scene, received) && scene.at("observed_at").get<double>() > after) {
        const auto stream = scene.at("stream_id").get<std::string>();
        const auto frame = scene.at("frame_id").get<int64_t>();
        if (stream == previous_stream && frame <= previous_frame) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        if (stream != previous_stream) {confirmations = 0;}
        previous_stream = stream;
        previous_frame = frame;
        if (!scene.at("objects").contains(object)) {confirmations = 0;}
        else {
          const auto & p = scene.at("objects").at(object);
          saw_object = true;
          last_xy_error = std::hypot(
            p.at(0).get<double>() - expected.x, p.at(1).get<double>() - expected.y);
          last_z_error = std::abs(p.at(2).get<double>() - expected.z);
          // A side-oblique RGB-D view has slightly larger centroid error than a nadir view.
          // Keep the tolerance below half the spacing between adjacent destination zones.
          const bool matches = last_xy_error < 0.055 &&
            last_z_error < (lifted ? 0.050 : 0.040);
          confirmations = matches ? confirmations + 1 : 0;
          if (confirmations >= 3U) {return true;}
        }
      }
    } catch (const nlohmann::json::exception &) {confirmations = 0;}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (saw_object) {
    RCLCPP_ERROR(node_->get_logger(),
      "Camera saw %s but its measured pose missed the destination (XY error %.3f m, Z error %.3f m).",
      object.c_str(), last_xy_error, last_z_error);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Camera did not detect %s after placement.", object.c_str());
  }
  return false;
}

bool RobotSkills::verifyHeldObject(const std::string & object)
{
  const double after = wallTimeSeconds();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  unsigned int confirmations = 0;
  std::string previous_stream;
  int64_t previous_frame = -1;
  int64_t last_pixel_count = 0;
  bool received_fresh_frame = false;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    nlohmann::json observation;
    std::chrono::steady_clock::time_point received;
    {
      std::lock_guard<std::mutex> lock(wrist_camera_mutex_);
      observation = wrist_camera_observation_;
      received = wrist_camera_received_;
    }
    try {
      if (wristCameraObservationFresh(observation, received) &&
        observation.at("observed_at").get<double>() > after)
      {
        received_fresh_frame = true;
        const auto stream = observation.at("stream_id").get<std::string>();
        const auto frame = observation.at("frame_id").get<int64_t>();
        if (stream != previous_stream || frame > previous_frame) {
          previous_stream = stream;
          previous_frame = frame;
          const auto & detections = observation.at("detections");
          const bool visible = detections.contains(object);
          double depth = std::numeric_limits<double>::infinity();
          if (visible) {
            const auto & detection = detections.at(object);
            last_pixel_count = detection.at("pixels").get<int64_t>();
            depth = detection.at("depth_m").get<double>();
          } else {
            last_pixel_count = 0;
          }
          const bool grasp_range = visible && std::isfinite(depth) &&
            depth >= 0.04 && depth <= 0.16 && last_pixel_count >= 24;
          confirmations = grasp_range ? confirmations + 1U : 0U;
          if (confirmations >= kHeldObjectCameraConfirmations) {
            RCLCPP_INFO(node_->get_logger(),
              "Wrist camera confirms %s in the gripper (%ld color pixels, depth %.3f m, %u frames).",
              object.c_str(), static_cast<long>(last_pixel_count), depth, confirmations);
            return true;
          }
        }
      }
    } catch (const nlohmann::json::exception &) {confirmations = 0;}
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_ERROR(node_->get_logger(),
    "Wrist camera could not confirm %s in the jaw range (%s, last count %ld pixels).",
    object.c_str(), received_fresh_frame ? "fresh image received" : "no fresh image",
    static_cast<long>(last_pixel_count));
  return false;
}

bool RobotSkills::canReachDestination(const geometry_msgs::msg::Point & target)
{
  auto current = getLatestState("destination preflight");
  if (!current) {return false;}
  const auto * group = current->getJointModelGroup(move_group_.getName());
  if (!group) {return false;}
  const auto valid = [this](moveit::core::RobotState * state,
    const moveit::core::JointModelGroup * candidate_group, const double * values) {
      state->setJointGroupPositions(candidate_group, values);
      state->update();
      return state->satisfiesBounds(candidate_group) && isStateValid(*state, candidate_group->getName());
    };
  const Eigen::Quaterniond base(
    default_tool_orientation_.w, default_tool_orientation_.x,
    default_tool_orientation_.y, default_tool_orientation_.z);
  for (const double yaw : {0.0, M_PI / 2.0, -M_PI / 2.0, M_PI}) {
    auto pose = makeToolPose(target);
    pose.position.z += approach_height_ + placement_clearance_;
    const Eigen::Quaterniond orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())) * base;
    pose.orientation.x = orientation.x(); pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z(); pose.orientation.w = orientation.w();
    moveit::core::RobotState candidate(*current);
    if (candidate.setFromIK(group, pose, end_effector_link_, ik_timeout_, valid)) {return true;}
    candidate = *current;
    candidate.setToDefaultValues(group, ik_seed_target_);
    if (candidate.setFromIK(group, pose, end_effector_link_, ik_timeout_, valid)) {return true;}
  }
  return false;
}

bool RobotSkills::acquireFullCameraScene()
{
  const auto observe = [this](const double after, const double timeout) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
      do {
        bool newer = false;
        try {
          std::lock_guard<std::mutex> lock(camera_mutex_);
          newer = camera_scene_.at("observed_at").get<double>() > after;
        } catch (const nlohmann::json::exception &) {}
        if (newer && refreshCameraScene()) {return true;}
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
      return false;
    };
  if (observe(0.0, 1.0)) {return true;}
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!held_object_.empty()) {return false;}
  }
  // Retreat above the previous zone can still occlude the next block. Reposition
  // the empty arm once, before the next pick, to obtain a complete camera view.
  const auto recovery = home();
  publishSubskillStatus("camera_observation_home", recovery);
  if (recovery != SkillStatus::SUCCESS) {return false;}
  if (!observe(wallTimeSeconds(), 3.0)) {
    publishSubskillStatus("camera_full_scene", SkillStatus::INVALID_STATE);
    return false;
  }
  publishSubskillStatus("camera_full_scene", SkillStatus::SUCCESS);
  return true;
}

bool RobotSkills::checkTransfer(
  const std::string & object, std::string & destination)
{
  pending_destination_.clear();
  if (!acquireFullCameraScene() || object_positions_.count(object) == 0U ||
    zone_positions_.count(destination) == 0U)
  {return false;}
  const auto usable = [this, &object](const std::string & candidate) {
      const auto & target = zone_positions_.at(candidate);
      const double clearance = candidate.rfind("temp_", 0U) == 0U ? 0.12 : 0.095;
      return destinationFree(target, object, clearance) && canReachDestination(target);
    };
  if (!usable(destination)) {
    if (destination.rfind("temp_", 0U) != 0U) {
      publishSubskillStatus("transfer_destination_occupied(" + destination + ")", SkillStatus::INVALID_STATE);
      return false;
    }
    std::string alternative;
    for (std::size_t i = 0; i < 20U; ++i) {
      const auto candidate = "temp_" + std::to_string(i);
      if (candidate != destination && usable(candidate)) {
        alternative = candidate;
        break;
      }
    }
    if (alternative.empty()) {
      publishSubskillStatus("transfer_no_free_temporary_slot", SkillStatus::INVALID_STATE);
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "Temporary slot %s is unavailable; using %s.",
      destination.c_str(), alternative.c_str());
    destination = alternative;
  }
  pending_destination_ = destination;
  return true;
}

bool RobotSkills::verifyFinalPlan(const nlohmann::json & plan)
{
  std::map<std::string, std::string> final_destinations;
  for (const auto & step : plan.at("plan")) {
    if (step.at("skill") == "place") {
      final_destinations[step.at("object").get<std::string>()] =
        step.at("zone").get<std::string>();
    }
  }
  for (const auto & entry : final_destinations) {
    if (!verifyObserved(entry.first, zone_positions_.at(entry.second))) {return false;}
  }
  return refreshCameraScene();
}

bool RobotSkills::validateEnvironmentPlan(nlohmann::json & plan)
{
  {std::lock_guard<std::mutex> lock(state_mutex_); if (!held_object_.empty()) {return false;}}
  if (!execute_motion_ || !refreshCameraScene()) {return false;}
  auto simulated = object_positions_;
  std::string held;
  for (auto & step : plan.at("plan")) {
    const auto skill = step.at("skill").get<std::string>();
    if (skill == "pick") {held = step.at("object").get<std::string>();}
    else if (skill == "place") {
      const auto free_and_reachable = [this, &simulated, &held](const std::string & name) {
        const auto & point = zone_positions_.at(name);
        for (const auto & entry : simulated) {
          const double clearance = name.rfind("temp_", 0U) == 0U ? 0.12 : 0.095;
          if (entry.first != held &&
            std::hypot(entry.second.x-point.x, entry.second.y-point.y) < clearance)
          {return false;}
        }
        return canReachDestination(point);
      };
      auto destination = step.at("zone").get<std::string>();
      if (!free_and_reachable(destination)) {
        if (destination.rfind("temp_", 0U) != 0U) {return false;}
        std::string alternative;
        for (std::size_t i = 0; i < 20U; ++i) {
          const auto name = "temp_" + std::to_string(i);
          if (name != destination && free_and_reachable(name)) {
            alternative = name;
            break;
          }
        }
        if (alternative.empty()) {
          RCLCPP_ERROR(node_->get_logger(), "No free, reachable temporary destination.");
          return false;
        }
        destination = alternative;
        step["zone"] = destination;
      }
      const auto & target = zone_positions_.at(destination);
      simulated[held] = target;
      held.clear();
    } else if (!held.empty()) {return false;}
  }
  return true;
}
}  // namespace ur3_llm_control
