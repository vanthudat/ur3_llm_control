#include <atomic>
#include <chrono>
#include <exception>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ur3_llm_control/robot_skills.hpp"

namespace ur3_llm_control
{
namespace
{
const std::set<std::string> kObjects{"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"};
const std::set<std::string> kZones{
  "zone_a", "zone_b", "zone_c",
  "temp_0", "temp_1", "temp_2", "temp_3", "temp_4", "temp_5", "temp_6",
  "temp_7", "temp_8", "temp_9", "temp_10", "temp_11", "temp_12", "temp_13",
  "temp_14", "temp_15", "temp_16", "temp_17", "temp_18", "temp_19"};
constexpr std::size_t kMaxPlanSteps = 40U;

bool hasExactFields(const nlohmann::json & value, const std::set<std::string> & fields)
{
  if (!value.is_object() || value.size() != fields.size()) {
    return false;
  }
  for (const auto & field : fields) {
    if (!value.contains(field)) {
      return false;
    }
  }
  return true;
}

std::string validatePlan(const nlohmann::json & root)
{
  if (!hasExactFields(root, {"plan"}) || !root.at("plan").is_array() ||
    root.at("plan").empty() || root.at("plan").size() > kMaxPlanSteps)
  {
    return "top level must contain a non-empty 'plan' array with at most 40 steps";
  }

  std::string held_object;
  std::size_t index = 0U;
  for (const auto & step : root.at("plan")) {
    const std::string label = "plan[" + std::to_string(index) + "]";
    if (!step.is_object() || !step.contains("skill") || !step.at("skill").is_string()) {
      return label + " has no string skill";
    }
    const auto skill = step.at("skill").get<std::string>();
    if (skill == "home") {
      if (!hasExactFields(step, {"skill"})) {
        return label + " has unexpected home fields";
      }
    } else if (skill == "open_gripper" || skill == "close_gripper") {
      if (!hasExactFields(step, {"skill"})) {
        return label + " has unexpected gripper fields";
      }
    } else if (skill == "move_above") {
      if (!hasExactFields(step, {"skill", "object"}) || !step.at("object").is_string()) {
        return label + " has invalid move_above fields";
      }
      if (kObjects.count(step.at("object").get<std::string>()) == 0U) {
        return label + " contains an invalid object";
      }
    } else if (skill == "move_to_zone") {
      if (!hasExactFields(step, {"skill", "zone"}) || !step.at("zone").is_string()) {
        return label + " has invalid move_to_zone fields";
      }
      if (kZones.count(step.at("zone").get<std::string>()) == 0U) {
        return label + " contains an invalid zone";
      }
    } else if (skill == "pick") {
      if (!hasExactFields(step, {"skill", "object"}) || !step.at("object").is_string()) {
        return label + " has invalid pick fields";
      }
      const auto object = step.at("object").get<std::string>();
      if (kObjects.count(object) == 0U) {
        return label + " contains an invalid object";
      }
      if (!held_object.empty()) {
        return label + " tries to pick while another object is held";
      }
      held_object = object;
    } else if (skill == "place") {
      if (!hasExactFields(step, {"skill", "object", "zone"}) ||
        !step.at("object").is_string() || !step.at("zone").is_string())
      {
        return label + " has invalid place fields";
      }
      const auto object = step.at("object").get<std::string>();
      const auto zone = step.at("zone").get<std::string>();
      if (kObjects.count(object) == 0U || kZones.count(zone) == 0U) {
        return label + " contains an invalid object or zone";
      }
      if (held_object != object) {
        return label + " places an object that is not held";
      }
      held_object.clear();
    } else {
      return label + " contains a non-whitelisted skill";
    }
    if (skill == "home" && index + 1U != root.at("plan").size()) {
      return label + " must be the final step";
    }
    ++index;
  }
  if (!held_object.empty()) {
    return "plan ends while still holding " + held_object;
  }
  return {};
}

std::string formatStep(const nlohmann::json & step)
{
  const auto skill = step.at("skill").get<std::string>();
  if (skill == "home" || skill == "open_gripper" || skill == "close_gripper") {
    return skill + "()";
  }
  if (skill == "pick" || skill == "move_above") {
    return skill + "(" + step.at("object").get<std::string>() + ")";
  }
  if (skill == "move_to_zone") {
    return skill + "(" + step.at("zone").get<std::string>() + ")";
  }
  return "place(" + step.at("object").get<std::string>() + ", " +
         step.at("zone").get<std::string>() + ")";
}
}  // namespace

class SkillExecutor
{
public:
  SkillExecutor(
    const rclcpp::Node::SharedPtr & node, const std::shared_ptr<RobotSkills> & robot_skills)
  : node_(node), robot_skills_(robot_skills)
  {
    const auto plan_topic = parameter("plan_topic", std::string("/validated_plan"));
    const auto status_topic = parameter("status_topic", std::string("/task_status"));
    inter_object_pause_seconds_ = parameter("inter_object_pause_seconds", 1.0);
    if (inter_object_pause_seconds_ < 0.0) {
      throw std::runtime_error("inter_object_pause_seconds must be non-negative");
    }
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(status_topic, 10);
    plan_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      plan_topic, 10,
      [this](const std_msgs::msg::String::SharedPtr message) {executePlan(message->data);});
    RCLCPP_INFO(node_->get_logger(), "Skill executor ready on %s.", plan_topic.c_str());
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    if (node_->has_parameter(name)) {
      return node_->get_parameter(name).get_value<T>();
    }
    return node_->declare_parameter<T>(name, default_value);
  }

  void publishStatus(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_publisher_->publish(message);
  }

  void executePlan(const std::string & encoded_plan)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(node_->get_logger(), "EXECUTION REJECTED: executor busy.");
      publishStatus("EXECUTION_REJECTED: executor busy");
      return;
    }

    try {
      auto plan = nlohmann::json::parse(encoded_plan);
      const auto validation_error = validatePlan(plan);
      if (!validation_error.empty()) {
        RCLCPP_ERROR(
          node_->get_logger(), "EXECUTION REJECTED: %s", validation_error.c_str());
        publishStatus("EXECUTION_REJECTED: " + validation_error);
        busy_ = false;
        return;
      }

      if (!robot_skills_->validateEnvironmentPlan(plan)) {
        publishStatus("EXECUTION_REJECTED: camera scene or destination preconditions invalid");
        busy_ = false;
        return;
      }
      // Preflight may replace a temporary slot with another observed, reachable slot.
      RCLCPP_DEBUG(node_->get_logger(), "EXECUTION PLAN: %s", plan.dump().c_str());
      publishStatus("EXECUTION PLAN: " + plan.dump());
      std::size_t index = 0U;
      for (auto & step : plan.at("plan")) {
        const auto skill = step.at("skill").get<std::string>();
        SkillStatus result = SkillStatus::FAILED;
        if (skill == "home") {
          result = robot_skills_->home();
        } else if (skill == "move_above") {
          result = robot_skills_->moveAbove(step.at("object").get<std::string>());
        } else if (skill == "open_gripper") {
          result = robot_skills_->openGripper();
        } else if (skill == "close_gripper") {
          result = robot_skills_->closeGripper();
        } else if (skill == "move_to_zone") {
          result = robot_skills_->moveToZone(step.at("zone").get<std::string>());
        } else if (skill == "pick") {
          auto & next = plan.at("plan").at(index + 1U);
          const auto planned_destination = next.at("zone").get<std::string>();
          auto destination = next.at("zone").get<std::string>();
          if (robot_skills_->checkTransfer(
              step.at("object").get<std::string>(), destination))
          {
            next["zone"] = destination;
            if (destination != planned_destination) {
              const auto update = "EXECUTION PLAN UPDATE: temporary slot " +
                planned_destination + " unavailable; use " + destination;
              RCLCPP_WARN(node_->get_logger(), "%s", update.c_str());
              publishStatus(update);
            }
            result = robot_skills_->pick(step.at("object").get<std::string>());
          } else {
            result = SkillStatus::INVALID_STATE;
          }
        } else if (skill == "place") {
          result = robot_skills_->place(
            step.at("object").get<std::string>(), step.at("zone").get<std::string>());
        }

        const auto step_label = formatStep(step);
        RCLCPP_INFO(
          node_->get_logger(), "EXECUTION: %s ........ %s", step_label.c_str(),
          toString(result));
        publishStatus("EXECUTION: " + step_label + " ........ " + toString(result));
        if (result != SkillStatus::SUCCESS) {
          RCLCPP_ERROR(node_->get_logger(), "TASK FAILED: %s", toString(result));
          publishStatus(std::string("TASK_FAILED: ") + toString(result));
          busy_ = false;
          return;
        }
        const bool pause_before_next_pick =
          skill == "place" && inter_object_pause_seconds_ > 0.0 &&
          index + 1U < plan.at("plan").size() &&
          plan.at("plan").at(index + 1U).at("skill").get<std::string>() == "pick";
        if (pause_before_next_pick) {
          RCLCPP_DEBUG(
            node_->get_logger(), "Pausing %.1f seconds before the next object.",
            inter_object_pause_seconds_);
          publishStatus("EXECUTION: pause " +
            std::to_string(inter_object_pause_seconds_) + " seconds before next pick");
          std::this_thread::sleep_for(
            std::chrono::duration<double>(inter_object_pause_seconds_));
        }
        ++index;
      }
      if (!robot_skills_->verifyFinalPlan(plan)) {
        publishStatus("TASK_FAILED: final camera observation does not confirm the plan");
        busy_ = false;
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "TASK SUCCESS");
      publishStatus("TASK SUCCESS");
    } catch (const nlohmann::json::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "EXECUTION REJECTED: invalid JSON: %s", error.what());
      publishStatus("EXECUTION_REJECTED: invalid JSON");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "TASK FAILED: %s", error.what());
      publishStatus(std::string("TASK_FAILED: ") + error.what());
    }
    busy_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<RobotSkills> robot_skills_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  double inter_object_pause_seconds_{1.0};
  std::atomic_bool busy_{false};
};
}  // namespace ur3_llm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("skill_executor", options);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    auto robot_skills = std::make_shared<ur3_llm_control::RobotSkills>(node);
    auto skill_executor = std::make_shared<ur3_llm_control::SkillExecutor>(node, robot_skills);
    spinner.join();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Could not start skill executor: %s", error.what());
    exit_code = 1;
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
  }
  rclcpp::shutdown();
  return exit_code;
}
