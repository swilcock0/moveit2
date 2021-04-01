#include <streambuf>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/pose_stamped.h>

namespace ph = std::placeholders;
using namespace std::chrono_literals;

auto SUCCESS = moveit::planning_interface::MoveItErrorCode::SUCCESS;

// Parameters
const std::string PLANNING_GROUP = "panda_arm";

const std::string node_name = "move_group_interface_tutorial";

static const rclcpp::Logger logger_ = rclcpp::get_logger(node_name);

class moveServer : public rclcpp::Node
{
public:
  moveServer()
    : Node(node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    , move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP)
    , planning_scene_interface_()
  {
    brick_num = 0;
    attached = false;

    // Config moveit params
    this->move_group_.setPlanningTime(5.0);
    this->move_group_.setMaxVelocityScalingFactor(1.0);
    this->move_group_.setMaxAccelerationScalingFactor(1.0);
    this->move_group_.setNumPlanningAttempts(2);
    this->move_group_.allowReplanning(true);

    // Configure callback groups for multithreading
    callback_group_movement_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_movement_;

    subscriber_move_with_brick = this->create_subscription<geometry_msgs::msg::Pose>(
        "move_with_brick", 10, std::bind(&moveServer::callbackMoveWithBrick, this, ph::_1), sub_opt);
  }

  /// Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;

  /// Planning scene interface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

private:
  bool pick()
  {
    // pose_target currently unused, hardcoded the brick in
    RCLCPP_INFO(logger_, "            PICKING");
    RCLCPP_INFO(logger_, "-----------------------------");
    geometry_msgs::msg::Pose brick_pose;

    brick_pose.position.x = 0;
    brick_pose.position.y = 0.4;
    brick_pose.position.z = 0 + 0.065 / 2;
    brick_pose.orientation.x = -1;
    brick_pose.orientation.y = 0.0;
    brick_pose.orientation.z = 0.0;
    brick_pose.orientation.w = 0.0;

    this->addBrick(brick_pose.position.x, brick_pose.position.y, brick_pose.position.z, brick_pose.orientation.x,
                   brick_pose.orientation.y, brick_pose.orientation.z, brick_pose.orientation.w);
    geometry_msgs::msg::Pose pre_grasp_pose;
    geometry_msgs::msg::Pose grasp_pose;
    geometry_msgs::msg::Pose post_grasp_pose;
    grasp_pose = brick_pose;
    grasp_pose.position.z += 0.065 / 2;

    pre_grasp_pose = grasp_pose;
    pre_grasp_pose.position.z += 0.25;
    post_grasp_pose = pre_grasp_pose;

    bool success;
    double fraction = 1.0;

    success = this->moveToPose(pre_grasp_pose);

    success = this->moveToPose(grasp_pose);
    if (success)
    {
      if (this->move_group_.attachObject("brick" + std::to_string(this->brick_num - 1)))
      {
        attached = true;
      }
      success = this->moveToPose(post_grasp_pose);
    }

    return success;
  }

  bool place(geometry_msgs::msg::Pose pose_target)
  {
    RCLCPP_INFO(logger_, "            PLACING");
    RCLCPP_INFO(logger_, "-----------------------------");
    geometry_msgs::msg::Pose pre_place_pose;
    geometry_msgs::msg::Pose place_pose = pose_target;
    geometry_msgs::msg::Pose post_place_pose;

    place_pose.position.z += 0.065;

    pre_place_pose = place_pose;
    pre_place_pose.position.z += 0.1;
    post_place_pose = pre_place_pose;

    bool success = false;
    double fraction = 1.0;

    success = this->moveToPose(pre_place_pose);

    success = this->moveToPose(place_pose);
    if (success)
    {
      if (this->move_group_.detachObject("brick" + std::to_string(this->brick_num - 1)))
      {
        attached = false;
      }
      fraction = this->moveToPose(post_place_pose);
    }

    return success;
  }

  void addBrick(double x, double y, double z, double x_o, double y_o,
                double z_o, double w)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = this->move_group_.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "brick" + std::to_string(this->brick_num);
    this->brick_num = this->brick_num + 1;

    // Define a box to add to the world.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.215;
    primitive.dimensions[1] = 0.103;
    primitive.dimensions[2] = 0.065;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::msg::Pose brick_pose;

    brick_pose.position.x = x;
    brick_pose.position.y = y;
    brick_pose.position.z = z;
    brick_pose.orientation.x = x_o;
    brick_pose.orientation.y = y_o;
    brick_pose.orientation.z = z_o;
    brick_pose.orientation.w = w;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(brick_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    brick_id_vector.push_back(collision_object.id);

    // Now, let's add the collision object into the world
    RCLCPP_INFO(logger_, "Adding brick #%u into the world at %f %f %f {%f %f %f %f}", this->brick_num - 1, x, y, z, x_o, y_o, z_o, w);
    planning_scene_interface_.applyCollisionObjects(collision_objects);
  }

  void callbackMoveWithBrick(const geometry_msgs::msg::Pose::SharedPtr pose_ptr)
  {
    // Pick up a brick from a set location, then move it to the requested position
    auto pose_target = *pose_ptr;
    if (attached == false)
    {
      this->pick();
    }

    if (attached == true)
    {
      this->place(pose_target);
    }
  }

  bool moveToPose(geometry_msgs::msg::Pose pose_target)
  {
    // clearPathConstraints();
    this->move_group_.clearPoseTarget();
    this->move_group_.setStartState(*this->move_group_.getCurrentState(5));

    RCLCPP_INFO(logger_, "Targeting pose");
    this->move_group_.setPoseTarget(pose_target);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (this->move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success){
      this->move_group_.execute(my_plan);
    }

    RCLCPP_INFO(logger_, "Pose goal %s ", success ? "SUCCEEDED" : "FAILED");

    return success;
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_move_with_brick;

  std::vector<std::string> brick_id_vector;

  rclcpp::CallbackGroup::SharedPtr callback_group_movement_;

  bool attached;
  int brick_num;
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto move_group_class = std::make_shared<moveServer>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(move_group_class);

  RCLCPP_INFO(logger_, "Motion server has been started.");

  exec.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}