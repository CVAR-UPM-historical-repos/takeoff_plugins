/*!*******************************************************************************************
 *  \file       takeoff_trajectory.cpp
 *  \brief      This file contains the implementation of the take off behaviour trajectory plugin
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>
#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/actions.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_msgs/action/trajectory_generator.hpp"
#include "as2_msgs/msg/trajectory_waypoints_with_id.hpp"
#include "motion_reference_handlers/position_motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "takeoff_base.hpp"

namespace takeoff_plugin_trajectory {

class Plugin : public takeoff_base::TakeOffBase {
  using TrajectoryGeneratorAction     = as2_msgs::action::TrajectoryGenerator;
  using GoalHandleTrajectoryGenerator = rclcpp_action::ClientGoalHandle<TrajectoryGeneratorAction>;

public:
  void ownInit() {
    position_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);

    trajectory_generator_client_ = rclcpp_action::create_client<TrajectoryGeneratorAction>(
        node_ptr_, as2_names::actions::behaviours::trajectorygenerator);

    send_goal_options_.goal_response_callback =
        std::bind(&Plugin::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
        std::bind(&Plugin::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
        std::bind(&Plugin::result_callback, this, std::placeholders::_1);
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff canceled, set to hover");
    sendHover();
    return true;
  }

  bool on_pause(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be paused");
    return false;
  }

  bool on_resume(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be resumed");
    return false;
  }

  bool own_activate(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override {
    if (!trajectory_generator_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Trajectory generator action server not available");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generator action server available");

    as2_msgs::msg::TrajectoryWaypointsWithID takeoff_goal = getActionGoal(*goal);

    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff to position: %f, %f, %f",
                takeoff_goal.poses[0].pose.position.x, takeoff_goal.poses[0].pose.position.y,
                takeoff_goal.poses[0].pose.position.z);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with angle: %f", takeoff_goal.yaw.angle);
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff with speed: %f", takeoff_goal.max_speed);

    as2_msgs::action::TrajectoryGenerator::Goal goal_msg = TrajectoryGeneratorAction::Goal();
    goal_msg.trajectory_waypoints                        = takeoff_goal;

    goal_handle_future_ =
        trajectory_generator_client_->async_send_goal(goal_msg, send_goal_options_);

    if (!goal_handle_future_.valid()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
      return false;
    }
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff accepted");
    return true;
  }

  bool own_modify(std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be modified");
    return false;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (goal_handle_future_.valid() &&
        goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto result = goal_handle_future_.get();
      if (result) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff result finished");
        result_.takeoff_success = true;
        return as2_behavior::ExecutionStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Takeoff result failed");
        result_.takeoff_success = false;
        return as2_behavior::ExecutionStatus::FAILURE;
      }
    } else {
      RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for trajectory generator result");
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff end");
    return;
  }

  void goal_response_callback(const GoalHandleTrajectoryGenerator::SharedPtr &goal_handle) {
    goal_received_ = true;
    if (!goal_handle.get()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Goal was rejected by server");
      return;
    }
    goal_accepted_ = true;
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal accepted by server, waiting for result");
    return;
  };

  void feedback_callback(
      GoalHandleTrajectoryGenerator::SharedPtr,
      const std::shared_ptr<const TrajectoryGeneratorAction::Feedback> feedback) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Feedback received");
    action_feedback_ = *feedback;
    return;
  };

  void result_callback(const GoalHandleTrajectoryGenerator::WrappedResult &result) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Result received");
    action_result_ = result;
    return;
  };

private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;
  std::shared_ptr<rclcpp_action::Client<TrajectoryGeneratorAction>> trajectory_generator_client_ =
      nullptr;
  rclcpp_action::Client<TrajectoryGeneratorAction>::SendGoalOptions send_goal_options_;

  // future
  std::shared_future<GoalHandleTrajectoryGenerator::SharedPtr> goal_handle_future_;

  TrajectoryGeneratorAction::Feedback action_feedback_;
  GoalHandleTrajectoryGenerator::WrappedResult action_result_;

  bool goal_accepted_   = false;
  bool goal_received_   = false;
  bool result_received_ = false;

private:
  as2_msgs::msg::TrajectoryWaypointsWithID getActionGoal(
      const as2_msgs::action::TakeOff::Goal &_goal) {
    as2_msgs::msg::TrajectoryWaypointsWithID trajectory;

    std_msgs::msg::Header header;
    header.frame_id = as2::tf::generateTfName(node_ptr_, "odom");
    header.stamp    = node_ptr_->now();

    as2_msgs::msg::YawMode yaw;
    yaw.mode = as2_msgs::msg::YawMode::FIXED_YAW;
    // yaw.angle = as2::frame::getYawFromQuaternion(actual_pose_);

    as2_msgs::msg::PoseStampedWithID pose;
    pose.header          = header;
    pose.pose.position.x = actual_pose_.pose.position.x;
    pose.pose.position.y = actual_pose_.pose.position.y;
    pose.pose.position.z = actual_pose_.pose.position.z + _goal.takeoff_height;

    trajectory.header    = header;
    trajectory.yaw       = yaw;
    trajectory.max_speed = _goal.takeoff_speed;
    trajectory.poses.push_back(pose);

    pose.pose.position.z += _goal.takeoff_height;
    trajectory.poses.push_back(pose);

    return trajectory;
  }

  bool checkGoalCondition() {
    if (localization_received_) {
      if (fabs(goal_.takeoff_height - feedback_.actual_takeoff_height) <
          params_.takeoff_height_threshold)
        return true;
    }
    return false;
  }

};  // Plugin class
}  // namespace takeoff_plugin_trajectory

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_trajectory::Plugin, takeoff_base::TakeOffBase)
