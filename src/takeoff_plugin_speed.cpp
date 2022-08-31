/*!*******************************************************************************************
 *  \file       takeoff_plugin_speed.cpp
 *  \brief      Plugin for takeoff with speed control
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
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

#include "motion_reference_handlers/hover_motion.hpp"
#include "motion_reference_handlers/speed_motion.hpp"
#include "takeoff_base.hpp"
#include "mbzirc_msgs/srv/set_pose_stamped.hpp"

namespace takeoff_plugin_speed
{
    class Plugin : public takeoff_base::TakeOffBase
    {
    public:

        // // std::shared_ptr<rclcpp::Client<mbzirc_msgs::srv::SetPoseStamped>> service_client_;
        // std::shared_ptr<as2::SynchronousServiceClient<mbzirc_msgs::srv::SetPoseStamped>> reset_cli_;
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override
        {
            desired_speed_ = goal->takeoff_speed;
            desired_height_ = goal->takeoff_height;

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            odom_received_ = false;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        bool checkGoalCondition()
        {
            if ((desired_height_ - actual_heigth_) <= 0 + this->takeoff_height_threshold_)
            {
                return true;
            }
            return false;
        }

        bool onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            rclcpp::Rate loop_rate(10);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<as2_msgs::action::TakeOff::Feedback>();
            auto result = std::make_shared<as2_msgs::action::TakeOff::Result>();

            static as2::motionReferenceHandlers::SpeedMotion motion_handler_speed(node_ptr_);
            static as2::motionReferenceHandlers::HoverMotion motion_handler_hover(node_ptr_);

            while (!odom_received_)
            {
                if (goal_handle->is_canceling())
                {
                    result->takeoff_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_WARN(node_ptr_->get_logger(), "Goal canceled");
                    motion_handler_hover.sendHover();
                    return false;
                }
                loop_rate.sleep();
            }

            desired_height_ += actual_heigth_;
            
            // Check if goal is done
            while (!checkGoalCondition())
            {
                if (goal_handle->is_canceling())
                {
                    result->takeoff_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_WARN(node_ptr_->get_logger(), "Goal canceled");
                    // TODO: change this to hover
                    motion_handler_speed.sendSpeedCommandWithYawSpeed(0.0, 0.0, 0.0, 0.0);
                    return false;
                }

                motion_handler_speed.sendSpeedCommandWithYawSpeed(0.0, 0.0, desired_speed_, 0.0);

                // FREEZES BEHAVIOUR TREE RESULT
                // feedback->actual_takeoff_height = actual_heigth_;
                // feedback->actual_takeoff_speed = actual_z_speed_;
                // goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            // motion_handler_hover.sendHover();
            motion_handler_speed.sendSpeedCommandWithYawSpeed(0.0, 0.0, 0.0, 0.0);

            rclcpp::Time start_time = node_ptr_->now();
            rclcpp::Time end_time = node_ptr_->now();
            rclcpp::Duration duration = end_time - start_time;
            RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for %f seconds", 4.0f);
            while (duration.seconds() < 4.0f)
            {
                end_time = node_ptr_->now();
                duration = end_time - start_time;
            }

            auto request =  mbzirc_msgs::srv::SetPoseStamped::Request();
            auto response = mbzirc_msgs::srv::SetPoseStamped::Response();
            geometry_msgs::msg::PoseStamped pose_;
            pose_.header.frame_id = "earth";
            pose_.header.stamp = node_ptr_->now();
            
            pose_mutex_.lock();
            request.pose.pose.position.x = actual_position_.x();
            request.pose.pose.position.y = actual_position_.y();
            request.pose.pose.position.z = actual_position_.z();
            pose_mutex_.unlock();
            auto reset_cli_ = as2::SynchronousServiceClient<mbzirc_msgs::srv::SetPoseStamped>(
                "reset");
            bool out = reset_cli_.sendRequest(request, response);

            // motion_handler_hover.sendHover();
            motion_handler_speed.sendSpeedCommandWithYawSpeed(0.0, 0.0, 0.0, 0.0);
            result->takeoff_success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
            return true;
        }

    }; // Plugin class
} // takeoff_plugin_speed namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_speed::Plugin, takeoff_base::TakeOffBase)
