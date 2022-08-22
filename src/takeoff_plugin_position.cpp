/*!*******************************************************************************************
 *  \file       takeoff_plugin_position.cpp
 *  \brief      Plugin for takeoff with position control
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
#include "motion_reference_handlers/position_motion.hpp"
#include "takeoff_base.hpp"

namespace takeoff_plugin_position
{
    class Plugin : public takeoff_base::TakeOffBase
    {
    public:
        rclcpp_action::GoalResponse onAccepted(const std::shared_ptr<const as2_msgs::action::TakeOff::Goal> goal) override
        {
            desired_speed_ = goal->takeoff_speed;
            RCLCPP_INFO(node_ptr_->get_logger(), "Take off speed: %f", desired_speed_);
            desired_height_ = goal->takeoff_height;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            odom_received_ = false;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        bool onExecute(const std::shared_ptr<GoalHandleTakeoff> goal_handle) override
        {
            rclcpp::Rate loop_rate(10);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<as2_msgs::action::TakeOff::Feedback>();
            auto result = std::make_shared<as2_msgs::action::TakeOff::Result>();

            static as2::motionReferenceHandlers::PositionMotion motion_handler_pose(node_ptr_);
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
                RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for odometry...");
            }

            desired_height_ += actual_heigth_;

            pose_mutex_.lock();
            float desired_pos_x = actual_position_.x();
            float desired_pos_y = actual_position_.y();
            float desired_yaw = as2::FrameUtils::getYawFromQuaternion(actual_q_);
            pose_mutex_.unlock();

            RCLCPP_INFO(node_ptr_->get_logger(), "Desired take off position: %f, %f, %f", desired_pos_x, desired_pos_y, desired_height_);

            // Check if goal is done
            while (!checkGoalCondition())
            {
                if (goal_handle->is_canceling())
                {
                    result->takeoff_success = false;
                    goal_handle->canceled(result);
                    RCLCPP_WARN(node_ptr_->get_logger(), "Goal canceled");
                    motion_handler_hover.sendHover();
                    return false;
                }

                motion_handler_pose.sendPositionCommandWithYawAngle(desired_pos_x, desired_pos_y, desired_height_, desired_yaw, desired_speed_, desired_speed_, desired_speed_);

                feedback->actual_takeoff_height = actual_heigth_;
                feedback->actual_takeoff_speed = actual_z_speed_;
                goal_handle->publish_feedback(feedback);

                loop_rate.sleep();
            }

            result->takeoff_success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
            // TODO: change this to hover?
            motion_handler_pose.sendPositionCommandWithYawAngle(desired_pos_x, desired_pos_y, desired_height_, desired_yaw, desired_speed_, desired_speed_, desired_speed_);
            return true;
        }

    }; // Plugin class
} // takeoff_plugin_position namespace

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_position::Plugin, takeoff_base::TakeOffBase)
