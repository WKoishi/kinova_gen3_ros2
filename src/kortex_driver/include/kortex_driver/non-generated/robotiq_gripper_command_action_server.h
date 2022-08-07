#ifndef _KORTEX_ROBOTIQ_GRIPPER_COMMAND_ACTION_SERVER_H_
#define _KORTEX_ROBOTIQ_GRIPPER_COMMAND_ACTION_SERVER_H_

/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include <chrono>
#include <mutex>
#include <thread>

#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "Errors.pb.h"

#include "kortex_math_util.h"

// Duration timeout for a gripper trajectory (in seconds) 
#define GRIPPER_TRAJECTORY_TIME_LIMIT 2.0

#define MAX_GRIPPER_RELATIVE_ERROR 0.05
#define MAX_CONSECUTIVE_POSITION_DIFFERENCE 0.01
#define MAX_CONSECUTIVE_IDENTICAL_POSITIONS 4

class RobotiqGripperCommandActionServer
{
    public:
        RobotiqGripperCommandActionServer() = delete;
        RobotiqGripperCommandActionServer(const std::string& server_name, const std::string& gripper_joint_name, double gripper_joint_limit_min, double gripper_joint_limit_max, rclcpp::Node::SharedPtr nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic);
        ~RobotiqGripperCommandActionServer();

        using GripperCommandAction = control_msgs::action::GripperCommand;
        using GoalHandle_GripperCommandAction = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
   
    private:
        // Members
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp_action::Server<GripperCommandAction>::SharedPtr m_server;

        Kinova::Api::Common::NotificationHandle m_sub_action_notif_handle;

        Kinova::Api::Base::GripperCommand           m_cancel_gripper_command;

        Kinova::Api::Base::BaseClient*              m_base;
        Kinova::Api::BaseCyclic::BaseCyclicClient*  m_base_cyclic;

        std::string m_server_name;

        GripperCommandAction::Feedback    m_feedback;
        // actionlib::ActionServer<control_msgs::GripperCommandAction>::GoalHandle m_goal;
        std::chrono::system_clock::time_point m_trajectory_start_time;
        
        bool m_is_trajectory_running;
        std::mutex m_is_trajectory_running_lock;
        std::thread m_gripper_position_polling_thread;

        KortexMathUtil m_math_util;

        // ROS Params
        std::string m_gripper_joint_name;
        double      m_gripper_joint_limit_min;
        double      m_gripper_joint_limit_max;

        // Action Server Callbacks
        rclcpp_action::GoalResponse ros_goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const GripperCommandAction::Goal> goal);
        rclcpp_action::CancelResponse ros_cancel_callback(const std::shared_ptr<GoalHandle_GripperCommandAction> goal_handle);
        void ros_accepted_callback(const std::shared_ptr<GoalHandle_GripperCommandAction> new_goal_handle);

        // Polling thread function
        void gripper_position_polling_thread(const std::shared_ptr<GoalHandle_GripperCommandAction> goal_handle);

        // Private methods
        bool is_goal_acceptable(const std::shared_ptr<const GripperCommandAction::Goal> goal);
        bool is_goal_tolerance_respected(const std::shared_ptr<const GripperCommandAction::Goal> goal);
        void stop_all_movement();
        void join_polling_thread();
};

#endif