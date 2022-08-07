#ifndef _KORTEX_CARTESIAN_TRAJECTORY_ACTION_SERVER_H_
#define _KORTEX_CARTESIAN_TRAJECTORY_ACTION_SERVER_H_

/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2021 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
#include <mutex>

#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "Errors.pb.h"
#include "kortex_driver/non-generated/kortex_math_util.h"
#include "kortex_driver/action/follow_cartesian_trajectory.hpp"

class CartesianTrajectoryActionServer
{
    public:

        enum ActionServerState
        {
            INITIALIZING = 0,
            IDLE,
            PRE_PROCESSING_PENDING,
            PRE_PROCESSING_IN_PROGRESS,
            TRAJECTORY_EXECUTION_PENDING,
            TRAJECTORY_EXECUTION_IN_PROGRESS,
        };

        CartesianTrajectoryActionServer() = delete;
        CartesianTrajectoryActionServer(const std::string& server_name, rclcpp::Node::SharedPtr nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic);
        ~CartesianTrajectoryActionServer();

        ActionServerState getState() {return m_server_state;};

        const char* actionServerStateNames[int(ActionServerState::TRAJECTORY_EXECUTION_IN_PROGRESS) + 1] =
        {
            "INITIALIZING",
            "IDLE",
            "PRE_PROCESSING_PENDING",
            "PRE_PROCESSING_IN_PROGRESS",
            "TRAJECTORY_EXECUTION_PENDING",
            "TRAJECTORY_EXECUTION_IN_PROGRESS"
        };

        using FollowCartesianTrajectoryAction = kortex_driver::action::FollowCartesianTrajectory;
        using GoalHandle_FollowCartesianTrajectoryAction = rclcpp_action::ServerGoalHandle<FollowCartesianTrajectoryAction>;

    private:
        // Members
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp_action::Server<FollowCartesianTrajectoryAction>::SharedPtr m_server;

        Kinova::Api::Common::NotificationHandle m_sub_action_notif_handle;
        
        uint32_t m_currentActionID;

        Kinova::Api::Base::BaseClient* m_base;

        std::string m_server_name;

        std::chrono::system_clock::time_point m_trajectory_start_time;
        std::chrono::system_clock::time_point m_trajectory_end_time;

        std::mutex m_server_state_lock;
        std::mutex m_action_notification_thread_lock;
        ActionServerState m_server_state;

        // ROS Params
        std::string m_prefix;

        // Action Server Callbacks
        rclcpp_action::GoalResponse ros_goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal);
        rclcpp_action::CancelResponse ros_cancel_callback(const std::shared_ptr<GoalHandle_FollowCartesianTrajectoryAction> goal_handle);
        void ros_accepted_callback(const std::shared_ptr<GoalHandle_FollowCartesianTrajectoryAction> new_goal_handle);

        struct KortexCallback {
            std::shared_ptr<GoalHandle_FollowCartesianTrajectoryAction> goal_handle;
        } kortex_callback;
        
        // Kortex Notifications Callbacks
        void action_notif_callback(Kinova::Api::Base::ActionNotification notif);

        // Private methods
        bool is_goal_acceptable(const std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal);
        bool is_goal_tolerance_respected(bool enable_prints, bool check_time_tolerance);
        void stop_all_movement();

        void set_server_state(ActionServerState s);
};

#endif //_KORTEX_CARTESIAN_TRAJECTORY_ACTION_SERVER_H_