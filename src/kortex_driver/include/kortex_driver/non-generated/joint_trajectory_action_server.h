#ifndef _KORTEX_JOINT_TRAJECTORY_ACTION_SERVER_H_
#define _KORTEX_JOINT_TRAJECTORY_ACTION_SERVER_H_

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

#include "kortex_driver/non-generated/kortex_math_util.h"

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ControlConfigClientRpc.h>
#include <ControlConfig.pb.h>
#include <Errors.pb.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include <google/protobuf/util/message_differencer.h>

#include <chrono>
#include <mutex>

struct AngularTrajectorySoftLimits
{
    Kinova::Api::ControlConfig::JointSpeedSoftLimits m_vel;
    Kinova::Api::ControlConfig::JointAccelerationSoftLimits m_acc;
    AngularTrajectorySoftLimits() = default;
    AngularTrajectorySoftLimits(const Kinova::Api::ControlConfig::JointSpeedSoftLimits& vel, Kinova::Api::ControlConfig::JointAccelerationSoftLimits acc):
        m_vel(vel), m_acc(acc) {}
    inline bool empty() const 
    {
        return google::protobuf::util::MessageDifferencer::Equals(m_vel, Kinova::Api::ControlConfig::JointSpeedSoftLimits::default_instance())
                && google::protobuf::util::MessageDifferencer::Equals(m_acc, Kinova::Api::ControlConfig::JointAccelerationSoftLimits::default_instance());
    }
};

class JointTrajectoryActionServer
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

        JointTrajectoryActionServer() = delete;
        JointTrajectoryActionServer(const std::string& server_name, rclcpp::Node::SharedPtr nh, Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic, Kinova::Api::ControlConfig::ControlConfigClient* control_config, bool use_hard_limits = false);
        ~JointTrajectoryActionServer();

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

        using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;
        using GoalHandle_FollowJointTrajectoryAction = rclcpp_action::ServerGoalHandle<FollowJointTrajectoryAction>;

    private:
        // Members
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp_action::Server<FollowJointTrajectoryAction>::SharedPtr m_server;

        Kinova::Api::Common::NotificationHandle m_sub_action_notif_handle;

        Kinova::Api::Base::BaseClient*                    m_base = nullptr;
        Kinova::Api::BaseCyclic::BaseCyclicClient*        m_base_cyclic = nullptr;
        Kinova::Api::ControlConfig::ControlConfigClient*  m_control_config = nullptr;

        std::string m_server_name;

        FollowJointTrajectoryAction::Feedback m_feedback;
        // actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle m_goal;
        std::chrono::system_clock::time_point m_trajectory_start_time;
        std::chrono::system_clock::time_point m_trajectory_end_time;

        std::mutex m_server_state_lock;
        std::mutex m_action_notification_thread_lock;
        ActionServerState m_server_state;

        // ROS Params
        double m_default_goal_time_tolerance;
        double m_default_goal_tolerance;
        std::vector<std::string> m_joint_names;
        std::string m_prefix;

        // Limits handling
        bool m_use_hard_limits;
        AngularTrajectorySoftLimits m_soft_limits;
        Kinova::Api::ControlConfig::KinematicLimits m_hard_limits;

        // Action Server Callbacks
        rclcpp_action::GoalResponse ros_goal_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal);
        rclcpp_action::CancelResponse ros_cancel_callback(const std::shared_ptr<GoalHandle_FollowJointTrajectoryAction> goal_handle);
        void ros_accepted_callback(const std::shared_ptr<GoalHandle_FollowJointTrajectoryAction> new_goal_handle);

        struct KortexCallback {
            std::shared_ptr<GoalHandle_FollowJointTrajectoryAction> goal_handle;
        } kortex_callback;

        // Kortex Notifications Callbacks
        void action_notif_callback(Kinova::Api::Base::ActionNotification notif);

        // Private methods
        bool is_goal_acceptable(const std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal);
        bool is_goal_tolerance_respected(const std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal, bool enable_prints, bool check_time_tolerance);
        void stop_all_movement();
        
        AngularTrajectorySoftLimits getAngularTrajectorySoftLimits();
        void setAngularTrajectorySoftLimitsToMax();
        void setAngularTrajectorySoftLimits(const AngularTrajectorySoftLimits& limits);

        void set_server_state(ActionServerState s);

};

#endif //_KORTEX_JOINT_TRAJECTORY_ACTION_SERVER_H_
