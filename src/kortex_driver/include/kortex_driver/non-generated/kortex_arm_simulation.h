#ifndef _KORTEX_ARM_SIMULATION_H_
#define _KORTEX_ARM_SIMULATION_H_

/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2020 Kinova inc. All rights reserved.
*
* This software may be modified and distributed under the
* terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

// ROS
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/Empty.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/action_client.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/velocityprofile_trap.hpp>

// Standard
#include <unordered_map>
#include <thread>
#include <mutex>
#include <atomic>

// Kortex
#include "kortex_driver/non-generated/kortex_math_util.h"

#include "kortex_driver/msg/action_type.hpp"
#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/msg/base_cyclic_feedback.hpp"

#include "kortex_driver/srv/create_action.hpp"
#include "kortex_driver/srv/read_action.hpp"
#include "kortex_driver/srv/read_all_actions.hpp"
#include "kortex_driver/srv/delete_action.hpp"
#include "kortex_driver/srv/update_action.hpp"
#include "kortex_driver/srv/execute_action_from_reference.hpp"
#include "kortex_driver/srv/execute_action.hpp"
#include "kortex_driver/srv/pause_action.hpp"
#include "kortex_driver/srv/stop_action.hpp"
#include "kortex_driver/srv/resume_action.hpp"

#include "kortex_driver/srv/play_cartesian_trajectory.hpp"
#include "kortex_driver/srv/stop.hpp"
#include "kortex_driver/srv/get_measured_cartesian_pose.hpp"
#include "kortex_driver/srv/send_twist_command.hpp"
#include "kortex_driver/srv/play_joint_trajectory.hpp"
#include "kortex_driver/srv/send_joint_speeds_command.hpp"
#include "kortex_driver/srv/send_gripper_command.hpp"
#include "kortex_driver/srv/apply_emergency_stop.hpp"

enum class ControllerType
{
  kTrajectory, // this is for the JointTrajectoryController
  kIndividual  // this is for the individual JointPositionController's
};

class KortexArmSimulation
{
  public:
    KortexArmSimulation() = delete;
    KortexArmSimulation(ros::NodeHandle& nh);
    ~KortexArmSimulation();
    std::unordered_map<uint32_t, kortex_driver::Action> GetActionsMap() const {return m_map_actions;}
    int GetDOF() const {return m_degrees_of_freedom;}

    kortex_driver::msg::BaseCyclicFeedback GetFeedback();

    // Handlers for simulated Kortex API functions
    // Actions API
    kortex_driver::srv::CreateAction::Response CreateAction(const kortex_driver::srv::CreateAction::Request& req);
    kortex_driver::srv::ReadAction::Response ReadAction(const kortex_driver::srv::ReadAction::Request& req);
    kortex_driver::srv::ReadAllActions::Response ReadAllActions(const kortex_driver::srv::ReadAllActions::Request& req);
    kortex_driver::srv::DeleteAction::Response DeleteAction(const kortex_driver::srv::DeleteAction::Request& req);
    kortex_driver::srv::UpdateAction::Response UpdateAction(const kortex_driver::srv::UpdateAction::Request& req);
    kortex_driver::srv::ExecuteActionFromReference::Response ExecuteActionFromReference(const kortex_driver::srv::ExecuteActionFromReference::Request& req);
    kortex_driver::srv::ExecuteAction::Response ExecuteAction(const kortex_driver::srv::ExecuteAction::Request& req);
    kortex_driver::srv::StopAction::Response StopAction(const kortex_driver::srv::StopAction::Request& req);
    // Other RPCs
    kortex_driver::srv::PlayCartesianTrajectory::Response PlayCartesianTrajectory(const kortex_driver::srv::PlayCartesianTrajectory::Request& req);
    kortex_driver::srv::SendTwistCommand::Response SendTwistCommand(const kortex_driver::srv::SendTwistCommand::Request& req);
    kortex_driver::srv::PlayJointTrajectory::Response PlayJointTrajectory(const kortex_driver::srv::PlayJointTrajectory::Request& req);
    kortex_driver::srv::SendJointSpeedsCommand::Response SendJointSpeedsCommand(const kortex_driver::srv::SendJointSpeedsCommand::Request& req);
    kortex_driver::srv::SendGripperCommand::Response SendGripperCommand(const kortex_driver::srv::SendGripperCommand::Request& req);
    kortex_driver::srv::Stop::Response Stop(const kortex_driver::srv::Stop::Request& req);
    kortex_driver::srv::ApplyEmergencyStop::Response ApplyEmergencyStop(const kortex_driver::srv::ApplyEmergencyStop::Request& req);

  private:
    // ROS
    ros::NodeHandle m_node_handle;

    // Publishers
    ros::Publisher m_pub_action_topic;
    std::vector<ros::Publisher> m_pub_position_controllers;

    // Subscribers
    ros::Subscriber m_sub_joint_trajectory_controller_state;
    ros::Subscriber m_sub_joint_state;
    ros::Subscriber m_sub_joint_speeds;
    ros::Subscriber m_sub_twist;
    ros::Subscriber m_sub_clear_faults;
    ros::Subscriber m_sub_stop;
    ros::Subscriber m_sub_emergency_stop;

    // Service clients
    ros::ServiceClient m_client_switch_controllers;
    ControllerType m_active_controller_type;
    std::vector<std::string> m_trajectory_controllers_list;
    std::vector<std::string> m_position_controllers_list;
    std::vector<double> m_velocity_commands;
    kortex_driver::msg::Twist m_twist_command;

    // Action clients
    std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> m_follow_joint_trajectory_action_client;
    std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> m_gripper_action_client;

    // Namespacing and prefixing information
    std::string m_prefix;
    std::string m_robot_name;

    // Arm information
    std::string m_arm_name;
    std::vector<std::string> m_arm_joint_names;
    std::vector<double> m_arm_joint_limits_min;
    std::vector<double> m_arm_joint_limits_max;
    std::vector<float> m_arm_velocity_max_limits;
    std::vector<float> m_arm_acceleration_max_limits;
    float m_max_cartesian_twist_linear;
    float m_max_cartesian_twist_angular;
    float m_max_cartesian_acceleration_linear;
    float m_max_cartesian_acceleration_angular;

    // Gripper information
    std::string m_gripper_name;
    std::vector<std::string> m_gripper_joint_names;
    std::vector<float> m_gripper_joint_limits_max;
    std::vector<float> m_gripper_joint_limits_min;
    int m_degrees_of_freedom;

    // The indexes of the first arm joint and of the gripper joint in the "joint_states" feedback
    int m_first_arm_joint_index;
    int m_gripper_joint_index;

    // Action-related
    std::unordered_map<uint32_t, kortex_driver::Action> m_map_actions;

    // Math utility
    KortexMathUtil m_math_util;

    // KDL chain, solvers and motions
    KDL::Chain m_chain;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> m_fk_solver;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> m_ik_pos_solver;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> m_ik_vel_solver;
    std::vector<KDL::VelocityProfile_Trap> m_velocity_trap_profiles;

    // Threading
    std::atomic<bool> m_is_action_being_executed;
    std::atomic<bool> m_action_preempted;
    std::atomic<int> m_current_action_type;
    std::thread m_action_executor_thread;

    // MoveIt-related
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_moveit_arm_interface;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> m_moveit_gripper_interface;

    // Subscription callbacks and data structures with their mutexes
    void cb_joint_states(const sensor_msgs::JointState& state);
    sensor_msgs::JointState m_current_state;
    bool m_first_state_received;
    kortex_driver::msg::BaseCyclicFeedback m_feedback;
    std::mutex m_state_mutex;

    // Helper functions
    bool IsGripperPresent() const {return !m_gripper_name.empty();}
    void CreateDefaultActions();
    bool SwitchControllerType(ControllerType new_type);
    void PlayAction(const kortex_driver::Action& action);
    void JoinThreadAndCancelAction();
    kortex_driver::msg::KortexError FillKortexError(uint32_t code, uint32_t subCode, const std::string& description = std::string()) const;
    kortex_driver::msg::KortexError ExecuteReachJointAngles(const kortex_driver::Action& action);
    kortex_driver::msg::KortexError ExecuteReachPose(const kortex_driver::Action& action);
    kortex_driver::msg::KortexError ExecuteSendJointSpeeds(const kortex_driver::Action& action);
    kortex_driver::msg::KortexError ExecuteSendTwist(const kortex_driver::Action& action);
    kortex_driver::msg::KortexError ExecuteSendGripperCommand(const kortex_driver::Action& action);
    kortex_driver::msg::KortexError ExecuteTimeDelay(const kortex_driver::Action& action);

    // Callbacks
    void new_joint_speeds_cb(const kortex_driver::Base_JointSpeeds& joint_speeds);
    void new_twist_cb(const kortex_driver::TwistCommand& twist);
    void clear_faults_cb(const std_msgs::Empty& empty);
    void stop_cb(const std_msgs::Empty& empty);
    void emergency_stop_cb(const std_msgs::Empty& empty);
};

#endif //_KORTEX_ARM_SIMULATION_H_
