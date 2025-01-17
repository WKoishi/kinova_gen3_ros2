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

#include "kortex_driver/non-generated/kortex_arm_simulation.h"
#include "kortex_driver/msg/error_codes.hpp"
#include "kortex_driver/msg/sub_error_codes.hpp"
#include "kortex_driver/msg/action_notification.hpp"
#include "kortex_driver/msg/action_event.hpp"
#include "kortex_driver/msg/joint_trajectory_constraint_type.hpp"
#include "kortex_driver/msg/gripper_mode.hpp"
#include "kortex_driver/msg/cartesian_reference_frame.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_msgs/msg/float64.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>

#include <urdf/model.h>

#include <set>
#include <chrono>
#include <unordered_set>

namespace 
{
    static const std::string ARM_PLANNING_GROUP = "arm";
    static const std::string GRIPPER_PLANNING_GROUP = "gripper";
    static constexpr unsigned int FIRST_CREATED_ACTION_ID = 10000;
    static const std::set<unsigned int> DEFAULT_ACTIONS_IDENTIFIERS{1,2,3};
    static constexpr double JOINT_TRAJECTORY_TIMESTEP_SECONDS = 0.01;
    static constexpr double MINIMUM_JOINT_VELOCITY_RAD_PER_SECONDS = 0.001;
}

KortexArmSimulation::KortexArmSimulation(rclcpp::Node::SharedPtr node_handle): m_node_handle(node_handle),
                                                                        m_map_actions{},
                                                                        m_is_action_being_executed{false},
                                                                        m_action_preempted{false},
                                                                        m_current_action_type{0},
                                                                        m_first_state_received{false},
                                                                        m_active_controller_type{ControllerType::kTrajectory}
{
    // Namespacing and prefixing information
    m_node_handle->get_parameter("~robot_name", m_robot_name);
    m_node_handle->get_parameter("~prefix", m_prefix);

    // Arm information
    urdf::Model model;
    model.initFile("/" + m_robot_name + "/robot_description");  // TODO: not sure this could work
    m_node_handle->get_parameter("~dof", m_degrees_of_freedom);
    m_node_handle->get_parameter("~arm", m_arm_name);
    m_node_handle->get_parameter("~joint_names", m_arm_joint_names);
    m_node_handle->get_parameter("~maximum_velocities", m_arm_velocity_max_limits);
    m_node_handle->get_parameter("~maximum_accelerations", m_arm_acceleration_max_limits);
    m_arm_joint_limits_min.resize(GetDOF());
    m_arm_joint_limits_max.resize(GetDOF());
    for (int i = 0; i < GetDOF(); i++)
    {
        auto joint = model.getJoint(m_arm_joint_names[i]);
        m_arm_joint_limits_min[i] = joint->limits->lower;
        m_arm_joint_limits_max[i] = joint->limits->upper;
    }

    // Cartesian Twist limits
    m_node_handle->get_parameter("~maximum_linear_velocity", m_max_cartesian_twist_linear);
    m_node_handle->get_parameter("~maximum_angular_velocity", m_max_cartesian_twist_angular);
    m_node_handle->get_parameter("~maximum_linear_acceleration", m_max_cartesian_acceleration_linear);
    m_node_handle->get_parameter("~maximum_angular_acceleration", m_max_cartesian_acceleration_angular);
    
    // Gripper information
    m_node_handle->get_parameter("~gripper", m_gripper_name);
    if (IsGripperPresent())
    {
        m_node_handle->get_parameter("~gripper_joint_names", m_gripper_joint_names);

        // The joint_states feedback uses alphabetical order for the indexes
        // If the gripper joint is before
        if (m_gripper_joint_names[0] < m_arm_joint_names[0])
        {
            m_gripper_joint_index = 0;
            m_first_arm_joint_index = 1;
        }
        // If the gripper joint is after the arm joints
        else
        {
            m_gripper_joint_index = GetDOF();
            m_first_arm_joint_index = 0;
        }

        for (auto s : m_gripper_joint_names)
        {
            s.insert(0, m_prefix);
        }
        m_node_handle->get_parameter("~gripper_joint_limits_max", m_gripper_joint_limits_max);
        m_node_handle->get_parameter("~gripper_joint_limits_min", m_gripper_joint_limits_min);
    }

    // Print out simulation configuration
    RCLCPP_INFO(m_node_handle->get_logger(), "Simulating arm with following characteristics:");
    RCLCPP_INFO(m_node_handle->get_logger(), "Arm type : %s", m_arm_name.c_str());
    RCLCPP_INFO(m_node_handle->get_logger(), "Gripper type : %s", m_gripper_name.empty() ? "None" : m_gripper_name.c_str());
    RCLCPP_INFO(m_node_handle->get_logger(), "Arm namespace : %s", m_robot_name.c_str());
    RCLCPP_INFO(m_node_handle->get_logger(), "URDF prefix : %s", m_prefix.c_str());

    // Building the KDL chain from the robot description
    // The chain goes from 'base_link' to 'tool_frame'
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree))  // TODO: not sure this could work
    {
        const std::string error_string("Failed to parse robot_description parameter to build the kinematic tree!"); 
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", error_string.c_str());
        throw(std::runtime_error(error_string));
    }
    if (!tree.getChain(m_prefix + "base_link", m_prefix + "tool_frame", m_chain))
    {
        const std::string error_string("Failed to extract kinematic chain from parsed tree!"); 
        RCLCPP_ERROR(m_node_handle->get_logger(), "%s", error_string.c_str());
        throw(std::runtime_error(error_string));
    }
    m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
    m_ik_vel_solver.reset(new KDL::ChainIkSolverVel_pinv(m_chain));
    m_ik_pos_solver.reset(new KDL::ChainIkSolverPos_NR(m_chain, *m_fk_solver, *m_ik_vel_solver));

    // Build the velocity profile for each joint using the max velocities and max accelerations
    for (int i = 0; i < GetDOF(); i++)
    {
        m_velocity_trap_profiles.push_back(KDL::VelocityProfile_Trap(m_arm_velocity_max_limits[i], m_arm_acceleration_max_limits[i]));
    }

    // Start MoveIt client
    m_moveit_arm_interface.reset(new moveit::planning_interface::MoveGroupInterface(m_node_handle, ARM_PLANNING_GROUP));
    if (IsGripperPresent())
    {
        m_moveit_gripper_interface.reset(new moveit::planning_interface::MoveGroupInterface(m_node_handle, GRIPPER_PLANNING_GROUP));   
    }

    // Create default actions
    CreateDefaultActions();

    // Create publishers and subscribers
    for (int i = 0; i < GetDOF(); i++)
    {
        m_pub_position_controllers.push_back(m_node_handle->create_publisher<std_msgs::msg::Float64>(
            "/" + m_robot_name + "/" + m_prefix + "joint_" + std::to_string(i+1) + "_position_controller/command", 1000));
    }
    m_pub_action_topic = m_node_handle->create_publisher<kortex_driver::msg::ActionNotification>("action_topic", 1000);
    m_sub_joint_state = m_node_handle->create_subscription<sensor_msgs::msg::JointState>("/" + m_robot_name + "/" + "joint_states", 10, std::bind(&KortexArmSimulation::cb_joint_states, this, std::placeholders::_1));
    m_feedback.actuators.resize(GetDOF());
    m_feedback.interconnect.oneof_tool_feedback.gripper_feedback.resize(1);
    m_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor.resize(1);

    m_sub_joint_speeds = m_node_handle->create_subscription<kortex_driver::msg::BaseJointSpeeds>("in/joint_velocity", 10, std::bind(&KortexArmSimulation::new_joint_speeds_cb, this, std::placeholders::_1));
    m_sub_twist = m_node_handle->create_subscription<kortex_driver::msg::TwistCommand>("in/cartesian_velocity", 10, std::bind(&KortexArmSimulation::new_twist_cb, this, std::placeholders::_1));
    m_sub_clear_faults = m_node_handle->create_subscription<std_msgs::msg::Empty>("in/clear_faults", 10, std::bind(&KortexArmSimulation::clear_faults_cb, this, std::placeholders::_1));
    m_sub_stop = m_node_handle->create_subscription<std_msgs::msg::Empty>("in/stop", 10, std::bind(&KortexArmSimulation::stop_cb, this, std::placeholders::_1));
    m_sub_emergency_stop = m_node_handle->create_subscription<std_msgs::msg::Empty>("in/emergency_stop", 10, std::bind(&KortexArmSimulation::emergency_stop_cb, this, std::placeholders::_1));

    // Create service clients
    m_client_switch_controllers = m_node_handle->create_client<controller_manager_msgs::srv::SwitchController>
        ("/" + m_robot_name + "/controller_manager/switch_controller");
    
    // Fill controllers'names
    m_trajectory_controllers_list.push_back(m_prefix + m_arm_name + "_joint_trajectory_controller"); // only one trajectory controller
    m_position_controllers_list.resize(GetDOF()); // one position controller per 
    std::generate(m_position_controllers_list.begin(), 
                    m_position_controllers_list.end(), 
                    [this]() -> std::string
                    {
                        static int i = 1;
                        return m_prefix + "joint_" + std::to_string(i++) + "_position_controller";
                    });

    // Initialize the velocity commands to null
    m_velocity_commands.resize(GetDOF());
    std::fill(m_velocity_commands.begin(), m_velocity_commands.end(), 0.0);

    // Create and connect action clients
    m_follow_joint_trajectory_action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(m_node_handle,
        "/" + m_robot_name + "/" + m_prefix + m_arm_name + "_joint_trajectory_controller" + "/follow_joint_trajectory");
    m_follow_joint_trajectory_action_client->wait_for_action_server();
    if (IsGripperPresent())
    {
        m_gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(m_node_handle, 
            "/" + m_robot_name + "/" + m_prefix + m_gripper_name + "_gripper_controller" + "/gripper_cmd");
        m_gripper_action_client->wait_for_action_server();
    }

    // Create usual ROS parameters
    m_node_handle->set_parameter(rclcpp::Parameter("degrees_of_freedom", m_degrees_of_freedom));
    m_node_handle->set_parameter(rclcpp::Parameter("is_gripper_present", IsGripperPresent()));
    m_node_handle->set_parameter(rclcpp::Parameter("gripper_joint_names", m_gripper_joint_names));
	m_node_handle->set_parameter(rclcpp::Parameter("has_vision_module", false));
    m_node_handle->set_parameter(rclcpp::Parameter("has_interconnect_module", false));
}

KortexArmSimulation::~KortexArmSimulation()
{
    JoinThreadAndCancelAction();
}

kortex_driver::msg::BaseCyclicFeedback KortexArmSimulation::GetFeedback()
{
    // If the feedback is not yet received, return now
    if (!m_first_state_received)
    {
        return m_feedback;
    }

    // Make a copy of current state
    sensor_msgs::msg::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }

    // Fill joint angles feedback
    for (int i = 0; i < GetDOF(); i++)
    {
        m_feedback.actuators[i].position = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(current.position[m_first_arm_joint_index + i]));
        m_feedback.actuators[i].velocity = m_math_util.toDeg(current.velocity[m_first_arm_joint_index + i]);
    }

    // Calculate FK to get end effector pose
    auto frame = KDL::Frame();
    Eigen::VectorXd positions_eigen(GetDOF());
    for (int i = 0; i < GetDOF(); i++)
    {
        positions_eigen[i] = current.position[m_first_arm_joint_index + i];
    }
    KDL::JntArray current_kdl(GetDOF());
    current_kdl.data = positions_eigen;
    m_fk_solver->JntToCart(current_kdl, frame);
    m_feedback.base.tool_pose_x = frame.p.x();
    m_feedback.base.commanded_tool_pose_x = frame.p.x();
    m_feedback.base.tool_pose_y = frame.p.y();
    m_feedback.base.commanded_tool_pose_y = frame.p.y();
    m_feedback.base.tool_pose_z = frame.p.z();
    m_feedback.base.commanded_tool_pose_z = frame.p.z();
    double alpha, beta, gamma;
    frame.M.GetEulerZYX(alpha, beta, gamma);
    m_feedback.base.tool_pose_theta_x = m_math_util.toDeg(gamma);
    m_feedback.base.commanded_tool_pose_theta_x = m_math_util.toDeg(gamma);
    m_feedback.base.tool_pose_theta_y = m_math_util.toDeg(beta);
    m_feedback.base.commanded_tool_pose_theta_y = m_math_util.toDeg(beta);
    m_feedback.base.tool_pose_theta_z = m_math_util.toDeg(alpha);
    m_feedback.base.commanded_tool_pose_theta_z = m_math_util.toDeg(alpha);

    // Fill gripper information
    if (IsGripperPresent())
    {
        // Gripper index is right after last actuator and is expressed in % in the base feedback (not in absolute position like in joint_states)
        m_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position = 100.0 * m_math_util.relative_position_from_absolute(current.position[m_gripper_joint_index], m_gripper_joint_limits_min[0], m_gripper_joint_limits_max[0]);
    }

    return m_feedback;
}

std::shared_ptr<kortex_driver::srv::CreateAction::Response> KortexArmSimulation::CreateAction(const std::shared_ptr<kortex_driver::srv::CreateAction::Request> req)
{
    auto new_action = req->input;
    unsigned int identifier = FIRST_CREATED_ACTION_ID;
    bool identifier_taken = true;
    // Find unique identifier for new action
    while (identifier_taken)
    {
        identifier_taken = m_map_actions.count(identifier) == 1;
        if (identifier_taken)
        {
            ++identifier;
        }
    }
    // Add Action to map if type is supported
    switch (new_action.handle.action_type)
    {
        case kortex_driver::msg::ActionType::REACH_JOINT_ANGLES:
        case kortex_driver::msg::ActionType::REACH_POSE:
        case kortex_driver::msg::ActionType::SEND_GRIPPER_COMMAND:
        case kortex_driver::msg::ActionType::TIME_DELAY:
            new_action.handle.identifier = identifier;
            new_action.handle.permission = 7;
            m_map_actions.emplace(std::make_pair(identifier, new_action));
            break;
        default:
            RCLCPP_ERROR(m_node_handle->get_logger(), "Unsupported action type %d : could not create simulated action.", new_action.handle.action_type);
            break;
    }
    // Return ActionHandle for added action
    auto response = std::make_shared<kortex_driver::srv::CreateAction::Response>();
    response->output = new_action.handle;
    return response;
}

std::shared_ptr<kortex_driver::srv::ReadAction::Response> KortexArmSimulation::ReadAction(const std::shared_ptr<kortex_driver::srv::ReadAction::Request> req)
{
    auto input = req->input;
    auto response = std::make_shared<kortex_driver::srv::ReadAction::Response>();
    auto it = m_map_actions.find(input.identifier);
    if (it != m_map_actions.end())
    {
        response->output = it->second;
    }
    return response;
}

std::shared_ptr<kortex_driver::srv::ReadAllActions::Response> KortexArmSimulation::ReadAllActions(const std::shared_ptr<kortex_driver::srv::ReadAllActions::Request> req)
{
    auto input = req->input;
    auto response = std::make_shared<kortex_driver::srv::ReadAllActions::Response>();
    kortex_driver::msg::ActionList action_list;
    for (auto a : m_map_actions)
    {
        // If requested action type is specified and matches iterated action's type, add it to the list
        if (input.action_type == 0 || input.action_type == a.second.handle.action_type)
        {
            action_list.action_list.push_back(a.second);
        }
        
    }
    response->output = action_list;
    return response;
}

std::shared_ptr<kortex_driver::srv::DeleteAction::Response> KortexArmSimulation::DeleteAction(const std::shared_ptr<kortex_driver::srv::DeleteAction::Request> req)
{
    auto handle = req->input;
    // If the action is not a default action
    if (DEFAULT_ACTIONS_IDENTIFIERS.find(handle.identifier) == DEFAULT_ACTIONS_IDENTIFIERS.end())
    {
        auto it = m_map_actions.find(handle.identifier);
        if (it != m_map_actions.end())
        {
            m_map_actions.erase(it);
            RCLCPP_INFO(m_node_handle->get_logger(), "Simulated action #%u properly deleted.", handle.identifier);
        }
        else
        {
            RCLCPP_WARN(m_node_handle->get_logger(), "Could not find simulated action #%u to delete in actions map.", handle.identifier);
        }
    }
    else
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Cannot delete default simulated actions.");
    }
    
    return std::make_shared<kortex_driver::srv::DeleteAction::Response>();
}

std::shared_ptr<kortex_driver::srv::UpdateAction::Response> KortexArmSimulation::UpdateAction(const std::shared_ptr<kortex_driver::srv::UpdateAction::Request> req)
{
    auto action = req->input;
    // If the action is not a default action
    if (DEFAULT_ACTIONS_IDENTIFIERS.find(action.handle.identifier) == DEFAULT_ACTIONS_IDENTIFIERS.end())
    {
        auto it = m_map_actions.find(action.handle.identifier);
        if (it != m_map_actions.end())
        {
            if (it->second.handle.action_type == action.handle.action_type)
            {
                it->second = action;
                RCLCPP_INFO(m_node_handle->get_logger(), "Simulated action #%u properly updated.", action.handle.identifier);
            }
            else
            {
                RCLCPP_ERROR(m_node_handle->get_logger(), "Cannot update action with different type.");
            }
        }
        else
        {
            RCLCPP_ERROR(m_node_handle->get_logger(), "Could not find simulated action #%u to update in actions map.", action.handle.identifier);
        }
    }
    else
    {
       RCLCPP_ERROR(m_node_handle->get_logger(), "Cannot update default simulated actions."); 
    }

    return std::make_shared<kortex_driver::srv::UpdateAction::Response>();
}

std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Response> KortexArmSimulation::ExecuteActionFromReference(const std::shared_ptr<kortex_driver::srv::ExecuteActionFromReference::Request> req)
{
    auto handle = req->input;
    auto it = m_map_actions.find(handle.identifier);
    if (it != m_map_actions.end())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
        m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, it->second);
    }
    else
    {
        RCLCPP_ERROR(m_node_handle->get_logger(), "Could not find action with given identifier %d", handle.identifier);
    }
    
    return std::make_shared<kortex_driver::srv::ExecuteActionFromReference::Response>();
}

std::shared_ptr<kortex_driver::srv::ExecuteAction::Response> KortexArmSimulation::ExecuteAction(const std::shared_ptr<kortex_driver::srv::ExecuteAction::Request> req)
{
    auto action = req->input;
    // Add Action to map if type is supported
    switch (action.handle.action_type)
    {
        case kortex_driver::msg::ActionType::REACH_JOINT_ANGLES:
        case kortex_driver::msg::ActionType::REACH_POSE:
        case kortex_driver::msg::ActionType::SEND_GRIPPER_COMMAND:
        case kortex_driver::msg::ActionType::TIME_DELAY:
            JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
            m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
            break;
        default:
            RCLCPP_ERROR(m_node_handle->get_logger(), "Unsupported action type %d : could not execute simulated action.", action.handle.action_type);
            break;
    }

    return std::make_shared<kortex_driver::srv::ExecuteAction::Response>();
}

std::shared_ptr<kortex_driver::srv::StopAction::Response> KortexArmSimulation::StopAction(const std::shared_ptr<kortex_driver::srv::StopAction::Request> req)
{
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }

    m_follow_joint_trajectory_action_client->async_cancel_all_goals();
    if (IsGripperPresent())
    {
        m_gripper_action_client->async_cancel_all_goals();
    }
    
    return std::make_shared<kortex_driver::srv::StopAction::Response>();
}

std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Response> KortexArmSimulation::PlayCartesianTrajectory(const std::shared_ptr<kortex_driver::srv::PlayCartesianTrajectory::Request> req)
{
    auto constrained_pose = req->input;
    kortex_driver::msg::Action action;
    action.name = "PlayCartesianTrajectory";
    action.handle.action_type = kortex_driver::msg::ActionType::REACH_POSE;
    action.oneof_action_parameters.reach_pose.push_back(constrained_pose);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return std::make_shared<kortex_driver::srv::PlayCartesianTrajectory::Response>();
}

std::shared_ptr<kortex_driver::srv::SendTwistCommand::Response> KortexArmSimulation::SendTwistCommand(const std::shared_ptr<kortex_driver::srv::SendTwistCommand::Request> req)
{
    auto twist_command = req->input;
    kortex_driver::msg::Action action;
    action.name = "SendTwistCommand";
    action.handle.action_type = kortex_driver::msg::ActionType::SEND_TWIST_COMMAND;
    action.oneof_action_parameters.send_twist_command.push_back(twist_command);

    // Convert orientations to rad
    m_twist_command.angular_x = m_math_util.toRad(m_twist_command.angular_x);
    m_twist_command.angular_x = m_math_util.toRad(m_twist_command.angular_y);
    m_twist_command.angular_x = m_math_util.toRad(m_twist_command.angular_z);
    
    // Fill the twist command
    m_twist_command = twist_command.twist;

    // If we are already executing twist control, don't cancel the thread
    if (m_current_action_type != kortex_driver::msg::ActionType::SEND_TWIST_COMMAND)
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
        m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    }

    return std::make_shared<kortex_driver::srv::SendTwistCommand::Response>();
}

std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Response> KortexArmSimulation::PlayJointTrajectory(const std::shared_ptr<kortex_driver::srv::PlayJointTrajectory::Request> req)
{
    auto constrained_joint_angles = req->input;
    kortex_driver::msg::Action action;
    action.name = "PlayJointTrajectory";
    action.handle.action_type = kortex_driver::msg::ActionType::REACH_JOINT_ANGLES;
    action.oneof_action_parameters.reach_joint_angles.push_back(constrained_joint_angles);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return std::make_shared<kortex_driver::srv::PlayJointTrajectory::Response>();
}

std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Response> KortexArmSimulation::SendJointSpeedsCommand(const std::shared_ptr<kortex_driver::srv::SendJointSpeedsCommand::Request> req)
{
    auto joint_speeds = req->input;
    kortex_driver::msg::Action action;
    action.name = "SendJointSpeedsCommand";
    action.handle.action_type = kortex_driver::msg::ActionType::SEND_JOINT_SPEEDS;

    // Convert radians in degrees
    for (unsigned int i = 0; i < joint_speeds.joint_speeds.size(); i++)
    {
        joint_speeds.joint_speeds[i].value = m_math_util.toDeg(joint_speeds.joint_speeds[i].value);
    }
    action.oneof_action_parameters.send_joint_speeds.push_back(joint_speeds);

    // Fill the velocity commands vector
    int n = 0;
    std::generate(m_velocity_commands.begin(),
        m_velocity_commands.end(), 
        [this, &action, &n]() -> double
        {
            return m_math_util.toRad(action.oneof_action_parameters.send_joint_speeds[0].joint_speeds[n++].value);
        });

    // If we are already executing joint speed control, don't cancel the thread
    if (m_current_action_type != kortex_driver::msg::ActionType::SEND_JOINT_SPEEDS)
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
        m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    }
    
    return std::make_shared<kortex_driver::srv::SendJointSpeedsCommand::Response>();
}

std::shared_ptr<kortex_driver::srv::SendGripperCommand::Response> KortexArmSimulation::SendGripperCommand(const std::shared_ptr<kortex_driver::srv::SendGripperCommand::Request> req)
{
    auto gripper_command = req->input;
    kortex_driver::msg::Action action;
    action.name = "GripperCommand";
    action.handle.action_type = kortex_driver::msg::ActionType::SEND_GRIPPER_COMMAND;
    action.oneof_action_parameters.send_gripper_command.push_back(gripper_command);

    JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    m_action_executor_thread = std::thread(&KortexArmSimulation::PlayAction, this, action);
    
    return std::make_shared<kortex_driver::srv::SendGripperCommand::Response>();
}

std::shared_ptr<kortex_driver::srv::Stop::Response> KortexArmSimulation::Stop(const std::shared_ptr<kortex_driver::srv::Stop::Request> req)
{
    // If an action is ongoing, cancel it first
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }
    m_follow_joint_trajectory_action_client->async_cancel_all_goals();
    if (IsGripperPresent())
    {
        m_gripper_action_client->async_cancel_all_goals();
    }
    return std::make_shared<kortex_driver::srv::Stop::Response>();
}

std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Response> KortexArmSimulation::ApplyEmergencyStop(const std::shared_ptr<kortex_driver::srv::ApplyEmergencyStop::Request> req)
{
    // If an action is ongoing, cancel it first
    if (m_is_action_being_executed.load())
    {
        JoinThreadAndCancelAction(); // this will block until the thread is joined and current action finished
    }
    m_follow_joint_trajectory_action_client->async_cancel_all_goals();
    if (IsGripperPresent())
    {
        m_gripper_action_client->async_cancel_all_goals();
    }
    return std::make_shared<kortex_driver::srv::ApplyEmergencyStop::Response>();
}

void KortexArmSimulation::cb_joint_states(const std::shared_ptr<sensor_msgs::msg::JointState> state)
{
    const std::lock_guard<std::mutex> lock(m_state_mutex);
    m_first_state_received = true;
    m_current_state = *state;
}

void KortexArmSimulation::CreateDefaultActions()
{
    kortex_driver::msg::Action retract, home, zero;
    kortex_driver::msg::ConstrainedJointAngles retract_angles, home_angles, zero_angles;
    // Retract
    retract.handle.identifier = 1;
    retract.handle.action_type = kortex_driver::msg::ActionType::REACH_JOINT_ANGLES;
    retract.handle.permission = 7;
    retract.name = "Retract";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::msg::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("retract");
        double moveit_angle = named_target[m_prefix + "joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        retract_angles.joint_angles.joint_angles.push_back(a);
    }
    retract.oneof_action_parameters.reach_joint_angles.push_back(retract_angles);
    // Home
    home.handle.identifier = 2;
    home.handle.action_type = kortex_driver::msg::ActionType::REACH_JOINT_ANGLES;
    home.handle.permission = 7;
    home.name = "Home";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::msg::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("home");
        double moveit_angle = named_target[m_prefix + "joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        home_angles.joint_angles.joint_angles.push_back(a);
    }
    home.oneof_action_parameters.reach_joint_angles.push_back(home_angles);
    // Zero
    zero.handle.identifier = 3;
    zero.handle.action_type = kortex_driver::msg::ActionType::REACH_JOINT_ANGLES;
    zero.handle.permission = 7;
    zero.name = "Zero";
    for (int i = 0; i < m_degrees_of_freedom; i++)
    {
        kortex_driver::msg::JointAngle a;
        a.joint_identifier = i;
        auto named_target = m_moveit_arm_interface->getNamedTargetValues("vertical");
        double moveit_angle = named_target[m_prefix + "joint_"+std::to_string(i+1)]; // rad
        a.value = m_math_util.wrapDegreesFromZeroTo360(m_math_util.toDeg(moveit_angle));
        zero_angles.joint_angles.joint_angles.push_back(a);
    }
    zero.oneof_action_parameters.reach_joint_angles.push_back(zero_angles);
    // Add actions
    m_map_actions.emplace(std::make_pair(retract.handle.identifier, retract));
    m_map_actions.emplace(std::make_pair(home.handle.identifier, home));
    m_map_actions.emplace(std::make_pair(zero.handle.identifier, zero));
}

bool KortexArmSimulation::SwitchControllerType(ControllerType new_type)
{
    bool success = true;
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->strictness = request->STRICT;
    if (m_active_controller_type != new_type)
    {
        // Set the controllers we want to switch to
        switch (new_type)
        {
            case ControllerType::kTrajectory:
                request->start_controllers = m_trajectory_controllers_list;
                request->stop_controllers = m_position_controllers_list;
                break;
            case ControllerType::kIndividual:
                request->start_controllers = m_position_controllers_list;
                request->stop_controllers = m_trajectory_controllers_list;
                break;
            default:
                RCLCPP_ERROR(m_node_handle->get_logger(), "Kortex arm simulator : Unsupported controller type %d", int(new_type));
                return false;
        }

        // Call the service
        // if (!m_client_switch_controllers.call(service))
        // {
        //     RCLCPP_ERROR(m_node_handle->get_logger(), "Failed to call the service for switching controllers");
        //     success = false;
        // }
        // else
        // {
        //     success = service.response.ok;
        // }
        auto result = m_client_switch_controllers->async_send_request(request);
        if (rclcpp::spin_until_future_complete(m_node_handle, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            success = result.get()->ok;
        }
        else
        {
            RCLCPP_ERROR(m_node_handle->get_logger(), "Failed to call the service for switching controllers");
            success = false;
        }

        // Update active type if the switch was successful
        if (success)
        {
            m_active_controller_type = new_type;
        }
    }

    return success;
}

kortex_driver::msg::KortexError KortexArmSimulation::FillKortexError(uint32_t code, uint32_t subCode, const std::string& description) const
{
    kortex_driver::msg::KortexError error;
    error.code = code;
    error.sub_code = subCode;
    error.description = description;
    return error;
}

void KortexArmSimulation::JoinThreadAndCancelAction()
{
    // Tell the thread to stop and join it
    m_action_preempted = true;
    if (m_action_executor_thread.joinable())
    {
        m_action_executor_thread.join();
    }
    m_current_action_type = 0;
    m_action_preempted = false;
}

void KortexArmSimulation::PlayAction(const kortex_driver::msg::Action& action)
{
    auto action_result = FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_NONE, kortex_driver::msg::SubErrorCodes::SUB_ERROR_NONE);

    // Notify action started
    kortex_driver::msg::ActionNotification start_notif;
    start_notif.handle = action.handle;
    start_notif.action_event = kortex_driver::msg::ActionEvent::ACTION_START;
    m_pub_action_topic->publish(start_notif);
    m_is_action_being_executed = true;
    m_current_action_type = action.handle.action_type;
    
    // Switch executor on the action type
    switch (action.handle.action_type)
    {
        case kortex_driver::msg::ActionType::REACH_JOINT_ANGLES:
            action_result = ExecuteReachJointAngles(action);
            break;
        case kortex_driver::msg::ActionType::REACH_POSE:
            action_result = ExecuteReachPose(action);
            break;
        case kortex_driver::msg::ActionType::SEND_JOINT_SPEEDS:
            action_result = ExecuteSendJointSpeeds(action);
            break;
        case kortex_driver::msg::ActionType::SEND_TWIST_COMMAND:
            action_result = ExecuteSendTwist(action);
            break;
        case kortex_driver::msg::ActionType::SEND_GRIPPER_COMMAND:
            action_result = ExecuteSendGripperCommand(action);
            break;
        case kortex_driver::msg::ActionType::TIME_DELAY:
            action_result = ExecuteTimeDelay(action);
            break;
        default:
            action_result = FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE, kortex_driver::msg::SubErrorCodes::UNSUPPORTED_ACTION);
            break;
    }
    
    // Oddly enough, gripper actions don't send notifications through Kortex API when they end
    if (action.handle.action_type != kortex_driver::msg::ActionType::SEND_GRIPPER_COMMAND)
    {
        kortex_driver::msg::ActionNotification end_notif;
        end_notif.handle = action.handle;
        // Action was cancelled by user and is not a velocity command
        if (m_action_preempted.load() && action.handle.action_type != kortex_driver::msg::ActionType::SEND_JOINT_SPEEDS)
        {
            // Notify ACTION_ABORT
            end_notif.action_event = kortex_driver::msg::ActionEvent::ACTION_ABORT;
            RCLCPP_WARN(m_node_handle->get_logger(), "Action was aborted by user.");
        }
        // Action ended on its own
        else
        {
            if (action_result.code != kortex_driver::msg::ErrorCodes::ERROR_NONE)
            {
                // Notify ACTION_ABORT
                end_notif.action_event = kortex_driver::msg::ActionEvent::ACTION_ABORT;
                end_notif.abort_details = action_result.sub_code;
                RCLCPP_WARN(m_node_handle->get_logger(), "Action was failed : \nError code is %d\nSub-error code is %d\nError description is : %s", 
                            action_result.code,
                            action_result.sub_code,
                            action_result.description.c_str());
            }
            else
            {
                // Notify ACTION_END
                end_notif.action_event = kortex_driver::msg::ActionEvent::ACTION_END;
            }
        }
        m_pub_action_topic->publish(end_notif);
    }
    
    m_is_action_being_executed = false;
}

kortex_driver::msg::KortexError KortexArmSimulation::ExecuteReachJointAngles(const kortex_driver::msg::Action& action)
{
    auto result = FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_NONE, kortex_driver::msg::SubErrorCodes::SUB_ERROR_NONE);
    if (action.oneof_action_parameters.reach_joint_angles.size() != 1)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                    kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                    "Error playing joint angles action : action is malformed.");
    }
    auto constrained_joint_angles = action.oneof_action_parameters.reach_joint_angles[0];
    if (constrained_joint_angles.joint_angles.joint_angles.size() != GetDOF())
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing joint angles action : action contains " + std::to_string(constrained_joint_angles.joint_angles.joint_angles.size()) + " joint angles but arm has " + std::to_string(GetDOF()));
    }

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kTrajectory))
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::METHOD_FAILED,
                                "Error playing joint angles action : simulated trajectory controller could not be switched to.");
    }

    // Initialize trajectory object
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.frame_id = m_prefix + "base_link";
    for (int i = 0; i < constrained_joint_angles.joint_angles.joint_angles.size(); i++)
    {
        const std::string joint_name = m_prefix + "joint_" + std::to_string(i+1); //joint names are 1-based
        traj.joint_names.push_back(joint_name);
    }

    // Get current position
    sensor_msgs::msg::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }

    // Transform kortex structure to trajectory_msgs to fill endpoint structure
    trajectory_msgs::msg::JointTrajectoryPoint endpoint;
    std::unordered_set<int> limited_joints; // joints limited in range
    int degrees_of_freedom = constrained_joint_angles.joint_angles.joint_angles.size();
    if (degrees_of_freedom == 6)
    {
        limited_joints = {1,2,4};
    }
    else if (degrees_of_freedom == 7)
    {
        limited_joints = {1,3,5};
    }
    else 
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Unsupported number of joints, expected 6 or 7");
    }
    for (int i = 0; i < constrained_joint_angles.joint_angles.joint_angles.size(); i++)
    {
        // If the current actuator has turned on itself many times, we need the endpoint to follow that trend too
        int n_turns = 0;
        double rad_wrapped_goal;
        if (limited_joints.count(i))
        {
            rad_wrapped_goal = m_math_util.wrapRadiansFromMinusPiToPi(m_math_util.toRad(constrained_joint_angles.joint_angles.joint_angles[i].value));
        }
        else
        {
            rad_wrapped_goal = m_math_util.wrapRadiansFromMinusPiToPi(m_math_util.toRad(constrained_joint_angles.joint_angles.joint_angles[i].value), n_turns);
        }
        endpoint.positions.push_back(rad_wrapped_goal + double(n_turns) * 2.0 * M_PI);
        endpoint.velocities.push_back(0.0);
        endpoint.accelerations.push_back(0.0);
    }

    // Calculate velocity profiles to know how much time this trajectory must last
    switch (constrained_joint_angles.constraint.type)
    {
        // If the duration is supplied, set the duration of the velocity profiles with that value
        case kortex_driver::msg::JointTrajectoryConstraintType::JOINT_CONSTRAINT_DURATION:
        {
            // Error check on the given duration
            if (constrained_joint_angles.constraint.value <= 0.0f)
            {
                return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Invalid duration constraint : it has to be higher than 0.0!");
            }
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.position[m_first_arm_joint_index + i], endpoint.positions[i], constrained_joint_angles.constraint.value);
            }
            endpoint.time_from_start = rclcpp::Duration::from_seconds(constrained_joint_angles.constraint.value);
            RCLCPP_DEBUG(m_node_handle->get_logger(), "Using supplied duration : %2.2f", constrained_joint_angles.constraint.value);
            break;
        }
        // If a max velocity is supplied for each joint, we need to find the limiting duration with this velocity constraint
        case kortex_driver::msg::JointTrajectoryConstraintType::JOINT_CONSTRAINT_SPEED:
        {
            // Error check on the given velocity
            float max_velocity = m_math_util.toRad(constrained_joint_angles.constraint.value);
            if (max_velocity <= 0.0f)
            {
                return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Invalid velocity constraint : it has to be higher than 0.0!");
            }
            // Find the limiting duration with given velocity
            double max_duration = 0.0;
            for (int i = 0; i < GetDOF(); i++)
            {
                double velocity_ratio = std::min(1.0, double(max_velocity)/m_arm_velocity_max_limits[i]);
                m_velocity_trap_profiles[i].SetProfileVelocity(current.position[m_first_arm_joint_index + i], endpoint.positions[i], velocity_ratio);
                max_duration = std::max(max_duration, m_velocity_trap_profiles[i].Duration());
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Joint %d moving from %2.2f to %2.2f gives duration %2.2f", i, current.position[m_first_arm_joint_index + i], endpoint.positions[i], m_velocity_trap_profiles[i].Duration());
            }
            RCLCPP_DEBUG(m_node_handle->get_logger(), "max_duration is : %2.2f", max_duration);
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.position[m_first_arm_joint_index + i], endpoint.positions[i], max_duration);
            }
            endpoint.time_from_start = rclcpp::Duration::from_seconds(max_duration);
            break;
        }
        default:
        {
            // Find the optimal duration based on actual velocity and acceleration limits
            double optimal_duration = 0.0;
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfile(current.position[m_first_arm_joint_index + i], endpoint.positions[i]);
                optimal_duration = std::max(optimal_duration, m_velocity_trap_profiles[i].Duration());
                RCLCPP_DEBUG(m_node_handle->get_logger(), "Joint %d moving from %2.2f to %2.2f gives duration %2.2f", i, current.position[m_first_arm_joint_index + i], endpoint.positions[i], m_velocity_trap_profiles[i].Duration());
            }
            RCLCPP_DEBUG(m_node_handle->get_logger(), "optimal_duration is : %2.2f", optimal_duration);
            // Set the velocity profiles
            for (int i = 0; i < GetDOF(); i++)
            {
                m_velocity_trap_profiles[i].SetProfileDuration(current.position[m_first_arm_joint_index + i], endpoint.positions[i], optimal_duration);
            }
            endpoint.time_from_start = rclcpp::Duration::from_seconds(optimal_duration);
            break;
        }
    }

    // Copy velocity profile data into trajectory using JOINT_TRAJECTORY_TIMESTEP_SECONDS timesteps
    // For each timestep
    for (double t = JOINT_TRAJECTORY_TIMESTEP_SECONDS; t < m_velocity_trap_profiles[0].Duration(); t += JOINT_TRAJECTORY_TIMESTEP_SECONDS)
    {
        // Create trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.time_from_start = rclcpp::Duration::from_seconds(t);
        // Add position, velocity, acceleration from each velocity profile
        for (int i = 0; i < GetDOF(); i++)
        {
            p.positions.push_back(m_velocity_trap_profiles[i].Pos(t));
            p.velocities.push_back(m_velocity_trap_profiles[i].Vel(t));
            p.accelerations.push_back(m_velocity_trap_profiles[i].Acc(t));
        }
        // Add trajectory point to goal
        traj.points.push_back(p);
    }
    // Finally, add endpoint to trajectory
    // Add position, velocity, acceleration from each velocity profile
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.time_from_start = rclcpp::Duration::from_seconds(m_velocity_trap_profiles[0].Duration());
    for (int i = 0; i < GetDOF(); i++)
    {
        p.positions.push_back(m_velocity_trap_profiles[i].Pos(m_velocity_trap_profiles[i].Duration()));
        p.velocities.push_back(m_velocity_trap_profiles[i].Vel(m_velocity_trap_profiles[i].Duration()));
        p.accelerations.push_back(m_velocity_trap_profiles[i].Acc(m_velocity_trap_profiles[i].Duration()));
    }
    // Add trajectory point to goal
    traj.points.push_back(p);

    // Verify if goal has been cancelled before sending it
    if (m_action_preempted.load())
    {
        return result;
    }
    
    // Send goal
    control_msgs::action::FollowJointTrajectory::Goal goal;
    traj.header.stamp = m_node_handle->get_clock()->now();
    goal.trajectory = traj;
    auto action_result = m_follow_joint_trajectory_action_client->async_send_goal(goal);

    // Wait for goal to be done, or for preempt to be called (check every 100ms)
    while(!m_action_preempted.load())
    {
        if (rclcpp::spin_until_future_complete(m_node_handle, action_result, std::chrono::duration<double>(0.1f)) ==
                rclcpp::FutureReturnCode::SUCCESS)
        {
            // Sometimes an error is thrown related to a bad cast in a ros::time structure inside the SimpleActionClient
            // See https://answers.ros.org/question/209452/exception-thrown-while-processing-service-call-time-is-out-of-dual-32-bit-range/
            // If this error happens here we just send the goal again with an updated timestamp
            auto status = m_follow_joint_trajectory_action_client->async_get_result(action_result.get()).get().result;
            if (status->error_string == "Time is out of dual 32-bit range")
            {
                traj.header.stamp = m_node_handle->get_clock()->now();
                goal.trajectory = traj;
                action_result = m_follow_joint_trajectory_action_client->async_send_goal(goal);
            }
            else
            {
                break;
            }
        }
    }

    // If we got out of the loop because we're preempted, cancel the goal before returning
    if (m_action_preempted.load())
    {
        m_follow_joint_trajectory_action_client->async_cancel_all_goals();
    }
    // Fill result depending on action final status if user didn't cancel
    else
    {
        auto status = m_follow_joint_trajectory_action_client->async_get_result(action_result.get()).get().result;
        if (status->error_code != status->SUCCESSFUL)
        {
            result = FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                        kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                        status->error_string);
        }
    }
    return result;
}

kortex_driver::msg::KortexError KortexArmSimulation::ExecuteReachPose(const kortex_driver::msg::Action& action)
{
    kortex_driver::msg::KortexError result;
    result.code = kortex_driver::msg::ErrorCodes::ERROR_NONE;
    result.sub_code = kortex_driver::msg::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.reach_pose.size() != 1)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing pose action : action is malformed.");
    }
    auto constrained_pose = action.oneof_action_parameters.reach_pose[0];

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kTrajectory))
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::METHOD_FAILED,
                                "Error playing pose action : simulated trajectory controller could not be switched to.");
    }

    // Get current position
    sensor_msgs::msg::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }
    
    // Get Start frame
    // For the Rotation part : ThetaX = gamma, ThetaY = beta, ThetaZ = alpha 
    auto start = KDL::Frame();
    Eigen::VectorXd positions_eigen(m_degrees_of_freedom);
    for (int i = 0; i < GetDOF(); i++)
    {
        positions_eigen[i] = current.position[m_first_arm_joint_index + i];
    }
    KDL::JntArray current_kdl(GetDOF());
    current_kdl.data = positions_eigen;
    m_fk_solver->JntToCart(current_kdl, start);

    {
    RCLCPP_DEBUG(m_node_handle->get_logger(), "START FRAME :");
    RCLCPP_DEBUG(m_node_handle->get_logger(), "X=%2.4f Y=%2.4f Z=%2.4f", start.p[0], start.p[1], start.p[2]);
    double sa, sb, sg; start.M.GetEulerZYX(sa, sb, sg);
    RCLCPP_DEBUG(m_node_handle->get_logger(), "ALPHA=%2.4f BETA=%2.4f GAMMA=%2.4f", m_math_util.toDeg(sa), m_math_util.toDeg(sb), m_math_util.toDeg(sg));
    KDL::Vector axis;
    RCLCPP_DEBUG(m_node_handle->get_logger(), "start rot = %2.4f", start.M.GetRotAngle(axis));
    }

    // Get End frame
    auto end_pos = KDL::Vector(constrained_pose.target_pose.x, constrained_pose.target_pose.y, constrained_pose.target_pose.z);
    auto end_rot = KDL::Rotation::EulerZYX(m_math_util.toRad(constrained_pose.target_pose.theta_z), m_math_util.toRad(constrained_pose.target_pose.theta_y), m_math_util.toRad(constrained_pose.target_pose.theta_x));
    KDL::Frame end(end_rot, end_pos);

    {
    RCLCPP_DEBUG(m_node_handle->get_logger(), "END FRAME :");
    RCLCPP_DEBUG(m_node_handle->get_logger(), "X=%2.4f Y=%2.4f Z=%2.4f", end_pos[0], end_pos[1], end_pos[2]);
    double ea, eb, eg; end_rot.GetEulerZYX(ea, eb, eg);
    RCLCPP_DEBUG(m_node_handle->get_logger(), "ALPHA=%2.4f BETA=%2.4f GAMMA=%2.4f", m_math_util.toDeg(ea), m_math_util.toDeg(eb), m_math_util.toDeg(eg));
    KDL::Vector axis;
    RCLCPP_DEBUG(m_node_handle->get_logger(), "end rot = %2.4f", end_rot.GetRotAngle(axis));
    }

    // If different speed limits than the default ones are provided, use them instead
    float translation_speed_limit = m_max_cartesian_twist_linear;
    float rotation_speed_limit = m_max_cartesian_twist_angular;
    if (!constrained_pose.constraint.oneof_type.speed.empty())
    {
        // If a max velocity is supplied for each joint, we need to find the limiting duration with this velocity constraint
        translation_speed_limit = std::min(constrained_pose.constraint.oneof_type.speed[0].translation, translation_speed_limit);
        rotation_speed_limit = std::min(constrained_pose.constraint.oneof_type.speed[0].orientation, rotation_speed_limit);
    }

    // Calculate norm of translation movement and minimum duration to move this amount given max translation speed
    double delta_pos = (end_pos - start.p).Norm();
    double minimum_translation_duration = delta_pos / m_max_cartesian_twist_linear; // in seconds

    // Calculate angle variation of rotation movement and minimum duration to move this amount given max rotation speed
    KDL::Vector axis; // we need to create this variable to access the RotAngle for start and end frames'rotation components
    KDL::Rotation dR = end_rot * start.M.Inverse();
    double delta_rot = dR.GetRotAngle(axis);
    double minimum_rotation_duration = delta_rot / m_max_cartesian_twist_angular; // in seconds

    RCLCPP_INFO(m_node_handle->get_logger(), "trans : %2.4f rot : %2.4f", minimum_translation_duration, minimum_rotation_duration);

    // The default value for the duration will be the longer duration of the two
    double duration = std::max(minimum_translation_duration, minimum_rotation_duration);

    // eq_radius is chosen here to make it so translations and rotations are normalised
    // Here is a good explanation for it : https://github.com/zakharov/BRICS_RN/blob/master/navigation_trajectory_common/include/navigation_trajectory_common/Conversions.h#L358-L382
    float eq_radius = translation_speed_limit / rotation_speed_limit;

    // Create Path_Line object
    // I know this is ugly but the RotationalInterpolation object is mandatory and needs to be created as such
    KDL::Path_Line line(start, end, new KDL::RotationalInterpolation_SingleAxis(), eq_radius);

    // Create a trapezoidal velocity profile for the Cartesian trajectory to parametrize it in time
    KDL::VelocityProfile_Trap velocity_profile(std::min(translation_speed_limit, rotation_speed_limit), m_max_cartesian_acceleration_linear);
    velocity_profile.SetProfile(0.0, line.PathLength());
    duration = std::max(duration, velocity_profile.Duration());

    // If duration is not supplied, use the one we just calculated
    // If the duration is supplied, simply use it
    if (!constrained_pose.constraint.oneof_type.duration.empty())
    {
        double supplied_duration = constrained_pose.constraint.oneof_type.duration[0];
        if (duration > supplied_duration)
        {
            RCLCPP_WARN(m_node_handle->get_logger(), "Cannot use supplied duration %2.4f because the minimum duration based on velocity limits is %2.4f",
                        supplied_duration,
                        duration);
        }
        duration = std::max(duration, supplied_duration);
    }

    // Set the velocity profile duration
    velocity_profile.SetProfileDuration(0.0, line.PathLength(), duration);
    KDL::Trajectory_Segment segment(&line, &velocity_profile, false);
    RCLCPP_DEBUG(m_node_handle->get_logger(), "Duration of trajectory will be %2.4f seconds", duration);

    // Initialize trajectory object
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.frame_id = m_prefix + "base_link";
    traj.header.stamp = m_node_handle->get_clock()->now();
    for (int i = 0; i < GetDOF(); i++)
    {
        const std::string joint_name = m_prefix + "joint_" + std::to_string(i+1); //joint names are 1-based
        traj.joint_names.push_back(joint_name);
    }

    // Fill trajectory object
    KDL::JntArray previous = current_kdl; // Position i - 1, initialise to starting angles
    KDL::JntArray current_joints(GetDOF()); // Position i
    for (float t = JOINT_TRAJECTORY_TIMESTEP_SECONDS; t < duration; t += JOINT_TRAJECTORY_TIMESTEP_SECONDS)
    {
        // First get the next Cartesian position and twists
        auto pos = segment.Pos(t);
        
        // Position is enough to have a smooth trajectory it seems but if eventually vel and acc are useful somehow this is how to get them:
        // auto vel = segment.Vel(t);
        // auto acc = segment.Acc(t);

        // Create trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.time_from_start = rclcpp::Duration::from_seconds(t);

        // Use inverse IK solver
        int code = m_ik_pos_solver->CartToJnt(previous, pos, current_joints);
        if (code != m_ik_pos_solver->E_NOERROR)
        {
            RCLCPP_ERROR(m_node_handle->get_logger(), "IK ERROR CODE = %d", code);
        }

        for (int i = 0; i < GetDOF(); i++)
        {
            p.positions.push_back(current_joints(i));
        }
        
        // Add trajectory point to goal
        traj.points.push_back(p);
        previous = current_joints;
    }

    // Add last point
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.time_from_start = rclcpp::Duration::from_seconds(segment.Duration());
    auto pos = segment.Pos(segment.Duration());
    int code = m_ik_pos_solver->CartToJnt(previous, pos, current_joints);
    for (int i = 0; i < GetDOF(); i++)
    {
        p.positions.push_back(current_joints(i));
    }
    
    // Add trajectory point to goal
    traj.points.push_back(p);

    // Verify if goal has been cancelled before sending it
    if (m_action_preempted.load())
    {
        return result;
    }
    
    // Send goal
    control_msgs::action::FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;
    auto action_result = m_follow_joint_trajectory_action_client->async_send_goal(goal);

    // Wait for goal to be done, or for preempt to be called (check every 10ms)
    while(!m_action_preempted.load() && 
        rclcpp::spin_until_future_complete(m_node_handle, action_result, std::chrono::duration<double>(0.01f)) != rclcpp::FutureReturnCode::SUCCESS) {}

    // If we got out of the loop because we're preempted, cancel the goal before returning
    if (m_action_preempted.load())
    {
        m_follow_joint_trajectory_action_client->async_cancel_all_goals();
    }
    // Fill result depending on action final status if user didn't cancel
    else
    {
        auto status = m_follow_joint_trajectory_action_client->async_get_result(action_result.get()).get().result;
        if (status->error_code != status->SUCCESSFUL)
        {
            result = FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                        kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                        status->error_string);
        }
    }
    return result;
}

kortex_driver::msg::KortexError KortexArmSimulation::ExecuteSendJointSpeeds(const kortex_driver::msg::Action& action)
{
    kortex_driver::msg::KortexError result;
    result.code = kortex_driver::msg::ErrorCodes::ERROR_NONE;
    result.sub_code = kortex_driver::msg::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_joint_speeds.size() != 1)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing joints speeds : action is malformed.");
    }
    auto joint_speeds = action.oneof_action_parameters.send_joint_speeds[0];
    if (joint_speeds.joint_speeds.size() != GetDOF())
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing joint speeds action : action contains " + std::to_string(joint_speeds.joint_speeds.size()) + " joint speeds but arm has " + std::to_string(GetDOF()));
    }

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kIndividual))
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::METHOD_FAILED,
                                "Error playing joint speeds action : simulated positions controllers could not be switched to.");
    }

    // Get current position
    sensor_msgs::msg::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }
    
    // Initialise commands
    std::vector<double> commands(GetDOF(), 0.0); // in rad
    for (int i = 0; i < GetDOF(); i++)
    {
        commands[i] = current.position[m_first_arm_joint_index + i];
    }
    std::vector<double> previous_commands = commands; // in rad
    std::vector<double> velocity_commands(GetDOF(), 0.0); // in rad/s
    std::vector<double> previous_velocity_commands(GetDOF(), 0.0); // in rad/s
    std::vector<bool> stopped(GetDOF(), false);
    
    // While we're not done
    while (true)
    {
        // If action is preempted, set the velocities to 0
        if (m_action_preempted.load())
        {
            std::fill(m_velocity_commands.begin(), m_velocity_commands.end(), 0.0);
        }
        // For each joint
        for (int i = 0; i < GetDOF(); i++)
        {
            // Calculate real position increment
            // This helps to know if we hit joint limits or if we stopped
            {
            const std::lock_guard<std::mutex> lock(m_state_mutex);
            current = m_current_state;
            }

            // Calculate permitted velocity command because we don't have infinite acceleration
            double vel_delta = m_velocity_commands[i] - previous_velocity_commands[i];
            double max_delta = std::copysign(m_arm_acceleration_max_limits[i] * JOINT_TRAJECTORY_TIMESTEP_SECONDS, vel_delta);
            
            // If the velocity change is within acceleration limits for this timestep
            double velocity_command;
            if (fabs(vel_delta) < fabs(max_delta))
            {
                velocity_command = m_velocity_commands[i];
            }
            // If we cannot instantly accelerate to this velocity
            else
            {
                velocity_command = previous_velocity_commands[i] + max_delta;
            }

            // Cap to the velocity limit for the joint
            velocity_command = std::copysign(std::min(fabs(velocity_command), double(fabs(m_arm_velocity_max_limits[i]))), velocity_command);

            // Check if velocity command is in fact too small
            if (fabs(velocity_command) < MINIMUM_JOINT_VELOCITY_RAD_PER_SECONDS)
            {
                velocity_command = 0.0;
                commands[i] = current.position[m_first_arm_joint_index + i];
                stopped[i] = true;
            }
            // Else calculate the position increment and send it
            else
            {
                commands[i] = previous_commands[i] + velocity_command * JOINT_TRAJECTORY_TIMESTEP_SECONDS;
                stopped[i] = false;

                // Cap the command to the joint limit
                if (m_arm_joint_limits_min[i] != 0.0 && commands[i] < m_arm_joint_limits_min[i])
                {
                    commands[i] = m_arm_joint_limits_min[i];
                    velocity_command = std::max(velocity_command, 0.0);
                }
                else if (m_arm_joint_limits_max[i] != 0.0 && commands[i] > m_arm_joint_limits_max[i])
                {
                    commands[i] = m_arm_joint_limits_max[i];
                    velocity_command = std::min(velocity_command, 0.0);
                }

                // Send the position increments to the controllers
                std_msgs::msg::Float64 message;
                message.data = commands[i];
                m_pub_position_controllers[i]->publish(message);
            }

            // Remember actual command as previous and actual velocity command as previous
            previous_commands[i] = commands[i];
            previous_velocity_commands[i] = velocity_command;
        }

        // Sleep for TIMESTEP
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000*JOINT_TRAJECTORY_TIMESTEP_SECONDS)));

        // If the action is preempted and we're back to zero velocity, we're done here
        if (m_action_preempted.load())
        {
            // Check if all joints are stopped and break if yes
            if (std::all_of(stopped.begin(), stopped.end(), [](bool b){return b;}))
            {
                break;
            }
        }
    }

    return result;
}

kortex_driver::msg::KortexError KortexArmSimulation::ExecuteSendTwist(const kortex_driver::msg::Action& action)
{
    kortex_driver::msg::KortexError result;
    result.code = kortex_driver::msg::ErrorCodes::ERROR_NONE;
    result.sub_code = kortex_driver::msg::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_twist_command.size() != 1)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing twist action : action is malformed.");
    }
    auto twist = action.oneof_action_parameters.send_twist_command[0];

    // Switch to trajectory controller
    if (!SwitchControllerType(ControllerType::kIndividual))
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::METHOD_FAILED,
                                "Error playing joint speeds action : simulated positions controllers could not be switched to.");
    }

    // Only mixed frame is supported in simulation
    if (twist.reference_frame != kortex_driver::msg::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing twist action : only mixed frame is supported in simulation.");
    }

    // Get current position
    sensor_msgs::msg::JointState current;
    {
        const std::lock_guard<std::mutex> lock(m_state_mutex);
        current = m_current_state;
    }
    
    // Initialise commands
    std::vector<double> commands(GetDOF(), 0.0); // in rad
    for (int i = 0; i < GetDOF(); i++)
    {
        commands[i] = current.position[m_first_arm_joint_index + i];
    }
    std::vector<double> previous_commands = commands; // in rad
    std::vector<double> previous_velocity_commands(GetDOF(), 0.0); // in rad/s
    std::vector<bool> stopped(GetDOF(), false);
    kortex_driver::msg::Twist twist_command = m_twist_command;
    kortex_driver::msg::Twist previous_twist_command;

    // While we're not done
    while (rclcpp::ok())
    {
        // If action is preempted, set the velocities to 0
        if (m_action_preempted.load())
        {
            m_twist_command = kortex_driver::msg::Twist();
        }

        // Calculate actual twist command considering max linear and angular accelerations
        double max_linear_twist_delta = JOINT_TRAJECTORY_TIMESTEP_SECONDS * m_max_cartesian_acceleration_linear;
        double max_angular_twist_delta = JOINT_TRAJECTORY_TIMESTEP_SECONDS * m_max_cartesian_acceleration_angular;
        kortex_driver::msg::Twist delta_twist = m_math_util.substractTwists(m_twist_command, previous_twist_command);

        // If the velocity change is within acceleration limits for this timestep
        if (fabs(delta_twist.linear_x) < fabs(max_linear_twist_delta))
        {
            twist_command.linear_x = m_twist_command.linear_x;
        }
        // If we cannot instantly accelerate to this velocity
        else
        {
            twist_command.linear_x = previous_twist_command.linear_x + std::copysign(max_linear_twist_delta, delta_twist.linear_x);
        }
        // same for linear_y
        if (fabs(delta_twist.linear_y) < fabs(max_linear_twist_delta))
        {
            twist_command.linear_y = m_twist_command.linear_y;
        }
        else
        {
            twist_command.linear_y = previous_twist_command.linear_y + std::copysign(max_linear_twist_delta, delta_twist.linear_y);
        }
        // same for linear_z
        if (fabs(delta_twist.linear_z) < fabs(max_linear_twist_delta))
        {
            twist_command.linear_z = m_twist_command.linear_z;
        }
        else
        {
            twist_command.linear_z = previous_twist_command.linear_z + std::copysign(max_linear_twist_delta, delta_twist.linear_z);
        }
        // same for angular_x
        if (fabs(delta_twist.angular_x) < fabs(max_angular_twist_delta))
        {
            twist_command.angular_x = m_twist_command.angular_x;
        }
        else
        {
            twist_command.angular_x = previous_twist_command.angular_x + std::copysign(max_angular_twist_delta, delta_twist.angular_x);
        }
        // same for angular_y
        if (fabs(delta_twist.angular_y) < fabs(max_angular_twist_delta))
        {
            twist_command.angular_y = m_twist_command.angular_y;
        }
        else
        {
            twist_command.angular_y = previous_twist_command.angular_y + std::copysign(max_angular_twist_delta, delta_twist.angular_y);
        }
        // same for angular_z
        if (fabs(delta_twist.angular_z) < fabs(max_angular_twist_delta))
        {
            twist_command.angular_z = m_twist_command.angular_z;
        }
        else
        {
            twist_command.angular_z = previous_twist_command.angular_z + std::copysign(max_angular_twist_delta, delta_twist.angular_z);
        }

        // Cap to the velocity limit
        twist_command.linear_x = std::copysign(std::min(fabs(twist_command.linear_x), fabs(m_max_cartesian_twist_linear)), twist_command.linear_x);
        twist_command.linear_y = std::copysign(std::min(fabs(twist_command.linear_y), fabs(m_max_cartesian_twist_linear)), twist_command.linear_y);
        twist_command.linear_z = std::copysign(std::min(fabs(twist_command.linear_z), fabs(m_max_cartesian_twist_linear)), twist_command.linear_z);
        twist_command.angular_x = std::copysign(std::min(fabs(twist_command.angular_x), fabs(m_max_cartesian_twist_angular)), twist_command.angular_x);
        twist_command.angular_y = std::copysign(std::min(fabs(twist_command.angular_y), fabs(m_max_cartesian_twist_angular)), twist_command.angular_y);
        twist_command.angular_z = std::copysign(std::min(fabs(twist_command.angular_z), fabs(m_max_cartesian_twist_angular)), twist_command.angular_z);

        // Fill current joint position commands KDL structure
        KDL::JntArray commands_kdl(GetDOF());
        Eigen::VectorXd commands_eigen(GetDOF());
        for (int i = 0; i < GetDOF(); i++)
        {
            commands_eigen[i] = commands[i];
        }
        commands_kdl.data = commands_eigen;
        
        // Fill KDL Twist structure with Kortex twist
        KDL::Twist twist_kdl = KDL::Twist(KDL::Vector(twist_command.linear_x, twist_command.linear_y, twist_command.linear_z),
                                            KDL::Vector(twist_command.angular_x, twist_command.angular_y, twist_command.angular_z));
        
        // Call IK and fill joint velocity commands
        KDL::JntArray joint_velocities(GetDOF());
        int ik_result = m_ik_vel_solver->CartToJnt(commands_kdl, twist_kdl, joint_velocities);
        if (ik_result != m_ik_vel_solver->E_NOERROR)
        {
            RCLCPP_WARN(m_node_handle->get_logger(), "IK ERROR = %d", ik_result);
        }
        
        // We need to know if the joint velocities have to be adjusted, and by what ratio
        double ratio = 1.0;
        for (int i = 0; i < GetDOF(); i++)
        {
            // Calculate permitted velocity command because we don't have infinite acceleration
            double vel_delta = joint_velocities(i) - previous_velocity_commands[i];
            double max_delta = std::copysign(m_arm_acceleration_max_limits[i] * JOINT_TRAJECTORY_TIMESTEP_SECONDS, vel_delta);
            
            // If we cannot instantly accelerate to this velocity
            if (fabs(vel_delta) > fabs(max_delta))
            {
                ratio = std::max(ratio, fabs(vel_delta / max_delta));
            }
        }

        // Command the velocities
        // For each joint
        for (int i = 0; i < GetDOF(); i++)
        {
            // Calculate position increment
            commands[i] = previous_commands[i] + joint_velocities(i) / ratio * JOINT_TRAJECTORY_TIMESTEP_SECONDS;

            // Cap the command to the joint limit
            if (m_arm_joint_limits_min[i] != 0.0 && commands[i] < 0.0)
            {
                commands[i] = std::max(m_arm_joint_limits_min[i], commands[i]);
            }
            else if (m_arm_joint_limits_max[i] != 0.0 && commands[i] > 0.0)
            {
                commands[i] = std::min(m_arm_joint_limits_max[i], commands[i]);
            }
            
            // Send the position increments to the controllers
            std_msgs::msg::Float64 message;
            message.data = commands[i];
            m_pub_position_controllers[i]->publish(message);

            // Check if joint is stopped or not
            stopped[i] = fabs(joint_velocities(i)) < MINIMUM_JOINT_VELOCITY_RAD_PER_SECONDS;

            // Remember actual command as previous
            previous_commands[i] = commands[i];
            previous_velocity_commands[i] = joint_velocities(i);
        }

        // Put current values to previous
        previous_twist_command = twist_command;

        // Sleep for TIMESTEP
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000*JOINT_TRAJECTORY_TIMESTEP_SECONDS)));

        // If the action is preempted and we're back to zero velocity, we're done here
        if (m_action_preempted.load())
        {
            // Check if all joints are stopped and break if yes
            if (std::all_of(stopped.begin(), stopped.end(), [](bool b){return b;}))
            {
                break;
            }
        }
    }

    return result;
}

kortex_driver::msg::KortexError KortexArmSimulation::ExecuteSendGripperCommand(const kortex_driver::msg::Action& action)
{
    kortex_driver::msg::KortexError result;
    result.code = kortex_driver::msg::ErrorCodes::ERROR_NONE;
    result.sub_code = kortex_driver::msg::SubErrorCodes::SUB_ERROR_NONE;
    if (action.oneof_action_parameters.send_gripper_command.size() != 1)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing gripper command action : action is malformed.");
    }
    auto gripper_command = action.oneof_action_parameters.send_gripper_command[0];

    if (gripper_command.gripper.finger.size() != 1)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                kortex_driver::msg::SubErrorCodes::INVALID_PARAM,
                                "Error playing gripper command action : there must be exactly one finger");
    }

    if (gripper_command.mode != kortex_driver::msg::GripperMode::GRIPPER_POSITION)
    {
        return FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                            kortex_driver::msg::SubErrorCodes::UNSUPPORTED_ACTION,
                            "Error playing gripper command action : gripper mode " + std::to_string(gripper_command.mode) + " is not supported; only position is.");
    }

    // The incoming command is relative [0,1] and we need to put it in absolute unit [m_gripper_joint_limits_min[0], m_gripper_joint_limits_max[0]]:
    double absolute_gripper_command = m_math_util.absolute_position_from_relative(gripper_command.gripper.finger[0].value, m_gripper_joint_limits_min[0], m_gripper_joint_limits_max[0]);

    // Create the goal
    control_msgs::action::GripperCommand::Goal goal;
    goal.command.position = absolute_gripper_command;

    // Verify if goal has been cancelled before sending it
    if (m_action_preempted.load())
    {
        return result;
    }

    // Send goal
    auto action_result = m_gripper_action_client->async_send_goal(goal);

    // Wait for goal to be done, or for preempt to be called (check every 10ms)
    while(!m_action_preempted.load() && 
        rclcpp::spin_until_future_complete(m_node_handle, action_result, std::chrono::duration<double>(0.01f)) != rclcpp::FutureReturnCode::SUCCESS) {}

    // If we got out of the loop because we're preempted, cancel the goal before returning
    if (m_action_preempted.load())
    {
        m_gripper_action_client->async_cancel_all_goals();
    }
    // Fill result depending on action final status if user didn't cancel
    else
    {
        auto status = m_gripper_action_client->async_get_result(action_result.get()).get().result;
        
        if (!status->reached_goal)
        {
            result = FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_DEVICE,
                                        kortex_driver::msg::SubErrorCodes::METHOD_FAILED,
                                        "The gripper command failed during execution.");
        }
    }
    return result;
}

kortex_driver::msg::KortexError KortexArmSimulation::ExecuteTimeDelay(const kortex_driver::msg::Action& action)
{
    auto result = FillKortexError(kortex_driver::msg::ErrorCodes::ERROR_NONE,
                                kortex_driver::msg::SubErrorCodes::SUB_ERROR_NONE);
    if (!action.oneof_action_parameters.delay.empty())
    {
        auto start = std::chrono::system_clock::now();
        uint32_t delay_seconds = action.oneof_action_parameters.delay[0].duration;
        // While not preempted and duration not elapsed
        while (!m_action_preempted.load() && (std::chrono::system_clock::now() - start) < std::chrono::seconds(delay_seconds))
        {
            // sleep a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    else
    {
        result.code = kortex_driver::msg::ErrorCodes::ERROR_DEVICE;
        result.sub_code = kortex_driver::msg::SubErrorCodes::INVALID_PARAM;
        result.description = "Error playing time delay action : action is malformed.";
    }
    return result;
}

void KortexArmSimulation::new_joint_speeds_cb(const std::shared_ptr<kortex_driver::msg::BaseJointSpeeds> joint_speeds)
{
    auto req = std::make_shared<kortex_driver::srv::SendJointSpeedsCommand::Request>();
    req->input = *joint_speeds;
    SendJointSpeedsCommand(req);
}

void KortexArmSimulation::new_twist_cb(const std::shared_ptr<kortex_driver::msg::TwistCommand> twist)
{
    // TODO Implement
}

void KortexArmSimulation::clear_faults_cb(const std::shared_ptr<std_msgs::msg::Empty> empty)
{
    // does nothing
}

void KortexArmSimulation::stop_cb(const std::shared_ptr<std_msgs::msg::Empty> empty)
{
    Stop(std::make_shared<kortex_driver::srv::Stop::Request>());
}

void KortexArmSimulation::emergency_stop_cb(const std::shared_ptr<std_msgs::msg::Empty> empty)
{
    ApplyEmergencyStop(std::make_shared<kortex_driver::srv::ApplyEmergencyStop::Request>());
}
