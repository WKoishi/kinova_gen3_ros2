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

#include "kortex_driver/non-generated/kortex_subscribers.h"

KortexSubscribers::KortexSubscribers(rclcpp::Node::SharedPtr node_handle, Kinova::Api::Base::BaseClient* base):
m_node_handle(node_handle), m_base(base)
{
    std::string robot_name;
    m_node_handle->get_parameter("~robot_name", robot_name);
    m_joint_speeds_sub = m_node_handle->create_subscription<kortex_driver::msg::BaseJointSpeeds>("in/joint_velocity", 10, std::bind(&KortexSubscribers::new_joint_speeds_cb, this, std::placeholders::_1));
    m_twist_sub = m_node_handle->create_subscription<kortex_driver::msg::TwistCommand>("in/cartesian_velocity", 10, std::bind(&KortexSubscribers::new_twist_cb, this, std::placeholders::_1));
    m_clear_faults_sub = m_node_handle->create_subscription<std_msgs::msg::Empty>("in/clear_faults", 10, std::bind(&KortexSubscribers::clear_faults_cb, this, std::placeholders::_1));
    m_stop_sub = m_node_handle->create_subscription<std_msgs::msg::Empty>("in/stop", 10, std::bind(&KortexSubscribers::stop_cb, this, std::placeholders::_1));
    m_emergency_stop_sub = m_node_handle->create_subscription<std_msgs::msg::Empty>("in/emergency_stop", 10, std::bind(&KortexSubscribers::emergency_stop_cb, this, std::placeholders::_1));
}

KortexSubscribers::~KortexSubscribers()
{
}

void KortexSubscribers::new_joint_speeds_cb(const std::shared_ptr<kortex_driver::msg::BaseJointSpeeds> joint_speeds)
{
    Kinova::Api::Base::JointSpeeds speeds;
    kortex_driver::msg::BaseJointSpeeds joint_speeds_in_rad(*joint_speeds); // Since joint_speeds is const we need this copy

    // Convert radians in degrees
    for (unsigned int i = 0; i < joint_speeds->joint_speeds.size(); i++)
    {
        joint_speeds_in_rad.joint_speeds[i].value = KortexMathUtil::toDeg(joint_speeds->joint_speeds[i].value);
    }
    ToProtoData(joint_speeds_in_rad, &speeds);

    try
    {
        m_base->SendJointSpeedsCommand(speeds);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Kortex exception while sending joint speeds");
        RCLCPP_WARN(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_DEBUG(m_node_handle->get_logger(), "Runtime exception detected while sending joint speeds!");
        RCLCPP_DEBUG(m_node_handle->get_logger(), "%s", ex_runtime.what());
    }
}

void KortexSubscribers::new_twist_cb(const std::shared_ptr<kortex_driver::msg::TwistCommand> twist)
{
    Kinova::Api::Base::TwistCommand twist_command;
    ToProtoData(*twist, &twist_command);

    // Convert radians to degrees
    twist_command.mutable_twist()->set_angular_x(KortexMathUtil::toDeg(twist_command.twist().angular_x()));
    twist_command.mutable_twist()->set_angular_y(KortexMathUtil::toDeg(twist_command.twist().angular_y()));
    twist_command.mutable_twist()->set_angular_z(KortexMathUtil::toDeg(twist_command.twist().angular_z()));

    try
    {
        m_base->SendTwistCommand(twist_command);
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Kortex exception while sending twist command");
        RCLCPP_WARN(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_DEBUG(m_node_handle->get_logger(), "Runtime exception detected while sending twist command!");
        RCLCPP_DEBUG(m_node_handle->get_logger(), "%s", ex_runtime.what());
    }
}

void KortexSubscribers::clear_faults_cb(const std::shared_ptr<std_msgs::msg::Empty> dummy)
{
    try
    {
        m_base->ClearFaults();
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Kortex exception while clearing the faults");
        RCLCPP_WARN(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_DEBUG(m_node_handle->get_logger(), "Runtime exception detected while clearing the faults!");
        RCLCPP_DEBUG(m_node_handle->get_logger(), "%s", ex_runtime.what());
    }
}

void KortexSubscribers::stop_cb(const std::shared_ptr<std_msgs::msg::Empty> dummy)
{
    try
    {
        m_base->Stop();
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Kortex exception while clearing the faults");
        RCLCPP_WARN(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_DEBUG(m_node_handle->get_logger(), "Runtime exception detected while clearing the faults!");
        RCLCPP_DEBUG(m_node_handle->get_logger(), "%s", ex_runtime.what());
    }
}

void KortexSubscribers::emergency_stop_cb(const std::shared_ptr<std_msgs::msg::Empty> dummy)
{
    try
    {
        m_base->ApplyEmergencyStop();
    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        RCLCPP_WARN(m_node_handle->get_logger(), "Kortex exception while clearing the faults");
        RCLCPP_WARN(m_node_handle->get_logger(), "Error code: %s\n", Kinova::Api::ErrorCodes_Name(ex.getErrorInfo().getError().error_code()).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error sub code: %s\n", Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())).c_str());
        RCLCPP_WARN(m_node_handle->get_logger(), "Error description: %s\n", ex.what());
    }
    catch (std::runtime_error& ex_runtime)
    {
        RCLCPP_DEBUG(m_node_handle->get_logger(), "Runtime exception detected while clearing the faults!");
        RCLCPP_DEBUG(m_node_handle->get_logger(), "%s", ex_runtime.what());
    }
}

