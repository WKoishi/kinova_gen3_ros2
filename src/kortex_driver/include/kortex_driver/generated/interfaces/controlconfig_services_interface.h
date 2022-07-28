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

/*
 * This file has been auto-generated and should not be modified.
 */
 
#ifndef _KORTEX_CONTROLCONFIG_SERVICES_INTERFACE_H_
#define _KORTEX_CONTROLCONFIG_SERVICES_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/srv/set_gravity_vector.hpp"
#include "kortex_driver/srv/get_gravity_vector.hpp"
#include "kortex_driver/srv/set_payload_information.hpp"
#include "kortex_driver/srv/get_payload_information.hpp"
#include "kortex_driver/srv/set_tool_configuration.hpp"
#include "kortex_driver/srv/get_tool_configuration.hpp"
#include "kortex_driver/srv/on_notification_control_configuration_topic.hpp"
#include "kortex_driver/msg/control_configuration_notification.hpp"
#include "kortex_driver/srv/control_config_unsubscribe.hpp"
#include "kortex_driver/srv/set_cartesian_reference_frame.hpp"
#include "kortex_driver/srv/get_cartesian_reference_frame.hpp"
#include "kortex_driver/srv/control_config_get_control_mode.hpp"
#include "kortex_driver/srv/set_joint_speed_soft_limits.hpp"
#include "kortex_driver/srv/set_twist_linear_soft_limit.hpp"
#include "kortex_driver/srv/set_twist_angular_soft_limit.hpp"
#include "kortex_driver/srv/set_joint_acceleration_soft_limits.hpp"
#include "kortex_driver/srv/get_kinematic_hard_limits.hpp"
#include "kortex_driver/srv/get_kinematic_soft_limits.hpp"
#include "kortex_driver/srv/get_all_kinematic_soft_limits.hpp"
#include "kortex_driver/srv/set_desired_linear_twist.hpp"
#include "kortex_driver/srv/set_desired_angular_twist.hpp"
#include "kortex_driver/srv/set_desired_joint_speeds.hpp"
#include "kortex_driver/srv/get_desired_speeds.hpp"
#include "kortex_driver/srv/reset_gravity_vector.hpp"
#include "kortex_driver/srv/reset_payload_information.hpp"
#include "kortex_driver/srv/reset_tool_configuration.hpp"
#include "kortex_driver/srv/reset_joint_speed_soft_limits.hpp"
#include "kortex_driver/srv/reset_twist_linear_soft_limit.hpp"
#include "kortex_driver/srv/reset_twist_angular_soft_limit.hpp"
#include "kortex_driver/srv/reset_joint_acceleration_soft_limits.hpp"
#include "kortex_driver/srv/control_config_on_notification_control_mode_topic.hpp"
#include "kortex_driver/msg/control_config_control_mode_notification.hpp"

#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/srv/set_device_id.hpp"
#include "kortex_driver/srv/set_api_options.hpp"
#include "kortex_driver/msg/api_options.hpp"

using namespace std;

class IControlConfigServices
{
    public:
        IControlConfigServices(rclcpp::Node::SharedPtr node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) = 0;
        virtual bool SetGravityVector(kortex_driver::srv::SetGravityVector::Request  &req, kortex_driver::srv::SetGravityVector::Response &res) = 0;
        virtual bool GetGravityVector(kortex_driver::srv::GetGravityVector::Request  &req, kortex_driver::srv::GetGravityVector::Response &res) = 0;
        virtual bool SetPayloadInformation(kortex_driver::srv::SetPayloadInformation::Request  &req, kortex_driver::srv::SetPayloadInformation::Response &res) = 0;
        virtual bool GetPayloadInformation(kortex_driver::srv::GetPayloadInformation::Request  &req, kortex_driver::srv::GetPayloadInformation::Response &res) = 0;
        virtual bool SetToolConfiguration(kortex_driver::srv::SetToolConfiguration::Request  &req, kortex_driver::srv::SetToolConfiguration::Response &res) = 0;
        virtual bool GetToolConfiguration(kortex_driver::srv::GetToolConfiguration::Request  &req, kortex_driver::srv::GetToolConfiguration::Response &res) = 0;
        virtual bool OnNotificationControlConfigurationTopic(kortex_driver::srv::OnNotificationControlConfigurationTopic::Request  &req, kortex_driver::srv::OnNotificationControlConfigurationTopic::Response &res) = 0;
        virtual void cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif) = 0;
        virtual bool ControlConfig_Unsubscribe(kortex_driver::srv::ControlConfigUnsubscribe::Request  &req, kortex_driver::srv::ControlConfigUnsubscribe::Response &res) = 0;
        virtual bool SetCartesianReferenceFrame(kortex_driver::srv::SetCartesianReferenceFrame::Request  &req, kortex_driver::srv::SetCartesianReferenceFrame::Response &res) = 0;
        virtual bool GetCartesianReferenceFrame(kortex_driver::srv::GetCartesianReferenceFrame::Request  &req, kortex_driver::srv::GetCartesianReferenceFrame::Response &res) = 0;
        virtual bool ControlConfig_GetControlMode(kortex_driver::srv::ControlConfigGetControlMode::Request  &req, kortex_driver::srv::ControlConfigGetControlMode::Response &res) = 0;
        virtual bool SetJointSpeedSoftLimits(kortex_driver::srv::SetJointSpeedSoftLimits::Request  &req, kortex_driver::srv::SetJointSpeedSoftLimits::Response &res) = 0;
        virtual bool SetTwistLinearSoftLimit(kortex_driver::srv::SetTwistLinearSoftLimit::Request  &req, kortex_driver::srv::SetTwistLinearSoftLimit::Response &res) = 0;
        virtual bool SetTwistAngularSoftLimit(kortex_driver::srv::SetTwistAngularSoftLimit::Request  &req, kortex_driver::srv::SetTwistAngularSoftLimit::Response &res) = 0;
        virtual bool SetJointAccelerationSoftLimits(kortex_driver::srv::SetJointAccelerationSoftLimits::Request  &req, kortex_driver::srv::SetJointAccelerationSoftLimits::Response &res) = 0;
        virtual bool GetKinematicHardLimits(kortex_driver::srv::GetKinematicHardLimits::Request  &req, kortex_driver::srv::GetKinematicHardLimits::Response &res) = 0;
        virtual bool GetKinematicSoftLimits(kortex_driver::srv::GetKinematicSoftLimits::Request  &req, kortex_driver::srv::GetKinematicSoftLimits::Response &res) = 0;
        virtual bool GetAllKinematicSoftLimits(kortex_driver::srv::GetAllKinematicSoftLimits::Request  &req, kortex_driver::srv::GetAllKinematicSoftLimits::Response &res) = 0;
        virtual bool SetDesiredLinearTwist(kortex_driver::srv::SetDesiredLinearTwist::Request  &req, kortex_driver::srv::SetDesiredLinearTwist::Response &res) = 0;
        virtual bool SetDesiredAngularTwist(kortex_driver::srv::SetDesiredAngularTwist::Request  &req, kortex_driver::srv::SetDesiredAngularTwist::Response &res) = 0;
        virtual bool SetDesiredJointSpeeds(kortex_driver::srv::SetDesiredJointSpeeds::Request  &req, kortex_driver::srv::SetDesiredJointSpeeds::Response &res) = 0;
        virtual bool GetDesiredSpeeds(kortex_driver::srv::GetDesiredSpeeds::Request  &req, kortex_driver::srv::GetDesiredSpeeds::Response &res) = 0;
        virtual bool ResetGravityVector(kortex_driver::srv::ResetGravityVector::Request  &req, kortex_driver::srv::ResetGravityVector::Response &res) = 0;
        virtual bool ResetPayloadInformation(kortex_driver::srv::ResetPayloadInformation::Request  &req, kortex_driver::srv::ResetPayloadInformation::Response &res) = 0;
        virtual bool ResetToolConfiguration(kortex_driver::srv::ResetToolConfiguration::Request  &req, kortex_driver::srv::ResetToolConfiguration::Response &res) = 0;
        virtual bool ResetJointSpeedSoftLimits(kortex_driver::srv::ResetJointSpeedSoftLimits::Request  &req, kortex_driver::srv::ResetJointSpeedSoftLimits::Response &res) = 0;
        virtual bool ResetTwistLinearSoftLimit(kortex_driver::srv::ResetTwistLinearSoftLimit::Request  &req, kortex_driver::srv::ResetTwistLinearSoftLimit::Response &res) = 0;
        virtual bool ResetTwistAngularSoftLimit(kortex_driver::srv::ResetTwistAngularSoftLimit::Request  &req, kortex_driver::srv::ResetTwistAngularSoftLimit::Response &res) = 0;
        virtual bool ResetJointAccelerationSoftLimits(kortex_driver::srv::ResetJointAccelerationSoftLimits::Request  &req, kortex_driver::srv::ResetJointAccelerationSoftLimits::Response &res) = 0;
        virtual bool ControlConfig_OnNotificationControlModeTopic(kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request  &req, kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response &res) = 0;
        virtual void cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif) = 0;

protected:
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp::Publisher<kortex_driver::msg::KortexError>::SharedPtr m_pub_Error;
        rclcpp::Publisher<kortex_driver::msg::ControlConfigurationNotification>::SharedPtr m_pub_ControlConfigurationTopic;
        bool m_is_activated_ControlConfigurationTopic;
        rclcpp::Publisher<kortex_driver::msg::ControlConfigControlModeNotification>::SharedPtr m_pub_ControlModeTopic;
        bool m_is_activated_ControlModeTopic;

        rclcpp::Service<kortex_driver::srv::SetDeviceID>::SharedPtr m_serviceSetDeviceID;
        rclcpp::Service<kortex_driver::srv::SetApiOptions>::SharedPtr m_serviceSetApiOptions;

	rclcpp::Service<kortex_driver::srv::SetGravityVector>::SharedPtr m_serviceSetGravityVector;
	rclcpp::Service<kortex_driver::srv::GetGravityVector>::SharedPtr m_serviceGetGravityVector;
	rclcpp::Service<kortex_driver::srv::SetPayloadInformation>::SharedPtr m_serviceSetPayloadInformation;
	rclcpp::Service<kortex_driver::srv::GetPayloadInformation>::SharedPtr m_serviceGetPayloadInformation;
	rclcpp::Service<kortex_driver::srv::SetToolConfiguration>::SharedPtr m_serviceSetToolConfiguration;
	rclcpp::Service<kortex_driver::srv::GetToolConfiguration>::SharedPtr m_serviceGetToolConfiguration;
	rclcpp::Service<kortex_driver::srv::OnNotificationControlConfigurationTopic>::SharedPtr m_serviceOnNotificationControlConfigurationTopic;
	rclcpp::Service<kortex_driver::srv::ControlConfigUnsubscribe>::SharedPtr m_serviceControlConfig_Unsubscribe;
	rclcpp::Service<kortex_driver::srv::SetCartesianReferenceFrame>::SharedPtr m_serviceSetCartesianReferenceFrame;
	rclcpp::Service<kortex_driver::srv::GetCartesianReferenceFrame>::SharedPtr m_serviceGetCartesianReferenceFrame;
	rclcpp::Service<kortex_driver::srv::ControlConfigGetControlMode>::SharedPtr m_serviceControlConfig_GetControlMode;
	rclcpp::Service<kortex_driver::srv::SetJointSpeedSoftLimits>::SharedPtr m_serviceSetJointSpeedSoftLimits;
	rclcpp::Service<kortex_driver::srv::SetTwistLinearSoftLimit>::SharedPtr m_serviceSetTwistLinearSoftLimit;
	rclcpp::Service<kortex_driver::srv::SetTwistAngularSoftLimit>::SharedPtr m_serviceSetTwistAngularSoftLimit;
	rclcpp::Service<kortex_driver::srv::SetJointAccelerationSoftLimits>::SharedPtr m_serviceSetJointAccelerationSoftLimits;
	rclcpp::Service<kortex_driver::srv::GetKinematicHardLimits>::SharedPtr m_serviceGetKinematicHardLimits;
	rclcpp::Service<kortex_driver::srv::GetKinematicSoftLimits>::SharedPtr m_serviceGetKinematicSoftLimits;
	rclcpp::Service<kortex_driver::srv::GetAllKinematicSoftLimits>::SharedPtr m_serviceGetAllKinematicSoftLimits;
	rclcpp::Service<kortex_driver::srv::SetDesiredLinearTwist>::SharedPtr m_serviceSetDesiredLinearTwist;
	rclcpp::Service<kortex_driver::srv::SetDesiredAngularTwist>::SharedPtr m_serviceSetDesiredAngularTwist;
	rclcpp::Service<kortex_driver::srv::SetDesiredJointSpeeds>::SharedPtr m_serviceSetDesiredJointSpeeds;
	rclcpp::Service<kortex_driver::srv::GetDesiredSpeeds>::SharedPtr m_serviceGetDesiredSpeeds;
	rclcpp::Service<kortex_driver::srv::ResetGravityVector>::SharedPtr m_serviceResetGravityVector;
	rclcpp::Service<kortex_driver::srv::ResetPayloadInformation>::SharedPtr m_serviceResetPayloadInformation;
	rclcpp::Service<kortex_driver::srv::ResetToolConfiguration>::SharedPtr m_serviceResetToolConfiguration;
	rclcpp::Service<kortex_driver::srv::ResetJointSpeedSoftLimits>::SharedPtr m_serviceResetJointSpeedSoftLimits;
	rclcpp::Service<kortex_driver::srv::ResetTwistLinearSoftLimit>::SharedPtr m_serviceResetTwistLinearSoftLimit;
	rclcpp::Service<kortex_driver::srv::ResetTwistAngularSoftLimit>::SharedPtr m_serviceResetTwistAngularSoftLimit;
	rclcpp::Service<kortex_driver::srv::ResetJointAccelerationSoftLimits>::SharedPtr m_serviceResetJointAccelerationSoftLimits;
	rclcpp::Service<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic>::SharedPtr m_serviceControlConfig_OnNotificationControlModeTopic;
};
#endif
