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
 
#include "kortex_driver/generated/robot/common_proto_converter.h"
#include "kortex_driver/generated/robot/common_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorconfig_ros_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/actuatorcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_proto_converter.h"
#include "kortex_driver/generated/robot/productconfiguration_ros_converter.h"
#include "kortex_driver/generated/robot/base_proto_converter.h"
#include "kortex_driver/generated/robot/base_ros_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_proto_converter.h"
#include "kortex_driver/generated/robot/grippercyclic_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectcyclic_ros_converter.h"
#include "kortex_driver/generated/robot/basecyclic_proto_converter.h"
#include "kortex_driver/generated/robot/basecyclic_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_proto_converter.h"
#include "kortex_driver/generated/robot/controlconfig_ros_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_proto_converter.h"
#include "kortex_driver/generated/robot/deviceconfig_ros_converter.h"
#include "kortex_driver/generated/robot/devicemanager_proto_converter.h"
#include "kortex_driver/generated/robot/devicemanager_ros_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_proto_converter.h"
#include "kortex_driver/generated/robot/interconnectconfig_ros_converter.h"
#include "kortex_driver/generated/robot/visionconfig_proto_converter.h"
#include "kortex_driver/generated/robot/visionconfig_ros_converter.h"
#include "kortex_driver/generated/robot/controlconfig_services.h"

ControlConfigRobotServices::ControlConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::ControlConfig::ControlConfigClient* controlconfig, uint32_t device_id, uint32_t timeout_ms): 
	IControlConfigServices(node_handle),
	m_controlconfig(controlconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle->create_publisher<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_ControlConfigurationTopic = m_node_handle->create_publisher<kortex_driver::msg::ControlConfigurationNotification>("control_configuration_topic", 1000);
	m_is_activated_ControlConfigurationTopic = false;
	m_pub_ControlModeTopic = m_node_handle->create_publisher<kortex_driver::msg::ControlConfigControlModeNotification>("control_mode_topic", 1000);
	m_is_activated_ControlModeTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("control_config/set_device_id", std::bind(&ControlConfigRobotServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("control_config/set_api_options", std::bind(&ControlConfigRobotServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceSetGravityVector = m_node_handle->create_service<kortex_driver::srv::SetGravityVector>("control_config/set_gravity_vector", std::bind(&ControlConfigRobotServices::SetGravityVector, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetGravityVector = m_node_handle->create_service<kortex_driver::srv::GetGravityVector>("control_config/get_gravity_vector", std::bind(&ControlConfigRobotServices::GetGravityVector, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetPayloadInformation = m_node_handle->create_service<kortex_driver::srv::SetPayloadInformation>("control_config/set_payload_information", std::bind(&ControlConfigRobotServices::SetPayloadInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetPayloadInformation = m_node_handle->create_service<kortex_driver::srv::GetPayloadInformation>("control_config/get_payload_information", std::bind(&ControlConfigRobotServices::GetPayloadInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetToolConfiguration = m_node_handle->create_service<kortex_driver::srv::SetToolConfiguration>("control_config/set_tool_configuration", std::bind(&ControlConfigRobotServices::SetToolConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetToolConfiguration = m_node_handle->create_service<kortex_driver::srv::GetToolConfiguration>("control_config/get_tool_configuration", std::bind(&ControlConfigRobotServices::GetToolConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationControlConfigurationTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationControlConfigurationTopic>("control_config/activate_publishing_of_control_configuration_topic", std::bind(&ControlConfigRobotServices::OnNotificationControlConfigurationTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceControlConfig_Unsubscribe = m_node_handle->create_service<kortex_driver::srv::ControlConfigUnsubscribe>("control_config/unsubscribe", std::bind(&ControlConfigRobotServices::ControlConfig_Unsubscribe, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetCartesianReferenceFrame = m_node_handle->create_service<kortex_driver::srv::SetCartesianReferenceFrame>("control_config/set_cartesian_reference_frame", std::bind(&ControlConfigRobotServices::SetCartesianReferenceFrame, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetCartesianReferenceFrame = m_node_handle->create_service<kortex_driver::srv::GetCartesianReferenceFrame>("control_config/get_cartesian_reference_frame", std::bind(&ControlConfigRobotServices::GetCartesianReferenceFrame, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceControlConfig_GetControlMode = m_node_handle->create_service<kortex_driver::srv::ControlConfigGetControlMode>("control_config/get_control_mode", std::bind(&ControlConfigRobotServices::ControlConfig_GetControlMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetJointSpeedSoftLimits = m_node_handle->create_service<kortex_driver::srv::SetJointSpeedSoftLimits>("control_config/set_joint_speed_soft_limits", std::bind(&ControlConfigRobotServices::SetJointSpeedSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetTwistLinearSoftLimit = m_node_handle->create_service<kortex_driver::srv::SetTwistLinearSoftLimit>("control_config/set_twist_linear_soft_limit", std::bind(&ControlConfigRobotServices::SetTwistLinearSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetTwistAngularSoftLimit = m_node_handle->create_service<kortex_driver::srv::SetTwistAngularSoftLimit>("control_config/set_twist_angular_soft_limit", std::bind(&ControlConfigRobotServices::SetTwistAngularSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetJointAccelerationSoftLimits = m_node_handle->create_service<kortex_driver::srv::SetJointAccelerationSoftLimits>("control_config/set_joint_acceleration_soft_limits", std::bind(&ControlConfigRobotServices::SetJointAccelerationSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetKinematicHardLimits = m_node_handle->create_service<kortex_driver::srv::GetKinematicHardLimits>("control_config/get_kinematic_hard_limits", std::bind(&ControlConfigRobotServices::GetKinematicHardLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetKinematicSoftLimits = m_node_handle->create_service<kortex_driver::srv::GetKinematicSoftLimits>("control_config/get_kinematic_soft_limits", std::bind(&ControlConfigRobotServices::GetKinematicSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllKinematicSoftLimits = m_node_handle->create_service<kortex_driver::srv::GetAllKinematicSoftLimits>("control_config/get_all_kinematic_soft_limits", std::bind(&ControlConfigRobotServices::GetAllKinematicSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetDesiredLinearTwist = m_node_handle->create_service<kortex_driver::srv::SetDesiredLinearTwist>("control_config/set_desired_linear_twist", std::bind(&ControlConfigRobotServices::SetDesiredLinearTwist, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetDesiredAngularTwist = m_node_handle->create_service<kortex_driver::srv::SetDesiredAngularTwist>("control_config/set_desired_angular_twist", std::bind(&ControlConfigRobotServices::SetDesiredAngularTwist, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetDesiredJointSpeeds = m_node_handle->create_service<kortex_driver::srv::SetDesiredJointSpeeds>("control_config/set_desired_joint_speeds", std::bind(&ControlConfigRobotServices::SetDesiredJointSpeeds, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetDesiredSpeeds = m_node_handle->create_service<kortex_driver::srv::GetDesiredSpeeds>("control_config/get_desired_speeds", std::bind(&ControlConfigRobotServices::GetDesiredSpeeds, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetGravityVector = m_node_handle->create_service<kortex_driver::srv::ResetGravityVector>("control_config/reset_gravity_vector", std::bind(&ControlConfigRobotServices::ResetGravityVector, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetPayloadInformation = m_node_handle->create_service<kortex_driver::srv::ResetPayloadInformation>("control_config/reset_payload_information", std::bind(&ControlConfigRobotServices::ResetPayloadInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetToolConfiguration = m_node_handle->create_service<kortex_driver::srv::ResetToolConfiguration>("control_config/reset_tool_configuration", std::bind(&ControlConfigRobotServices::ResetToolConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetJointSpeedSoftLimits = m_node_handle->create_service<kortex_driver::srv::ResetJointSpeedSoftLimits>("control_config/reset_joint_speed_soft_limits", std::bind(&ControlConfigRobotServices::ResetJointSpeedSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetTwistLinearSoftLimit = m_node_handle->create_service<kortex_driver::srv::ResetTwistLinearSoftLimit>("control_config/reset_twist_linear_soft_limit", std::bind(&ControlConfigRobotServices::ResetTwistLinearSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetTwistAngularSoftLimit = m_node_handle->create_service<kortex_driver::srv::ResetTwistAngularSoftLimit>("control_config/reset_twist_angular_soft_limit", std::bind(&ControlConfigRobotServices::ResetTwistAngularSoftLimit, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetJointAccelerationSoftLimits = m_node_handle->create_service<kortex_driver::srv::ResetJointAccelerationSoftLimits>("control_config/reset_joint_acceleration_soft_limits", std::bind(&ControlConfigRobotServices::ResetJointAccelerationSoftLimits, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceControlConfig_OnNotificationControlModeTopic = m_node_handle->create_service<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic>("control_config/activate_publishing_of_control_mode_topic", std::bind(&ControlConfigRobotServices::ControlConfig_OnNotificationControlModeTopic, this, std::placeholders::_1, std::placeholders::_2));
}

bool ControlConfigRobotServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	m_current_device_id = req->device_id;

	return true;
}

bool ControlConfigRobotServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	m_api_options.timeout_ms = req->input.timeout_ms;

	return true;
}


bool ControlConfigRobotServices::SetGravityVector(const std::shared_ptr<kortex_driver::srv::SetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::SetGravityVector::Response> res)
{
	
	Kinova::Api::ControlConfig::GravityVector input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetGravityVector(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::GetGravityVector(const std::shared_ptr<kortex_driver::srv::GetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::GetGravityVector::Response> res)
{
	
	Kinova::Api::ControlConfig::GravityVector output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetGravityVector(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::SetPayloadInformation(const std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::SetPayloadInformation::Response> res)
{
	
	Kinova::Api::ControlConfig::PayloadInformation input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetPayloadInformation(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::GetPayloadInformation(const std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetPayloadInformation::Response> res)
{
	
	Kinova::Api::ControlConfig::PayloadInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetPayloadInformation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::SetToolConfiguration(const std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetToolConfiguration::Response> res)
{
	
	Kinova::Api::ControlConfig::ToolConfiguration input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetToolConfiguration(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::GetToolConfiguration(const std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetToolConfiguration::Response> res)
{
	
	Kinova::Api::ControlConfig::ToolConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetToolConfiguration(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::OnNotificationControlConfigurationTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationControlConfigurationTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControlConfigurationTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::ControlConfig::ControlConfigurationNotification) > callback = std::bind(&ControlConfigRobotServices::cb_ControlConfigurationTopic, this, std::placeholders::_1);
		output = m_controlconfig->OnNotificationControlConfigurationTopic(callback, input, m_current_device_id);
		m_is_activated_ControlConfigurationTopic = true;
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}
void ControlConfigRobotServices::cb_ControlConfigurationTopic(Kinova::Api::ControlConfig::ControlConfigurationNotification notif)
{
	kortex_driver::msg::ControlConfigurationNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlConfigurationTopic->publish(ros_msg);
}

bool ControlConfigRobotServices::ControlConfig_Unsubscribe(const std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigUnsubscribe::Response> res)
{
	
	Kinova::Api::Common::NotificationHandle input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->Unsubscribe(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::SetCartesianReferenceFrame(const std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Request> req, std::shared_ptr<kortex_driver::srv::SetCartesianReferenceFrame::Response> res)
{
	
	Kinova::Api::ControlConfig::CartesianReferenceFrameInfo input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetCartesianReferenceFrame(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::GetCartesianReferenceFrame(const std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Request> req, std::shared_ptr<kortex_driver::srv::GetCartesianReferenceFrame::Response> res)
{
	
	Kinova::Api::ControlConfig::CartesianReferenceFrameInfo output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetCartesianReferenceFrame(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ControlConfig_GetControlMode(const std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigGetControlMode::Response> res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetControlMode(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::SetJointSpeedSoftLimits(const std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::SetJointSpeedSoftLimits::Response> res)
{
	
	Kinova::Api::ControlConfig::JointSpeedSoftLimits input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetJointSpeedSoftLimits(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::SetTwistLinearSoftLimit(const std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::SetTwistLinearSoftLimit::Response> res)
{
	
	Kinova::Api::ControlConfig::TwistLinearSoftLimit input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetTwistLinearSoftLimit(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::SetTwistAngularSoftLimit(const std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::SetTwistAngularSoftLimit::Response> res)
{
	
	Kinova::Api::ControlConfig::TwistAngularSoftLimit input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetTwistAngularSoftLimit(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::SetJointAccelerationSoftLimits(const std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::SetJointAccelerationSoftLimits::Response> res)
{
	
	Kinova::Api::ControlConfig::JointAccelerationSoftLimits input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetJointAccelerationSoftLimits(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::GetKinematicHardLimits(const std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetKinematicHardLimits::Response> res)
{
	
	Kinova::Api::ControlConfig::KinematicLimits output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetKinematicHardLimits(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::GetKinematicSoftLimits(const std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetKinematicSoftLimits::Response> res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req->input, &input);
	Kinova::Api::ControlConfig::KinematicLimits output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetKinematicSoftLimits(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::GetAllKinematicSoftLimits(const std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::GetAllKinematicSoftLimits::Response> res)
{
	
	Kinova::Api::ControlConfig::KinematicLimitsList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetAllKinematicSoftLimits(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::SetDesiredLinearTwist(const std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredLinearTwist::Response> res)
{
	
	Kinova::Api::ControlConfig::LinearTwist input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetDesiredLinearTwist(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::SetDesiredAngularTwist(const std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredAngularTwist::Response> res)
{
	
	Kinova::Api::ControlConfig::AngularTwist input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetDesiredAngularTwist(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::SetDesiredJointSpeeds(const std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Request> req, std::shared_ptr<kortex_driver::srv::SetDesiredJointSpeeds::Response> res)
{
	
	Kinova::Api::ControlConfig::JointSpeeds input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_controlconfig->SetDesiredJointSpeeds(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	return true;
}

bool ControlConfigRobotServices::GetDesiredSpeeds(const std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Request> req, std::shared_ptr<kortex_driver::srv::GetDesiredSpeeds::Response> res)
{
	
	Kinova::Api::ControlConfig::DesiredSpeeds output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->GetDesiredSpeeds(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ResetGravityVector(const std::shared_ptr<kortex_driver::srv::ResetGravityVector::Request> req, std::shared_ptr<kortex_driver::srv::ResetGravityVector::Response> res)
{
	
	Kinova::Api::ControlConfig::GravityVector output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetGravityVector(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ResetPayloadInformation(const std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Request> req, std::shared_ptr<kortex_driver::srv::ResetPayloadInformation::Response> res)
{
	
	Kinova::Api::ControlConfig::PayloadInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetPayloadInformation(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ResetToolConfiguration(const std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::ResetToolConfiguration::Response> res)
{
	
	Kinova::Api::ControlConfig::ToolConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetToolConfiguration(m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ResetJointSpeedSoftLimits(const std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::ResetJointSpeedSoftLimits::Response> res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req->input, &input);
	Kinova::Api::ControlConfig::JointSpeedSoftLimits output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetJointSpeedSoftLimits(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ResetTwistLinearSoftLimit(const std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::ResetTwistLinearSoftLimit::Response> res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req->input, &input);
	Kinova::Api::ControlConfig::TwistLinearSoftLimit output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetTwistLinearSoftLimit(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ResetTwistAngularSoftLimit(const std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Request> req, std::shared_ptr<kortex_driver::srv::ResetTwistAngularSoftLimit::Response> res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req->input, &input);
	Kinova::Api::ControlConfig::TwistAngularSoftLimit output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetTwistAngularSoftLimit(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ResetJointAccelerationSoftLimits(const std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Request> req, std::shared_ptr<kortex_driver::srv::ResetJointAccelerationSoftLimits::Response> res)
{
	
	Kinova::Api::ControlConfig::ControlModeInformation input;
	ToProtoData(req->input, &input);
	Kinova::Api::ControlConfig::JointAccelerationSoftLimits output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_controlconfig->ResetJointAccelerationSoftLimits(input, m_current_device_id, m_api_options);
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}

bool ControlConfigRobotServices::ControlConfig_OnNotificationControlModeTopic(const std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Request> req, std::shared_ptr<kortex_driver::srv::ControlConfigOnNotificationControlModeTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_ControlModeTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::ControlConfig::ControlModeNotification) > callback = std::bind(&ControlConfigRobotServices::cb_ControlModeTopic, this, std::placeholders::_1);
		output = m_controlconfig->OnNotificationControlModeTopic(callback, input, m_current_device_id);
		m_is_activated_ControlModeTopic = true;
	}

	catch (Kinova::Api::KDetailedException& ex)
	{
		result_error.sub_code = ex.getErrorInfo().getError().error_sub_code();
		result_error.code = ex.getErrorInfo().getError().error_code();
		result_error.description = ex.toString();
		m_pub_Error->publish(result_error);
		RCLCPP_INFO(m_node_handle->get_logger(), "Kortex exception");
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
		RCLCPP_INFO(m_node_handle->get_logger(), "KINOVA exception description: %s\n", ex.what());
		return false;
	}
	catch (std::runtime_error& ex2)
	{
		RCLCPP_INFO(m_node_handle->get_logger(), "%s", ex2.what());
		return false;
	}
	ToRosData(output, res->output);
	return true;
}
void ControlConfigRobotServices::cb_ControlModeTopic(Kinova::Api::ControlConfig::ControlModeNotification notif)
{
	kortex_driver::msg::ControlConfigControlModeNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_ControlModeTopic->publish(ros_msg);
}
