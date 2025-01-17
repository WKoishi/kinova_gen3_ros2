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
#include "kortex_driver/generated/robot/visionconfig_services.h"

VisionConfigRobotServices::VisionConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::VisionConfig::VisionConfigClient* visionconfig, uint32_t device_id, uint32_t timeout_ms): 
	IVisionConfigServices(node_handle),
	m_visionconfig(visionconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle->create_publisher<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_VisionTopic = m_node_handle->create_publisher<kortex_driver::msg::VisionNotification>("vision_topic", 1000);
	m_is_activated_VisionTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("vision_config/set_device_id", std::bind(&VisionConfigRobotServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("vision_config/set_api_options", std::bind(&VisionConfigRobotServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceSetSensorSettings = m_node_handle->create_service<kortex_driver::srv::SetSensorSettings>("vision_config/set_sensor_settings", std::bind(&VisionConfigRobotServices::SetSensorSettings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSensorSettings = m_node_handle->create_service<kortex_driver::srv::GetSensorSettings>("vision_config/get_sensor_settings", std::bind(&VisionConfigRobotServices::GetSensorSettings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetOptionValue = m_node_handle->create_service<kortex_driver::srv::GetOptionValue>("vision_config/get_option_value", std::bind(&VisionConfigRobotServices::GetOptionValue, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetOptionValue = m_node_handle->create_service<kortex_driver::srv::SetOptionValue>("vision_config/set_option_value", std::bind(&VisionConfigRobotServices::SetOptionValue, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetOptionInformation = m_node_handle->create_service<kortex_driver::srv::GetOptionInformation>("vision_config/get_option_information", std::bind(&VisionConfigRobotServices::GetOptionInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationVisionTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationVisionTopic>("vision_config/activate_publishing_of_vision_topic", std::bind(&VisionConfigRobotServices::OnNotificationVisionTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDoSensorFocusAction = m_node_handle->create_service<kortex_driver::srv::DoSensorFocusAction>("vision_config/do_sensor_focus_action", std::bind(&VisionConfigRobotServices::DoSensorFocusAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIntrinsicParameters = m_node_handle->create_service<kortex_driver::srv::GetIntrinsicParameters>("vision_config/get_intrinsic_parameters", std::bind(&VisionConfigRobotServices::GetIntrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIntrinsicParametersProfile = m_node_handle->create_service<kortex_driver::srv::GetIntrinsicParametersProfile>("vision_config/get_intrinsic_parameters_profile", std::bind(&VisionConfigRobotServices::GetIntrinsicParametersProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetIntrinsicParameters = m_node_handle->create_service<kortex_driver::srv::SetIntrinsicParameters>("vision_config/set_intrinsic_parameters", std::bind(&VisionConfigRobotServices::SetIntrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetExtrinsicParameters = m_node_handle->create_service<kortex_driver::srv::GetExtrinsicParameters>("vision_config/get_extrinsic_parameters", std::bind(&VisionConfigRobotServices::GetExtrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetExtrinsicParameters = m_node_handle->create_service<kortex_driver::srv::SetExtrinsicParameters>("vision_config/set_extrinsic_parameters", std::bind(&VisionConfigRobotServices::SetExtrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
}

bool VisionConfigRobotServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	m_current_device_id = req->device_id;

	return true;
}

bool VisionConfigRobotServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	m_api_options.timeout_ms = req->input.timeout_ms;

	return true;
}


bool VisionConfigRobotServices::SetSensorSettings(const std::shared_ptr<kortex_driver::srv::SetSensorSettings::Request> req, std::shared_ptr<kortex_driver::srv::SetSensorSettings::Response> res)
{
	
	Kinova::Api::VisionConfig::SensorSettings input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_visionconfig->SetSensorSettings(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetSensorSettings(const std::shared_ptr<kortex_driver::srv::GetSensorSettings::Request> req, std::shared_ptr<kortex_driver::srv::GetSensorSettings::Response> res)
{
	
	Kinova::Api::VisionConfig::SensorIdentifier input;
	ToProtoData(req->input, &input);
	Kinova::Api::VisionConfig::SensorSettings output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetSensorSettings(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetOptionValue(const std::shared_ptr<kortex_driver::srv::GetOptionValue::Request> req, std::shared_ptr<kortex_driver::srv::GetOptionValue::Response> res)
{
	
	Kinova::Api::VisionConfig::OptionIdentifier input;
	ToProtoData(req->input, &input);
	Kinova::Api::VisionConfig::OptionValue output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetOptionValue(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::SetOptionValue(const std::shared_ptr<kortex_driver::srv::SetOptionValue::Request> req, std::shared_ptr<kortex_driver::srv::SetOptionValue::Response> res)
{
	
	Kinova::Api::VisionConfig::OptionValue input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_visionconfig->SetOptionValue(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetOptionInformation(const std::shared_ptr<kortex_driver::srv::GetOptionInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetOptionInformation::Response> res)
{
	
	Kinova::Api::VisionConfig::OptionIdentifier input;
	ToProtoData(req->input, &input);
	Kinova::Api::VisionConfig::OptionInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetOptionInformation(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::OnNotificationVisionTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationVisionTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationVisionTopic::Response> res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_VisionTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req->input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::VisionConfig::VisionNotification) > callback = std::bind(&VisionConfigRobotServices::cb_VisionTopic, this, std::placeholders::_1);
		output = m_visionconfig->OnNotificationVisionTopic(callback, input, m_current_device_id);
		m_is_activated_VisionTopic = true;
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
void VisionConfigRobotServices::cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif)
{
	kortex_driver::msg::VisionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_VisionTopic->publish(ros_msg);
}

bool VisionConfigRobotServices::DoSensorFocusAction(const std::shared_ptr<kortex_driver::srv::DoSensorFocusAction::Request> req, std::shared_ptr<kortex_driver::srv::DoSensorFocusAction::Response> res)
{
	
	Kinova::Api::VisionConfig::SensorFocusAction input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_visionconfig->DoSensorFocusAction(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetIntrinsicParameters(const std::shared_ptr<kortex_driver::srv::GetIntrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetIntrinsicParameters::Response> res)
{
	
	Kinova::Api::VisionConfig::SensorIdentifier input;
	ToProtoData(req->input, &input);
	Kinova::Api::VisionConfig::IntrinsicParameters output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetIntrinsicParameters(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetIntrinsicParametersProfile(const std::shared_ptr<kortex_driver::srv::GetIntrinsicParametersProfile::Request> req, std::shared_ptr<kortex_driver::srv::GetIntrinsicParametersProfile::Response> res)
{
	
	Kinova::Api::VisionConfig::IntrinsicProfileIdentifier input;
	ToProtoData(req->input, &input);
	Kinova::Api::VisionConfig::IntrinsicParameters output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetIntrinsicParametersProfile(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::SetIntrinsicParameters(const std::shared_ptr<kortex_driver::srv::SetIntrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetIntrinsicParameters::Response> res)
{
	
	Kinova::Api::VisionConfig::IntrinsicParameters input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_visionconfig->SetIntrinsicParameters(input, m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::GetExtrinsicParameters(const std::shared_ptr<kortex_driver::srv::GetExtrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetExtrinsicParameters::Response> res)
{
	
	Kinova::Api::VisionConfig::ExtrinsicParameters output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_visionconfig->GetExtrinsicParameters(m_current_device_id, m_api_options);
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

bool VisionConfigRobotServices::SetExtrinsicParameters(const std::shared_ptr<kortex_driver::srv::SetExtrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetExtrinsicParameters::Response> res)
{
	
	Kinova::Api::VisionConfig::ExtrinsicParameters input;
	ToProtoData(req->input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_visionconfig->SetExtrinsicParameters(input, m_current_device_id, m_api_options);
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
