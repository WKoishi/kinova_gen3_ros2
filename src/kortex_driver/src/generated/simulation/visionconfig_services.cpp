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
#include "kortex_driver/generated/simulation/visionconfig_services.h"

VisionConfigSimulationServices::VisionConfigSimulationServices(rclcpp::Node::SharedPtr node_handle): 
	IVisionConfigServices(node_handle)
{
	m_pub_Error = m_node_handle->create_publisher<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_VisionTopic = m_node_handle->create_publisher<kortex_driver::msg::VisionNotification>("vision_topic", 1000);
	m_is_activated_VisionTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("vision_config/set_device_id", std::bind(&VisionConfigSimulationServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("vision_config/set_api_options", std::bind(&VisionConfigSimulationServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceSetSensorSettings = m_node_handle->create_service<kortex_driver::srv::SetSensorSettings>("vision_config/set_sensor_settings", std::bind(&VisionConfigSimulationServices::SetSensorSettings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSensorSettings = m_node_handle->create_service<kortex_driver::srv::GetSensorSettings>("vision_config/get_sensor_settings", std::bind(&VisionConfigSimulationServices::GetSensorSettings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetOptionValue = m_node_handle->create_service<kortex_driver::srv::GetOptionValue>("vision_config/get_option_value", std::bind(&VisionConfigSimulationServices::GetOptionValue, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetOptionValue = m_node_handle->create_service<kortex_driver::srv::SetOptionValue>("vision_config/set_option_value", std::bind(&VisionConfigSimulationServices::SetOptionValue, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetOptionInformation = m_node_handle->create_service<kortex_driver::srv::GetOptionInformation>("vision_config/get_option_information", std::bind(&VisionConfigSimulationServices::GetOptionInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationVisionTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationVisionTopic>("vision_config/activate_publishing_of_vision_topic", std::bind(&VisionConfigSimulationServices::OnNotificationVisionTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDoSensorFocusAction = m_node_handle->create_service<kortex_driver::srv::DoSensorFocusAction>("vision_config/do_sensor_focus_action", std::bind(&VisionConfigSimulationServices::DoSensorFocusAction, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIntrinsicParameters = m_node_handle->create_service<kortex_driver::srv::GetIntrinsicParameters>("vision_config/get_intrinsic_parameters", std::bind(&VisionConfigSimulationServices::GetIntrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIntrinsicParametersProfile = m_node_handle->create_service<kortex_driver::srv::GetIntrinsicParametersProfile>("vision_config/get_intrinsic_parameters_profile", std::bind(&VisionConfigSimulationServices::GetIntrinsicParametersProfile, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetIntrinsicParameters = m_node_handle->create_service<kortex_driver::srv::SetIntrinsicParameters>("vision_config/set_intrinsic_parameters", std::bind(&VisionConfigSimulationServices::SetIntrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetExtrinsicParameters = m_node_handle->create_service<kortex_driver::srv::GetExtrinsicParameters>("vision_config/get_extrinsic_parameters", std::bind(&VisionConfigSimulationServices::GetExtrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetExtrinsicParameters = m_node_handle->create_service<kortex_driver::srv::SetExtrinsicParameters>("vision_config/set_extrinsic_parameters", std::bind(&VisionConfigSimulationServices::SetExtrinsicParameters, this, std::placeholders::_1, std::placeholders::_2));
}

bool VisionConfigSimulationServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool VisionConfigSimulationServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool VisionConfigSimulationServices::SetSensorSettings(const std::shared_ptr<kortex_driver::srv::SetSensorSettings::Request> req, std::shared_ptr<kortex_driver::srv::SetSensorSettings::Response> res)
{
	
	
	if (SetSensorSettingsHandler)
	{
		res = SetSensorSettingsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/set_sensor_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetSensorSettings(const std::shared_ptr<kortex_driver::srv::GetSensorSettings::Request> req, std::shared_ptr<kortex_driver::srv::GetSensorSettings::Response> res)
{
	
	
	if (GetSensorSettingsHandler)
	{
		res = GetSensorSettingsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/get_sensor_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetOptionValue(const std::shared_ptr<kortex_driver::srv::GetOptionValue::Request> req, std::shared_ptr<kortex_driver::srv::GetOptionValue::Response> res)
{
	
	
	if (GetOptionValueHandler)
	{
		res = GetOptionValueHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/get_option_value is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::SetOptionValue(const std::shared_ptr<kortex_driver::srv::SetOptionValue::Request> req, std::shared_ptr<kortex_driver::srv::SetOptionValue::Response> res)
{
	
	
	if (SetOptionValueHandler)
	{
		res = SetOptionValueHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/set_option_value is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetOptionInformation(const std::shared_ptr<kortex_driver::srv::GetOptionInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetOptionInformation::Response> res)
{
	
	
	if (GetOptionInformationHandler)
	{
		res = GetOptionInformationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/get_option_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::OnNotificationVisionTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationVisionTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationVisionTopic::Response> res)
{
	
	m_is_activated_VisionTopic = true;
	
	if (OnNotificationVisionTopicHandler)
	{
		res = OnNotificationVisionTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/activate_publishing_of_vision_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void VisionConfigSimulationServices::cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif)
{
	kortex_driver::msg::VisionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_VisionTopic->publish(ros_msg);
}

bool VisionConfigSimulationServices::DoSensorFocusAction(const std::shared_ptr<kortex_driver::srv::DoSensorFocusAction::Request> req, std::shared_ptr<kortex_driver::srv::DoSensorFocusAction::Response> res)
{
	
	
	if (DoSensorFocusActionHandler)
	{
		res = DoSensorFocusActionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/do_sensor_focus_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetIntrinsicParameters(const std::shared_ptr<kortex_driver::srv::GetIntrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetIntrinsicParameters::Response> res)
{
	
	
	if (GetIntrinsicParametersHandler)
	{
		res = GetIntrinsicParametersHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/get_intrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetIntrinsicParametersProfile(const std::shared_ptr<kortex_driver::srv::GetIntrinsicParametersProfile::Request> req, std::shared_ptr<kortex_driver::srv::GetIntrinsicParametersProfile::Response> res)
{
	
	
	if (GetIntrinsicParametersProfileHandler)
	{
		res = GetIntrinsicParametersProfileHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/get_intrinsic_parameters_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::SetIntrinsicParameters(const std::shared_ptr<kortex_driver::srv::SetIntrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetIntrinsicParameters::Response> res)
{
	
	
	if (SetIntrinsicParametersHandler)
	{
		res = SetIntrinsicParametersHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/set_intrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetExtrinsicParameters(const std::shared_ptr<kortex_driver::srv::GetExtrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::GetExtrinsicParameters::Response> res)
{
	
	
	if (GetExtrinsicParametersHandler)
	{
		res = GetExtrinsicParametersHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/get_extrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::SetExtrinsicParameters(const std::shared_ptr<kortex_driver::srv::SetExtrinsicParameters::Request> req, std::shared_ptr<kortex_driver::srv::SetExtrinsicParameters::Response> res)
{
	
	
	if (SetExtrinsicParametersHandler)
	{
		res = SetExtrinsicParametersHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for vision_config/set_extrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}
