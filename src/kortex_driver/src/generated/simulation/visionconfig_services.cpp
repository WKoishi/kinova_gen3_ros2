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
	m_pub_Error = m_node_handle.advertise<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_VisionTopic = m_node_handle.advertise<kortex_driver::msg::VisionNotification>("vision_topic", 1000);
	m_is_activated_VisionTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("vision_config/set_device_id", &VisionConfigSimulationServices::SetDeviceID, this);
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("vision_config/set_api_options", &VisionConfigSimulationServices::SetApiOptions, this);

	m_serviceSetSensorSettings = m_node_handle->create_service<kortex_driver::srv::SetSensorSettings>("vision_config/set_sensor_settings", &VisionConfigSimulationServices::SetSensorSettings, this);
	m_serviceGetSensorSettings = m_node_handle->create_service<kortex_driver::srv::GetSensorSettings>("vision_config/get_sensor_settings", &VisionConfigSimulationServices::GetSensorSettings, this);
	m_serviceGetOptionValue = m_node_handle->create_service<kortex_driver::srv::GetOptionValue>("vision_config/get_option_value", &VisionConfigSimulationServices::GetOptionValue, this);
	m_serviceSetOptionValue = m_node_handle->create_service<kortex_driver::srv::SetOptionValue>("vision_config/set_option_value", &VisionConfigSimulationServices::SetOptionValue, this);
	m_serviceGetOptionInformation = m_node_handle->create_service<kortex_driver::srv::GetOptionInformation>("vision_config/get_option_information", &VisionConfigSimulationServices::GetOptionInformation, this);
	m_serviceOnNotificationVisionTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationVisionTopic>("vision_config/activate_publishing_of_vision_topic", &VisionConfigSimulationServices::OnNotificationVisionTopic, this);
	m_serviceDoSensorFocusAction = m_node_handle->create_service<kortex_driver::srv::DoSensorFocusAction>("vision_config/do_sensor_focus_action", &VisionConfigSimulationServices::DoSensorFocusAction, this);
	m_serviceGetIntrinsicParameters = m_node_handle->create_service<kortex_driver::srv::GetIntrinsicParameters>("vision_config/get_intrinsic_parameters", &VisionConfigSimulationServices::GetIntrinsicParameters, this);
	m_serviceGetIntrinsicParametersProfile = m_node_handle->create_service<kortex_driver::srv::GetIntrinsicParametersProfile>("vision_config/get_intrinsic_parameters_profile", &VisionConfigSimulationServices::GetIntrinsicParametersProfile, this);
	m_serviceSetIntrinsicParameters = m_node_handle->create_service<kortex_driver::srv::SetIntrinsicParameters>("vision_config/set_intrinsic_parameters", &VisionConfigSimulationServices::SetIntrinsicParameters, this);
	m_serviceGetExtrinsicParameters = m_node_handle->create_service<kortex_driver::srv::GetExtrinsicParameters>("vision_config/get_extrinsic_parameters", &VisionConfigSimulationServices::GetExtrinsicParameters, this);
	m_serviceSetExtrinsicParameters = m_node_handle->create_service<kortex_driver::srv::SetExtrinsicParameters>("vision_config/set_extrinsic_parameters", &VisionConfigSimulationServices::SetExtrinsicParameters, this);
}

bool VisionConfigSimulationServices::SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool VisionConfigSimulationServices::SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res)
{
	ROS_WARN_ONCE("The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool VisionConfigSimulationServices::SetSensorSettings(kortex_driver::srv::SetSensorSettings::Request  &req, kortex_driver::srv::SetSensorSettings::Response &res)
{
	
	
	if (SetSensorSettingsHandler)
	{
		res = SetSensorSettingsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/set_sensor_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetSensorSettings(kortex_driver::srv::GetSensorSettings::Request  &req, kortex_driver::srv::GetSensorSettings::Response &res)
{
	
	
	if (GetSensorSettingsHandler)
	{
		res = GetSensorSettingsHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/get_sensor_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetOptionValue(kortex_driver::srv::GetOptionValue::Request  &req, kortex_driver::srv::GetOptionValue::Response &res)
{
	
	
	if (GetOptionValueHandler)
	{
		res = GetOptionValueHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/get_option_value is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::SetOptionValue(kortex_driver::srv::SetOptionValue::Request  &req, kortex_driver::srv::SetOptionValue::Response &res)
{
	
	
	if (SetOptionValueHandler)
	{
		res = SetOptionValueHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/set_option_value is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetOptionInformation(kortex_driver::srv::GetOptionInformation::Request  &req, kortex_driver::srv::GetOptionInformation::Response &res)
{
	
	
	if (GetOptionInformationHandler)
	{
		res = GetOptionInformationHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/get_option_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::OnNotificationVisionTopic(kortex_driver::srv::OnNotificationVisionTopic::Request  &req, kortex_driver::srv::OnNotificationVisionTopic::Response &res)
{
	
	m_is_activated_VisionTopic = true;
	
	if (OnNotificationVisionTopicHandler)
	{
		res = OnNotificationVisionTopicHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/activate_publishing_of_vision_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void VisionConfigSimulationServices::cb_VisionTopic(Kinova::Api::VisionConfig::VisionNotification notif)
{
	kortex_driver::msg::VisionNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_VisionTopic->publish(ros_msg);
}

bool VisionConfigSimulationServices::DoSensorFocusAction(kortex_driver::srv::DoSensorFocusAction::Request  &req, kortex_driver::srv::DoSensorFocusAction::Response &res)
{
	
	
	if (DoSensorFocusActionHandler)
	{
		res = DoSensorFocusActionHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/do_sensor_focus_action is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetIntrinsicParameters(kortex_driver::srv::GetIntrinsicParameters::Request  &req, kortex_driver::srv::GetIntrinsicParameters::Response &res)
{
	
	
	if (GetIntrinsicParametersHandler)
	{
		res = GetIntrinsicParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/get_intrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetIntrinsicParametersProfile(kortex_driver::srv::GetIntrinsicParametersProfile::Request  &req, kortex_driver::srv::GetIntrinsicParametersProfile::Response &res)
{
	
	
	if (GetIntrinsicParametersProfileHandler)
	{
		res = GetIntrinsicParametersProfileHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/get_intrinsic_parameters_profile is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::SetIntrinsicParameters(kortex_driver::srv::SetIntrinsicParameters::Request  &req, kortex_driver::srv::SetIntrinsicParameters::Response &res)
{
	
	
	if (SetIntrinsicParametersHandler)
	{
		res = SetIntrinsicParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/set_intrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::GetExtrinsicParameters(kortex_driver::srv::GetExtrinsicParameters::Request  &req, kortex_driver::srv::GetExtrinsicParameters::Response &res)
{
	
	
	if (GetExtrinsicParametersHandler)
	{
		res = GetExtrinsicParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/get_extrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool VisionConfigSimulationServices::SetExtrinsicParameters(kortex_driver::srv::SetExtrinsicParameters::Request  &req, kortex_driver::srv::SetExtrinsicParameters::Response &res)
{
	
	
	if (SetExtrinsicParametersHandler)
	{
		res = SetExtrinsicParametersHandler(req);
	}
	else
	{
		ROS_WARN_ONCE("The simulation handler for vision_config/set_extrinsic_parameters is not implemented, so the service calls will return the default response.");
	}
	return true;
}
