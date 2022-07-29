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
#include "kortex_driver/generated/robot/deviceconfig_services.h"

DeviceConfigRobotServices::DeviceConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::DeviceConfig::DeviceConfigClient* deviceconfig, uint32_t device_id, uint32_t timeout_ms): 
	IDeviceConfigServices(node_handle),
	m_deviceconfig(deviceconfig),
	m_current_device_id(device_id)
{
	m_api_options.timeout_ms = timeout_ms;

	m_pub_Error = m_node_handle.advertise<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_SafetyTopic = m_node_handle.advertise<kortex_driver::msg::SafetyNotification>("safety_topic", 1000);
	m_is_activated_SafetyTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("device_config/set_device_id", &DeviceConfigRobotServices::SetDeviceID);
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("device_config/set_api_options", &DeviceConfigRobotServices::SetApiOptions);

	m_serviceGetRunMode = m_node_handle->create_service<kortex_driver::srv::GetRunMode>("device_config/get_run_mode", &DeviceConfigRobotServices::GetRunMode);
	m_serviceSetRunMode = m_node_handle->create_service<kortex_driver::srv::SetRunMode>("device_config/set_run_mode", &DeviceConfigRobotServices::SetRunMode);
	m_serviceGetDeviceType = m_node_handle->create_service<kortex_driver::srv::GetDeviceType>("device_config/get_device_type", &DeviceConfigRobotServices::GetDeviceType);
	m_serviceGetFirmwareVersion = m_node_handle->create_service<kortex_driver::srv::GetFirmwareVersion>("device_config/get_firmware_version", &DeviceConfigRobotServices::GetFirmwareVersion);
	m_serviceGetBootloaderVersion = m_node_handle->create_service<kortex_driver::srv::GetBootloaderVersion>("device_config/get_bootloader_version", &DeviceConfigRobotServices::GetBootloaderVersion);
	m_serviceGetModelNumber = m_node_handle->create_service<kortex_driver::srv::GetModelNumber>("device_config/get_model_number", &DeviceConfigRobotServices::GetModelNumber);
	m_serviceGetPartNumber = m_node_handle->create_service<kortex_driver::srv::GetPartNumber>("device_config/get_part_number", &DeviceConfigRobotServices::GetPartNumber);
	m_serviceGetSerialNumber = m_node_handle->create_service<kortex_driver::srv::GetSerialNumber>("device_config/get_serial_number", &DeviceConfigRobotServices::GetSerialNumber);
	m_serviceGetMACAddress = m_node_handle->create_service<kortex_driver::srv::GetMACAddress>("device_config/get_m_a_c_address", &DeviceConfigRobotServices::GetMACAddress);
	m_serviceGetIPv4Settings = m_node_handle->create_service<kortex_driver::srv::GetIPv4Settings>("device_config/get_i_pv4_settings", &DeviceConfigRobotServices::GetIPv4Settings);
	m_serviceSetIPv4Settings = m_node_handle->create_service<kortex_driver::srv::SetIPv4Settings>("device_config/set_i_pv4_settings", &DeviceConfigRobotServices::SetIPv4Settings);
	m_serviceGetPartNumberRevision = m_node_handle->create_service<kortex_driver::srv::GetPartNumberRevision>("device_config/get_part_number_revision", &DeviceConfigRobotServices::GetPartNumberRevision);
	m_serviceRebootRequest = m_node_handle->create_service<kortex_driver::srv::RebootRequest>("device_config/reboot_request", &DeviceConfigRobotServices::RebootRequest);
	m_serviceSetSafetyEnable = m_node_handle->create_service<kortex_driver::srv::SetSafetyEnable>("device_config/set_safety_enable", &DeviceConfigRobotServices::SetSafetyEnable);
	m_serviceSetSafetyErrorThreshold = m_node_handle->create_service<kortex_driver::srv::SetSafetyErrorThreshold>("device_config/set_safety_error_threshold", &DeviceConfigRobotServices::SetSafetyErrorThreshold);
	m_serviceSetSafetyWarningThreshold = m_node_handle->create_service<kortex_driver::srv::SetSafetyWarningThreshold>("device_config/set_safety_warning_threshold", &DeviceConfigRobotServices::SetSafetyWarningThreshold);
	m_serviceSetSafetyConfiguration = m_node_handle->create_service<kortex_driver::srv::SetSafetyConfiguration>("device_config/set_safety_configuration", &DeviceConfigRobotServices::SetSafetyConfiguration);
	m_serviceGetSafetyConfiguration = m_node_handle->create_service<kortex_driver::srv::GetSafetyConfiguration>("device_config/get_safety_configuration", &DeviceConfigRobotServices::GetSafetyConfiguration);
	m_serviceGetSafetyInformation = m_node_handle->create_service<kortex_driver::srv::GetSafetyInformation>("device_config/get_safety_information", &DeviceConfigRobotServices::GetSafetyInformation);
	m_serviceGetSafetyEnable = m_node_handle->create_service<kortex_driver::srv::GetSafetyEnable>("device_config/get_safety_enable", &DeviceConfigRobotServices::GetSafetyEnable);
	m_serviceGetSafetyStatus = m_node_handle->create_service<kortex_driver::srv::GetSafetyStatus>("device_config/get_safety_status", &DeviceConfigRobotServices::GetSafetyStatus);
	m_serviceClearAllSafetyStatus = m_node_handle->create_service<kortex_driver::srv::ClearAllSafetyStatus>("device_config/clear_all_safety_status", &DeviceConfigRobotServices::ClearAllSafetyStatus);
	m_serviceClearSafetyStatus = m_node_handle->create_service<kortex_driver::srv::ClearSafetyStatus>("device_config/clear_safety_status", &DeviceConfigRobotServices::ClearSafetyStatus);
	m_serviceGetAllSafetyConfiguration = m_node_handle->create_service<kortex_driver::srv::GetAllSafetyConfiguration>("device_config/get_all_safety_configuration", &DeviceConfigRobotServices::GetAllSafetyConfiguration);
	m_serviceGetAllSafetyInformation = m_node_handle->create_service<kortex_driver::srv::GetAllSafetyInformation>("device_config/get_all_safety_information", &DeviceConfigRobotServices::GetAllSafetyInformation);
	m_serviceResetSafetyDefaults = m_node_handle->create_service<kortex_driver::srv::ResetSafetyDefaults>("device_config/reset_safety_defaults", &DeviceConfigRobotServices::ResetSafetyDefaults);
	m_serviceOnNotificationSafetyTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationSafetyTopic>("device_config/activate_publishing_of_safety_topic", &DeviceConfigRobotServices::OnNotificationSafetyTopic);
	m_serviceExecuteCalibration = m_node_handle->create_service<kortex_driver::srv::ExecuteCalibration>("device_config/execute_calibration", &DeviceConfigRobotServices::ExecuteCalibration);
	m_serviceGetCalibrationResult = m_node_handle->create_service<kortex_driver::srv::GetCalibrationResult>("device_config/get_calibration_result", &DeviceConfigRobotServices::GetCalibrationResult);
	m_serviceStopCalibration = m_node_handle->create_service<kortex_driver::srv::StopCalibration>("device_config/stop_calibration", &DeviceConfigRobotServices::StopCalibration);
	m_serviceDeviceConfig_SetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::DeviceConfigSetCapSenseConfig>("device_config/set_cap_sense_config", &DeviceConfigRobotServices::DeviceConfig_SetCapSenseConfig);
	m_serviceDeviceConfig_GetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::DeviceConfigGetCapSenseConfig>("device_config/get_cap_sense_config", &DeviceConfigRobotServices::DeviceConfig_GetCapSenseConfig);
}

bool DeviceConfigRobotServices::SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res)
{
	m_current_device_id = req.device_id;

	return true;
}

bool DeviceConfigRobotServices::SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res)
{
	m_api_options.timeout_ms = req.input.timeout_ms;

	return true;
}


bool DeviceConfigRobotServices::GetRunMode(kortex_driver::srv::GetRunMode::Request  &req, kortex_driver::srv::GetRunMode::Response &res)
{
	
	Kinova::Api::DeviceConfig::RunMode output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetRunMode(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::SetRunMode(kortex_driver::srv::SetRunMode::Request  &req, kortex_driver::srv::SetRunMode::Response &res)
{
	
	Kinova::Api::DeviceConfig::RunMode input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetRunMode(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::GetDeviceType(kortex_driver::srv::GetDeviceType::Request  &req, kortex_driver::srv::GetDeviceType::Response &res)
{
	
	Kinova::Api::DeviceConfig::DeviceType output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetDeviceType(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetFirmwareVersion(kortex_driver::srv::GetFirmwareVersion::Request  &req, kortex_driver::srv::GetFirmwareVersion::Response &res)
{
	
	Kinova::Api::DeviceConfig::FirmwareVersion output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetFirmwareVersion(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetBootloaderVersion(kortex_driver::srv::GetBootloaderVersion::Request  &req, kortex_driver::srv::GetBootloaderVersion::Response &res)
{
	
	Kinova::Api::DeviceConfig::BootloaderVersion output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetBootloaderVersion(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetModelNumber(kortex_driver::srv::GetModelNumber::Request  &req, kortex_driver::srv::GetModelNumber::Response &res)
{
	
	Kinova::Api::DeviceConfig::ModelNumber output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetModelNumber(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetPartNumber(kortex_driver::srv::GetPartNumber::Request  &req, kortex_driver::srv::GetPartNumber::Response &res)
{
	
	Kinova::Api::DeviceConfig::PartNumber output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetPartNumber(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSerialNumber(kortex_driver::srv::GetSerialNumber::Request  &req, kortex_driver::srv::GetSerialNumber::Response &res)
{
	
	Kinova::Api::DeviceConfig::SerialNumber output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSerialNumber(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetMACAddress(kortex_driver::srv::GetMACAddress::Request  &req, kortex_driver::srv::GetMACAddress::Response &res)
{
	
	Kinova::Api::DeviceConfig::MACAddress output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetMACAddress(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetIPv4Settings(kortex_driver::srv::GetIPv4Settings::Request  &req, kortex_driver::srv::GetIPv4Settings::Response &res)
{
	
	Kinova::Api::DeviceConfig::IPv4Settings output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetIPv4Settings(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::SetIPv4Settings(kortex_driver::srv::SetIPv4Settings::Request  &req, kortex_driver::srv::SetIPv4Settings::Response &res)
{
	
	Kinova::Api::DeviceConfig::IPv4Settings input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetIPv4Settings(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::GetPartNumberRevision(kortex_driver::srv::GetPartNumberRevision::Request  &req, kortex_driver::srv::GetPartNumberRevision::Response &res)
{
	
	Kinova::Api::DeviceConfig::PartNumberRevision output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetPartNumberRevision(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::RebootRequest(kortex_driver::srv::RebootRequest::Request  &req, kortex_driver::srv::RebootRequest::Response &res)
{
	
	Kinova::Api::DeviceConfig::RebootRqst input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->RebootRequest(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::SetSafetyEnable(kortex_driver::srv::SetSafetyEnable::Request  &req, kortex_driver::srv::SetSafetyEnable::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyEnable input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyEnable(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::SetSafetyErrorThreshold(kortex_driver::srv::SetSafetyErrorThreshold::Request  &req, kortex_driver::srv::SetSafetyErrorThreshold::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyThreshold input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyErrorThreshold(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::SetSafetyWarningThreshold(kortex_driver::srv::SetSafetyWarningThreshold::Request  &req, kortex_driver::srv::SetSafetyWarningThreshold::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyThreshold input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyWarningThreshold(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::SetSafetyConfiguration(kortex_driver::srv::SetSafetyConfiguration::Request  &req, kortex_driver::srv::SetSafetyConfiguration::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyConfiguration input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetSafetyConfiguration(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::GetSafetyConfiguration(kortex_driver::srv::GetSafetyConfiguration::Request  &req, kortex_driver::srv::GetSafetyConfiguration::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyConfiguration output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyConfiguration(input, m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSafetyInformation(kortex_driver::srv::GetSafetyInformation::Request  &req, kortex_driver::srv::GetSafetyInformation::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyInformation output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyInformation(input, m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSafetyEnable(kortex_driver::srv::GetSafetyEnable::Request  &req, kortex_driver::srv::GetSafetyEnable::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyEnable output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyEnable(input, m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetSafetyStatus(kortex_driver::srv::GetSafetyStatus::Request  &req, kortex_driver::srv::GetSafetyStatus::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::SafetyStatus output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetSafetyStatus(input, m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::ClearAllSafetyStatus(kortex_driver::srv::ClearAllSafetyStatus::Request  &req, kortex_driver::srv::ClearAllSafetyStatus::Response &res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->ClearAllSafetyStatus(m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::ClearSafetyStatus(kortex_driver::srv::ClearSafetyStatus::Request  &req, kortex_driver::srv::ClearSafetyStatus::Response &res)
{
	
	Kinova::Api::Common::SafetyHandle input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->ClearSafetyStatus(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::GetAllSafetyConfiguration(kortex_driver::srv::GetAllSafetyConfiguration::Request  &req, kortex_driver::srv::GetAllSafetyConfiguration::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyConfigurationList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetAllSafetyConfiguration(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::GetAllSafetyInformation(kortex_driver::srv::GetAllSafetyInformation::Request  &req, kortex_driver::srv::GetAllSafetyInformation::Response &res)
{
	
	Kinova::Api::DeviceConfig::SafetyInformationList output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetAllSafetyInformation(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::ResetSafetyDefaults(kortex_driver::srv::ResetSafetyDefaults::Request  &req, kortex_driver::srv::ResetSafetyDefaults::Response &res)
{
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->ResetSafetyDefaults(m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::OnNotificationSafetyTopic(kortex_driver::srv::OnNotificationSafetyTopic::Request  &req, kortex_driver::srv::OnNotificationSafetyTopic::Response &res)
{
	
	// If the notification is already activated, don't activate multiple times
	if (m_is_activated_SafetyTopic)
		return true;
	Kinova::Api::Common::NotificationOptions input;
	ToProtoData(req.input, &input);
	Kinova::Api::Common::NotificationHandle output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		std::function< void (Kinova::Api::Common::SafetyNotification) > callback = std::bind(&DeviceConfigRobotServices::cb_SafetyTopic, this, std::placeholders::_1);
		output = m_deviceconfig->OnNotificationSafetyTopic(callback, input, m_current_device_id);
		m_is_activated_SafetyTopic = true;
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
	ToRosData(output, res.output);
	return true;
}
void DeviceConfigRobotServices::cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif)
{
	kortex_driver::msg::SafetyNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SafetyTopic->publish(ros_msg);
}

bool DeviceConfigRobotServices::ExecuteCalibration(kortex_driver::srv::ExecuteCalibration::Request  &req, kortex_driver::srv::ExecuteCalibration::Response &res)
{
	
	Kinova::Api::DeviceConfig::Calibration input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->ExecuteCalibration(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::GetCalibrationResult(kortex_driver::srv::GetCalibrationResult::Request  &req, kortex_driver::srv::GetCalibrationResult::Response &res)
{
	
	Kinova::Api::DeviceConfig::CalibrationElement input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::CalibrationResult output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetCalibrationResult(input, m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::StopCalibration(kortex_driver::srv::StopCalibration::Request  &req, kortex_driver::srv::StopCalibration::Response &res)
{
	
	Kinova::Api::DeviceConfig::Calibration input;
	ToProtoData(req.input, &input);
	Kinova::Api::DeviceConfig::CalibrationResult output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->StopCalibration(input, m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}

bool DeviceConfigRobotServices::DeviceConfig_SetCapSenseConfig(kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response &res)
{
	
	Kinova::Api::DeviceConfig::CapSenseConfig input;
	ToProtoData(req.input, &input);
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		m_deviceconfig->SetCapSenseConfig(input, m_current_device_id, m_api_options);
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

bool DeviceConfigRobotServices::DeviceConfig_GetCapSenseConfig(kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response &res)
{
	
	Kinova::Api::DeviceConfig::CapSenseConfig output;
	
	kortex_driver::msg::KortexError result_error;
	
	try
	{
		output = m_deviceconfig->GetCapSenseConfig(m_current_device_id, m_api_options);
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
	ToRosData(output, res.output);
	return true;
}
