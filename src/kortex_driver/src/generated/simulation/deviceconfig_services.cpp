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
#include "kortex_driver/generated/simulation/deviceconfig_services.h"

DeviceConfigSimulationServices::DeviceConfigSimulationServices(rclcpp::Node::SharedPtr node_handle): 
	IDeviceConfigServices(node_handle)
{
	m_pub_Error = m_node_handle->create_publisher<kortex_driver::msg::KortexError>("kortex_error", 1000);
	m_pub_SafetyTopic = m_node_handle->create_publisher<kortex_driver::msg::SafetyNotification>("safety_topic", 1000);
	m_is_activated_SafetyTopic = false;

	m_serviceSetDeviceID = m_node_handle->create_service<kortex_driver::srv::SetDeviceID>("device_config/set_device_id", std::bind(&DeviceConfigSimulationServices::SetDeviceID, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetApiOptions = m_node_handle->create_service<kortex_driver::srv::SetApiOptions>("device_config/set_api_options", std::bind(&DeviceConfigSimulationServices::SetApiOptions, this, std::placeholders::_1, std::placeholders::_2));

	m_serviceGetRunMode = m_node_handle->create_service<kortex_driver::srv::GetRunMode>("device_config/get_run_mode", std::bind(&DeviceConfigSimulationServices::GetRunMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetRunMode = m_node_handle->create_service<kortex_driver::srv::SetRunMode>("device_config/set_run_mode", std::bind(&DeviceConfigSimulationServices::SetRunMode, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetDeviceType = m_node_handle->create_service<kortex_driver::srv::GetDeviceType>("device_config/get_device_type", std::bind(&DeviceConfigSimulationServices::GetDeviceType, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetFirmwareVersion = m_node_handle->create_service<kortex_driver::srv::GetFirmwareVersion>("device_config/get_firmware_version", std::bind(&DeviceConfigSimulationServices::GetFirmwareVersion, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetBootloaderVersion = m_node_handle->create_service<kortex_driver::srv::GetBootloaderVersion>("device_config/get_bootloader_version", std::bind(&DeviceConfigSimulationServices::GetBootloaderVersion, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetModelNumber = m_node_handle->create_service<kortex_driver::srv::GetModelNumber>("device_config/get_model_number", std::bind(&DeviceConfigSimulationServices::GetModelNumber, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetPartNumber = m_node_handle->create_service<kortex_driver::srv::GetPartNumber>("device_config/get_part_number", std::bind(&DeviceConfigSimulationServices::GetPartNumber, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSerialNumber = m_node_handle->create_service<kortex_driver::srv::GetSerialNumber>("device_config/get_serial_number", std::bind(&DeviceConfigSimulationServices::GetSerialNumber, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetMACAddress = m_node_handle->create_service<kortex_driver::srv::GetMACAddress>("device_config/get_m_a_c_address", std::bind(&DeviceConfigSimulationServices::GetMACAddress, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetIPv4Settings = m_node_handle->create_service<kortex_driver::srv::GetIPv4Settings>("device_config/get_i_pv4_settings", std::bind(&DeviceConfigSimulationServices::GetIPv4Settings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetIPv4Settings = m_node_handle->create_service<kortex_driver::srv::SetIPv4Settings>("device_config/set_i_pv4_settings", std::bind(&DeviceConfigSimulationServices::SetIPv4Settings, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetPartNumberRevision = m_node_handle->create_service<kortex_driver::srv::GetPartNumberRevision>("device_config/get_part_number_revision", std::bind(&DeviceConfigSimulationServices::GetPartNumberRevision, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceRebootRequest = m_node_handle->create_service<kortex_driver::srv::RebootRequest>("device_config/reboot_request", std::bind(&DeviceConfigSimulationServices::RebootRequest, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetSafetyEnable = m_node_handle->create_service<kortex_driver::srv::SetSafetyEnable>("device_config/set_safety_enable", std::bind(&DeviceConfigSimulationServices::SetSafetyEnable, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetSafetyErrorThreshold = m_node_handle->create_service<kortex_driver::srv::SetSafetyErrorThreshold>("device_config/set_safety_error_threshold", std::bind(&DeviceConfigSimulationServices::SetSafetyErrorThreshold, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetSafetyWarningThreshold = m_node_handle->create_service<kortex_driver::srv::SetSafetyWarningThreshold>("device_config/set_safety_warning_threshold", std::bind(&DeviceConfigSimulationServices::SetSafetyWarningThreshold, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceSetSafetyConfiguration = m_node_handle->create_service<kortex_driver::srv::SetSafetyConfiguration>("device_config/set_safety_configuration", std::bind(&DeviceConfigSimulationServices::SetSafetyConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSafetyConfiguration = m_node_handle->create_service<kortex_driver::srv::GetSafetyConfiguration>("device_config/get_safety_configuration", std::bind(&DeviceConfigSimulationServices::GetSafetyConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSafetyInformation = m_node_handle->create_service<kortex_driver::srv::GetSafetyInformation>("device_config/get_safety_information", std::bind(&DeviceConfigSimulationServices::GetSafetyInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSafetyEnable = m_node_handle->create_service<kortex_driver::srv::GetSafetyEnable>("device_config/get_safety_enable", std::bind(&DeviceConfigSimulationServices::GetSafetyEnable, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetSafetyStatus = m_node_handle->create_service<kortex_driver::srv::GetSafetyStatus>("device_config/get_safety_status", std::bind(&DeviceConfigSimulationServices::GetSafetyStatus, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceClearAllSafetyStatus = m_node_handle->create_service<kortex_driver::srv::ClearAllSafetyStatus>("device_config/clear_all_safety_status", std::bind(&DeviceConfigSimulationServices::ClearAllSafetyStatus, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceClearSafetyStatus = m_node_handle->create_service<kortex_driver::srv::ClearSafetyStatus>("device_config/clear_safety_status", std::bind(&DeviceConfigSimulationServices::ClearSafetyStatus, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllSafetyConfiguration = m_node_handle->create_service<kortex_driver::srv::GetAllSafetyConfiguration>("device_config/get_all_safety_configuration", std::bind(&DeviceConfigSimulationServices::GetAllSafetyConfiguration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetAllSafetyInformation = m_node_handle->create_service<kortex_driver::srv::GetAllSafetyInformation>("device_config/get_all_safety_information", std::bind(&DeviceConfigSimulationServices::GetAllSafetyInformation, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceResetSafetyDefaults = m_node_handle->create_service<kortex_driver::srv::ResetSafetyDefaults>("device_config/reset_safety_defaults", std::bind(&DeviceConfigSimulationServices::ResetSafetyDefaults, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceOnNotificationSafetyTopic = m_node_handle->create_service<kortex_driver::srv::OnNotificationSafetyTopic>("device_config/activate_publishing_of_safety_topic", std::bind(&DeviceConfigSimulationServices::OnNotificationSafetyTopic, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceExecuteCalibration = m_node_handle->create_service<kortex_driver::srv::ExecuteCalibration>("device_config/execute_calibration", std::bind(&DeviceConfigSimulationServices::ExecuteCalibration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceGetCalibrationResult = m_node_handle->create_service<kortex_driver::srv::GetCalibrationResult>("device_config/get_calibration_result", std::bind(&DeviceConfigSimulationServices::GetCalibrationResult, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceStopCalibration = m_node_handle->create_service<kortex_driver::srv::StopCalibration>("device_config/stop_calibration", std::bind(&DeviceConfigSimulationServices::StopCalibration, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeviceConfig_SetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::DeviceConfigSetCapSenseConfig>("device_config/set_cap_sense_config", std::bind(&DeviceConfigSimulationServices::DeviceConfig_SetCapSenseConfig, this, std::placeholders::_1, std::placeholders::_2));
	m_serviceDeviceConfig_GetCapSenseConfig = m_node_handle->create_service<kortex_driver::srv::DeviceConfigGetCapSenseConfig>("device_config/get_cap_sense_config", std::bind(&DeviceConfigSimulationServices::DeviceConfig_GetCapSenseConfig, this, std::placeholders::_1, std::placeholders::_2));
}

bool DeviceConfigSimulationServices::SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}

bool DeviceConfigSimulationServices::SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res)
{
	RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The SetDeviceID service is not implemented in simulation, and has no effect.");
	return true;
}


bool DeviceConfigSimulationServices::GetRunMode(const std::shared_ptr<kortex_driver::srv::GetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::GetRunMode::Response> res)
{
	
	
	if (GetRunModeHandler)
	{
		res = GetRunModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_run_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetRunMode(const std::shared_ptr<kortex_driver::srv::SetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::SetRunMode::Response> res)
{
	
	
	if (SetRunModeHandler)
	{
		res = SetRunModeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/set_run_mode is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetDeviceType(const std::shared_ptr<kortex_driver::srv::GetDeviceType::Request> req, std::shared_ptr<kortex_driver::srv::GetDeviceType::Response> res)
{
	
	
	if (GetDeviceTypeHandler)
	{
		res = GetDeviceTypeHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_device_type is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetFirmwareVersion(const std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Response> res)
{
	
	
	if (GetFirmwareVersionHandler)
	{
		res = GetFirmwareVersionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_firmware_version is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetBootloaderVersion(const std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Response> res)
{
	
	
	if (GetBootloaderVersionHandler)
	{
		res = GetBootloaderVersionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_bootloader_version is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetModelNumber(const std::shared_ptr<kortex_driver::srv::GetModelNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetModelNumber::Response> res)
{
	
	
	if (GetModelNumberHandler)
	{
		res = GetModelNumberHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_model_number is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetPartNumber(const std::shared_ptr<kortex_driver::srv::GetPartNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumber::Response> res)
{
	
	
	if (GetPartNumberHandler)
	{
		res = GetPartNumberHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_part_number is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSerialNumber(const std::shared_ptr<kortex_driver::srv::GetSerialNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetSerialNumber::Response> res)
{
	
	
	if (GetSerialNumberHandler)
	{
		res = GetSerialNumberHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_serial_number is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetMACAddress(const std::shared_ptr<kortex_driver::srv::GetMACAddress::Request> req, std::shared_ptr<kortex_driver::srv::GetMACAddress::Response> res)
{
	
	
	if (GetMACAddressHandler)
	{
		res = GetMACAddressHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_m_a_c_address is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetIPv4Settings(const std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Response> res)
{
	
	
	if (GetIPv4SettingsHandler)
	{
		res = GetIPv4SettingsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_i_pv4_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetIPv4Settings(const std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Response> res)
{
	
	
	if (SetIPv4SettingsHandler)
	{
		res = SetIPv4SettingsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/set_i_pv4_settings is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetPartNumberRevision(const std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Response> res)
{
	
	
	if (GetPartNumberRevisionHandler)
	{
		res = GetPartNumberRevisionHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_part_number_revision is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::RebootRequest(const std::shared_ptr<kortex_driver::srv::RebootRequest::Request> req, std::shared_ptr<kortex_driver::srv::RebootRequest::Response> res)
{
	
	
	if (RebootRequestHandler)
	{
		res = RebootRequestHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/reboot_request is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyEnable(const std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Response> res)
{
	
	
	if (SetSafetyEnableHandler)
	{
		res = SetSafetyEnableHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/set_safety_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyErrorThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Response> res)
{
	
	
	if (SetSafetyErrorThresholdHandler)
	{
		res = SetSafetyErrorThresholdHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/set_safety_error_threshold is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyWarningThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Response> res)
{
	
	
	if (SetSafetyWarningThresholdHandler)
	{
		res = SetSafetyWarningThresholdHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/set_safety_warning_threshold is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::SetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Response> res)
{
	
	
	if (SetSafetyConfigurationHandler)
	{
		res = SetSafetyConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/set_safety_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Response> res)
{
	
	
	if (GetSafetyConfigurationHandler)
	{
		res = GetSafetyConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_safety_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Response> res)
{
	
	
	if (GetSafetyInformationHandler)
	{
		res = GetSafetyInformationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_safety_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyEnable(const std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Response> res)
{
	
	
	if (GetSafetyEnableHandler)
	{
		res = GetSafetyEnableHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_safety_enable is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetSafetyStatus(const std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Response> res)
{
	
	
	if (GetSafetyStatusHandler)
	{
		res = GetSafetyStatusHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_safety_status is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::ClearAllSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Response> res)
{
	
	
	if (ClearAllSafetyStatusHandler)
	{
		res = ClearAllSafetyStatusHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/clear_all_safety_status is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::ClearSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Response> res)
{
	
	
	if (ClearSafetyStatusHandler)
	{
		res = ClearSafetyStatusHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/clear_safety_status is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetAllSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Response> res)
{
	
	
	if (GetAllSafetyConfigurationHandler)
	{
		res = GetAllSafetyConfigurationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_all_safety_configuration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetAllSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Response> res)
{
	
	
	if (GetAllSafetyInformationHandler)
	{
		res = GetAllSafetyInformationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_all_safety_information is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::ResetSafetyDefaults(const std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Request> req, std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Response> res)
{
	
	
	if (ResetSafetyDefaultsHandler)
	{
		res = ResetSafetyDefaultsHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/reset_safety_defaults is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::OnNotificationSafetyTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Response> res)
{
	
	m_is_activated_SafetyTopic = true;
	
	if (OnNotificationSafetyTopicHandler)
	{
		res = OnNotificationSafetyTopicHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/activate_publishing_of_safety_topic is not implemented, so the service calls will return the default response.");
	}
	return true;
}
void DeviceConfigSimulationServices::cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif)
{
	kortex_driver::msg::SafetyNotification ros_msg;
	ToRosData(notif, ros_msg);
	m_pub_SafetyTopic->publish(ros_msg);
}

bool DeviceConfigSimulationServices::ExecuteCalibration(const std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Response> res)
{
	
	
	if (ExecuteCalibrationHandler)
	{
		res = ExecuteCalibrationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/execute_calibration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::GetCalibrationResult(const std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Request> req, std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Response> res)
{
	
	
	if (GetCalibrationResultHandler)
	{
		res = GetCalibrationResultHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_calibration_result is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::StopCalibration(const std::shared_ptr<kortex_driver::srv::StopCalibration::Request> req, std::shared_ptr<kortex_driver::srv::StopCalibration::Response> res)
{
	
	
	if (StopCalibrationHandler)
	{
		res = StopCalibrationHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/stop_calibration is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::DeviceConfig_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response> res)
{
	
	
	if (DeviceConfig_SetCapSenseConfigHandler)
	{
		res = DeviceConfig_SetCapSenseConfigHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/set_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}

bool DeviceConfigSimulationServices::DeviceConfig_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response> res)
{
	
	
	if (DeviceConfig_GetCapSenseConfigHandler)
	{
		res = DeviceConfig_GetCapSenseConfigHandler(req);
	}
	else
	{
		RCLCPP_WARN_ONCE(m_node_handle->get_logger(), "The simulation handler for device_config/get_cap_sense_config is not implemented, so the service calls will return the default response.");
	}
	return true;
}
