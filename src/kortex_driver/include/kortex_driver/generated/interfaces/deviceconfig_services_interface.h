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
 
#ifndef _KORTEX_DEVICECONFIG_SERVICES_INTERFACE_H_
#define _KORTEX_DEVICECONFIG_SERVICES_INTERFACE_H_

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <chrono>
#include "kortex_driver/srv/get_run_mode.hpp"
#include "kortex_driver/srv/set_run_mode.hpp"
#include "kortex_driver/srv/get_device_type.hpp"
#include "kortex_driver/srv/get_firmware_version.hpp"
#include "kortex_driver/srv/get_bootloader_version.hpp"
#include "kortex_driver/srv/get_model_number.hpp"
#include "kortex_driver/srv/get_part_number.hpp"
#include "kortex_driver/srv/get_serial_number.hpp"
#include "kortex_driver/srv/get_mac_address.hpp"
#include "kortex_driver/srv/get_i_pv4_settings.hpp"
#include "kortex_driver/srv/set_i_pv4_settings.hpp"
#include "kortex_driver/srv/get_part_number_revision.hpp"
#include "kortex_driver/srv/reboot_request.hpp"
#include "kortex_driver/srv/set_safety_enable.hpp"
#include "kortex_driver/srv/set_safety_error_threshold.hpp"
#include "kortex_driver/srv/set_safety_warning_threshold.hpp"
#include "kortex_driver/srv/set_safety_configuration.hpp"
#include "kortex_driver/srv/get_safety_configuration.hpp"
#include "kortex_driver/srv/get_safety_information.hpp"
#include "kortex_driver/srv/get_safety_enable.hpp"
#include "kortex_driver/srv/get_safety_status.hpp"
#include "kortex_driver/srv/clear_all_safety_status.hpp"
#include "kortex_driver/srv/clear_safety_status.hpp"
#include "kortex_driver/srv/get_all_safety_configuration.hpp"
#include "kortex_driver/srv/get_all_safety_information.hpp"
#include "kortex_driver/srv/reset_safety_defaults.hpp"
#include "kortex_driver/srv/on_notification_safety_topic.hpp"
#include "kortex_driver/msg/safety_notification.hpp"
#include "kortex_driver/srv/execute_calibration.hpp"
#include "kortex_driver/srv/get_calibration_result.hpp"
#include "kortex_driver/srv/stop_calibration.hpp"
#include "kortex_driver/srv/device_config_set_cap_sense_config.hpp"
#include "kortex_driver/srv/device_config_get_cap_sense_config.hpp"

#include "kortex_driver/msg/kortex_error.hpp"
#include "kortex_driver/srv/set_device_id.hpp"
#include "kortex_driver/srv/set_api_options.hpp"
#include "kortex_driver/msg/api_options.hpp"

using namespace std;

class IDeviceConfigServices
{
    public:
        IDeviceConfigServices(ros::NodeHandle& node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) = 0;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) = 0;
        virtual bool GetRunMode(kortex_driver::srv::GetRunMode::Request  &req, kortex_driver::srv::GetRunMode::Response &res) = 0;
        virtual bool SetRunMode(kortex_driver::srv::SetRunMode::Request  &req, kortex_driver::srv::SetRunMode::Response &res) = 0;
        virtual bool GetDeviceType(kortex_driver::srv::GetDeviceType::Request  &req, kortex_driver::srv::GetDeviceType::Response &res) = 0;
        virtual bool GetFirmwareVersion(kortex_driver::srv::GetFirmwareVersion::Request  &req, kortex_driver::srv::GetFirmwareVersion::Response &res) = 0;
        virtual bool GetBootloaderVersion(kortex_driver::srv::GetBootloaderVersion::Request  &req, kortex_driver::srv::GetBootloaderVersion::Response &res) = 0;
        virtual bool GetModelNumber(kortex_driver::srv::GetModelNumber::Request  &req, kortex_driver::srv::GetModelNumber::Response &res) = 0;
        virtual bool GetPartNumber(kortex_driver::srv::GetPartNumber::Request  &req, kortex_driver::srv::GetPartNumber::Response &res) = 0;
        virtual bool GetSerialNumber(kortex_driver::srv::GetSerialNumber::Request  &req, kortex_driver::srv::GetSerialNumber::Response &res) = 0;
        virtual bool GetMACAddress(kortex_driver::srv::GetMACAddress::Request  &req, kortex_driver::srv::GetMACAddress::Response &res) = 0;
        virtual bool GetIPv4Settings(kortex_driver::srv::GetIPv4Settings::Request  &req, kortex_driver::srv::GetIPv4Settings::Response &res) = 0;
        virtual bool SetIPv4Settings(kortex_driver::srv::SetIPv4Settings::Request  &req, kortex_driver::srv::SetIPv4Settings::Response &res) = 0;
        virtual bool GetPartNumberRevision(kortex_driver::srv::GetPartNumberRevision::Request  &req, kortex_driver::srv::GetPartNumberRevision::Response &res) = 0;
        virtual bool RebootRequest(kortex_driver::srv::RebootRequest::Request  &req, kortex_driver::srv::RebootRequest::Response &res) = 0;
        virtual bool SetSafetyEnable(kortex_driver::srv::SetSafetyEnable::Request  &req, kortex_driver::srv::SetSafetyEnable::Response &res) = 0;
        virtual bool SetSafetyErrorThreshold(kortex_driver::srv::SetSafetyErrorThreshold::Request  &req, kortex_driver::srv::SetSafetyErrorThreshold::Response &res) = 0;
        virtual bool SetSafetyWarningThreshold(kortex_driver::srv::SetSafetyWarningThreshold::Request  &req, kortex_driver::srv::SetSafetyWarningThreshold::Response &res) = 0;
        virtual bool SetSafetyConfiguration(kortex_driver::srv::SetSafetyConfiguration::Request  &req, kortex_driver::srv::SetSafetyConfiguration::Response &res) = 0;
        virtual bool GetSafetyConfiguration(kortex_driver::srv::GetSafetyConfiguration::Request  &req, kortex_driver::srv::GetSafetyConfiguration::Response &res) = 0;
        virtual bool GetSafetyInformation(kortex_driver::srv::GetSafetyInformation::Request  &req, kortex_driver::srv::GetSafetyInformation::Response &res) = 0;
        virtual bool GetSafetyEnable(kortex_driver::srv::GetSafetyEnable::Request  &req, kortex_driver::srv::GetSafetyEnable::Response &res) = 0;
        virtual bool GetSafetyStatus(kortex_driver::srv::GetSafetyStatus::Request  &req, kortex_driver::srv::GetSafetyStatus::Response &res) = 0;
        virtual bool ClearAllSafetyStatus(kortex_driver::srv::ClearAllSafetyStatus::Request  &req, kortex_driver::srv::ClearAllSafetyStatus::Response &res) = 0;
        virtual bool ClearSafetyStatus(kortex_driver::srv::ClearSafetyStatus::Request  &req, kortex_driver::srv::ClearSafetyStatus::Response &res) = 0;
        virtual bool GetAllSafetyConfiguration(kortex_driver::srv::GetAllSafetyConfiguration::Request  &req, kortex_driver::srv::GetAllSafetyConfiguration::Response &res) = 0;
        virtual bool GetAllSafetyInformation(kortex_driver::srv::GetAllSafetyInformation::Request  &req, kortex_driver::srv::GetAllSafetyInformation::Response &res) = 0;
        virtual bool ResetSafetyDefaults(kortex_driver::srv::ResetSafetyDefaults::Request  &req, kortex_driver::srv::ResetSafetyDefaults::Response &res) = 0;
        virtual bool OnNotificationSafetyTopic(kortex_driver::srv::OnNotificationSafetyTopic::Request  &req, kortex_driver::srv::OnNotificationSafetyTopic::Response &res) = 0;
        virtual void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif) = 0;
        virtual bool ExecuteCalibration(kortex_driver::srv::ExecuteCalibration::Request  &req, kortex_driver::srv::ExecuteCalibration::Response &res) = 0;
        virtual bool GetCalibrationResult(kortex_driver::srv::GetCalibrationResult::Request  &req, kortex_driver::srv::GetCalibrationResult::Response &res) = 0;
        virtual bool StopCalibration(kortex_driver::srv::StopCalibration::Request  &req, kortex_driver::srv::StopCalibration::Response &res) = 0;
        virtual bool DeviceConfig_SetCapSenseConfig(kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response &res) = 0;
        virtual bool DeviceConfig_GetCapSenseConfig(kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response &res) = 0;

protected:
        ros::NodeHandle m_node_handle;
        ros::Publisher m_pub_Error;
        ros::Publisher m_pub_SafetyTopic;
        bool m_is_activated_SafetyTopic;

        ros::ServiceServer m_serviceSetDeviceID;
        ros::ServiceServer m_serviceSetApiOptions;

	ros::ServiceServer m_serviceGetRunMode;
	ros::ServiceServer m_serviceSetRunMode;
	ros::ServiceServer m_serviceGetDeviceType;
	ros::ServiceServer m_serviceGetFirmwareVersion;
	ros::ServiceServer m_serviceGetBootloaderVersion;
	ros::ServiceServer m_serviceGetModelNumber;
	ros::ServiceServer m_serviceGetPartNumber;
	ros::ServiceServer m_serviceGetSerialNumber;
	ros::ServiceServer m_serviceGetMACAddress;
	ros::ServiceServer m_serviceGetIPv4Settings;
	ros::ServiceServer m_serviceSetIPv4Settings;
	ros::ServiceServer m_serviceGetPartNumberRevision;
	ros::ServiceServer m_serviceRebootRequest;
	ros::ServiceServer m_serviceSetSafetyEnable;
	ros::ServiceServer m_serviceSetSafetyErrorThreshold;
	ros::ServiceServer m_serviceSetSafetyWarningThreshold;
	ros::ServiceServer m_serviceSetSafetyConfiguration;
	ros::ServiceServer m_serviceGetSafetyConfiguration;
	ros::ServiceServer m_serviceGetSafetyInformation;
	ros::ServiceServer m_serviceGetSafetyEnable;
	ros::ServiceServer m_serviceGetSafetyStatus;
	ros::ServiceServer m_serviceClearAllSafetyStatus;
	ros::ServiceServer m_serviceClearSafetyStatus;
	ros::ServiceServer m_serviceGetAllSafetyConfiguration;
	ros::ServiceServer m_serviceGetAllSafetyInformation;
	ros::ServiceServer m_serviceResetSafetyDefaults;
	ros::ServiceServer m_serviceOnNotificationSafetyTopic;
	ros::ServiceServer m_serviceExecuteCalibration;
	ros::ServiceServer m_serviceGetCalibrationResult;
	ros::ServiceServer m_serviceStopCalibration;
	ros::ServiceServer m_serviceDeviceConfig_SetCapSenseConfig;
	ros::ServiceServer m_serviceDeviceConfig_GetCapSenseConfig;
};
#endif
