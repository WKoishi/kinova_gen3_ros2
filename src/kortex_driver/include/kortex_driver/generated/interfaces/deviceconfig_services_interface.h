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

#include <memory>
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
        IDeviceConfigServices(rclcpp::Node::SharedPtr node_handle) : m_node_handle(node_handle) {}

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) = 0;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) = 0;
        virtual bool GetRunMode(const std::shared_ptr<kortex_driver::srv::GetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::GetRunMode::Response> res) = 0;
        virtual bool SetRunMode(const std::shared_ptr<kortex_driver::srv::SetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::SetRunMode::Response> res) = 0;
        virtual bool GetDeviceType(const std::shared_ptr<kortex_driver::srv::GetDeviceType::Request> req, std::shared_ptr<kortex_driver::srv::GetDeviceType::Response> res) = 0;
        virtual bool GetFirmwareVersion(const std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Response> res) = 0;
        virtual bool GetBootloaderVersion(const std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Response> res) = 0;
        virtual bool GetModelNumber(const std::shared_ptr<kortex_driver::srv::GetModelNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetModelNumber::Response> res) = 0;
        virtual bool GetPartNumber(const std::shared_ptr<kortex_driver::srv::GetPartNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumber::Response> res) = 0;
        virtual bool GetSerialNumber(const std::shared_ptr<kortex_driver::srv::GetSerialNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetSerialNumber::Response> res) = 0;
        virtual bool GetMACAddress(const std::shared_ptr<kortex_driver::srv::GetMACAddress::Request> req, std::shared_ptr<kortex_driver::srv::GetMACAddress::Response> res) = 0;
        virtual bool GetIPv4Settings(const std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Response> res) = 0;
        virtual bool SetIPv4Settings(const std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Response> res) = 0;
        virtual bool GetPartNumberRevision(const std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Response> res) = 0;
        virtual bool RebootRequest(const std::shared_ptr<kortex_driver::srv::RebootRequest::Request> req, std::shared_ptr<kortex_driver::srv::RebootRequest::Response> res) = 0;
        virtual bool SetSafetyEnable(const std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Response> res) = 0;
        virtual bool SetSafetyErrorThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Response> res) = 0;
        virtual bool SetSafetyWarningThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Response> res) = 0;
        virtual bool SetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Response> res) = 0;
        virtual bool GetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Response> res) = 0;
        virtual bool GetSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Response> res) = 0;
        virtual bool GetSafetyEnable(const std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Response> res) = 0;
        virtual bool GetSafetyStatus(const std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Response> res) = 0;
        virtual bool ClearAllSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Response> res) = 0;
        virtual bool ClearSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Response> res) = 0;
        virtual bool GetAllSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Response> res) = 0;
        virtual bool GetAllSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Response> res) = 0;
        virtual bool ResetSafetyDefaults(const std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Request> req, std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Response> res) = 0;
        virtual bool OnNotificationSafetyTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Response> res) = 0;
        virtual void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif) = 0;
        virtual bool ExecuteCalibration(const std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Response> res) = 0;
        virtual bool GetCalibrationResult(const std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Request> req, std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Response> res) = 0;
        virtual bool StopCalibration(const std::shared_ptr<kortex_driver::srv::StopCalibration::Request> req, std::shared_ptr<kortex_driver::srv::StopCalibration::Response> res) = 0;
        virtual bool DeviceConfig_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response> res) = 0;
        virtual bool DeviceConfig_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response> res) = 0;

protected:
        rclcpp::Node::SharedPtr m_node_handle;
        rclcpp::Publisher<kortex_driver::msg::KortexError>::SharedPtr m_pub_Error;
        rclcpp::Publisher<kortex_driver::msg::SafetyNotification>::SharedPtr m_pub_SafetyTopic;
        bool m_is_activated_SafetyTopic;

        rclcpp::Service<kortex_driver::srv::SetDeviceID>::SharedPtr m_serviceSetDeviceID;
        rclcpp::Service<kortex_driver::srv::SetApiOptions>::SharedPtr m_serviceSetApiOptions;

	rclcpp::Service<kortex_driver::srv::GetRunMode>::SharedPtr m_serviceGetRunMode;
	rclcpp::Service<kortex_driver::srv::SetRunMode>::SharedPtr m_serviceSetRunMode;
	rclcpp::Service<kortex_driver::srv::GetDeviceType>::SharedPtr m_serviceGetDeviceType;
	rclcpp::Service<kortex_driver::srv::GetFirmwareVersion>::SharedPtr m_serviceGetFirmwareVersion;
	rclcpp::Service<kortex_driver::srv::GetBootloaderVersion>::SharedPtr m_serviceGetBootloaderVersion;
	rclcpp::Service<kortex_driver::srv::GetModelNumber>::SharedPtr m_serviceGetModelNumber;
	rclcpp::Service<kortex_driver::srv::GetPartNumber>::SharedPtr m_serviceGetPartNumber;
	rclcpp::Service<kortex_driver::srv::GetSerialNumber>::SharedPtr m_serviceGetSerialNumber;
	rclcpp::Service<kortex_driver::srv::GetMACAddress>::SharedPtr m_serviceGetMACAddress;
	rclcpp::Service<kortex_driver::srv::GetIPv4Settings>::SharedPtr m_serviceGetIPv4Settings;
	rclcpp::Service<kortex_driver::srv::SetIPv4Settings>::SharedPtr m_serviceSetIPv4Settings;
	rclcpp::Service<kortex_driver::srv::GetPartNumberRevision>::SharedPtr m_serviceGetPartNumberRevision;
	rclcpp::Service<kortex_driver::srv::RebootRequest>::SharedPtr m_serviceRebootRequest;
	rclcpp::Service<kortex_driver::srv::SetSafetyEnable>::SharedPtr m_serviceSetSafetyEnable;
	rclcpp::Service<kortex_driver::srv::SetSafetyErrorThreshold>::SharedPtr m_serviceSetSafetyErrorThreshold;
	rclcpp::Service<kortex_driver::srv::SetSafetyWarningThreshold>::SharedPtr m_serviceSetSafetyWarningThreshold;
	rclcpp::Service<kortex_driver::srv::SetSafetyConfiguration>::SharedPtr m_serviceSetSafetyConfiguration;
	rclcpp::Service<kortex_driver::srv::GetSafetyConfiguration>::SharedPtr m_serviceGetSafetyConfiguration;
	rclcpp::Service<kortex_driver::srv::GetSafetyInformation>::SharedPtr m_serviceGetSafetyInformation;
	rclcpp::Service<kortex_driver::srv::GetSafetyEnable>::SharedPtr m_serviceGetSafetyEnable;
	rclcpp::Service<kortex_driver::srv::GetSafetyStatus>::SharedPtr m_serviceGetSafetyStatus;
	rclcpp::Service<kortex_driver::srv::ClearAllSafetyStatus>::SharedPtr m_serviceClearAllSafetyStatus;
	rclcpp::Service<kortex_driver::srv::ClearSafetyStatus>::SharedPtr m_serviceClearSafetyStatus;
	rclcpp::Service<kortex_driver::srv::GetAllSafetyConfiguration>::SharedPtr m_serviceGetAllSafetyConfiguration;
	rclcpp::Service<kortex_driver::srv::GetAllSafetyInformation>::SharedPtr m_serviceGetAllSafetyInformation;
	rclcpp::Service<kortex_driver::srv::ResetSafetyDefaults>::SharedPtr m_serviceResetSafetyDefaults;
	rclcpp::Service<kortex_driver::srv::OnNotificationSafetyTopic>::SharedPtr m_serviceOnNotificationSafetyTopic;
	rclcpp::Service<kortex_driver::srv::ExecuteCalibration>::SharedPtr m_serviceExecuteCalibration;
	rclcpp::Service<kortex_driver::srv::GetCalibrationResult>::SharedPtr m_serviceGetCalibrationResult;
	rclcpp::Service<kortex_driver::srv::StopCalibration>::SharedPtr m_serviceStopCalibration;
	rclcpp::Service<kortex_driver::srv::DeviceConfigSetCapSenseConfig>::SharedPtr m_serviceDeviceConfig_SetCapSenseConfig;
	rclcpp::Service<kortex_driver::srv::DeviceConfigGetCapSenseConfig>::SharedPtr m_serviceDeviceConfig_GetCapSenseConfig;
};
#endif
