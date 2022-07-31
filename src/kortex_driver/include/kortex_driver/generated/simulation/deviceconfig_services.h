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
 
#ifndef _KORTEX_DEVICECONFIG_SIMULATION_SERVICES_H_
#define _KORTEX_DEVICECONFIG_SIMULATION_SERVICES_H_

#include "kortex_driver/generated/interfaces/deviceconfig_services_interface.h"

using namespace std;

class DeviceConfigSimulationServices : public IDeviceConfigServices
{
    public:
        DeviceConfigSimulationServices(rclcpp::Node::SharedPtr node_handle);

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetRunMode::Response>(const std::shared_ptr<kortex_driver::srv::GetRunMode::Request>)> GetRunModeHandler = nullptr;
        virtual bool GetRunMode(const std::shared_ptr<kortex_driver::srv::GetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::GetRunMode::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetRunMode::Response>(const std::shared_ptr<kortex_driver::srv::SetRunMode::Request>)> SetRunModeHandler = nullptr;
        virtual bool SetRunMode(const std::shared_ptr<kortex_driver::srv::SetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::SetRunMode::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetDeviceType::Response>(const std::shared_ptr<kortex_driver::srv::GetDeviceType::Request>)> GetDeviceTypeHandler = nullptr;
        virtual bool GetDeviceType(const std::shared_ptr<kortex_driver::srv::GetDeviceType::Request> req, std::shared_ptr<kortex_driver::srv::GetDeviceType::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Response>(const std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Request>)> GetFirmwareVersionHandler = nullptr;
        virtual bool GetFirmwareVersion(const std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Response>(const std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Request>)> GetBootloaderVersionHandler = nullptr;
        virtual bool GetBootloaderVersion(const std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetModelNumber::Response>(const std::shared_ptr<kortex_driver::srv::GetModelNumber::Request>)> GetModelNumberHandler = nullptr;
        virtual bool GetModelNumber(const std::shared_ptr<kortex_driver::srv::GetModelNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetModelNumber::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetPartNumber::Response>(const std::shared_ptr<kortex_driver::srv::GetPartNumber::Request>)> GetPartNumberHandler = nullptr;
        virtual bool GetPartNumber(const std::shared_ptr<kortex_driver::srv::GetPartNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumber::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetSerialNumber::Response>(const std::shared_ptr<kortex_driver::srv::GetSerialNumber::Request>)> GetSerialNumberHandler = nullptr;
        virtual bool GetSerialNumber(const std::shared_ptr<kortex_driver::srv::GetSerialNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetSerialNumber::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetMACAddress::Response>(const std::shared_ptr<kortex_driver::srv::GetMACAddress::Request>)> GetMACAddressHandler = nullptr;
        virtual bool GetMACAddress(const std::shared_ptr<kortex_driver::srv::GetMACAddress::Request> req, std::shared_ptr<kortex_driver::srv::GetMACAddress::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Response>(const std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Request>)> GetIPv4SettingsHandler = nullptr;
        virtual bool GetIPv4Settings(const std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Response>(const std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Request>)> SetIPv4SettingsHandler = nullptr;
        virtual bool SetIPv4Settings(const std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Response>(const std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Request>)> GetPartNumberRevisionHandler = nullptr;
        virtual bool GetPartNumberRevision(const std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::RebootRequest::Response>(const std::shared_ptr<kortex_driver::srv::RebootRequest::Request>)> RebootRequestHandler = nullptr;
        virtual bool RebootRequest(const std::shared_ptr<kortex_driver::srv::RebootRequest::Request> req, std::shared_ptr<kortex_driver::srv::RebootRequest::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Response>(const std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Request>)> SetSafetyEnableHandler = nullptr;
        virtual bool SetSafetyEnable(const std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Response>(const std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Request>)> SetSafetyErrorThresholdHandler = nullptr;
        virtual bool SetSafetyErrorThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Response>(const std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Request>)> SetSafetyWarningThresholdHandler = nullptr;
        virtual bool SetSafetyWarningThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Response>(const std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Request>)> SetSafetyConfigurationHandler = nullptr;
        virtual bool SetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Response>(const std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Request>)> GetSafetyConfigurationHandler = nullptr;
        virtual bool GetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Response>(const std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Request>)> GetSafetyInformationHandler = nullptr;
        virtual bool GetSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Response>(const std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Request>)> GetSafetyEnableHandler = nullptr;
        virtual bool GetSafetyEnable(const std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Response>(const std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Request>)> GetSafetyStatusHandler = nullptr;
        virtual bool GetSafetyStatus(const std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Response>(const std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Request>)> ClearAllSafetyStatusHandler = nullptr;
        virtual bool ClearAllSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Response>(const std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Request>)> ClearSafetyStatusHandler = nullptr;
        virtual bool ClearSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Response>(const std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Request>)> GetAllSafetyConfigurationHandler = nullptr;
        virtual bool GetAllSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Response>(const std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Request>)> GetAllSafetyInformationHandler = nullptr;
        virtual bool GetAllSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Response>(const std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Request>)> ResetSafetyDefaultsHandler = nullptr;
        virtual bool ResetSafetyDefaults(const std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Request> req, std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Response>(const std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Request>)> OnNotificationSafetyTopicHandler = nullptr;
        virtual bool OnNotificationSafetyTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Response> res) override;
        virtual void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif) override;
        std::function<std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Response>(const std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Request>)> ExecuteCalibrationHandler = nullptr;
        virtual bool ExecuteCalibration(const std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Response>(const std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Request>)> GetCalibrationResultHandler = nullptr;
        virtual bool GetCalibrationResult(const std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Request> req, std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::StopCalibration::Response>(const std::shared_ptr<kortex_driver::srv::StopCalibration::Request>)> StopCalibrationHandler = nullptr;
        virtual bool StopCalibration(const std::shared_ptr<kortex_driver::srv::StopCalibration::Request> req, std::shared_ptr<kortex_driver::srv::StopCalibration::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response>(const std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request>)> DeviceConfig_SetCapSenseConfigHandler = nullptr;
        virtual bool DeviceConfig_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response> res) override;
        std::function<std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response>(const std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request>)> DeviceConfig_GetCapSenseConfigHandler = nullptr;
        virtual bool DeviceConfig_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response> res) override;

};
#endif
