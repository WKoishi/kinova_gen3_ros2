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

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) override;
        std::function<kortex_driver::srv::GetRunMode::Response(const kortex_driver::srv::GetRunMode::Request&)> GetRunModeHandler = nullptr;
        virtual bool GetRunMode(kortex_driver::srv::GetRunMode::Request  &req, kortex_driver::srv::GetRunMode::Response &res) override;
        std::function<kortex_driver::srv::SetRunMode::Response(const kortex_driver::srv::SetRunMode::Request&)> SetRunModeHandler = nullptr;
        virtual bool SetRunMode(kortex_driver::srv::SetRunMode::Request  &req, kortex_driver::srv::SetRunMode::Response &res) override;
        std::function<kortex_driver::srv::GetDeviceType::Response(const kortex_driver::srv::GetDeviceType::Request&)> GetDeviceTypeHandler = nullptr;
        virtual bool GetDeviceType(kortex_driver::srv::GetDeviceType::Request  &req, kortex_driver::srv::GetDeviceType::Response &res) override;
        std::function<kortex_driver::srv::GetFirmwareVersion::Response(const kortex_driver::srv::GetFirmwareVersion::Request&)> GetFirmwareVersionHandler = nullptr;
        virtual bool GetFirmwareVersion(kortex_driver::srv::GetFirmwareVersion::Request  &req, kortex_driver::srv::GetFirmwareVersion::Response &res) override;
        std::function<kortex_driver::srv::GetBootloaderVersion::Response(const kortex_driver::srv::GetBootloaderVersion::Request&)> GetBootloaderVersionHandler = nullptr;
        virtual bool GetBootloaderVersion(kortex_driver::srv::GetBootloaderVersion::Request  &req, kortex_driver::srv::GetBootloaderVersion::Response &res) override;
        std::function<kortex_driver::srv::GetModelNumber::Response(const kortex_driver::srv::GetModelNumber::Request&)> GetModelNumberHandler = nullptr;
        virtual bool GetModelNumber(kortex_driver::srv::GetModelNumber::Request  &req, kortex_driver::srv::GetModelNumber::Response &res) override;
        std::function<kortex_driver::srv::GetPartNumber::Response(const kortex_driver::srv::GetPartNumber::Request&)> GetPartNumberHandler = nullptr;
        virtual bool GetPartNumber(kortex_driver::srv::GetPartNumber::Request  &req, kortex_driver::srv::GetPartNumber::Response &res) override;
        std::function<kortex_driver::srv::GetSerialNumber::Response(const kortex_driver::srv::GetSerialNumber::Request&)> GetSerialNumberHandler = nullptr;
        virtual bool GetSerialNumber(kortex_driver::srv::GetSerialNumber::Request  &req, kortex_driver::srv::GetSerialNumber::Response &res) override;
        std::function<kortex_driver::srv::GetMACAddress::Response(const kortex_driver::srv::GetMACAddress::Request&)> GetMACAddressHandler = nullptr;
        virtual bool GetMACAddress(kortex_driver::srv::GetMACAddress::Request  &req, kortex_driver::srv::GetMACAddress::Response &res) override;
        std::function<kortex_driver::srv::GetIPv4Settings::Response(const kortex_driver::srv::GetIPv4Settings::Request&)> GetIPv4SettingsHandler = nullptr;
        virtual bool GetIPv4Settings(kortex_driver::srv::GetIPv4Settings::Request  &req, kortex_driver::srv::GetIPv4Settings::Response &res) override;
        std::function<kortex_driver::srv::SetIPv4Settings::Response(const kortex_driver::srv::SetIPv4Settings::Request&)> SetIPv4SettingsHandler = nullptr;
        virtual bool SetIPv4Settings(kortex_driver::srv::SetIPv4Settings::Request  &req, kortex_driver::srv::SetIPv4Settings::Response &res) override;
        std::function<kortex_driver::srv::GetPartNumberRevision::Response(const kortex_driver::srv::GetPartNumberRevision::Request&)> GetPartNumberRevisionHandler = nullptr;
        virtual bool GetPartNumberRevision(kortex_driver::srv::GetPartNumberRevision::Request  &req, kortex_driver::srv::GetPartNumberRevision::Response &res) override;
        std::function<kortex_driver::srv::RebootRequest::Response(const kortex_driver::srv::RebootRequest::Request&)> RebootRequestHandler = nullptr;
        virtual bool RebootRequest(kortex_driver::srv::RebootRequest::Request  &req, kortex_driver::srv::RebootRequest::Response &res) override;
        std::function<kortex_driver::srv::SetSafetyEnable::Response(const kortex_driver::srv::SetSafetyEnable::Request&)> SetSafetyEnableHandler = nullptr;
        virtual bool SetSafetyEnable(kortex_driver::srv::SetSafetyEnable::Request  &req, kortex_driver::srv::SetSafetyEnable::Response &res) override;
        std::function<kortex_driver::srv::SetSafetyErrorThreshold::Response(const kortex_driver::srv::SetSafetyErrorThreshold::Request&)> SetSafetyErrorThresholdHandler = nullptr;
        virtual bool SetSafetyErrorThreshold(kortex_driver::srv::SetSafetyErrorThreshold::Request  &req, kortex_driver::srv::SetSafetyErrorThreshold::Response &res) override;
        std::function<kortex_driver::srv::SetSafetyWarningThreshold::Response(const kortex_driver::srv::SetSafetyWarningThreshold::Request&)> SetSafetyWarningThresholdHandler = nullptr;
        virtual bool SetSafetyWarningThreshold(kortex_driver::srv::SetSafetyWarningThreshold::Request  &req, kortex_driver::srv::SetSafetyWarningThreshold::Response &res) override;
        std::function<kortex_driver::srv::SetSafetyConfiguration::Response(const kortex_driver::srv::SetSafetyConfiguration::Request&)> SetSafetyConfigurationHandler = nullptr;
        virtual bool SetSafetyConfiguration(kortex_driver::srv::SetSafetyConfiguration::Request  &req, kortex_driver::srv::SetSafetyConfiguration::Response &res) override;
        std::function<kortex_driver::srv::GetSafetyConfiguration::Response(const kortex_driver::srv::GetSafetyConfiguration::Request&)> GetSafetyConfigurationHandler = nullptr;
        virtual bool GetSafetyConfiguration(kortex_driver::srv::GetSafetyConfiguration::Request  &req, kortex_driver::srv::GetSafetyConfiguration::Response &res) override;
        std::function<kortex_driver::srv::GetSafetyInformation::Response(const kortex_driver::srv::GetSafetyInformation::Request&)> GetSafetyInformationHandler = nullptr;
        virtual bool GetSafetyInformation(kortex_driver::srv::GetSafetyInformation::Request  &req, kortex_driver::srv::GetSafetyInformation::Response &res) override;
        std::function<kortex_driver::srv::GetSafetyEnable::Response(const kortex_driver::srv::GetSafetyEnable::Request&)> GetSafetyEnableHandler = nullptr;
        virtual bool GetSafetyEnable(kortex_driver::srv::GetSafetyEnable::Request  &req, kortex_driver::srv::GetSafetyEnable::Response &res) override;
        std::function<kortex_driver::srv::GetSafetyStatus::Response(const kortex_driver::srv::GetSafetyStatus::Request&)> GetSafetyStatusHandler = nullptr;
        virtual bool GetSafetyStatus(kortex_driver::srv::GetSafetyStatus::Request  &req, kortex_driver::srv::GetSafetyStatus::Response &res) override;
        std::function<kortex_driver::srv::ClearAllSafetyStatus::Response(const kortex_driver::srv::ClearAllSafetyStatus::Request&)> ClearAllSafetyStatusHandler = nullptr;
        virtual bool ClearAllSafetyStatus(kortex_driver::srv::ClearAllSafetyStatus::Request  &req, kortex_driver::srv::ClearAllSafetyStatus::Response &res) override;
        std::function<kortex_driver::srv::ClearSafetyStatus::Response(const kortex_driver::srv::ClearSafetyStatus::Request&)> ClearSafetyStatusHandler = nullptr;
        virtual bool ClearSafetyStatus(kortex_driver::srv::ClearSafetyStatus::Request  &req, kortex_driver::srv::ClearSafetyStatus::Response &res) override;
        std::function<kortex_driver::srv::GetAllSafetyConfiguration::Response(const kortex_driver::srv::GetAllSafetyConfiguration::Request&)> GetAllSafetyConfigurationHandler = nullptr;
        virtual bool GetAllSafetyConfiguration(kortex_driver::srv::GetAllSafetyConfiguration::Request  &req, kortex_driver::srv::GetAllSafetyConfiguration::Response &res) override;
        std::function<kortex_driver::srv::GetAllSafetyInformation::Response(const kortex_driver::srv::GetAllSafetyInformation::Request&)> GetAllSafetyInformationHandler = nullptr;
        virtual bool GetAllSafetyInformation(kortex_driver::srv::GetAllSafetyInformation::Request  &req, kortex_driver::srv::GetAllSafetyInformation::Response &res) override;
        std::function<kortex_driver::srv::ResetSafetyDefaults::Response(const kortex_driver::srv::ResetSafetyDefaults::Request&)> ResetSafetyDefaultsHandler = nullptr;
        virtual bool ResetSafetyDefaults(kortex_driver::srv::ResetSafetyDefaults::Request  &req, kortex_driver::srv::ResetSafetyDefaults::Response &res) override;
        std::function<kortex_driver::srv::OnNotificationSafetyTopic::Response(const kortex_driver::srv::OnNotificationSafetyTopic::Request&)> OnNotificationSafetyTopicHandler = nullptr;
        virtual bool OnNotificationSafetyTopic(kortex_driver::srv::OnNotificationSafetyTopic::Request  &req, kortex_driver::srv::OnNotificationSafetyTopic::Response &res) override;
        virtual void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif) override;
        std::function<kortex_driver::srv::ExecuteCalibration::Response(const kortex_driver::srv::ExecuteCalibration::Request&)> ExecuteCalibrationHandler = nullptr;
        virtual bool ExecuteCalibration(kortex_driver::srv::ExecuteCalibration::Request  &req, kortex_driver::srv::ExecuteCalibration::Response &res) override;
        std::function<kortex_driver::srv::GetCalibrationResult::Response(const kortex_driver::srv::GetCalibrationResult::Request&)> GetCalibrationResultHandler = nullptr;
        virtual bool GetCalibrationResult(kortex_driver::srv::GetCalibrationResult::Request  &req, kortex_driver::srv::GetCalibrationResult::Response &res) override;
        std::function<kortex_driver::srv::StopCalibration::Response(const kortex_driver::srv::StopCalibration::Request&)> StopCalibrationHandler = nullptr;
        virtual bool StopCalibration(kortex_driver::srv::StopCalibration::Request  &req, kortex_driver::srv::StopCalibration::Response &res) override;
        std::function<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response(const kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request&)> DeviceConfig_SetCapSenseConfigHandler = nullptr;
        virtual bool DeviceConfig_SetCapSenseConfig(kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response &res) override;
        std::function<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response(const kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request&)> DeviceConfig_GetCapSenseConfigHandler = nullptr;
        virtual bool DeviceConfig_GetCapSenseConfig(kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response &res) override;

};
#endif
