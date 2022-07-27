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
 
#ifndef _KORTEX_DEVICECONFIG_ROBOT_SERVICES_H_
#define _KORTEX_DEVICECONFIG_ROBOT_SERVICES_H_

#include "kortex_driver/generated/interfaces/deviceconfig_services_interface.h"

#include <DeviceConfig.pb.h>
#include <DeviceConfigClientRpc.h>

using namespace std;

class DeviceConfigRobotServices : public IDeviceConfigServices
{
    public:
        DeviceConfigRobotServices(rclcpp::Node::SharedPtr node_handle, Kinova::Api::DeviceConfig::DeviceConfigClient* deviceconfig, uint32_t device_id, uint32_t timeout_ms);

        virtual bool SetDeviceID(kortex_driver::srv::SetDeviceID::Request  &req, kortex_driver::srv::SetDeviceID::Response &res) override;
        virtual bool SetApiOptions(kortex_driver::srv::SetApiOptions::Request  &req, kortex_driver::srv::SetApiOptions::Response &res) override;
        virtual bool GetRunMode(kortex_driver::srv::GetRunMode::Request  &req, kortex_driver::srv::GetRunMode::Response &res) override;
        virtual bool SetRunMode(kortex_driver::srv::SetRunMode::Request  &req, kortex_driver::srv::SetRunMode::Response &res) override;
        virtual bool GetDeviceType(kortex_driver::srv::GetDeviceType::Request  &req, kortex_driver::srv::GetDeviceType::Response &res) override;
        virtual bool GetFirmwareVersion(kortex_driver::srv::GetFirmwareVersion::Request  &req, kortex_driver::srv::GetFirmwareVersion::Response &res) override;
        virtual bool GetBootloaderVersion(kortex_driver::srv::GetBootloaderVersion::Request  &req, kortex_driver::srv::GetBootloaderVersion::Response &res) override;
        virtual bool GetModelNumber(kortex_driver::srv::GetModelNumber::Request  &req, kortex_driver::srv::GetModelNumber::Response &res) override;
        virtual bool GetPartNumber(kortex_driver::srv::GetPartNumber::Request  &req, kortex_driver::srv::GetPartNumber::Response &res) override;
        virtual bool GetSerialNumber(kortex_driver::srv::GetSerialNumber::Request  &req, kortex_driver::srv::GetSerialNumber::Response &res) override;
        virtual bool GetMACAddress(kortex_driver::srv::GetMACAddress::Request  &req, kortex_driver::srv::GetMACAddress::Response &res) override;
        virtual bool GetIPv4Settings(kortex_driver::srv::GetIPv4Settings::Request  &req, kortex_driver::srv::GetIPv4Settings::Response &res) override;
        virtual bool SetIPv4Settings(kortex_driver::srv::SetIPv4Settings::Request  &req, kortex_driver::srv::SetIPv4Settings::Response &res) override;
        virtual bool GetPartNumberRevision(kortex_driver::srv::GetPartNumberRevision::Request  &req, kortex_driver::srv::GetPartNumberRevision::Response &res) override;
        virtual bool RebootRequest(kortex_driver::srv::RebootRequest::Request  &req, kortex_driver::srv::RebootRequest::Response &res) override;
        virtual bool SetSafetyEnable(kortex_driver::srv::SetSafetyEnable::Request  &req, kortex_driver::srv::SetSafetyEnable::Response &res) override;
        virtual bool SetSafetyErrorThreshold(kortex_driver::srv::SetSafetyErrorThreshold::Request  &req, kortex_driver::srv::SetSafetyErrorThreshold::Response &res) override;
        virtual bool SetSafetyWarningThreshold(kortex_driver::srv::SetSafetyWarningThreshold::Request  &req, kortex_driver::srv::SetSafetyWarningThreshold::Response &res) override;
        virtual bool SetSafetyConfiguration(kortex_driver::srv::SetSafetyConfiguration::Request  &req, kortex_driver::srv::SetSafetyConfiguration::Response &res) override;
        virtual bool GetSafetyConfiguration(kortex_driver::srv::GetSafetyConfiguration::Request  &req, kortex_driver::srv::GetSafetyConfiguration::Response &res) override;
        virtual bool GetSafetyInformation(kortex_driver::srv::GetSafetyInformation::Request  &req, kortex_driver::srv::GetSafetyInformation::Response &res) override;
        virtual bool GetSafetyEnable(kortex_driver::srv::GetSafetyEnable::Request  &req, kortex_driver::srv::GetSafetyEnable::Response &res) override;
        virtual bool GetSafetyStatus(kortex_driver::srv::GetSafetyStatus::Request  &req, kortex_driver::srv::GetSafetyStatus::Response &res) override;
        virtual bool ClearAllSafetyStatus(kortex_driver::srv::ClearAllSafetyStatus::Request  &req, kortex_driver::srv::ClearAllSafetyStatus::Response &res) override;
        virtual bool ClearSafetyStatus(kortex_driver::srv::ClearSafetyStatus::Request  &req, kortex_driver::srv::ClearSafetyStatus::Response &res) override;
        virtual bool GetAllSafetyConfiguration(kortex_driver::srv::GetAllSafetyConfiguration::Request  &req, kortex_driver::srv::GetAllSafetyConfiguration::Response &res) override;
        virtual bool GetAllSafetyInformation(kortex_driver::srv::GetAllSafetyInformation::Request  &req, kortex_driver::srv::GetAllSafetyInformation::Response &res) override;
        virtual bool ResetSafetyDefaults(kortex_driver::srv::ResetSafetyDefaults::Request  &req, kortex_driver::srv::ResetSafetyDefaults::Response &res) override;
        virtual bool OnNotificationSafetyTopic(kortex_driver::srv::OnNotificationSafetyTopic::Request  &req, kortex_driver::srv::OnNotificationSafetyTopic::Response &res) override;
        virtual void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif) override;
        virtual bool ExecuteCalibration(kortex_driver::srv::ExecuteCalibration::Request  &req, kortex_driver::srv::ExecuteCalibration::Response &res) override;
        virtual bool GetCalibrationResult(kortex_driver::srv::GetCalibrationResult::Request  &req, kortex_driver::srv::GetCalibrationResult::Response &res) override;
        virtual bool StopCalibration(kortex_driver::srv::StopCalibration::Request  &req, kortex_driver::srv::StopCalibration::Response &res) override;
        virtual bool DeviceConfig_SetCapSenseConfig(kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response &res) override;
        virtual bool DeviceConfig_GetCapSenseConfig(kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request  &req, kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response &res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::DeviceConfig::DeviceConfigClient* m_deviceconfig;
};
#endif
