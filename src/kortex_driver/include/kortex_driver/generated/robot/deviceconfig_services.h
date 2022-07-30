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

        virtual bool SetDeviceID(const std::shared_ptr<kortex_driver::srv::SetDeviceID::Request> req, std::shared_ptr<kortex_driver::srv::SetDeviceID::Response> res) override;
        virtual bool SetApiOptions(const std::shared_ptr<kortex_driver::srv::SetApiOptions::Request> req, std::shared_ptr<kortex_driver::srv::SetApiOptions::Response> res) override;
        virtual bool GetRunMode(const std::shared_ptr<kortex_driver::srv::GetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::GetRunMode::Response> res) override;
        virtual bool SetRunMode(const std::shared_ptr<kortex_driver::srv::SetRunMode::Request> req, std::shared_ptr<kortex_driver::srv::SetRunMode::Response> res) override;
        virtual bool GetDeviceType(const std::shared_ptr<kortex_driver::srv::GetDeviceType::Request> req, std::shared_ptr<kortex_driver::srv::GetDeviceType::Response> res) override;
        virtual bool GetFirmwareVersion(const std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetFirmwareVersion::Response> res) override;
        virtual bool GetBootloaderVersion(const std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Request> req, std::shared_ptr<kortex_driver::srv::GetBootloaderVersion::Response> res) override;
        virtual bool GetModelNumber(const std::shared_ptr<kortex_driver::srv::GetModelNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetModelNumber::Response> res) override;
        virtual bool GetPartNumber(const std::shared_ptr<kortex_driver::srv::GetPartNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumber::Response> res) override;
        virtual bool GetSerialNumber(const std::shared_ptr<kortex_driver::srv::GetSerialNumber::Request> req, std::shared_ptr<kortex_driver::srv::GetSerialNumber::Response> res) override;
        virtual bool GetMACAddress(const std::shared_ptr<kortex_driver::srv::GetMACAddress::Request> req, std::shared_ptr<kortex_driver::srv::GetMACAddress::Response> res) override;
        virtual bool GetIPv4Settings(const std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::GetIPv4Settings::Response> res) override;
        virtual bool SetIPv4Settings(const std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Request> req, std::shared_ptr<kortex_driver::srv::SetIPv4Settings::Response> res) override;
        virtual bool GetPartNumberRevision(const std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Request> req, std::shared_ptr<kortex_driver::srv::GetPartNumberRevision::Response> res) override;
        virtual bool RebootRequest(const std::shared_ptr<kortex_driver::srv::RebootRequest::Request> req, std::shared_ptr<kortex_driver::srv::RebootRequest::Response> res) override;
        virtual bool SetSafetyEnable(const std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyEnable::Response> res) override;
        virtual bool SetSafetyErrorThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyErrorThreshold::Response> res) override;
        virtual bool SetSafetyWarningThreshold(const std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyWarningThreshold::Response> res) override;
        virtual bool SetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::SetSafetyConfiguration::Response> res) override;
        virtual bool GetSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyConfiguration::Response> res) override;
        virtual bool GetSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyInformation::Response> res) override;
        virtual bool GetSafetyEnable(const std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyEnable::Response> res) override;
        virtual bool GetSafetyStatus(const std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::GetSafetyStatus::Response> res) override;
        virtual bool ClearAllSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearAllSafetyStatus::Response> res) override;
        virtual bool ClearSafetyStatus(const std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Request> req, std::shared_ptr<kortex_driver::srv::ClearSafetyStatus::Response> res) override;
        virtual bool GetAllSafetyConfiguration(const std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyConfiguration::Response> res) override;
        virtual bool GetAllSafetyInformation(const std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Request> req, std::shared_ptr<kortex_driver::srv::GetAllSafetyInformation::Response> res) override;
        virtual bool ResetSafetyDefaults(const std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Request> req, std::shared_ptr<kortex_driver::srv::ResetSafetyDefaults::Response> res) override;
        virtual bool OnNotificationSafetyTopic(const std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Request> req, std::shared_ptr<kortex_driver::srv::OnNotificationSafetyTopic::Response> res) override;
        virtual void cb_SafetyTopic(Kinova::Api::Common::SafetyNotification notif) override;
        virtual bool ExecuteCalibration(const std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Request> req, std::shared_ptr<kortex_driver::srv::ExecuteCalibration::Response> res) override;
        virtual bool GetCalibrationResult(const std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Request> req, std::shared_ptr<kortex_driver::srv::GetCalibrationResult::Response> res) override;
        virtual bool StopCalibration(const std::shared_ptr<kortex_driver::srv::StopCalibration::Request> req, std::shared_ptr<kortex_driver::srv::StopCalibration::Response> res) override;
        virtual bool DeviceConfig_SetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigSetCapSenseConfig::Response> res) override;
        virtual bool DeviceConfig_GetCapSenseConfig(const std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Request> req, std::shared_ptr<kortex_driver::srv::DeviceConfigGetCapSenseConfig::Response> res) override;

private:
        uint32_t m_current_device_id;
        Kinova::Api::RouterClientSendOptions m_api_options;

        Kinova::Api::DeviceConfig::DeviceConfigClient* m_deviceconfig;
};
#endif
